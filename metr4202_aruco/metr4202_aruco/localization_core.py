"""Pure ArUco pose-estimation and two-dimensional tracking logic."""

from dataclasses import dataclass
import math
from typing import Dict, Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class PoseEstimate:
    """A marker pose in the camera optical frame."""

    rvec: np.ndarray
    tvec: np.ndarray
    reprojection_error_px: float


@dataclass(frozen=True)
class TrackingParameters:
    """Configuration for the per-marker static-state estimator."""

    confirmation_count: int = 3
    process_noise_m2_per_s: float = 1.0e-4
    innovation_gate_squared: float = 9.21
    min_fusion_interval_s: float = 0.20
    min_observer_translation_m: float = 0.10
    min_observer_yaw_rad: float = math.radians(10.0)


@dataclass
class MarkerRecord:
    """Mutable estimate and diagnostics for one physical marker ID."""

    marker_id: int
    state: np.ndarray
    covariance: np.ndarray
    accepted_observation_count: int
    rejected_observation_count: int
    first_seen_s: float
    last_seen_s: float
    last_fused_s: float
    last_observer_pose: Tuple[float, float, float]
    confirmed: bool


@dataclass(frozen=True)
class UpdateResult:
    """Outcome of attempting to fuse one map-frame observation."""

    accepted: bool
    reason: str
    record: MarkerRecord


def estimate_square_pose(
    corners,
    camera_matrix,
    distortion,
    marker_length_m: float,
) -> Optional[PoseEstimate]:
    """Return the best positive-depth IPPE square solution."""
    if marker_length_m <= 0.0:
        raise ValueError('marker_length_m must be positive')

    image_points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(distortion, dtype=np.float64).reshape(-1, 1)
    half = marker_length_m / 2.0
    object_points = np.array([
        [-half, half, 0.0],
        [half, half, 0.0],
        [half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float64)

    result = cv2.solvePnPGeneric(
        object_points,
        image_points,
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not result or not result[0]:
        return None

    rvecs = result[1]
    tvecs = result[2]
    best = None
    for rvec, tvec in zip(rvecs, tvecs):
        rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
        tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
        if not np.all(np.isfinite(tvec)) or float(tvec[2, 0]) <= 0.0:
            continue
        projected, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            camera_matrix,
            distortion,
        )
        projected = projected.reshape(4, 2)
        error = float(np.sqrt(np.mean(np.sum(
            (projected - image_points) ** 2,
            axis=1,
        ))))
        estimate = PoseEstimate(rvec, tvec, error)
        if best is None or error < best.reprojection_error_px:
            best = estimate
    return best


def observation_variance(
    base_stddev_m: float,
    distance_m: float,
    reprojection_error_px: float,
    reference_distance_m: float = 1.0,
    reference_error_px: float = 1.0,
) -> float:
    """Return a conservative scalar map-position variance."""
    if base_stddev_m <= 0.0:
        raise ValueError('base_stddev_m must be positive')
    distance_scale = max(1.0, distance_m / max(reference_distance_m, 1.0e-6))
    error_scale = max(
        1.0,
        reprojection_error_px / max(reference_error_px, 1.0e-6),
    )
    return (base_stddev_m * distance_scale * error_scale) ** 2


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract planar yaw from a quaternion."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def angle_difference(first: float, second: float) -> float:
    """Return the absolute wrapped separation between two angles."""
    return abs(math.atan2(
        math.sin(first - second),
        math.cos(first - second),
    ))


class MarkerDatabase:
    """Maintain independent gated static-state estimators by marker ID."""

    def __init__(self, parameters: TrackingParameters) -> None:
        """Create an empty database."""
        if parameters.confirmation_count < 1:
            raise ValueError('confirmation_count must be at least one')
        self.parameters = parameters
        self._records: Dict[int, MarkerRecord] = {}

    def clear(self) -> None:
        """Start a new localization epoch."""
        self._records.clear()

    def records(self, confirmed_only: bool = False):
        """Return records ordered by marker ID."""
        records = [
            self._records[key]
            for key in sorted(self._records)
        ]
        if confirmed_only:
            return [record for record in records if record.confirmed]
        return records

    def observe(
        self,
        marker_id: int,
        measurement_xy,
        variance_m2: float,
        stamp_s: float,
        observer_pose: Tuple[float, float, float],
    ) -> UpdateResult:
        """Gate and fuse one map-frame observation."""
        measurement = np.asarray(measurement_xy, dtype=np.float64).reshape(2)
        if not np.all(np.isfinite(measurement)):
            raise ValueError('measurement must be finite')
        if not math.isfinite(variance_m2) or variance_m2 <= 0.0:
            raise ValueError('variance_m2 must be positive and finite')
        if not math.isfinite(stamp_s):
            raise ValueError('stamp_s must be finite')

        record = self._records.get(marker_id)
        if record is None:
            covariance = np.eye(2, dtype=np.float64) * variance_m2
            record = MarkerRecord(
                marker_id=marker_id,
                state=measurement.copy(),
                covariance=covariance,
                accepted_observation_count=1,
                rejected_observation_count=0,
                first_seen_s=stamp_s,
                last_seen_s=stamp_s,
                last_fused_s=stamp_s,
                last_observer_pose=observer_pose,
                confirmed=self.parameters.confirmation_count <= 1,
            )
            self._records[marker_id] = record
            return UpdateResult(True, 'initialized', record)

        record.last_seen_s = max(record.last_seen_s, stamp_s)
        elapsed = stamp_s - record.last_fused_s
        if elapsed < self.parameters.min_fusion_interval_s:
            record.rejected_observation_count += 1
            return UpdateResult(False, 'rate_limited', record)

        if record.confirmed and not self._observer_is_novel(
            record.last_observer_pose,
            observer_pose,
        ):
            record.rejected_observation_count += 1
            return UpdateResult(False, 'correlated_view', record)

        predicted_covariance = record.covariance + (
            np.eye(2, dtype=np.float64)
            * self.parameters.process_noise_m2_per_s
            * max(0.0, elapsed)
        )
        measurement_covariance = np.eye(2, dtype=np.float64) * variance_m2
        innovation = measurement - record.state
        innovation_covariance = predicted_covariance + measurement_covariance
        mahalanobis_squared = float(
            innovation.T
            @ np.linalg.solve(innovation_covariance, innovation)
        )
        if mahalanobis_squared > self.parameters.innovation_gate_squared:
            record.rejected_observation_count += 1
            return UpdateResult(False, 'innovation_outlier', record)

        gain = predicted_covariance @ np.linalg.inv(innovation_covariance)
        identity = np.eye(2, dtype=np.float64)
        record.state = record.state + gain @ innovation
        record.covariance = (
            (identity - gain)
            @ predicted_covariance
            @ (identity - gain).T
            + gain @ measurement_covariance @ gain.T
        )
        record.accepted_observation_count += 1
        record.last_fused_s = stamp_s
        record.last_observer_pose = observer_pose
        record.confirmed = (
            record.accepted_observation_count
            >= self.parameters.confirmation_count
        )
        return UpdateResult(True, 'updated', record)

    def _observer_is_novel(
        self,
        previous: Tuple[float, float, float],
        current: Tuple[float, float, float],
    ) -> bool:
        translation = math.hypot(
            current[0] - previous[0],
            current[1] - previous[1],
        )
        yaw = angle_difference(current[2], previous[2])
        return (
            translation >= self.parameters.min_observer_translation_m
            or yaw >= self.parameters.min_observer_yaw_rad
        )
