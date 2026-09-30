"""Tests for ArUco pose estimation and per-ID static tracking."""

import math

import cv2
import numpy as np

from metr4202_aruco.localization_core import (
    MarkerDatabase,
    RelativeObservation,
    TrackingParameters,
    estimate_square_pose,
    filter_projected_observations,
    observation_variance,
    project_relative_observation,
)


def _project_square(marker_length, rvec, tvec):
    half = marker_length / 2.0
    object_points = np.array([
        [-half, half, 0.0],
        [half, half, 0.0],
        [half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float64)
    camera_matrix = np.array([
        [600.0, 0.0, 320.0],
        [0.0, 600.0, 240.0],
        [0.0, 0.0, 1.0],
    ])
    corners, _ = cv2.projectPoints(
        object_points,
        rvec,
        tvec,
        camera_matrix,
        np.zeros(5),
    )
    return camera_matrix, corners.reshape(4, 2)


def test_ippe_recovers_positive_depth_square_pose():
    marker_length = 0.10
    expected_tvec = np.array([[0.04], [-0.02], [1.50]])
    camera_matrix, corners = _project_square(
        marker_length,
        np.array([[0.08], [-0.12], [0.03]]),
        expected_tvec,
    )

    estimate = estimate_square_pose(
        corners,
        camera_matrix,
        np.zeros(5),
        marker_length,
    )

    assert estimate is not None
    assert estimate.tvec[2, 0] > 0.0
    assert np.allclose(estimate.tvec, expected_tvec, atol=1.0e-5)
    assert estimate.reprojection_error_px < 1.0e-5


def test_pose_estimation_rejects_nonpositive_marker_length():
    camera_matrix, corners = _project_square(
        0.10,
        np.zeros((3, 1)),
        np.array([[0.0], [0.0], [1.0]]),
    )

    try:
        estimate_square_pose(corners, camera_matrix, np.zeros(5), 0.0)
    except ValueError as error:
        assert 'positive' in str(error)
    else:
        raise AssertionError('Expected invalid marker length to be rejected.')


def test_confirmation_then_rejects_correlated_confirmed_view():
    database = MarkerDatabase(TrackingParameters(
        confirmation_count=2,
        min_fusion_interval_s=0.10,
        min_observer_translation_m=0.10,
        min_observer_yaw_rad=math.radians(10.0),
    ))

    first = database.observe(7, (1.0, 2.0), 0.01, 1.0, (0.0, 0.0, 0.0))
    second = database.observe(7, (1.02, 1.98), 0.01, 1.2, (0.0, 0.0, 0.0))
    repeated = database.observe(
        7,
        (1.01, 1.99),
        0.01,
        1.4,
        (0.0, 0.0, 0.0),
    )

    assert first.accepted
    assert second.accepted
    assert second.record.confirmed
    assert not repeated.accepted
    assert repeated.reason == 'correlated_view'


def test_novel_view_updates_but_large_innovation_is_rejected():
    database = MarkerDatabase(TrackingParameters(
        confirmation_count=1,
        min_fusion_interval_s=0.0,
        min_observer_translation_m=0.10,
        min_observer_yaw_rad=0.10,
        innovation_gate_squared=9.21,
    ))
    database.observe(3, (2.0, 1.0), 0.01, 1.0, (0.0, 0.0, 0.0))

    update = database.observe(
        3,
        (2.02, 1.01),
        0.01,
        2.0,
        (0.2, 0.0, 0.0),
    )
    outlier = database.observe(
        3,
        (10.0, -8.0),
        0.01,
        3.0,
        (0.4, 0.0, 0.0),
    )

    assert update.accepted
    assert not outlier.accepted
    assert outlier.reason == 'innovation_outlier'
    assert np.linalg.norm(outlier.record.state - np.array([2.01, 1.005])) < 0.05


def test_marker_ids_are_independent_and_clear_starts_new_epoch():
    database = MarkerDatabase(TrackingParameters(confirmation_count=1))
    database.observe(2, (1.0, 1.0), 0.02, 1.0, (0.0, 0.0, 0.0))
    database.observe(9, (4.0, 3.0), 0.02, 1.0, (0.0, 0.0, 0.0))

    records = database.records(confirmed_only=True)
    assert [record.marker_id for record in records] == [2, 9]
    assert np.allclose(records[0].state, [1.0, 1.0])
    assert np.allclose(records[1].state, [4.0, 3.0])

    database.clear()
    assert database.records() == []


def test_observation_variance_increases_with_distance_and_pixel_error():
    near = observation_variance(0.05, 0.8, 0.5)
    far = observation_variance(0.05, 2.0, 0.5)
    noisy = observation_variance(0.05, 0.8, 3.0)

    assert far > near
    assert noisy > near


def test_relative_observation_uses_final_map_from_odom_transform():
    half_turn = math.pi / 2.0
    observation = RelativeObservation(
        marker_id=12,
        camera_position=(1.0, 0.0, 0.0),
        stamp_s=4.5,
        odom_from_camera_translation=(2.0, 0.0, 0.0),
        odom_from_camera_quaternion=(
            0.0,
            0.0,
            math.sin(half_turn / 2.0),
            math.cos(half_turn / 2.0),
        ),
        variance_m2=0.04,
    )

    projected = project_relative_observation(
        observation,
        map_from_odom_translation=(10.0, 0.0, 0.0),
        map_from_odom_quaternion=(
            0.0,
            0.0,
            math.sin(half_turn / 2.0),
            math.cos(half_turn / 2.0),
        ),
    )

    assert projected.marker_id == 12
    assert np.allclose(projected.measurement_xy, (9.0, 2.0))
    assert np.allclose(projected.observer_pose[:2], (10.0, 2.0))
    assert math.isclose(abs(projected.observer_pose[2]), math.pi)
    assert projected.variance_m2 == 0.04
    assert projected.stamp_s == 4.5


def test_batch_filter_removes_gross_outlier_and_preserves_order():
    observations = [
        project_relative_observation(
            RelativeObservation(
                marker_id=5,
                camera_position=position,
                stamp_s=stamp,
                odom_from_camera_translation=(0.0, 0.0, 0.0),
                odom_from_camera_quaternion=(0.0, 0.0, 0.0, 1.0),
                variance_m2=0.01,
            ),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )
        for position, stamp in (
            ((1.0, 1.0, 0.0), 1.0),
            ((20.0, -10.0, 0.0), 2.0),
            ((1.02, 0.99, 0.0), 3.0),
        )
    ]

    filtered = filter_projected_observations(observations)

    assert [item.stamp_s for item in filtered] == [1.0, 3.0]
