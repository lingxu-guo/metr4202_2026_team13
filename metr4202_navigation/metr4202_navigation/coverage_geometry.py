"""Pure geometry and state model for two-dimensional visual coverage."""

from dataclasses import dataclass, field
from enum import IntEnum
import math
from typing import Dict, Iterable, Optional, Sequence, Set, Tuple

import cv2
import numpy as np

from metr4202_navigation.grid_utils import (
    GridMap,
    GridPoint,
    classify_cells,
    connected_component,
    inflate_mask,
    supercover_cells,
)


class TargetState(IntEnum):
    """Lifecycle states for a boundary target."""

    UNCOVERED = 0
    COVERED = 1
    INITIAL_UNCOVERABLE = 2
    RUNTIME_UNCOVERABLE = 3


class CandidateState(IntEnum):
    """Execution states for a candidate viewpoint."""

    AVAILABLE = 0
    COMPLETED = 1
    UNAVAILABLE = 2


class ExecutionPhase(IntEnum):
    """Current phase of the active coverage transaction."""

    IDLE = 0
    NAVIGATING = 1
    SCANNING = 2


@dataclass
class Target:
    """One oriented boundary sample."""

    target_id: int
    position: GridPoint
    normal: GridPoint
    los_point: GridPoint
    segment_id: int
    state: TargetState = TargetState.UNCOVERED


@dataclass
class Candidate:
    """One safe robot-centre location and its immutable coverage set."""

    candidate_id: int
    position: GridPoint
    coverage_set: Set[int] = field(default_factory=set)
    state: CandidateState = CandidateState.AVAILABLE


@dataclass(frozen=True)
class CoverageParameters:
    """Parameters required to construct the frozen coverage model."""

    occupied_threshold: int = 50
    safety_radius: float = 0.30
    target_spacing: float = 0.0
    candidate_spacing: float = 0.20
    minimum_candidate_spacing: float = 0.0
    visual_distance_min: float = 0.20
    visual_distance_max: float = 2.00
    viewing_angle_max: float = math.radians(60.0)
    camera_radial_offset: float = 0.10
    contour_simplification: float = 0.0
    fixed_visit_cost: float = 0.50


@dataclass(frozen=True)
class CoverageSummary:
    """Counts reported to the ROS service response."""

    covered: int
    initial_uncoverable: int
    runtime_uncoverable: int
    total: int


@dataclass(frozen=True)
class TaskDecision:
    """ROS-independent description of the next motion task."""

    candidate_id: int
    task_type: int
    position: Optional[GridPoint] = None
    spin_yaw: float = 0.0


class CoverageModel:
    """Frozen geometry, visibility graph, and execution state."""

    NAVIGATE = 0
    SPIN = 1
    MAP_COVERED = 20
    MAP_UNCOVERED = 50
    MAP_INITIAL_UNCOVERABLE = 70
    MAP_RUNTIME_UNCOVERABLE = 85

    def __init__(
        self,
        grid: GridMap,
        robot_position: GridPoint,
        parameters: CoverageParameters,
    ) -> None:
        """Build all geometry from one immutable map snapshot."""
        self.grid = grid
        self.parameters = parameters
        self.targets: list[Target] = []
        self.candidates: list[Candidate] = []
        self.target_to_candidates: Dict[int, Set[int]] = {}
        self.active_candidate_id: Optional[int] = None
        self.phase = ExecutionPhase.IDLE

        free, occupied, unknown = classify_cells(
            grid, parameters.occupied_threshold
        )
        self.free = free
        self.occupied = occupied
        self.unknown = unknown
        self.visibility_blocked = [
            is_occupied or is_unknown
            for is_occupied, is_unknown in zip(occupied, unknown)
        ]

        inflated = inflate_mask(
            self.visibility_blocked,
            grid.width,
            grid.height,
            grid.resolution,
            parameters.safety_radius,
        )
        safe = []
        for index, (is_free, is_inflated) in enumerate(
            zip(free, inflated)
        ):
            row, column = divmod(index, grid.width)
            edge_distance = min(
                row + 0.5,
                column + 0.5,
                grid.height - row - 0.5,
                grid.width - column - 0.5,
            ) * grid.resolution
            safe.append(
                is_free and not is_inflated
                and edge_distance + 1e-9 >= parameters.safety_radius
            )
        robot_index = grid.world_to_index(robot_position)
        if robot_index is None:
            raise ValueError('Robot position is outside the coverage map.')
        if not safe[robot_index]:
            robot_index = min(
                (index for index, is_safe in enumerate(safe) if is_safe),
                key=lambda index: math.dist(
                    grid.index_to_world(index), robot_position
                ),
                default=None,
            )
            if robot_index is None:
                raise ValueError('Coverage map has no safe free space.')
        self.reachable = connected_component(
            safe, robot_index, grid.width, grid.height
        )

        self._extract_targets()
        self._generate_candidates()
        self._classify_initially_uncoverable()

    def summary(self) -> CoverageSummary:
        """Return current target-state totals."""
        return CoverageSummary(
            covered=sum(
                target.state == TargetState.COVERED
                for target in self.targets
            ),
            initial_uncoverable=sum(
                target.state == TargetState.INITIAL_UNCOVERABLE
                for target in self.targets
            ),
            runtime_uncoverable=sum(
                target.state == TargetState.RUNTIME_UNCOVERABLE
                for target in self.targets
            ),
            total=len(self.targets),
        )

    def coverage_grid_data(self) -> Tuple[int, ...]:
        """Overlay target states on the frozen occupancy-map snapshot."""
        data = list(self.grid.data)
        values = {
            TargetState.COVERED: self.MAP_COVERED,
            TargetState.UNCOVERED: self.MAP_UNCOVERED,
            TargetState.INITIAL_UNCOVERABLE:
                self.MAP_INITIAL_UNCOVERABLE,
            TargetState.RUNTIME_UNCOVERABLE:
                self.MAP_RUNTIME_UNCOVERABLE,
        }
        cell_values: Dict[int, int] = {}
        for target in self.targets:
            index = self.grid.world_to_index(target.los_point)
            if index is None:
                continue
            value = values[target.state]
            cell_values[index] = max(cell_values.get(index, -1), value)
        for index, value in cell_values.items():
            data[index] = value
        return tuple(data)

    def unfinished(self) -> bool:
        """Return whether any target still requires a scan."""
        return any(
            target.state == TargetState.UNCOVERED
            for target in self.targets
        )

    def next_task(self, robot_position: GridPoint) -> Optional[TaskDecision]:
        """Return the active phase task or dynamically select a candidate."""
        if self.phase == ExecutionPhase.NAVIGATING:
            candidate = self._active_candidate()
            return TaskDecision(
                candidate.candidate_id,
                self.NAVIGATE,
                position=candidate.position,
            )
        if self.phase == ExecutionPhase.SCANNING:
            candidate = self._active_candidate()
            return TaskDecision(
                candidate.candidate_id,
                self.SPIN,
                spin_yaw=math.tau,
            )

        candidate = self._select_candidate(robot_position)
        if candidate is None:
            self._update_all_runtime_uncoverable()
            return None
        self.active_candidate_id = candidate.candidate_id
        self.phase = ExecutionPhase.NAVIGATING
        return TaskDecision(
            candidate.candidate_id,
            self.NAVIGATE,
            position=candidate.position,
        )

    def submit_result(
        self,
        candidate_id: int,
        task_type: int,
        succeeded: bool,
    ) -> None:
        """Apply one final Manager result to the active transaction."""
        candidate = self._active_candidate()
        if candidate_id != candidate.candidate_id:
            raise ValueError(
                f'Result candidate {candidate_id} does not match active '
                f'candidate {candidate.candidate_id}.'
            )

        expected_type = (
            self.NAVIGATE
            if self.phase == ExecutionPhase.NAVIGATING
            else self.SPIN
        )
        if task_type != expected_type:
            raise ValueError(
                f'Result task type {task_type} does not match expected '
                f'type {expected_type}.'
            )

        if self.phase == ExecutionPhase.NAVIGATING and succeeded:
            self.phase = ExecutionPhase.SCANNING
            return

        if self.phase == ExecutionPhase.SCANNING and succeeded:
            candidate.state = CandidateState.COMPLETED
            for target_id in candidate.coverage_set:
                target = self.targets[target_id]
                if target.state == TargetState.UNCOVERED:
                    target.state = TargetState.COVERED
        else:
            candidate.state = CandidateState.UNAVAILABLE
            self._update_runtime_uncoverable(candidate.coverage_set)

        self.active_candidate_id = None
        self.phase = ExecutionPhase.IDLE

    def stalled(self) -> bool:
        """Return whether internal state cannot explain remaining targets."""
        if self.phase != ExecutionPhase.IDLE or not self.unfinished():
            return False
        if any(
            candidate.state == CandidateState.AVAILABLE
            and self._gain(candidate) > 0
            for candidate in self.candidates
        ):
            return False
        return any(
            target.state == TargetState.UNCOVERED
            for target in self.targets
        )

    def _extract_targets(self) -> None:
        """Trace and simplify occupied contours into oriented targets."""
        image = np.asarray(self.occupied, dtype=np.uint8).reshape(
            self.grid.height, self.grid.width
        )
        contours, _ = cv2.findContours(
            image,
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_NONE,
        )
        spacing = self.parameters.target_spacing or self.grid.resolution
        epsilon = (
            self.parameters.contour_simplification
            or 1.25 * self.grid.resolution
        )
        segment_id = 0

        for contour in contours:
            if len(contour) < 2:
                continue
            approximation = cv2.approxPolyDP(
                contour,
                epsilon / self.grid.resolution,
                True,
            )
            points = [
                (
                    (float(point[0][0]) + 0.5) * self.grid.resolution,
                    (float(point[0][1]) + 0.5) * self.grid.resolution,
                )
                for point in approximation
            ]
            if len(points) < 2:
                continue

            for start, end in _closed_segments(points):
                delta_x = end[0] - start[0]
                delta_y = end[1] - start[1]
                length = math.hypot(delta_x, delta_y)
                if length < 0.5 * self.grid.resolution:
                    continue
                normals = (
                    (-delta_y / length, delta_x / length),
                    (delta_y / length, -delta_x / length),
                )
                for normal_local in normals:
                    if not self._normal_faces_free(
                        start, end, normal_local
                    ):
                        continue
                    sample_count = max(1, int(math.ceil(length / spacing)))
                    for sample_index in range(sample_count):
                        fraction = (sample_index + 0.5) / sample_count
                        centre_local = (
                            start[0] + fraction * delta_x,
                            start[1] + fraction * delta_y,
                        )
                        surface_local = _offset(
                            centre_local,
                            normal_local,
                            0.5 * self.grid.resolution,
                        )
                        los_local = _offset(
                            centre_local,
                            normal_local,
                            1.1 * self.grid.resolution,
                        )
                        normal_world = _rotate(
                            normal_local, self.grid.origin_yaw
                        )
                        self.targets.append(Target(
                            target_id=len(self.targets),
                            position=self.grid.local_to_world(surface_local),
                            normal=normal_world,
                            los_point=self.grid.local_to_world(los_local),
                            segment_id=segment_id,
                        ))
                    segment_id += 1

        if not self.targets:
            self._extract_cell_face_targets()

    def _extract_cell_face_targets(self) -> None:
        """Fallback for isolated occupied cells that have no usable contour."""
        directions = (
            (-1, 0, 0.0, -1.0),
            (0, -1, -1.0, 0.0),
            (0, 1, 1.0, 0.0),
            (1, 0, 0.0, 1.0),
        )
        for index, is_occupied in enumerate(self.occupied):
            if not is_occupied:
                continue
            row, column = divmod(index, self.grid.width)
            centre = (
                (column + 0.5) * self.grid.resolution,
                (row + 0.5) * self.grid.resolution,
            )
            for row_delta, column_delta, nx, ny in directions:
                next_row = row + row_delta
                next_column = column + column_delta
                if not (
                    0 <= next_row < self.grid.height
                    and 0 <= next_column < self.grid.width
                ):
                    continue
                neighbor = next_row * self.grid.width + next_column
                if not self.free[neighbor]:
                    continue
                normal_local = (nx, ny)
                surface = _offset(
                    centre, normal_local, 0.5 * self.grid.resolution
                )
                los_point = _offset(
                    centre, normal_local, 1.0 * self.grid.resolution
                )
                self.targets.append(Target(
                    target_id=len(self.targets),
                    position=self.grid.local_to_world(surface),
                    normal=_rotate(normal_local, self.grid.origin_yaw),
                    los_point=self.grid.local_to_world(los_point),
                    segment_id=len(self.targets),
                ))

    def _normal_faces_free(
        self,
        start: GridPoint,
        end: GridPoint,
        normal: GridPoint,
    ) -> bool:
        midpoint = (
            0.5 * (start[0] + end[0]),
            0.5 * (start[1] + end[1]),
        )
        test_point = self.grid.local_to_world(_offset(
            midpoint, normal, 1.1 * self.grid.resolution
        ))
        index = self.grid.world_to_index(test_point)
        return index is not None and self.free[index]

    def _generate_candidates(self) -> None:
        """Sample reachable cells and locally refine uncovered targets."""
        initial_spacing = max(
            self.grid.resolution,
            self.parameters.candidate_spacing,
        )
        minimum_spacing = (
            self.parameters.minimum_candidate_spacing
            or self.grid.resolution
        )
        sampled_indices: Set[int] = set()
        self._add_candidate_level(
            initial_spacing, sampled_indices, targets=None
        )

        spacing = initial_spacing
        while spacing > minimum_spacing + 1e-9:
            missing = [
                target
                for target in self.targets
                if not self.target_to_candidates.get(target.target_id)
            ]
            if not missing:
                break
            spacing = max(minimum_spacing, spacing / 2.0)
            self._add_candidate_level(
                spacing, sampled_indices, targets=missing
            )

    def _add_candidate_level(
        self,
        spacing: float,
        sampled_indices: Set[int],
        targets: Optional[Sequence[Target]],
    ) -> None:
        stride = max(1, int(round(spacing / self.grid.resolution)))
        for row in range(0, self.grid.height, stride):
            for column in range(0, self.grid.width, stride):
                index = row * self.grid.width + column
                if index in sampled_indices or not self.reachable[index]:
                    continue
                position = self.grid.index_to_world(index)
                relevant_targets = self.targets if targets is None else targets
                if not any(
                    math.dist(position, target.position)
                    <= (
                        self.parameters.visual_distance_max
                        + self.parameters.camera_radial_offset
                        + self.grid.resolution
                    )
                    for target in relevant_targets
                ):
                    continue

                sampled_indices.add(index)
                candidate = Candidate(
                    candidate_id=len(self.candidates),
                    position=position,
                )
                for target in self.targets:
                    if self._visible(candidate.position, target):
                        candidate.coverage_set.add(target.target_id)
                        self.target_to_candidates.setdefault(
                            target.target_id, set()
                        ).add(candidate.candidate_id)
                if candidate.coverage_set:
                    self.candidates.append(candidate)

    def _visible(self, candidate: GridPoint, target: Target) -> bool:
        delta_x = candidate[0] - target.position[0]
        delta_y = candidate[1] - target.position[1]
        base_distance = math.hypot(delta_x, delta_y)
        camera_distance = (
            base_distance - self.parameters.camera_radial_offset
        )
        if not (
            self.parameters.visual_distance_min - 1e-9
            <= camera_distance
            <= self.parameters.visual_distance_max + 1e-9
        ):
            return False
        if base_distance <= self.parameters.camera_radial_offset + 1e-9:
            return False

        viewing_cosine = (
            target.normal[0] * delta_x
            + target.normal[1] * delta_y
        ) / base_distance
        minimum_cosine = math.cos(self.parameters.viewing_angle_max)
        if viewing_cosine + 1e-9 < minimum_cosine:
            return False

        toward_target = (
            -delta_x / base_distance,
            -delta_y / base_distance,
        )
        camera = (
            candidate[0]
            + self.parameters.camera_radial_offset * toward_target[0],
            candidate[1]
            + self.parameters.camera_radial_offset * toward_target[1],
        )
        cells = supercover_cells(self.grid, camera, target.los_point)
        return self._line_of_sight_clear(cells)

    def _line_of_sight_clear(
        self,
        cells: Optional[Sequence[int]],
    ) -> bool:
        """Reject rays crossing occupied or unexplored map cells."""
        if cells is None:
            return False
        return all(
            not self.visibility_blocked[index]
            for index in cells
        )

    def _classify_initially_uncoverable(self) -> None:
        for target in self.targets:
            if not self.target_to_candidates.get(target.target_id):
                target.state = TargetState.INITIAL_UNCOVERABLE

    def _select_candidate(
        self, robot_position: GridPoint
    ) -> Optional[Candidate]:
        best = None
        for candidate in self.candidates:
            if candidate.state != CandidateState.AVAILABLE:
                continue
            gain = self._gain(candidate)
            if gain <= 0:
                continue
            distance = math.dist(robot_position, candidate.position)
            score = gain / (
                distance + self.parameters.fixed_visit_cost
            )
            key = (score, gain, -distance, -candidate.candidate_id)
            if best is None or key > best[0]:
                best = (key, candidate)
        return None if best is None else best[1]

    def _gain(self, candidate: Candidate) -> int:
        return sum(
            self.targets[target_id].state == TargetState.UNCOVERED
            for target_id in candidate.coverage_set
        )

    def _active_candidate(self) -> Candidate:
        if self.active_candidate_id is None:
            raise ValueError('No active coverage candidate.')
        return self.candidates[self.active_candidate_id]

    def _update_runtime_uncoverable(
        self, target_ids: Iterable[int]
    ) -> None:
        for target_id in target_ids:
            target = self.targets[target_id]
            if target.state != TargetState.UNCOVERED:
                continue
            if not any(
                self.candidates[candidate_id].state
                == CandidateState.AVAILABLE
                for candidate_id in self.target_to_candidates.get(
                    target_id, set()
                )
            ):
                target.state = TargetState.RUNTIME_UNCOVERABLE

    def _update_all_runtime_uncoverable(self) -> None:
        self._update_runtime_uncoverable(
            target.target_id
            for target in self.targets
            if target.state == TargetState.UNCOVERED
        )


def _closed_segments(
    points: Sequence[GridPoint],
) -> Iterable[Tuple[GridPoint, GridPoint]]:
    for index, start in enumerate(points):
        yield start, points[(index + 1) % len(points)]


def _offset(
    point: GridPoint,
    direction: GridPoint,
    distance: float,
) -> GridPoint:
    return (
        point[0] + direction[0] * distance,
        point[1] + direction[1] * distance,
    )


def _rotate(vector: GridPoint, yaw: float) -> GridPoint:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        cosine * vector[0] - sine * vector[1],
        sine * vector[0] + cosine * vector[1],
    )
