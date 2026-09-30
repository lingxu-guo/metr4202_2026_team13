"""Unit tests for the Coverage V1 geometry and transaction state."""

import math

from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy

from metr4202_navigation.coverage import Coverage
from metr4202_navigation.coverage_geometry import (
    CandidateState,
    CoverageModel,
    CoverageParameters,
    Target,
    TargetState,
)
from metr4202_navigation.grid_utils import GridMap


def make_obstacle_map() -> OccupancyGrid:
    """Create free space containing one rectangular occupied object."""
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.frame_id = 'map'
    occupancy_grid.info.width = 30
    occupancy_grid.info.height = 30
    occupancy_grid.info.resolution = 0.1
    occupancy_grid.info.origin.orientation.w = 1.0
    data = [0] * 900
    for row in range(10, 20):
        for column in range(12, 18):
            data[row * 30 + column] = 100
    occupancy_grid.data = data
    return occupancy_grid


def make_model(**overrides) -> CoverageModel:
    """Build a deterministic model suitable for state-machine tests."""
    values = {
        'safety_radius': 0.20,
        'candidate_spacing': 0.20,
        'minimum_candidate_spacing': 0.10,
        'visual_distance_min': 0.20,
        'visual_distance_max': 1.50,
        'camera_radial_offset': 0.10,
    }
    values.update(overrides)
    return CoverageModel(
        GridMap.from_message(make_obstacle_map()),
        (0.45, 0.45),
        CoverageParameters(**values),
    )


def test_robot_near_obstacle_uses_nearest_safe_component() -> None:
    """Seed reachability from nearby safe space after Exploration."""
    model = CoverageModel(
        GridMap.from_message(make_obstacle_map()),
        (1.05, 0.95),
        CoverageParameters(safety_radius=0.30),
    )

    assert any(model.reachable)


def test_navigation_then_spin_is_one_candidate_transaction() -> None:
    """Cover targets only after the matching candidate finishes its spin."""
    model = make_model()
    navigation = model.next_task((0.45, 0.45))
    assert navigation is not None
    assert navigation.task_type == CoverageModel.NAVIGATE

    model.submit_result(
        navigation.candidate_id,
        navigation.task_type,
        succeeded=True,
    )
    assert model.summary().covered == 0

    spin = model.next_task((0.45, 0.45))
    assert spin is not None
    assert spin.candidate_id == navigation.candidate_id
    assert spin.task_type == CoverageModel.SPIN
    assert spin.spin_yaw == pytest.approx(math.tau)

    model.submit_result(spin.candidate_id, spin.task_type, succeeded=True)
    assert model.summary().covered > 0


def test_navigation_failure_never_returns_its_spin() -> None:
    """Invalidate a failed viewpoint and dynamically choose another one."""
    model = make_model()
    failed = model.next_task((0.45, 0.45))
    model.submit_result(
        failed.candidate_id,
        failed.task_type,
        succeeded=False,
    )

    assert (
        model.candidates[failed.candidate_id].state
        == CandidateState.UNAVAILABLE
    )
    replacement = model.next_task((0.45, 0.45))
    assert replacement is not None
    assert replacement.task_type == CoverageModel.NAVIGATE
    assert replacement.candidate_id != failed.candidate_id


def test_rejects_stale_or_mismatched_result() -> None:
    """Do not let a delayed Manager result update another candidate."""
    model = make_model()
    task = model.next_task((0.45, 0.45))
    with pytest.raises(ValueError, match='does not match active'):
        model.submit_result(
            task.candidate_id + 1,
            task.task_type,
            succeeded=True,
        )
    with pytest.raises(ValueError, match='does not match expected'):
        model.submit_result(
            task.candidate_id,
            CoverageModel.SPIN,
            succeeded=True,
        )


def test_camera_offset_changes_the_distance_test() -> None:
    """Evaluate visual range from the radial camera, not the robot centre."""
    target = Target(
        target_id=0,
        position=(1.45, 0.45),
        normal=(-1.0, 0.0),
        los_point=(1.35, 0.45),
        segment_id=0,
    )
    offset_model = make_model(
        visual_distance_min=0.85,
        visual_distance_max=0.95,
        camera_radial_offset=0.10,
    )
    centred_model = make_model(
        visual_distance_min=0.85,
        visual_distance_max=0.95,
        camera_radial_offset=0.0,
    )

    assert offset_model._visible((0.45, 0.45), target)
    assert not centred_model._visible((0.45, 0.45), target)


def test_obstacle_occludes_theoretical_camera_coverage() -> None:
    """Do not cover a target when an occupied wall blocks the camera ray."""
    model = make_model(
        visual_distance_max=3.0,
    )
    target = Target(
        target_id=0,
        position=(2.45, 1.45),
        normal=(-1.0, 0.0),
        los_point=(2.35, 1.45),
        segment_id=0,
    )

    assert not model._visible((0.45, 1.45), target)
    assert model._visible((2.05, 1.45), target)


def test_coverage_grid_encodes_every_target_state() -> None:
    """Overlay target states without losing the underlying occupancy map."""
    model = make_model()
    states = (
        TargetState.COVERED,
        TargetState.UNCOVERED,
        TargetState.INITIAL_UNCOVERABLE,
        TargetState.RUNTIME_UNCOVERABLE,
    )
    values = (
        CoverageModel.MAP_COVERED,
        CoverageModel.MAP_UNCOVERED,
        CoverageModel.MAP_INITIAL_UNCOVERABLE,
        CoverageModel.MAP_RUNTIME_UNCOVERABLE,
    )
    model.targets = []
    for target_id, state in enumerate(states):
        point = (0.55 + 0.10 * target_id, 0.55)
        model.targets.append(Target(
            target_id=target_id,
            position=point,
            normal=(1.0, 0.0),
            los_point=point,
            segment_id=target_id,
            state=state,
        ))

    data = model.coverage_grid_data()
    for target_id, value in enumerate(values):
        index = model.grid.world_to_index(model.targets[target_id].los_point)
        assert data[index] == value
    obstacle_index = 10 * model.grid.width + 12
    assert data[obstacle_index] == 100


def test_node_parameter_factory_is_not_shadowed_by_rclpy() -> None:
    """Keep Coverage helpers clear of private rclpy.Node attributes."""
    rclpy.init()
    node = Coverage()
    try:
        parameters = node._coverage_parameters()
        assert isinstance(parameters, CoverageParameters)
    finally:
        node.destroy_node()
        rclpy.shutdown()
