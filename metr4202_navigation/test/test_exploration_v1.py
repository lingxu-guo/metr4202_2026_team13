"""Unit tests for the Exploration V1 decision algorithm."""

import math
import threading
import time

from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.parameter import Parameter

from metr4202_interfaces.msg import MotionTask
from metr4202_interfaces.srv import Exploration as ExplorationSrv
from metr4202_navigation.exploration import Exploration


@pytest.fixture
def exploration_node():
    """Create an Exploration node for one test."""
    rclpy.init()
    node = Exploration()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def make_frontier_map() -> OccupancyGrid:
    """Make a free square surrounded by unknown cells."""
    occupancy_grid = OccupancyGrid()
    occupancy_grid.info.width = 10
    occupancy_grid.info.height = 10
    occupancy_grid.info.resolution = 0.1
    data = [-1] * 100
    for row in range(2, 8):
        for column in range(2, 8):
            data[row * 10 + column] = 0
    occupancy_grid.data = data
    return occupancy_grid


def configure_service_goal(exploration_node, occupancy_grid) -> None:
    """Make service goal generation deterministic without TF."""
    occupancy_grid.header.frame_id = 'map'
    exploration_node._wait_for_map_and_robot_position = lambda: (
        occupancy_grid,
        (0.0, 0.0),
        exploration_node._map_update_count,
    )
    exploration_node._find_nearest_candidate = lambda grid, robot: (
        0.25,
        0.35,
        8,
        0.43,
    )


def test_initial_request_returns_monotonic_goal_id(exploration_node) -> None:
    """Require the initial NO_RESULT transaction and issue goal ID one."""
    occupancy_grid = make_frontier_map()
    configure_service_goal(exploration_node, occupancy_grid)
    request = ExplorationSrv.Request()
    request.previous_result = ExplorationSrv.Request.NO_RESULT
    request.previous_goal_id = 0

    response = exploration_node._handle_get_next_task(
        request, ExplorationSrv.Response()
    )

    assert response.success
    assert not response.finished
    assert response.goal_id == 1
    assert response.task.task_type == MotionTask.NAVIGATE
    assert exploration_node._current_goal_id == 1
    assert exploration_node._next_goal_id == 2


def test_matching_success_consumes_goal_and_issues_next(
    exploration_node,
) -> None:
    """Evaluate progress only for a matching successful goal result."""
    occupancy_grid = make_frontier_map()
    configure_service_goal(exploration_node, occupancy_grid)
    exploration_node.set_parameters([
        Parameter('progress_map_updates', value=0),
    ])
    exploration_node._latest_map = occupancy_grid
    exploration_node._current_goal = (0.15, 0.15)
    exploration_node._current_goal_id = 1
    exploration_node._next_goal_id = 2
    exploration_node._known_cells_before = 30
    exploration_node._goal_start_map_update_count = 0
    request = ExplorationSrv.Request()
    request.previous_result = ExplorationSrv.Request.SUCCEEDED
    request.previous_goal_id = 1

    response = exploration_node._handle_get_next_task(
        request, ExplorationSrv.Response()
    )

    assert response.success
    assert response.goal_id == 2
    assert exploration_node._current_goal_id == 2
    assert (0.15, 0.15) not in exploration_node._ineffective_candidates


def test_rejects_stale_goal_id_without_state_change(exploration_node) -> None:
    """Reject a mismatched result without consuming the active goal."""
    exploration_node._current_goal = (0.25, 0.35)
    exploration_node._current_goal_id = 7
    exploration_node._known_cells_before = 20
    exploration_node._goal_start_map_update_count = 4
    request = ExplorationSrv.Request()
    request.previous_result = ExplorationSrv.Request.SUCCEEDED
    request.previous_goal_id = 6

    response = exploration_node._handle_get_next_task(
        request, ExplorationSrv.Response()
    )

    assert not response.success
    assert exploration_node._current_goal == (0.25, 0.35)
    assert exploration_node._current_goal_id == 7
    assert exploration_node._known_cells_before == 20
    assert exploration_node._goal_start_map_update_count == 4
    assert exploration_node._ineffective_candidates == []


def test_final_failure_is_immediate_and_marks_ineffective(
    exploration_node,
) -> None:
    """Consume FINAL_FAILURE without entering the map-update wait path."""
    occupancy_grid = make_frontier_map()
    occupancy_grid.header.frame_id = 'map'
    exploration_node._current_goal = (0.25, 0.35)
    exploration_node._current_goal_id = 3
    exploration_node._known_cells_before = 36
    exploration_node._goal_start_map_update_count = 1
    exploration_node._wait_for_map_and_robot_position = lambda: (
        occupancy_grid,
        (0.0, 0.0),
        1,
    )
    exploration_node._find_nearest_candidate = lambda grid, robot: None
    exploration_node._evaluate_previous_goal_after_map_updates = lambda: (
        pytest.fail('FINAL_FAILURE must not wait for map updates')
    )
    request = ExplorationSrv.Request()
    request.previous_result = ExplorationSrv.Request.FINAL_FAILURE
    request.previous_goal_id = 3

    start = time.monotonic()
    response = exploration_node._handle_get_next_task(
        request, ExplorationSrv.Response()
    )

    assert time.monotonic() - start < 0.1
    assert response.success
    assert response.finished
    assert exploration_node._ineffective_candidates == [(0.25, 0.35)]
    assert exploration_node._current_goal is None
    assert exploration_node._current_goal_id == 0


def test_candidates_and_ineffective_filter(exploration_node) -> None:
    """Generate multiple candidates and filter near an ineffective one."""
    occupancy_grid = make_frontier_map()
    exploration_node.set_parameters([
        Parameter('minimum_goal_distance', value=0.0),
    ])
    result = exploration_node._find_nearest_candidate(
        occupancy_grid, (0.45, 0.45)
    )
    assert result is not None
    goal_x, goal_y, cluster_size, _ = result
    assert cluster_size >= 5

    exploration_node._ineffective_candidates.append((goal_x, goal_y))
    next_result = exploration_node._find_nearest_candidate(
        occupancy_grid, (0.45, 0.45)
    )
    if next_result is not None:
        assert math.hypot(
            next_result[0] - goal_x, next_result[1] - goal_y
        ) >= 0.20


def test_candidate_stays_inside_known_free_space(exploration_node) -> None:
    """Keep a generated goal at least the configured distance from unknown."""
    occupancy_grid = make_frontier_map()
    exploration_node.set_parameters([
        Parameter('minimum_goal_distance', value=0.0),
        Parameter('unknown_clearance', value=0.20),
    ])

    result = exploration_node._find_nearest_candidate(
        occupancy_grid, (0.45, 0.45)
    )

    assert result is not None
    goal_x, goal_y = result[:2]
    unknown_centres = [
        ((index % 10 + 0.5) * 0.1, (index // 10 + 0.5) * 0.1)
        for index, value in enumerate(occupancy_grid.data)
        if value < 0
    ]
    assert min(
        math.hypot(goal_x - x, goal_y - y)
        for x, y in unknown_centres
    ) >= 0.20 - 1e-9


def test_filters_goals_inside_nav2_tolerance(exploration_node) -> None:
    """Do not return goals that Nav2 would accept without moving."""
    occupancy_grid = make_frontier_map()
    result = exploration_node._find_nearest_candidate(
        occupancy_grid, (0.45, 0.45)
    )
    assert result is None


def test_safety_margin_and_no_progress(exploration_node) -> None:
    """Enforce obstacle clearance and retain a no-progress goal."""
    free = [True] * 100
    occupied = [False] * 100
    occupied[55] = True
    free[55] = False
    safe = exploration_node._safe_cells(
        free, occupied, 10, 10, 0.1, 0.20
    )
    assert not safe[56]
    assert safe[57]

    occupancy_grid = make_frontier_map()
    exploration_node.set_parameters([
        Parameter('progress_map_updates', value=0),
    ])
    exploration_node._latest_map = occupancy_grid
    exploration_node._current_goal = (0.25, 0.25)
    exploration_node._known_cells_before = 36
    exploration_node._goal_start_map_update_count = 0
    exploration_node._evaluate_previous_goal_after_map_updates()
    assert exploration_node._ineffective_candidates == [(0.25, 0.25)]


def test_navigation_map_updates_avoid_post_arrival_wait(exploration_node) -> None:
    """Count map updates received while travelling toward the active goal."""
    occupancy_grid = make_frontier_map()
    exploration_node.set_parameters([
        Parameter('progress_map_updates', value=3),
        Parameter('progress_wait_timeout', value=1.0),
    ])
    exploration_node._latest_map = occupancy_grid
    exploration_node._map_update_count = 8
    exploration_node._current_goal = (0.25, 0.25)
    exploration_node._known_cells_before = 30
    exploration_node._goal_start_map_update_count = 5

    start = time.monotonic()
    exploration_node._evaluate_previous_goal_after_map_updates()

    assert time.monotonic() - start < 0.1
    assert exploration_node._ineffective_candidates == []
    assert exploration_node._goal_start_map_update_count is None


def test_progress_wait_has_bounded_timeout(exploration_node) -> None:
    """Do not block indefinitely when map updates stop arriving."""
    occupancy_grid = make_frontier_map()
    exploration_node.set_parameters([
        Parameter('progress_map_updates', value=3),
        Parameter('progress_wait_timeout', value=0.01),
    ])
    exploration_node._latest_map = occupancy_grid
    exploration_node._map_update_count = 4
    exploration_node._current_goal = (0.25, 0.25)
    exploration_node._known_cells_before = 36
    exploration_node._goal_start_map_update_count = 4

    start = time.monotonic()
    exploration_node._evaluate_previous_goal_after_map_updates()

    assert time.monotonic() - start < 0.2
    assert exploration_node._ineffective_candidates == [(0.25, 0.25)]


def test_waits_for_map_instead_of_returning_spin(exploration_node) -> None:
    """Wait for SLAM readiness rather than manufacture a SPIN task."""
    result = []
    exploration_node._robot_position = lambda frame_id: (0.0, 0.0)
    waiter = threading.Thread(
        target=lambda: result.append(
            exploration_node._wait_for_map_and_robot_position()
        )
    )
    waiter.start()
    time.sleep(0.1)
    assert waiter.is_alive()

    occupancy_grid = make_frontier_map()
    occupancy_grid.header.frame_id = 'map'
    exploration_node._map_callback(occupancy_grid)
    waiter.join(timeout=1.0)
    assert not waiter.is_alive()
    assert result[0] == (occupancy_grid, (0.0, 0.0), 1)
