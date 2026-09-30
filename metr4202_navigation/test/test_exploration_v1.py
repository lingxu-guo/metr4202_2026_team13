"""Unit tests for the Exploration V1 decision algorithm."""

import math
import threading
import time

from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.parameter import Parameter

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
    exploration_node._evaluate_previous_goal_after_map_updates()
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
    assert result[0] == (occupancy_grid, (0.0, 0.0))
