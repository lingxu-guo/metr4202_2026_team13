"""Exploration V1 frontier goal provider."""

from collections import deque
import math
import threading
import time
from typing import Iterable, Optional, Sequence, Tuple

from geometry_msgs.msg import PoseStamped
from metr4202_interfaces.msg import MotionTask
from metr4202_interfaces.srv import Exploration as ExplorationSrv
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


GridIndex = int
GridPoint = Tuple[float, float]


class Exploration(Node):
    """Detect frontiers and supply one safe navigation goal at a time."""

    def __init__(self) -> None:
        """Create the existing map, TF, and Mission Manager interfaces."""
        super().__init__('exploration')

        # Keep existing parameter names for launch/config compatibility.
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('frontier_minimum_size', 5)
        self.declare_parameter('obstacle_clearance', 0.27)
        self.declare_parameter('unknown_clearance', 0.20)
        self.declare_parameter('goal_separation', 0.20)
        self.declare_parameter('minimum_goal_distance', 0.50)
        self.declare_parameter('maximum_goal_distance', 12.0)
        self.declare_parameter('information_gain_weight', 1.0)
        self.declare_parameter('distance_weight', 0.35)
        self.declare_parameter('bootstrap_spin_yaw', math.tau)
        self.declare_parameter('candidate_search_radius', 0.5)
        self.declare_parameter('progress_map_updates', 3)
        self.declare_parameter('progress_known_cell_threshold', 5)
        self.declare_parameter('progress_wait_timeout', 2.0)

        self._map_topic = self.get_parameter('map_topic').value
        self._base_frame = self.get_parameter('base_frame').value
        self._map_condition = threading.Condition()
        self._latest_map: Optional[OccupancyGrid] = None
        self._map_update_count = 0

        self._current_goal: Optional[GridPoint] = None
        self._current_goal_id = 0
        self._next_goal_id = 1
        self._known_cells_before: Optional[int] = None
        self._goal_start_map_update_count: Optional[int] = None
        self._ineffective_candidates: list[GridPoint] = []

        map_group = MutuallyExclusiveCallbackGroup()
        service_group = MutuallyExclusiveCallbackGroup()
        self._callback_groups = (map_group, service_group)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            self._map_topic,
            self._map_callback,
            map_qos,
            callback_group=map_group,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._service = self.create_service(
            ExplorationSrv,
            'exploration/get_next_task',
            self._handle_get_next_task,
            callback_group=service_group,
        )
        self.get_logger().info(
            'Exploration V1 service ready at '
            '/exploration/get_next_task; waiting for map data.'
        )

    def _map_callback(self, occupancy_grid: OccupancyGrid) -> None:
        """Store a valid map and wake any progress-evaluation waiter."""
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        if width == 0 or height == 0 or len(occupancy_grid.data) != width * height:
            self.get_logger().warning('Ignoring malformed occupancy grid.')
            return
        with self._map_condition:
            self._latest_map = occupancy_grid
            self._map_update_count += 1
            self._map_condition.notify_all()

    def _handle_get_next_task(self, request, response):
        """Recompute frontiers from the latest map and return at most one goal."""
        if not self._process_previous_result(request, response):
            return response

        ready = self._wait_for_map_and_robot_position()
        if ready is None:
            response.success = False
            response.finished = False
            response.message = 'Exploration stopped while waiting for map and TF.'
            return response
        occupancy_grid, robot_position, map_update_count = ready

        result = self._find_nearest_candidate(occupancy_grid, robot_position)
        if result is None:
            response.success = True
            response.finished = True
            response.message = 'EXPLORATION_COMPLETE: no valid candidate remains.'
            self.get_logger().info(response.message)
            return response

        goal_x, goal_y, cluster_size, distance = result
        self._current_goal = (goal_x, goal_y)
        self._current_goal_id = self._next_goal_id
        self._next_goal_id += 1
        self._known_cells_before = self._count_known_cells(occupancy_grid)
        self._goal_start_map_update_count = map_update_count
        response.success = True
        response.finished = False
        response.goal_id = self._current_goal_id
        response.task.task_type = MotionTask.NAVIGATE
        response.task.goal_pose = self._make_goal(
            occupancy_grid.header.frame_id,
            occupancy_grid.header.stamp,
            goal_x,
            goal_y,
            math.atan2(
                goal_y - robot_position[1],
                goal_x - robot_position[0],
            ),
        )
        response.message = (
            f'Nearest safe candidate from a {cluster_size}-cell frontier cluster; '
            f'{distance:.2f} m from the robot.'
        )
        self.get_logger().info(
            f'Returning one exploration goal ({goal_x:.2f}, {goal_y:.2f}); '
            f'K_before={self._known_cells_before}. {response.message}'
        )
        return response

    def _process_previous_result(self, request, response) -> bool:
        """Validate and consume the result for the one active goal."""
        if self._current_goal is None:
            if (
                request.previous_result != ExplorationSrv.Request.NO_RESULT
                or request.previous_goal_id != 0
            ):
                response.success = False
                response.finished = False
                response.message = (
                    'No exploration goal is active; expected '
                    'NO_RESULT with goal_id 0.'
                )
                return False
            return True

        if request.previous_goal_id != self._current_goal_id:
            response.success = False
            response.finished = False
            response.message = (
                f'Exploration result goal_id {request.previous_goal_id} does '
                f'not match active goal_id {self._current_goal_id}.'
            )
            return False
        if request.previous_result == ExplorationSrv.Request.SUCCEEDED:
            self._evaluate_previous_goal_after_map_updates()
            return True
        if request.previous_result == ExplorationSrv.Request.FINAL_FAILURE:
            goal = self._current_goal
            self._ineffective_candidates.append(goal)
            self.get_logger().warning(
                f'Goal ({goal[0]:.2f}, {goal[1]:.2f}) failed permanently; '
                'retaining it without waiting for map updates.'
            )
            self._clear_active_goal()
            return True

        response.success = False
        response.finished = False
        response.message = (
            'An active exploration goal requires SUCCEEDED or FINAL_FAILURE.'
        )
        return False

    def _wait_for_map_and_robot_position(
        self,
    ) -> Optional[Tuple[OccupancyGrid, GridPoint, int]]:
        """Wait until SLAM has published a map and the robot pose is usable."""
        waiting_for_map_logged = False
        waiting_for_tf_logged = False
        while rclpy.ok():
            with self._map_condition:
                if self._latest_map is None:
                    if not waiting_for_map_logged:
                        self.get_logger().info(
                            'Waiting for the first SLAM occupancy grid; '
                            'no motion task will be returned yet.'
                        )
                        waiting_for_map_logged = True
                    self._map_condition.wait(timeout=0.2)
                    continue
                occupancy_grid = self._latest_map
                map_update_count = self._map_update_count

            robot_position = self._robot_position(
                occupancy_grid.header.frame_id
            )
            if robot_position is not None:
                return occupancy_grid, robot_position, map_update_count
            if not waiting_for_tf_logged:
                self.get_logger().info(
                    f'Waiting for TF from {occupancy_grid.header.frame_id!r} '
                    f'to {self._base_frame!r}; no motion task will be '
                    'returned yet.'
                )
                waiting_for_tf_logged = True
            with self._map_condition:
                self._map_condition.wait(timeout=0.1)
        return None

    def _evaluate_previous_goal_after_map_updates(self) -> None:
        """Wait for new maps, then permanently retain an ineffective goal."""
        update_count = max(
            0, int(self.get_parameter('progress_map_updates').value)
        )
        wait_timeout = max(
            0.0, float(self.get_parameter('progress_wait_timeout').value)
        )
        with self._map_condition:
            start_update = self._goal_start_map_update_count
            if start_update is None:
                start_update = self._map_update_count
            target_update = start_update + update_count
            deadline = time.monotonic() + wait_timeout
            while rclpy.ok() and self._map_update_count < target_update:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._map_condition.wait(timeout=min(0.5, remaining))
            occupancy_grid = self._latest_map
            received_updates = max(0, self._map_update_count - start_update)

        if received_updates < update_count:
            self.get_logger().warning(
                'Progress evaluation timed out after receiving '
                f'{received_updates}/{update_count} map updates since the goal '
                'was generated; using the latest map.'
            )

        goal = self._current_goal
        if occupancy_grid is not None and goal is not None:
            known_after = self._count_known_cells(occupancy_grid)
            known_before = self._known_cells_before or 0
            delta_known = known_after - known_before
            threshold = int(
                self.get_parameter('progress_known_cell_threshold').value
            )
            if delta_known <= threshold:
                self._ineffective_candidates.append(goal)
                self.get_logger().warning(
                    f'Goal ({goal[0]:.2f}, {goal[1]:.2f}) was ineffective: '
                    f'delta_K={delta_known}; retaining it permanently.'
                )
            else:
                self.get_logger().info(
                    f'Exploration goal made progress: delta_K={delta_known}.'
                )
        self._clear_active_goal()

    def _clear_active_goal(self) -> None:
        """Clear state only after the active goal result is consumed."""
        self._current_goal = None
        self._current_goal_id = 0
        self._known_cells_before = None
        self._goal_start_map_update_count = None

    def _robot_position(self, map_frame: str) -> Optional[GridPoint]:
        """Look up the robot position in the map frame without blocking."""
        try:
            transform = self._tf_buffer.lookup_transform(
                map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as error:
            self.get_logger().debug(f'Robot TF is not available yet: {error}')
            return None
        translation = transform.transform.translation
        return translation.x, translation.y

    def _find_nearest_candidate(
        self,
        occupancy_grid: OccupancyGrid,
        robot_position: GridPoint,
    ) -> Optional[Tuple[float, float, int, float]]:
        """Generate candidates from all frontier clusters and pick nearest."""
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        resolution = occupancy_grid.info.resolution
        if resolution <= 0.0:
            self.get_logger().warning('Ignoring occupancy grid with no resolution.')
            return None

        threshold = int(self.get_parameter('occupied_threshold').value)
        free = [0 <= value < threshold for value in occupancy_grid.data]
        unknown = [value < 0 for value in occupancy_grid.data]
        occupied = [value >= threshold for value in occupancy_grid.data]
        frontier = [
            is_free and any(
                unknown[neighbor]
                for neighbor in self._neighbors(index, width, height)
            )
            for index, is_free in enumerate(free)
        ]
        obstacle_safe = self._safe_cells(
            free,
            occupied,
            width,
            height,
            resolution,
            float(self.get_parameter('obstacle_clearance').value),
        )
        unknown_safe = self._safe_cells(
            free,
            unknown,
            width,
            height,
            resolution,
            float(self.get_parameter('unknown_clearance').value),
        )
        safe = [
            clear_of_obstacles and clear_of_unknown
            for clear_of_obstacles, clear_of_unknown in zip(
                obstacle_safe, unknown_safe
            )
        ]

        minimum_size = int(self.get_parameter('frontier_minimum_size').value)
        search_radius = float(
            self.get_parameter('candidate_search_radius').value
        )
        ineffective_radius = float(self.get_parameter('goal_separation').value)
        minimum_distance = float(
            self.get_parameter('minimum_goal_distance').value
        )
        maximum_distance = float(
            self.get_parameter('maximum_goal_distance').value
        )
        candidates: dict[GridIndex, int] = {}
        visited = [False] * len(frontier)

        for index, is_frontier in enumerate(frontier):
            if not is_frontier or visited[index]:
                continue
            component = self._component(index, frontier, visited, width, height)
            if len(component) < minimum_size:
                continue
            for frontier_index in component:
                candidate_index = self._candidate_from_frontier(
                    frontier_index,
                    free,
                    safe,
                    width,
                    height,
                    resolution,
                    search_radius,
                )
                if candidate_index is not None:
                    candidates[candidate_index] = len(component)

        best = None
        for candidate_index, cluster_size in candidates.items():
            goal_x, goal_y = self._index_to_world(occupancy_grid, candidate_index)
            if any(
                math.hypot(goal_x - old_x, goal_y - old_y)
                < ineffective_radius
                for old_x, old_y in self._ineffective_candidates
            ):
                continue
            distance = math.hypot(
                goal_x - robot_position[0], goal_y - robot_position[1]
            )
            if distance + 1e-9 < minimum_distance:
                continue
            if distance > maximum_distance:
                continue
            candidate = (distance, candidate_index, goal_x, goal_y, cluster_size)
            if best is None or candidate[:2] < best[:2]:
                best = candidate

        if best is None:
            return None
        distance, _, goal_x, goal_y, cluster_size = best
        return goal_x, goal_y, cluster_size, distance

    def _candidate_from_frontier(
        self,
        start: GridIndex,
        free: Sequence[bool],
        safe: Sequence[bool],
        width: int,
        height: int,
        resolution: float,
        maximum_radius: float,
    ) -> Optional[GridIndex]:
        """BFS from one frontier cell for a free, obstacle-clear candidate."""
        radius_cells = max(0.0, maximum_radius / resolution)
        start_row, start_column = divmod(start, width)
        queue = deque([start])
        visited = {start}
        while queue:
            index = queue.popleft()
            if safe[index]:
                return index
            for neighbor in self._neighbors(index, width, height):
                if neighbor in visited or not free[neighbor]:
                    continue
                row, column = divmod(neighbor, width)
                if math.hypot(
                    row - start_row, column - start_column
                ) > radius_cells + 1e-9:
                    continue
                visited.add(neighbor)
                queue.append(neighbor)
        return None

    @staticmethod
    def _safe_cells(
        free: Sequence[bool],
        blocked: Sequence[bool],
        width: int,
        height: int,
        resolution: float,
        safety_margin: float,
    ) -> list[bool]:
        """Return free cells at least the safety margin from blocked cells."""
        if safety_margin <= 0.0:
            return list(free)
        radius = int(math.ceil(safety_margin / resolution))
        offsets = [
            (row_delta, column_delta)
            for row_delta in range(-radius, radius + 1)
            for column_delta in range(-radius, radius + 1)
            if math.hypot(row_delta, column_delta) * resolution
            < safety_margin - 1e-9
        ]
        unsafe = [False] * len(free)
        for index, is_blocked in enumerate(blocked):
            if not is_blocked:
                continue
            row, column = divmod(index, width)
            for row_delta, column_delta in offsets:
                neighbor_row = row + row_delta
                neighbor_column = column + column_delta
                if 0 <= neighbor_row < height and 0 <= neighbor_column < width:
                    unsafe[neighbor_row * width + neighbor_column] = True
        return [
            is_free and not is_unsafe
            for is_free, is_unsafe in zip(free, unsafe)
        ]

    @staticmethod
    def _neighbors(index: GridIndex, width: int, height: int) -> Iterable[GridIndex]:
        """Yield eight-connected neighbour indices inside the grid."""
        row, column = divmod(index, width)
        for row_delta in (-1, 0, 1):
            for column_delta in (-1, 0, 1):
                if row_delta == 0 and column_delta == 0:
                    continue
                neighbor_row = row + row_delta
                neighbor_column = column + column_delta
                if 0 <= neighbor_row < height and 0 <= neighbor_column < width:
                    yield neighbor_row * width + neighbor_column

    def _component(
        self,
        start: GridIndex,
        frontier: Sequence[bool],
        visited: list[bool],
        width: int,
        height: int,
    ) -> list[GridIndex]:
        """Collect one eight-connected frontier component."""
        component = []
        queue = deque([start])
        visited[start] = True
        while queue:
            index = queue.popleft()
            component.append(index)
            for neighbor in self._neighbors(index, width, height):
                if frontier[neighbor] and not visited[neighbor]:
                    visited[neighbor] = True
                    queue.append(neighbor)
        return component

    @staticmethod
    def _count_known_cells(occupancy_grid: OccupancyGrid) -> int:
        """Count all known cells (free plus occupied) in the complete map."""
        return sum(value >= 0 for value in occupancy_grid.data)

    @staticmethod
    def _index_to_world(
        occupancy_grid: OccupancyGrid, index: GridIndex
    ) -> GridPoint:
        """Return the centre of a grid cell in the map frame."""
        row, column = divmod(index, occupancy_grid.info.width)
        origin = occupancy_grid.info.origin.position
        resolution = occupancy_grid.info.resolution
        return (
            origin.x + (column + 0.5) * resolution,
            origin.y + (row + 0.5) * resolution,
        )

    @staticmethod
    def _make_goal(
        frame_id: str,
        stamp,
        x: float,
        y: float,
        yaw: float = 0.0,
    ) -> PoseStamped:
        """Build a map-frame Nav2 goal facing its direction of travel."""
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = stamp
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal


def main(args=None) -> None:
    """Run Exploration with concurrent service and map callbacks."""
    rclpy.init(args=args)
    node = Exploration()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
