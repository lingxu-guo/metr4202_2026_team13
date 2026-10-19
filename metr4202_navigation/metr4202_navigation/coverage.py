"""Coverage V1 ROS node built on the existing task-provider architecture."""

import math
import threading
from typing import Optional

from geometry_msgs.msg import PoseStamped
from metr4202_interfaces.msg import MotionTask
from metr4202_interfaces.srv import Coverage as CoverageSrv
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from metr4202_navigation.coverage_geometry import (
    CoverageModel,
    CoverageParameters,
    TaskDecision,
)
from metr4202_navigation.grid_utils import GridMap, GridPoint


class Coverage(Node):
    """Provide map-derived NAVIGATE/SPIN tasks one at a time."""

    def __init__(self) -> None:
        """Create map, TF, and service interfaces."""
        super().__init__('coverage')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('coverage_map_topic', '/coverage/status_map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('robot_footprint_radius', 0.22)
        self.declare_parameter('obstacle_clearance', 0.08)
        self.declare_parameter('camera_radial_offset', 0.076)
        self.declare_parameter('camera_horizontal_fov_degrees', 62.2)
        self.declare_parameter('target_spacing', 0.0)
        self.declare_parameter('candidate_spacing', 0.20)
        self.declare_parameter('minimum_candidate_spacing', 0.0)
        self.declare_parameter('visual_distance_min', 0.20)
        self.declare_parameter('visual_distance_max', 2.00)
        self.declare_parameter('viewing_angle_max_degrees', 60.0)
        self.declare_parameter('contour_simplification', 0.0)
        self.declare_parameter('fixed_visit_cost', 0.50)

        self._map_topic = self.get_parameter('map_topic').value
        self._coverage_map_topic = self.get_parameter(
            'coverage_map_topic'
        ).value
        self._base_frame = self.get_parameter('base_frame').value
        self._map_condition = threading.Condition()
        self._latest_map: Optional[OccupancyGrid] = None
        self._model: Optional[CoverageModel] = None

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
        self._status_publisher = self.create_publisher(
            OccupancyGrid,
            self._coverage_map_topic,
            map_qos,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._service = self.create_service(
            CoverageSrv,
            'coverage/get_next_task',
            self._handle_request,
            callback_group=service_group,
        )
        self.get_logger().info(
            'Coverage V1 service ready at /coverage/get_next_task; '
            'waiting for INIT.'
        )

    def _map_callback(self, occupancy_grid: OccupancyGrid) -> None:
        """Store the latest valid map without changing an active snapshot."""
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        if (
            width == 0
            or height == 0
            or len(occupancy_grid.data) != width * height
        ):
            self.get_logger().warning('Ignoring malformed occupancy grid.')
            return
        with self._map_condition:
            self._latest_map = occupancy_grid
            self._map_condition.notify_all()

    def _handle_request(self, request, response):
        """Initialize coverage or apply a result and return one task."""
        if request.command == CoverageSrv.Request.INIT:
            return self._initialize(response)
        if request.command != CoverageSrv.Request.NEXT:
            response.success = False
            response.message = 'Unknown coverage command.'
            return response
        if self._model is None:
            response.success = False
            response.message = 'Coverage must be initialized before NEXT.'
            return response

        try:
            self._apply_previous_result(request)
            robot_position = self._robot_position(self._model.grid.frame_id)
            if robot_position is None:
                self._publish_coverage_map()
                response.success = False
                response.message = 'Robot TF is unavailable for selection.'
                return self._fill_summary(response)
            decision = self._model.next_task(robot_position)
            self._publish_coverage_map()
        except ValueError as error:
            response.success = False
            response.message = f'Coverage state error: {error}'
            return self._fill_summary(response)

        if decision is None:
            if self._model.unfinished():
                response.success = False
                response.finished = False
                response.message = 'COVERAGE_STALLED'
            else:
                response.success = True
                response.finished = True
                response.message = 'COVERAGE_COMPLETE'
            return self._fill_summary(response)

        response.success = True
        response.finished = False
        response.candidate_id = decision.candidate_id
        self._write_task(response.task, decision, robot_position)
        response.message = (
            f'Candidate {decision.candidate_id}: '
            f'{"NAVIGATE" if decision.task_type == MotionTask.NAVIGATE else "SPIN"}.'
        )
        return self._fill_summary(response)

    def _initialize(self, response):
        """Freeze the current map and construct the complete coverage model."""
        ready = self._wait_for_map_and_robot_position()
        if ready is None:
            response.success = False
            response.message = 'Coverage stopped while waiting for map and TF.'
            return response
        occupancy_grid, robot_position = ready
        try:
            snapshot = GridMap.from_message(occupancy_grid)
            self._model = CoverageModel(
                snapshot,
                robot_position,
                self._coverage_parameters(),
            )
        except ValueError as error:
            response.success = False
            response.message = f'Coverage initialization failed: {error}'
            return response

        self._publish_coverage_map()
        summary = self._model.summary()
        response.success = True
        response.finished = not self._model.unfinished()
        response.message = (
            f'Coverage initialized with {summary.total} targets and '
            f'{len(self._model.candidates)} candidates.'
        )
        self.get_logger().info(response.message)
        self.get_logger().info(
            f'Coverage status map published on {self._coverage_map_topic}; '
            'values: covered=20, uncovered=50, '
            'initial-uncoverable=70, runtime-uncoverable=85, '
            'occupied=100.'
        )
        return self._fill_summary(response)

    def _apply_previous_result(self, request) -> None:
        if request.previous_result == CoverageSrv.Request.NO_RESULT:
            return
        if request.previous_result not in (
            CoverageSrv.Request.SUCCEEDED,
            CoverageSrv.Request.FINAL_FAILURE,
        ):
            raise ValueError(
                f'Unknown previous result: {request.previous_result}.'
            )
        self._model.submit_result(
            int(request.previous_candidate_id),
            int(request.previous_task_type),
            request.previous_result == CoverageSrv.Request.SUCCEEDED,
        )

    def _write_task(
        self,
        task: MotionTask,
        decision: TaskDecision,
        robot_position: GridPoint,
    ) -> None:
        task.task_type = decision.task_type
        if decision.task_type == MotionTask.SPIN:
            task.spin_yaw = decision.spin_yaw
            return

        goal_x, goal_y = decision.position
        yaw = math.atan2(
            goal_y - robot_position[1],
            goal_x - robot_position[0],
        )
        task.goal_pose = self._make_goal(
            self._model.grid.frame_id,
            goal_x,
            goal_y,
            yaw,
        )

    def _coverage_parameters(self) -> CoverageParameters:
        footprint = float(
            self.get_parameter('robot_footprint_radius').value
        )
        clearance = float(self.get_parameter('obstacle_clearance').value)
        camera_offset = float(
            self.get_parameter('camera_radial_offset').value
        )
        return CoverageParameters(
            occupied_threshold=int(
                self.get_parameter('occupied_threshold').value
            ),
            safety_radius=max(footprint, camera_offset) + clearance,
            target_spacing=float(
                self.get_parameter('target_spacing').value
            ),
            candidate_spacing=float(
                self.get_parameter('candidate_spacing').value
            ),
            minimum_candidate_spacing=float(
                self.get_parameter('minimum_candidate_spacing').value
            ),
            visual_distance_min=float(
                self.get_parameter('visual_distance_min').value
            ),
            visual_distance_max=float(
                self.get_parameter('visual_distance_max').value
            ),
            viewing_angle_max=math.radians(float(
                self.get_parameter('viewing_angle_max_degrees').value
            )),
            camera_radial_offset=camera_offset,
            camera_horizontal_fov=math.radians(float(
                self.get_parameter('camera_horizontal_fov_degrees').value
            )),
            contour_simplification=float(
                self.get_parameter('contour_simplification').value
            ),
            fixed_visit_cost=float(
                self.get_parameter('fixed_visit_cost').value
            ),
        )

    def _wait_for_map_and_robot_position(
        self,
    ) -> Optional[tuple[OccupancyGrid, GridPoint]]:
        waiting_logged = False
        while rclpy.ok():
            with self._map_condition:
                occupancy_grid = self._latest_map
                if occupancy_grid is None:
                    if not waiting_logged:
                        self.get_logger().info(
                            'Waiting for the final Exploration map.'
                        )
                        waiting_logged = True
                    self._map_condition.wait(timeout=0.2)
                    continue
            frame_id = occupancy_grid.header.frame_id or 'map'
            robot_position = self._robot_position(frame_id)
            if robot_position is not None:
                return occupancy_grid, robot_position
            with self._map_condition:
                self._map_condition.wait(timeout=0.1)
        return None

    def _robot_position(self, map_frame: str) -> Optional[GridPoint]:
        try:
            transform = self._tf_buffer.lookup_transform(
                map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as error:
            self.get_logger().debug(f'Robot TF is unavailable: {error}')
            return None
        translation = transform.transform.translation
        return translation.x, translation.y

    def _publish_coverage_map(self) -> None:
        """Publish the frozen map with target lifecycle states overlaid."""
        if self._model is None:
            return
        grid = self._model.grid
        stamp = self.get_clock().now().to_msg()
        message = OccupancyGrid()
        message.header.frame_id = grid.frame_id
        message.header.stamp = stamp
        message.info.map_load_time = stamp
        message.info.width = grid.width
        message.info.height = grid.height
        message.info.resolution = grid.resolution
        message.info.origin.position.x = grid.origin_x
        message.info.origin.position.y = grid.origin_y
        message.info.origin.orientation.z = math.sin(
            grid.origin_yaw / 2.0
        )
        message.info.origin.orientation.w = math.cos(grid.origin_yaw / 2.0)
        message.data = list(self._model.coverage_grid_data())
        self._status_publisher.publish(message)

    def _fill_summary(self, response):
        if self._model is None:
            return response
        summary = self._model.summary()
        response.covered_count = summary.covered
        response.initial_uncoverable_count = summary.initial_uncoverable
        response.runtime_uncoverable_count = summary.runtime_uncoverable
        response.total_target_count = summary.total
        return response

    def _make_goal(
        self,
        frame_id: str,
        x: float,
        y: float,
        yaw: float,
    ) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal


def main(args=None) -> None:
    """Run Coverage with concurrent service and map callbacks."""
    rclpy.init(args=args)
    node = Coverage()
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
