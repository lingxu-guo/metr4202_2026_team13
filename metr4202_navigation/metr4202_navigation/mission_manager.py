"""Mission-level state-machine node."""

from action_msgs.msg import GoalStatus
import rclpy
from metr4202_interfaces.msg import MotionTask
from metr4202_interfaces.srv import Coverage, Exploration, Recovery
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger


class MissionManager(Node):
    """Coordinate task providers, Nav2, and recovery decisions."""

    def __init__(self) -> None:
        """Create clients and start the mission state machine."""
        super().__init__('mission_manager')
        self._state = 'WAITING_FOR_SERVICES'
        self._request_in_flight = False
        self._action_in_flight = False
        self._completed_task_count = 0
        self._waiting_message_logged = False
        self._active_task = None
        self._active_task_description = ''
        self._active_exploration_goal_id = 0
        self._pending_exploration_result = Exploration.Request.NO_RESULT
        self._pending_exploration_goal_id = 0
        self._active_candidate_id = 0
        self._pending_coverage_result = Coverage.Request.NO_RESULT
        self._pending_coverage_candidate_id = 0
        self._pending_coverage_task_type = MotionTask.NAVIGATE
        self._nav2_active = False
        self._nav2_active_request_in_flight = False
        self._aruco_waiting_logged = False

        self._exploration_client = self.create_client(
            Exploration,
            'exploration/get_next_task',
        )
        self._coverage_client = self.create_client(
            Coverage,
            'coverage/get_next_task',
        )
        self._recovery_client = self.create_client(
            Recovery,
            'recovery/handle_failure',
        )
        self._aruco_finalize_client = self.create_client(
            Trigger,
            'aruco/finalize_exploration',
        )
        self._nav2_active_client = self.create_client(
            Trigger,
            'lifecycle_manager_navigation/is_active',
        )
        self._navigate_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
        )
        self._spin_client = ActionClient(self, Spin, 'spin')

        self._timer = self.create_timer(0.5, self._advance_state_machine)
        self.get_logger().info('Mission manager started.')

    def _advance_state_machine(self) -> None:
        """Request the next task only when all required servers are ready."""
        if (
            self._request_in_flight
            or self._action_in_flight
            or self._state == 'DONE'
        ):
            return

        if self._state == 'WAITING_FOR_SERVICES':
            self._poll_nav2_active()
            if not self._all_servers_ready():
                if not self._waiting_message_logged:
                    self.get_logger().info(
                        'Waiting for task providers, Recovery, and Nav2 servers.'
                    )
                    self._waiting_message_logged = True
                return

            self.get_logger().info('Task providers, Recovery, and Nav2 are ready.')
            self._state = 'EXPLORATION'

        if self._state == 'EXPLORATION':
            self._request_exploration_task()
        elif self._state == 'ARUCO_FINALIZE':
            self._finalize_aruco_for_coverage()
        elif self._state == 'COVERAGE_INIT':
            self._initialize_coverage()
        elif self._state == 'COVERAGE':
            self._request_coverage_task()

    def _all_servers_ready(self) -> bool:
        """Return whether the service and action servers are available."""
        return (
            self._nav2_active
            and self._exploration_client.service_is_ready()
            and self._coverage_client.service_is_ready()
            and self._recovery_client.service_is_ready()
            and self._navigate_client.server_is_ready()
            and self._spin_client.server_is_ready()
        )

    def _poll_nav2_active(self) -> None:
        """Asynchronously wait for every Nav2 managed node to be active."""
        if (
            self._nav2_active
            or self._nav2_active_request_in_flight
            or not self._nav2_active_client.service_is_ready()
        ):
            return
        self._nav2_active_request_in_flight = True
        future = self._nav2_active_client.call_async(Trigger.Request())
        future.add_done_callback(self._handle_nav2_active_response)

    def _handle_nav2_active_response(self, future) -> None:
        """Record lifecycle-manager readiness without blocking the timer."""
        self._nav2_active_request_in_flight = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().debug(
                f'Nav2 lifecycle readiness check failed: {error}'
            )
            return
        if response.success:
            self._nav2_active = True
            self.get_logger().info('Nav2 managed nodes are active.')

    def _request_exploration_task(self) -> None:
        """Ask Exploration for its next task."""
        request = Exploration.Request()
        request.previous_result = self._pending_exploration_result
        request.previous_goal_id = self._pending_exploration_goal_id
        self._request_in_flight = True
        future = self._exploration_client.call_async(request)
        future.add_done_callback(self._handle_exploration_response)

    def _handle_exploration_response(self, future) -> None:
        """Process the response returned by the Exploration service."""
        self._request_in_flight = False
        try:
            response = future.result()
        except Exception as error:
            self._fail(f'Exploration service call failed: {error}')
            return

        if not response.success:
            self._fail(f'Exploration error: {response.message}')
            return

        self._pending_exploration_result = Exploration.Request.NO_RESULT
        self._pending_exploration_goal_id = 0
        if response.finished:
            self._active_exploration_goal_id = 0
            self.get_logger().info(
                'Exploration complete; finalizing saved ArUco observations.'
            )
            self._state = 'ARUCO_FINALIZE'
        else:
            self._active_exploration_goal_id = response.goal_id
            self._start_action_for_task(response.task, response.message)

    def _finalize_aruco_for_coverage(self) -> None:
        """Reproject exploration observations before Coverage."""
        if not self._aruco_finalize_client.service_is_ready():
            if not self._aruco_waiting_logged:
                self.get_logger().info(
                    'Waiting for ArUco finalization before Coverage.'
                )
                self._aruco_waiting_logged = True
            return

        self._request_in_flight = True
        future = self._aruco_finalize_client.call_async(Trigger.Request())
        future.add_done_callback(self._handle_aruco_finalize_response)

    def _handle_aruco_finalize_response(self, future) -> None:
        """Continue only after historical observations are reprojected."""
        self._request_in_flight = False
        try:
            response = future.result()
        except Exception as error:
            self._fail(f'ArUco finalization failed: {error}')
            return

        if not response.success:
            self._fail(f'ArUco finalization error: {response.message}')
            return

        self.get_logger().info(
            f'ArUco exploration observations finalized: {response.message}'
        )
        self._state = 'COVERAGE_INIT'

    def _initialize_coverage(self) -> None:
        """Ask Coverage to prepare its task sequence."""
        request = Coverage.Request()
        request.command = Coverage.Request.INIT
        self._request_in_flight = True
        future = self._coverage_client.call_async(request)
        future.add_done_callback(self._handle_coverage_init_response)

    def _handle_coverage_init_response(self, future) -> None:
        """Move to coverage execution after successful initialization."""
        self._request_in_flight = False
        try:
            response = future.result()
        except Exception as error:
            self._fail(f'Coverage INIT call failed: {error}')
            return

        if not response.success:
            self._fail(f'Coverage INIT error: {response.message}')
            return

        self.get_logger().info('Coverage initialized; requesting coverage tasks.')
        self._state = 'COVERAGE'

    def _request_coverage_task(self) -> None:
        """Ask Coverage for its next task."""
        request = Coverage.Request()
        request.command = Coverage.Request.NEXT
        request.previous_result = self._pending_coverage_result
        request.previous_candidate_id = self._pending_coverage_candidate_id
        request.previous_task_type = self._pending_coverage_task_type
        self._request_in_flight = True
        future = self._coverage_client.call_async(request)
        future.add_done_callback(self._handle_coverage_response)

    def _handle_coverage_response(self, future) -> None:
        """Process the response returned by the Coverage service."""
        self._request_in_flight = False
        try:
            response = future.result()
        except Exception as error:
            self._fail(f'Coverage NEXT call failed: {error}')
            return
        self._pending_coverage_result = Coverage.Request.NO_RESULT
        self._pending_coverage_candidate_id = 0
        self._pending_coverage_task_type = MotionTask.NAVIGATE

        if not response.success:
            self._fail(f'Coverage NEXT error: {response.message}')
        elif response.finished:
            self._state = 'DONE'
            self.get_logger().info('Coverage complete; mission finished.')
        else:
            self._active_candidate_id = response.candidate_id
            self._start_action_for_task(response.task, response.message)

    def _start_action_for_task(
        self,
        task: MotionTask,
        message: str,
        is_retry: bool = False,
    ) -> None:
        """Send a MotionTask to the Nav2 Action server that implements it."""
        if not is_retry:
            self._completed_task_count += 1
        self._active_task = task

        if task.task_type == MotionTask.NAVIGATE:
            position = task.goal_pose.pose.position
            self._active_task_description = (
                f'NAVIGATE to ({position.x:.2f}, {position.y:.2f}) '
                f'in {task.goal_pose.header.frame_id!r}'
            )
            goal = NavigateToPose.Goal()
            goal.pose = task.goal_pose
            future = self._navigate_client.send_goal_async(goal)
        elif task.task_type == MotionTask.SPIN:
            self._active_task_description = f'SPIN by {task.spin_yaw:.2f} rad'
            goal = Spin.Goal()
            goal.target_yaw = task.spin_yaw
            future = self._spin_client.send_goal_async(goal)
        else:
            self._fail(f'Unknown MotionTask type: {task.task_type}')
            return

        self._action_in_flight = True
        retry_prefix = 'Retrying' if is_retry else f'Task {self._completed_task_count}'
        self.get_logger().info(
            f'{retry_prefix}: {self._active_task_description}. {message}'
        )
        future.add_done_callback(self._handle_goal_response)

    def _handle_goal_response(self, future) -> None:
        """Wait for the accepted Nav2 goal to produce its final result."""
        try:
            goal_handle = future.result()
        except Exception as error:
            self._action_in_flight = False
            self._fail(f'Could not send Nav2 goal: {error}')
            return

        if not goal_handle.accepted:
            self._action_in_flight = False
            self._request_recovery('Nav2 rejected the goal.')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_action_result)

    def _handle_action_result(self, future) -> None:
        """Advance on success, or ask Recovery to handle a failed task."""
        self._action_in_flight = False
        try:
            result = future.result()
        except Exception as error:
            self._fail(f'Nav2 result failed: {error}')
            return

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self._request_recovery(f'Nav2 Action status {result.status}.')
            return

        if self._state == 'EXPLORATION' and self._active_exploration_goal_id:
            self._record_exploration_result(Exploration.Request.SUCCEEDED)
        elif self._state == 'COVERAGE':
            self._record_coverage_result(Coverage.Request.SUCCEEDED)
        self.get_logger().info(
            f'Nav2 completed {self._active_task_description}; requesting next task.'
        )
        self._active_task = None

    def _record_exploration_result(self, result: int) -> None:
        """Queue one final Exploration result for the next service request."""
        if self._active_task is None or not self._active_exploration_goal_id:
            self._fail('Exploration result recorded without an active goal.')
            return
        self._pending_exploration_result = result
        self._pending_exploration_goal_id = self._active_exploration_goal_id
        self._active_exploration_goal_id = 0

    def _record_coverage_result(self, result: int) -> None:
        """Queue one final Coverage result for the next service request."""
        if self._active_task is None:
            self._fail('Coverage result recorded without an active task.')
            return
        self._pending_coverage_result = result
        self._pending_coverage_candidate_id = self._active_candidate_id
        self._pending_coverage_task_type = self._active_task.task_type

    def _request_recovery(self, failure_reason: str) -> None:
        """Ask Recovery how to proceed after the current Nav2 task failed."""
        if self._active_task is None:
            self._fail('Recovery requested without an active MotionTask.')
            return

        request = Recovery.Request()
        request.phase = (
            Recovery.Request.EXPLORATION
            if self._state == 'EXPLORATION'
            else Recovery.Request.COVERAGE
        )
        request.failed_task = self._active_task
        request.failure_reason = failure_reason
        self._request_in_flight = True
        future = self._recovery_client.call_async(request)
        future.add_done_callback(self._handle_recovery_response)

    def _handle_recovery_response(self, future) -> None:
        """Apply Recovery's skip, retry, or replacement-task decision."""
        self._request_in_flight = False
        try:
            response = future.result()
        except Exception as error:
            self._fail(f'Recovery service call failed: {error}')
            return

        if not response.success:
            self._fail(f'Recovery error: {response.message}')
            return

        if response.decision == Recovery.Response.SKIP:
            if self._state == 'EXPLORATION' and self._active_exploration_goal_id:
                self._record_exploration_result(
                    Exploration.Request.FINAL_FAILURE
                )
            elif self._state == 'COVERAGE':
                self._record_coverage_result(Coverage.Request.FINAL_FAILURE)
            self.get_logger().warning(
                f'Recovery skipped {self._active_task_description}: '
                f'{response.message}'
            )
            self._active_task = None
        elif response.decision == Recovery.Response.RETRY_ORIGINAL:
            self.get_logger().info(
                f'Recovery retrying {self._active_task_description}: '
                f'{response.message}'
            )
            self._start_action_for_task(
                self._active_task,
                response.message,
                is_retry=True,
            )
        elif response.decision == Recovery.Response.REPLACE_WITH_TASK:
            if self._state == 'COVERAGE':
                self._record_coverage_result(Coverage.Request.FINAL_FAILURE)
                self.get_logger().warning(
                    'Recovery replacement cannot prove the original Coverage '
                    'task succeeded; reporting FINAL_FAILURE.'
                )
                self._active_task = None
                return

            if self._active_exploration_goal_id:
                self._record_exploration_result(
                    Exploration.Request.FINAL_FAILURE
                )

            self.get_logger().info(
                f'Recovery supplied a replacement task: {response.message}'
            )
            self._start_action_for_task(
                response.replacement_task,
                response.message,
            )
        else:
            self._fail(f'Unknown Recovery decision: {response.decision}')

    def _fail(self, message: str) -> None:
        """Stop state-machine progress after an unrecoverable error."""
        self._state = 'DONE'
        self.get_logger().error(message)


def main(args=None) -> None:
    """Run the mission manager node."""
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
