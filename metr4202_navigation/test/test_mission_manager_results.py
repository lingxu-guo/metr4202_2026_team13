"""Result-feedback tests for the Mission Manager state machine."""

from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from metr4202_interfaces.msg import MotionTask
from metr4202_interfaces.srv import Coverage, Exploration, Recovery
import pytest
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
import rclpy
from slam_toolbox.srv import Pause

from metr4202_navigation.mission_manager import MissionManager


class FinishedFuture:
    """Minimal future whose result is already available."""

    def __init__(self, result=None, error=None) -> None:
        """Store either a result or an exception."""
        self._result = result
        self._error = error

    def result(self):
        """Return the configured result or raise its exception."""
        if self._error is not None:
            raise self._error
        return self._result


class DeferredFuture:
    """Record a done callback so a test can complete it explicitly."""

    def __init__(self) -> None:
        """Initialize without a callback."""
        self.callback = None

    def add_done_callback(self, callback) -> None:
        """Save the callback registered by production code."""
        self.callback = callback


class RecordingClient:
    """Minimal client recording requests and returning deferred futures."""

    def __init__(self) -> None:
        """Initialize empty request and future collections."""
        self.requests = []
        self.futures = []

    def call_async(self, request):
        """Record request and return a controllable future."""
        future = DeferredFuture()
        self.requests.append(request)
        self.futures.append(future)
        return future


@pytest.fixture
def mission_manager():
    """Create a Mission Manager without running its timer."""
    rclpy.init()
    node = MissionManager()
    node._timer.cancel()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def navigation_task() -> MotionTask:
    """Return a simple navigation task for state-machine tests."""
    task = MotionTask()
    task.task_type = MotionTask.NAVIGATE
    task.goal_pose.header.frame_id = 'map'
    return task


def recovery_response(decision: int) -> Recovery.Response:
    """Return a successful Recovery decision."""
    response = Recovery.Response()
    response.success = True
    response.decision = decision
    response.message = 'test decision'
    return response


def slam_parameter_response(paused: bool) -> GetParameters.Response:
    """Return a valid paused_new_measurements parameter response."""
    response = GetParameters.Response()
    response.values = [ParameterValue(
        type=ParameterType.PARAMETER_BOOL,
        bool_value=paused,
    )]
    return response


def test_exploration_completion_enters_slam_freeze(mission_manager) -> None:
    """Freeze SLAM before ArUco finalization and Coverage initialization."""
    mission_manager._state = 'EXPLORATION'
    response = Exploration.Response()
    response.success = True
    response.finished = True

    mission_manager._handle_exploration_response(FinishedFuture(response))

    assert mission_manager._state == 'SLAM_FREEZE'


def test_already_frozen_slam_does_not_toggle_pause(mission_manager) -> None:
    """Avoid resuming SLAM when the toggle service is already paused."""
    pause_client = RecordingClient()
    mission_manager._slam_pause_client = pause_client
    mission_manager._state = 'SLAM_FREEZE'
    mission_manager._request_in_flight = True

    mission_manager._handle_slam_pause_state(FinishedFuture(
        slam_parameter_response(True)
    ))

    assert pause_client.requests == []
    assert mission_manager._request_in_flight is False
    assert mission_manager._state == 'ARUCO_FINALIZE'


def test_slam_pause_is_verified_before_aruco_finalize(
    mission_manager,
) -> None:
    """Toggle an active SLAM node and verify it reports being frozen."""
    pause_client = RecordingClient()
    parameter_client = RecordingClient()
    mission_manager._slam_pause_client = pause_client
    mission_manager._slam_parameters_client = parameter_client
    mission_manager._state = 'SLAM_FREEZE'
    mission_manager._request_in_flight = True

    mission_manager._handle_slam_pause_state(FinishedFuture(
        slam_parameter_response(False)
    ))

    assert len(pause_client.requests) == 1
    assert isinstance(pause_client.requests[0], Pause.Request)
    assert mission_manager._state == 'SLAM_FREEZE'

    pause_response = Pause.Response()
    pause_response.status = True
    pause_client.futures[0].callback(FinishedFuture(pause_response))

    assert len(parameter_client.requests) == 1
    assert parameter_client.requests[0].names == [
        'paused_new_measurements'
    ]
    parameter_client.futures[0].callback(FinishedFuture(
        slam_parameter_response(True)
    ))

    assert mission_manager._request_in_flight is False
    assert mission_manager._state == 'ARUCO_FINALIZE'


def test_exploration_response_clears_delivered_result_and_saves_goal_id(
    mission_manager,
) -> None:
    """A successful service response acknowledges the pending result."""
    mission_manager._state = 'EXPLORATION'
    mission_manager._pending_exploration_result = (
        Exploration.Request.SUCCEEDED
    )
    mission_manager._pending_exploration_goal_id = 4
    started = []
    mission_manager._start_action_for_task = (
        lambda task, message: started.append((task, message))
    )
    response = Exploration.Response()
    response.success = True
    response.goal_id = 5
    response.task = navigation_task()
    response.message = 'next goal'

    mission_manager._handle_exploration_response(FinishedFuture(response))

    assert mission_manager._pending_exploration_result == (
        Exploration.Request.NO_RESULT
    )
    assert mission_manager._pending_exploration_goal_id == 0
    assert mission_manager._active_exploration_goal_id == 5
    assert started == [(response.task, 'next goal')]


def test_exploration_service_error_preserves_pending_result(
    mission_manager,
) -> None:
    """Do not lose a Nav2 result when the service call raises."""
    mission_manager._pending_exploration_result = (
        Exploration.Request.SUCCEEDED
    )
    mission_manager._pending_exploration_goal_id = 9

    mission_manager._handle_exploration_response(
        FinishedFuture(error=RuntimeError('service unavailable'))
    )

    assert mission_manager._pending_exploration_result == (
        Exploration.Request.SUCCEEDED
    )
    assert mission_manager._pending_exploration_goal_id == 9


def test_exploration_nav2_success_queues_matching_goal(mission_manager) -> None:
    """Report a successful original Exploration goal exactly once."""
    mission_manager._state = 'EXPLORATION'
    mission_manager._active_task = navigation_task()
    mission_manager._active_task_description = 'test navigation'
    mission_manager._active_exploration_goal_id = 11
    result = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)

    mission_manager._handle_action_result(FinishedFuture(result))

    assert mission_manager._pending_exploration_result == (
        Exploration.Request.SUCCEEDED
    )
    assert mission_manager._pending_exploration_goal_id == 11
    assert mission_manager._active_exploration_goal_id == 0
    assert mission_manager._active_task is None


def test_exploration_recovery_skip_queues_final_failure(
    mission_manager,
) -> None:
    """SKIP permanently fails the original Exploration goal."""
    mission_manager._state = 'EXPLORATION'
    mission_manager._active_task = navigation_task()
    mission_manager._active_task_description = 'test navigation'
    mission_manager._active_exploration_goal_id = 12

    mission_manager._handle_recovery_response(FinishedFuture(
        recovery_response(Recovery.Response.SKIP)
    ))

    assert mission_manager._pending_exploration_result == (
        Exploration.Request.FINAL_FAILURE
    )
    assert mission_manager._pending_exploration_goal_id == 12
    assert mission_manager._active_exploration_goal_id == 0
    assert mission_manager._active_task is None


def test_exploration_recovery_retry_does_not_report_early(
    mission_manager,
) -> None:
    """RETRY_ORIGINAL keeps the goal active until its final Nav2 result."""
    original = navigation_task()
    mission_manager._state = 'EXPLORATION'
    mission_manager._active_task = original
    mission_manager._active_task_description = 'test navigation'
    mission_manager._active_exploration_goal_id = 13
    retries = []
    mission_manager._start_action_for_task = (
        lambda task, message, is_retry=False: retries.append(
            (task, message, is_retry)
        )
    )

    mission_manager._handle_recovery_response(FinishedFuture(
        recovery_response(Recovery.Response.RETRY_ORIGINAL)
    ))

    assert mission_manager._pending_exploration_result == (
        Exploration.Request.NO_RESULT
    )
    assert mission_manager._pending_exploration_goal_id == 0
    assert mission_manager._active_exploration_goal_id == 13
    assert retries == [(original, 'test decision', True)]


def test_exploration_replacement_cannot_report_original_success(
    mission_manager,
) -> None:
    """A replacement task cannot masquerade as the original goal."""
    original = navigation_task()
    replacement = navigation_task()
    replacement.goal_pose.pose.position.x = 2.0
    mission_manager._state = 'EXPLORATION'
    mission_manager._active_task = original
    mission_manager._active_task_description = 'original navigation'
    mission_manager._active_exploration_goal_id = 14
    started = []
    mission_manager._start_action_for_task = (
        lambda task, message: started.append((task, message))
    )
    response = recovery_response(Recovery.Response.REPLACE_WITH_TASK)
    response.replacement_task = replacement

    mission_manager._handle_recovery_response(FinishedFuture(response))

    assert mission_manager._pending_exploration_result == (
        Exploration.Request.FINAL_FAILURE
    )
    assert mission_manager._pending_exploration_goal_id == 14
    assert mission_manager._active_exploration_goal_id == 0
    assert started == [(replacement, 'test decision')]

    mission_manager._active_task = replacement
    mission_manager._active_task_description = 'replacement navigation'
    result = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)
    mission_manager._handle_action_result(FinishedFuture(result))
    assert mission_manager._pending_exploration_result == (
        Exploration.Request.FINAL_FAILURE
    )
    assert mission_manager._pending_exploration_goal_id == 14


def test_coverage_result_feedback_still_uses_candidate_and_task(
    mission_manager,
) -> None:
    """Keep the existing Coverage result transaction unchanged."""
    task = MotionTask()
    task.task_type = MotionTask.SPIN
    mission_manager._state = 'COVERAGE'
    mission_manager._active_task = task
    mission_manager._active_task_description = 'coverage spin'
    mission_manager._active_candidate_id = 21
    result = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)

    mission_manager._handle_action_result(FinishedFuture(result))

    assert mission_manager._pending_coverage_result == (
        Coverage.Request.SUCCEEDED
    )
    assert mission_manager._pending_coverage_candidate_id == 21
    assert mission_manager._pending_coverage_task_type == MotionTask.SPIN
