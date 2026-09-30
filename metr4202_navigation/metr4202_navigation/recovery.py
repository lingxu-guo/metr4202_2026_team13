"""Mission-level recovery node."""

import rclpy
from metr4202_interfaces.srv import Recovery as RecoverySrv
from rclpy.node import Node


class Recovery(Node):
    """Handle failed tasks and return a recovery decision to the manager."""

    def __init__(self) -> None:
        """Create the recovery service node."""
        super().__init__('recovery')
        self._service = self.create_service(
            RecoverySrv,
            'recovery/handle_failure',
            self._handle_failure,
        )
        self.get_logger().info(
            'Recovery service ready at /recovery/handle_failure.'
        )

    def _handle_failure(self, request, response):
        """Return a safe no-op decision until map-based recovery is added."""
        response.success = True
        response.decision = RecoverySrv.Response.SKIP
        response.message = (
            'No-op recovery selected SKIP for the failed task. '
            'Map-based replacement planning is not configured yet.'
        )
        self.get_logger().warning(
            f'Received failed task from phase {request.phase}: '
            f'{request.failure_reason}. Returning SKIP.'
        )
        return response


def main(args=None) -> None:
    """Run the recovery node."""
    rclpy.init(args=args)
    node = Recovery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
