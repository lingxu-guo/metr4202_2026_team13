"""Timestamp-correct ArUco localization and map-fusion node."""

import math

from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import PointStamped
from metr4202_aruco.localization_core import (
    MarkerDatabase,
    TrackingParameters,
    estimate_square_pose,
    observation_variance,
    quaternion_yaw,
)
from metr4202_interfaces.msg import ArucoMarker, ArucoMarkerArray
from metr4202_interfaces.msg import ArucoDetectionArray
from metr4202_interfaces.srv import GetArucoMap
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


class ArucoLocalization(Node):
    """Estimate, gate, and fuse static marker positions in the map frame."""

    def __init__(self) -> None:
        """Create subscriptions, TF support, services, and marker storage."""
        super().__init__('aruco_localization')
        self._declare_parameters()
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._marker_length = float(
            self.get_parameter('marker_length_m').value
        )
        self._minimum_depth = float(self.get_parameter('min_depth_m').value)
        self._maximum_depth = float(self.get_parameter('max_depth_m').value)
        self._maximum_reprojection_error = float(
            self.get_parameter('max_reprojection_error_px').value
        )
        self._measurement_stddev = float(
            self.get_parameter('measurement_stddev_m').value
        )
        self._tf_timeout = Duration(
            seconds=float(self.get_parameter('tf_timeout_s').value)
        )

        parameters = TrackingParameters(
            confirmation_count=int(
                self.get_parameter('confirmation_count').value
            ),
            process_noise_m2_per_s=float(
                self.get_parameter('process_noise_m2_per_s').value
            ),
            innovation_gate_squared=float(
                self.get_parameter('innovation_gate_squared').value
            ),
            min_fusion_interval_s=float(
                self.get_parameter('min_fusion_interval_s').value
            ),
            min_observer_translation_m=float(
                self.get_parameter('min_observer_translation_m').value
            ),
            min_observer_yaw_rad=math.radians(float(
                self.get_parameter('min_observer_yaw_degrees').value
            )),
        )
        self._database = MarkerDatabase(parameters)
        self._camera_info = None
        self._camera_warning_emitted = False

        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._marker_publisher = self.create_publisher(
            ArucoMarkerArray,
            str(self.get_parameter('marker_map_topic').value),
            marker_qos,
        )
        self._camera_subscription = self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self._detection_subscription = self.create_subscription(
            ArucoDetectionArray,
            str(self.get_parameter('detections_topic').value),
            self._detection_callback,
            qos_profile_sensor_data,
        )
        self._get_service = self.create_service(
            GetArucoMap,
            'aruco/get_marker_map',
            self._get_marker_map,
        )
        self._reset_service = self.create_service(
            Trigger,
            'aruco/reset_map',
            self._reset_marker_map,
        )
        self.get_logger().info(
            f'ArUco localization started; output frame is '
            f'{self._map_frame!r}.'
        )

    def _declare_parameters(self) -> None:
        """Declare all externally configurable localization parameters."""
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('detections_topic', '/aruco/detections')
        self.declare_parameter('marker_map_topic', '/aruco/marker_map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('marker_length_m', 0.10)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 4.0)
        self.declare_parameter('max_reprojection_error_px', 3.0)
        self.declare_parameter('measurement_stddev_m', 0.08)
        self.declare_parameter('confirmation_count', 3)
        self.declare_parameter('process_noise_m2_per_s', 1.0e-4)
        self.declare_parameter('innovation_gate_squared', 9.21)
        self.declare_parameter('min_fusion_interval_s', 0.20)
        self.declare_parameter('min_observer_translation_m', 0.10)
        self.declare_parameter('min_observer_yaw_degrees', 10.0)
        self.declare_parameter('tf_timeout_s', 0.10)

    def _camera_info_callback(self, message: CameraInfo) -> None:
        """Cache the most recent calibrated camera model."""
        if message.k[0] <= 0.0 or message.k[4] <= 0.0:
            self.get_logger().warning('Ignoring invalid CameraInfo intrinsics.')
            return
        self._camera_info = message
        self._camera_warning_emitted = False

    def _detection_callback(self, message: ArucoDetectionArray) -> None:
        """Localize every valid detection using TF at the image timestamp."""
        if not message.detections:
            return
        camera_info = self._camera_info
        if camera_info is None:
            if not self._camera_warning_emitted:
                self.get_logger().warning(
                    'Dropping ArUco detections until CameraInfo is available.'
                )
                self._camera_warning_emitted = True
            return
        if (
            message.image_width != camera_info.width
            or message.image_height != camera_info.height
        ):
            self.get_logger().warning(
                'Dropping detections because Image and CameraInfo '
                'dimensions differ.'
            )
            return

        source_frame = message.header.frame_id
        if not source_frame:
            source_frame = camera_info.header.frame_id
        if (
            camera_info.header.frame_id
            and source_frame != camera_info.header.frame_id
        ):
            self.get_logger().warning(
                'Dropping detections because Image and CameraInfo frames '
                'differ.'
            )
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                source_frame,
                Time.from_msg(message.header.stamp),
                timeout=self._tf_timeout,
            )
        except TransformException as error:
            self.get_logger().warning(
                f'No timestamped TF for ArUco frame: {error}'
            )
            return

        camera_matrix = np.asarray(camera_info.k).reshape(3, 3)
        distortion = np.asarray(camera_info.d, dtype=np.float64)
        observer = transform.transform.translation
        rotation = transform.transform.rotation
        observer_pose = (
            float(observer.x),
            float(observer.y),
            quaternion_yaw(
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ),
        )
        stamp_s = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )

        for detection in message.detections:
            corners = [
                [corner.x, corner.y]
                for corner in detection.corners
            ]
            estimate = estimate_square_pose(
                corners,
                camera_matrix,
                distortion,
                self._marker_length,
            )
            if estimate is None:
                continue
            depth = float(estimate.tvec[2, 0])
            if depth < self._minimum_depth or depth > self._maximum_depth:
                continue
            if (
                estimate.reprojection_error_px
                > self._maximum_reprojection_error
            ):
                continue

            camera_point = PointStamped()
            camera_point.header = message.header
            camera_point.header.frame_id = source_frame
            camera_point.point.x = float(estimate.tvec[0, 0])
            camera_point.point.y = float(estimate.tvec[1, 0])
            camera_point.point.z = float(estimate.tvec[2, 0])
            map_point = do_transform_point(camera_point, transform)

            distance = float(np.linalg.norm(estimate.tvec))
            variance = observation_variance(
                self._measurement_stddev,
                distance,
                estimate.reprojection_error_px,
            )
            result = self._database.observe(
                int(detection.id),
                (map_point.point.x, map_point.point.y),
                variance,
                stamp_s,
                observer_pose,
            )
            if not result.accepted:
                self.get_logger().debug(
                    f'Rejected marker {detection.id}: {result.reason}.'
                )

        self._marker_publisher.publish(
            self._build_marker_map(message.header.stamp)
        )

    def _build_marker_map(self, stamp: TimeMessage) -> ArucoMarkerArray:
        """Convert database records to the public marker-map message."""
        output = ArucoMarkerArray()
        output.header.frame_id = self._map_frame
        output.header.stamp = stamp
        for record in self._database.records():
            marker = ArucoMarker()
            marker.id = record.marker_id
            marker.x = float(record.state[0])
            marker.y = float(record.state[1])
            marker.covariance = [
                float(record.covariance[0, 0]),
                float(record.covariance[0, 1]),
                float(record.covariance[1, 0]),
                float(record.covariance[1, 1]),
            ]
            marker.accepted_observation_count = (
                record.accepted_observation_count
            )
            marker.rejected_observation_count = (
                record.rejected_observation_count
            )
            marker.first_seen = self._seconds_to_time(record.first_seen_s)
            marker.last_seen = self._seconds_to_time(record.last_seen_s)
            marker.confirmed = record.confirmed
            output.markers.append(marker)
        return output

    def _get_marker_map(self, request, response):
        """Return the latest marker estimates without changing state."""
        del request
        response.success = True
        response.marker_map = self._build_marker_map(
            self.get_clock().now().to_msg()
        )
        response.message = (
            f'Returned {len(response.marker_map.markers)} marker records.'
        )
        return response

    def _reset_marker_map(self, request, response):
        """Clear prior-SLAM observations and begin a final map epoch."""
        del request
        previous_count = len(self._database.records())
        self._database.clear()
        self._marker_publisher.publish(
            self._build_marker_map(self.get_clock().now().to_msg())
        )
        response.success = True
        response.message = f'Cleared {previous_count} marker records.'
        self.get_logger().info(
            'Started a new ArUco localization epoch: '
            f'{response.message}'
        )
        return response

    @staticmethod
    def _seconds_to_time(seconds: float) -> TimeMessage:
        """Convert a non-negative floating-point stamp to a ROS message."""
        seconds = max(0.0, seconds)
        whole = int(math.floor(seconds))
        nanoseconds = int(round((seconds - whole) * 1.0e9))
        if nanoseconds >= 1_000_000_000:
            whole += 1
            nanoseconds -= 1_000_000_000
        message = TimeMessage()
        message.sec = whole
        message.nanosec = nanoseconds
        return message


def main(args=None) -> None:
    """Run the ArUco localization node."""
    rclpy.init(args=args)
    node = ArucoLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
