"""Timestamp-correct ArUco localization and map-fusion node."""

import math

from builtin_interfaces.msg import Time as TimeMessage
from metr4202_aruco.localization_core import (
    MarkerDatabase,
    RelativeObservation,
    TrackingParameters,
    estimate_square_pose,
    filter_projected_observations,
    observation_variance,
    project_relative_observation,
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
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class ArucoLocalization(Node):
    """Estimate, gate, and fuse static marker positions in the map frame."""

    def __init__(self) -> None:
        """Create subscriptions, TF support, services, and marker storage."""
        super().__init__('aruco_localization')
        self._declare_parameters()
        self._odom_frame = str(self.get_parameter('odom_frame').value)
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
        self._batch_outlier_sigma = float(
            self.get_parameter('batch_outlier_sigma').value
        )
        self._batch_outlier_noise_floor_stddevs = float(
            self.get_parameter('batch_outlier_noise_floor_stddevs').value
        )
        self._tf_timeout = Duration(
            seconds=float(self.get_parameter('tf_timeout_s').value)
        )
        self._visualization_height = float(
            self.get_parameter('visualization_height_m').value
        )
        self._visualization_scale = float(
            self.get_parameter('visualization_scale_m').value
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
        self._relative_observations: list[RelativeObservation] = []
        self._final_map_from_odom = None

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
        self._visualization_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('visualization_topic').value),
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
        self._finalize_service = self.create_service(
            Trigger,
            'aruco/finalize_exploration',
            self._finalize_exploration,
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
        self.declare_parameter('visualization_topic', '/aruco/visualization')
        self.declare_parameter('visualization_height_m', 0.15)
        self.declare_parameter('visualization_scale_m', 0.18)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('marker_length_m', 0.10)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 4.0)
        self.declare_parameter('max_reprojection_error_px', 3.0)
        self.declare_parameter('measurement_stddev_m', 0.08)
        self.declare_parameter('batch_outlier_sigma', 3.5)
        self.declare_parameter('batch_outlier_noise_floor_stddevs', 3.0)
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

        stamp = Time.from_msg(message.header.stamp)
        try:
            odom_from_camera = self._tf_buffer.lookup_transform(
                self._odom_frame,
                source_frame,
                stamp,
                timeout=self._tf_timeout,
            )
        except TransformException as error:
            self.get_logger().warning(
                f'No timestamped {self._odom_frame} <- {source_frame} '
                f'TF for ArUco observation: {error}'
            )
            return

        camera_matrix = np.asarray(camera_info.k).reshape(3, 3)
        distortion = np.asarray(camera_info.d, dtype=np.float64)
        stamp_s = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        translation = odom_from_camera.transform.translation
        rotation = odom_from_camera.transform.rotation

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

            distance = float(np.linalg.norm(estimate.tvec))
            variance = observation_variance(
                self._measurement_stddev,
                distance,
                estimate.reprojection_error_px,
            )
            observation = RelativeObservation(
                marker_id=int(detection.id),
                camera_position=tuple(
                    float(value) for value in estimate.tvec[:, 0]
                ),
                stamp_s=stamp_s,
                odom_from_camera_translation=(
                    float(translation.x),
                    float(translation.y),
                    float(translation.z),
                ),
                odom_from_camera_quaternion=(
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                    float(rotation.w),
                ),
                variance_m2=variance,
            )
            self._relative_observations.append(observation)
            if self._final_map_from_odom is not None:
                self._fuse_relative_observation(observation)

        self._publish_maps(message.header.stamp)

    def _fuse_relative_observation(
        self,
        observation: RelativeObservation,
    ):
        """Project and fuse one saved observation using the final map frame."""
        if self._final_map_from_odom is None:
            raise RuntimeError('Final map <- odom transform is not available.')
        translation, quaternion = self._final_map_from_odom
        projected = project_relative_observation(
            observation,
            translation,
            quaternion,
        )
        return self._fuse_projected_observation(projected)

    def _fuse_projected_observation(self, projected):
        """Fuse one final-map observation into the persistent estimator."""
        result = self._database.observe(
            projected.marker_id,
            projected.measurement_xy,
            projected.variance_m2,
            projected.stamp_s,
            projected.observer_pose,
        )
        if not result.accepted:
            self.get_logger().debug(
                f'Rejected marker {projected.marker_id}: {result.reason}.'
            )
        return result

    def _publish_maps(self, stamp: TimeMessage) -> None:
        """Publish both the API map and its RViz representation."""
        self._marker_publisher.publish(self._build_marker_map(stamp))
        self._visualization_publisher.publish(
            self._build_visualization(stamp)
        )

    def _build_visualization(self, stamp: TimeMessage) -> MarkerArray:
        """Build RViz markers for all current ArUco position estimates."""
        output = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.frame_id = self._map_frame
        clear_marker.header.stamp = stamp
        clear_marker.action = Marker.DELETEALL
        output.markers.append(clear_marker)

        for record in self._database.records():
            position = Marker()
            position.header.frame_id = self._map_frame
            position.header.stamp = stamp
            position.ns = 'aruco_position'
            position.id = record.marker_id * 2
            position.type = Marker.SPHERE
            position.action = Marker.ADD
            position.pose.position.x = float(record.state[0])
            position.pose.position.y = float(record.state[1])
            position.pose.position.z = self._visualization_height
            position.pose.orientation.w = 1.0
            position.scale.x = self._visualization_scale
            position.scale.y = self._visualization_scale
            position.scale.z = self._visualization_scale
            if record.confirmed:
                position.color.r = 0.1
                position.color.g = 1.0
                position.color.b = 0.2
            else:
                position.color.r = 1.0
                position.color.g = 0.55
                position.color.b = 0.05
            position.color.a = 0.9
            output.markers.append(position)

            label = Marker()
            label.header.frame_id = self._map_frame
            label.header.stamp = stamp
            label.ns = 'aruco_label'
            label.id = record.marker_id * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(record.state[0])
            label.pose.position.y = float(record.state[1])
            label.pose.position.z = (
                self._visualization_height + self._visualization_scale
            )
            label.pose.orientation.w = 1.0
            label.scale.z = max(0.12, self._visualization_scale * 0.8)
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'ArUco {record.marker_id}'
            if not record.confirmed:
                label.text += ' (tentative)'
            output.markers.append(label)

        return output

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

    def _finalize_exploration(self, request, response):
        """Reproject all exploration observations using final map <- odom."""
        del request
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._odom_frame,
                Time(),
                timeout=self._tf_timeout,
            )
        except TransformException as error:
            response.success = False
            response.message = (
                f'Could not finalize ArUco observations: {error}'
            )
            return response

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self._final_map_from_odom = (
            (
                float(translation.x),
                float(translation.y),
                float(translation.z),
            ),
            (
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ),
        )
        self._database.clear()
        projected_observations = [
            project_relative_observation(
                observation,
                self._final_map_from_odom[0],
                self._final_map_from_odom[1],
            )
            for observation in self._relative_observations
        ]
        filtered_observations = filter_projected_observations(
            projected_observations,
            sigma=self._batch_outlier_sigma,
            noise_floor_stddevs=(
                self._batch_outlier_noise_floor_stddevs
            ),
        )
        batch_rejected_count = (
            len(projected_observations) - len(filtered_observations)
        )
        accepted_count = 0
        for observation in sorted(
            filtered_observations,
            key=lambda item: item.stamp_s,
        ):
            result = self._fuse_projected_observation(observation)
            accepted_count += int(result.accepted)

        self._publish_maps(self.get_clock().now().to_msg())
        response.success = True
        response.message = (
            f'Reprojected {len(self._relative_observations)} saved '
            f'observations using final map <- odom; '
            f'{batch_rejected_count} gross batch outliers removed; '
            f'{accepted_count} passed fusion gates.'
        )
        self.get_logger().info(response.message)
        return response

    def _reset_marker_map(self, request, response):
        """Explicitly clear observations, final transform, and estimates."""
        del request
        previous_record_count = len(self._database.records())
        previous_observation_count = len(self._relative_observations)
        self._database.clear()
        self._relative_observations.clear()
        self._final_map_from_odom = None
        self._publish_maps(self.get_clock().now().to_msg())
        response.success = True
        response.message = (
            f'Cleared {previous_observation_count} saved observations and '
            f'{previous_record_count} marker records.'
        )
        self.get_logger().info(
            'Explicitly reset ArUco localization: '
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
