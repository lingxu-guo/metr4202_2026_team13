"""Continuous camera-based ArUco detection node."""

import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
from metr4202_interfaces.msg import ArucoDetection as ArucoDetectionMessage
from metr4202_interfaces.msg import ArucoDetectionArray
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def resolve_dictionary(name: str):
    """Resolve an ArUco dictionary across OpenCV 4.5 and newer APIs."""
    value = getattr(cv2.aruco, name, None)
    if value is None:
        raise ValueError(f'Unsupported ArUco dictionary: {name}')
    getter = getattr(cv2.aruco, 'getPredefinedDictionary', None)
    if getter is None:
        getter = cv2.aruco.Dictionary_get
    return getter(value)


def create_detector_parameters():
    """Create detector parameters across legacy and current OpenCV APIs."""
    parameter_type = getattr(cv2.aruco, 'DetectorParameters', None)
    if parameter_type is not None:
        try:
            return parameter_type()
        except TypeError:
            pass
    return cv2.aruco.DetectorParameters_create()


class ArucoDetection(Node):
    """Detect marker IDs and image corners without performing localization."""

    def __init__(self) -> None:
        """Create publishers, parameters, and the camera subscription."""
        super().__init__('aruco_detection')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/aruco/detections')
        self.declare_parameter('debug_image_topic', '/aruco/debug_image')
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('min_marker_perimeter_px', 40.0)
        self.declare_parameter('max_detection_rate_hz', 0.0)
        self.declare_parameter('publish_debug_image', False)

        dictionary_name = str(self.get_parameter('dictionary').value)
        self._dictionary = resolve_dictionary(dictionary_name)
        self._detector_parameters = create_detector_parameters()
        detector_type = getattr(cv2.aruco, 'ArucoDetector', None)
        self._detector = (
            detector_type(self._dictionary, self._detector_parameters)
            if detector_type is not None
            else None
        )
        self._bridge = CvBridge()
        self._min_perimeter = float(
            self.get_parameter('min_marker_perimeter_px').value
        )
        maximum_rate = float(
            self.get_parameter('max_detection_rate_hz').value
        )
        self._minimum_period = (
            1.0 / maximum_rate if maximum_rate > 0.0 else 0.0
        )
        self._last_processed_stamp = None
        self._publish_debug = bool(
            self.get_parameter('publish_debug_image').value
        )

        detections_topic = str(
            self.get_parameter('detections_topic').value
        )
        self._detection_publisher = self.create_publisher(
            ArucoDetectionArray,
            detections_topic,
            qos_profile_sensor_data,
        )
        self._debug_publisher = None
        if self._publish_debug:
            self._debug_publisher = self.create_publisher(
                Image,
                str(self.get_parameter('debug_image_topic').value),
                qos_profile_sensor_data,
            )
        self._image_subscription = self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._image_callback,
            qos_profile_sensor_data,
        )
        detector_api = 'ArucoDetector' if self._detector else 'detectMarkers'
        self.get_logger().info(
            f'ArUco detection started with {dictionary_name} on '
            f'{self.get_parameter("image_topic").value}; API={detector_api}.'
        )

    def _image_callback(self, image_message: Image) -> None:
        """Detect all sufficiently large markers in one camera frame."""
        stamp = (
            float(image_message.header.stamp.sec)
            + float(image_message.header.stamp.nanosec) * 1.0e-9
        )
        if (
            self._last_processed_stamp is not None
            and stamp - self._last_processed_stamp < self._minimum_period
        ):
            return
        self._last_processed_stamp = stamp

        try:
            gray = self._bridge.imgmsg_to_cv2(
                image_message,
                desired_encoding='mono8',
            )
        except CvBridgeError as error:
            self.get_logger().error(f'Could not convert camera image: {error}')
            return

        if self._detector is not None:
            corners, identifiers, _ = self._detector.detectMarkers(gray)
        else:
            corners, identifiers, _ = cv2.aruco.detectMarkers(
                gray,
                self._dictionary,
                parameters=self._detector_parameters,
            )
        output = ArucoDetectionArray()
        output.header = image_message.header
        output.image_width = image_message.width
        output.image_height = image_message.height

        kept_corners = []
        kept_identifiers = []
        if identifiers is not None:
            for marker_corners, marker_id in zip(
                corners,
                identifiers.reshape(-1),
            ):
                points = np.asarray(marker_corners).reshape(4, 2)
                perimeter = sum(
                    math.hypot(
                        float(points[(index + 1) % 4, 0] - points[index, 0]),
                        float(points[(index + 1) % 4, 1] - points[index, 1]),
                    )
                    for index in range(4)
                )
                if perimeter < self._min_perimeter:
                    continue

                detection = ArucoDetectionMessage()
                detection.id = int(marker_id)
                detection.perimeter_px = float(perimeter)
                for destination, source in zip(detection.corners, points):
                    destination.x = float(source[0])
                    destination.y = float(source[1])
                    destination.z = 0.0
                output.detections.append(detection)
                kept_corners.append(marker_corners)
                kept_identifiers.append([int(marker_id)])

        self._detection_publisher.publish(output)
        if self._debug_publisher is not None:
            debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            if kept_identifiers:
                cv2.aruco.drawDetectedMarkers(
                    debug,
                    kept_corners,
                    np.asarray(kept_identifiers, dtype=np.int32),
                )
            debug_message = self._bridge.cv2_to_imgmsg(
                debug,
                encoding='bgr8',
            )
            debug_message.header = image_message.header
            self._debug_publisher.publish(debug_message)


def main(args=None) -> None:
    """Run the ArUco detection node."""
    rclpy.init(args=args)
    node = ArucoDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
