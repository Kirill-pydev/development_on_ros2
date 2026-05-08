#!/usr/bin/env python3
"""Детектор людей HOG (OpenCV), по занятию 19 курса."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


class PersonDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('person_detector')

        self.declare_parameter('image_topic', '/camera_node/image_raw')
        self.declare_parameter('publish_qos_depth', 5)

        topic = self.get_parameter('image_topic').get_parameter_value().string_value
        qos = int(self.get_parameter('publish_qos_depth').get_parameter_value().integer_value)

        self._bridge = CvBridge()
        self._hog = cv2.HOGDescriptor()
        self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self._pub_detected = self.create_publisher(Bool, '/person_detected', qos)
        self._pub_offset = self.create_publisher(Float32, '/person_offset_x', qos)

        self.create_subscription(Image, topic, self._image_cb, 10)

        self.get_logger().info(f'PersonDetector (HOG): подписка «{topic}»')

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge: {e}')
            return

        h, w = frame.shape[:2]
        small = cv2.resize(frame, (max(1, w // 2), max(1, h // 2)))
        boxes, _ = self._hog.detectMultiScale(
            small,
            winStride=(8, 8),
            padding=(4, 4),
            scale=1.05,
        )
        detected = len(boxes) > 0
        self._pub_detected.publish(Bool(data=detected))
        if detected:
            bx, by, bw, bh = boxes[0]
            sw = small.shape[1]
            center_x = (bx + bw / 2.0) / float(sw)
            offset = float(center_x - 0.5)
            self._pub_offset.publish(Float32(data=offset))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
