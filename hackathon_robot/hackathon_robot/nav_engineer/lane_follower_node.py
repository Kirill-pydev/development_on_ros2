#!/usr/bin/env python3
"""Следование по линии разметки: камера → оценка смещения линии в ROI → /cmd_vel."""

from __future__ import annotations

from collections import deque
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


class LaneFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('lane_follower')

        self.declare_parameter('image_topic', '/camera_node/image_raw')
        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('max_angular', 0.7)
        self.declare_parameter('kp', 1.8)
        self.declare_parameter('kd', 0.12)
        self.declare_parameter('roi_top_ratio', 0.55)
        self.declare_parameter('min_lane_pixels', 800)

        self.declare_parameter('use_white_mask', True)
        self.declare_parameter('hsv_white_low', [0, 0, 175])
        self.declare_parameter('hsv_white_high', [180, 55, 255])

        self.declare_parameter('use_yellow_mask', False)
        self.declare_parameter('hsv_yellow_low', [18, 80, 100])
        self.declare_parameter('hsv_yellow_high', [35, 255, 255])

        topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._linear = float(self.get_parameter('linear_speed').value)
        self._max_angular = float(self.get_parameter('max_angular').value)
        self._kp = float(self.get_parameter('kp').value)
        self._kd = float(self.get_parameter('kd').value)
        self._roi_top = float(self.get_parameter('roi_top_ratio').value)
        self._min_pix = int(self.get_parameter('min_lane_pixels').value)
        self._use_white = bool(self.get_parameter('use_white_mask').value)
        self._use_yellow = bool(self.get_parameter('use_yellow_mask').value)

        self._white_low = tuple(int(x) for x in self.get_parameter('hsv_white_low').value)
        self._white_high = tuple(int(x) for x in self.get_parameter('hsv_white_high').value)
        self._yl_low = tuple(int(x) for x in self.get_parameter('hsv_yellow_low').value)
        self._yl_high = tuple(int(x) for x in self.get_parameter('hsv_yellow_high').value)

        self._bridge = CvBridge()
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image, topic, self._image_cb, 10)
        self.create_timer(0.05, self._control_tick)

        self._err_hist: deque[float] = deque(maxlen=5)
        self._last_err = 0.0
        self._have_line = False
        self._last_img_ts = None
        self._no_frame_log_t = 0.0

        self.get_logger().info(f'Lane follower: «{topic}», white={self._use_white} yellow={self._use_yellow}')

    def _build_mask(self, hsv) -> object:
        mask = None
        if self._use_white:
            mw = cv2.inRange(hsv, self._white_low, self._white_high)
            mask = mw
        if self._use_yellow:
            my = cv2.inRange(hsv, self._yl_low, self._yl_high)
            mask = my if mask is None else cv2.bitwise_or(mask, my)
        if mask is None:
            mask = cv2.inRange(hsv, self._white_low, self._white_high)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, (5, 5), iterations=1)
        return mask

    def _image_cb(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge: {e}')
            return

        h, w = bgr.shape[:2]
        y0 = int(h * self._roi_top)
        roi = bgr[y0:h, 0:w]
        if roi.size == 0:
            self._have_line = False
            self._last_img_ts = self.get_clock().now()
            return

        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = self._build_mask(hsv)
        m = cv2.moments(mask)
        if m['m00'] < self._min_pix:
            self._have_line = False
            self._last_img_ts = self.get_clock().now()
            return

        cx = int(m['m10'] / m['m00'])
        roi_w = roi.shape[1]
        err = (cx - roi_w / 2.0) / max(roi_w / 2.0, 1.0)
        err = max(-1.0, min(1.0, err))
        self._last_err = err
        self._have_line = True
        self._last_img_ts = self.get_clock().now()

    def _control_tick(self) -> None:
        now = self.get_clock().now()
        cmd = Twist()
        if self._last_img_ts is None:
            self._cmd_pub.publish(cmd)
            return
        if (now - self._last_img_ts).nanoseconds > int(0.35e9):
            t = time.monotonic()
            if t - self._no_frame_log_t > 2.0:
                self.get_logger().warn('Нет свежего кадра камеры — останавливаюсь')
                self._no_frame_log_t = t
            self._cmd_pub.publish(cmd)
            return

        if not self._have_line:
            cmd.angular.z = 0.15
            cmd.linear.x = 0.0
            self._cmd_pub.publish(cmd)
            return

        self._err_hist.append(self._last_err)
        d_err = 0.0
        if len(self._err_hist) >= 2:
            d_err = self._err_hist[-1] - self._err_hist[-2]

        w_ang = -(self._kp * self._last_err + self._kd * d_err / 0.05)
        w_ang = max(-self._max_angular, min(self._max_angular, w_ang))
        slow = max(0.35, 1.0 - 0.55 * abs(self._last_err))
        cmd.linear.x = self._linear * slow
        cmd.angular.z = float(w_ang)
        self._cmd_pub.publish(cmd)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LaneFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
