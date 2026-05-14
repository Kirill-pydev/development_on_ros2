#!/usr/bin/env python3
"""GOD: едет вперёд, пока не появится человек — затем стоп.

События пишутся в файл и в ROS-лог (терминал) при mirror_logs_to_terminal=true.
Встроенный HOG или топик /person_detected (use_external_person_topic).
"""

from __future__ import annotations

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

import os
from datetime import datetime, timezone


def default_god_log_dir(cwd: str | None = None) -> str:
    base = cwd if cwd else os.getcwd()
    d = os.path.join(base, 'god_logs')
    os.makedirs(d, exist_ok=True)
    return d


def make_log_path(log_dir: str, stem: str) -> str:
    os.makedirs(log_dir, exist_ok=True)
    ts = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
    return os.path.join(log_dir, f'{stem}_{ts}.log')


class GodFileLog:
    """Файл + опционально зеркало в ROS-лог (терминал)."""

    def __init__(
        self,
        path: str,
        *,
        ros_logger: object | None = None,
        mirror_to_terminal: bool = True,
    ) -> None:
        self._path = path
        self._ros = ros_logger
        self._mirror = mirror_to_terminal
        self._fp = open(path, 'a', encoding='utf-8')
        self._fp.write(f'# started {datetime.now(timezone.utc).isoformat()}\n')
        self._fp.flush()
        if self._mirror and self._ros is not None:
            self._ros.info(f'[god] лог-файл: {path}')

    def close(self) -> None:
        try:
            self._fp.close()
        except OSError:
            pass

    def line(self, *fields: object) -> None:
        t = time.perf_counter()
        row = '\t'.join(str(x) for x in fields)
        self._fp.write(f'{t:.6f}\t{row}\n')
        self._fp.flush()
        if self._mirror and self._ros is not None:
            self._ros.info(f'[god] {row}')


class GodPersonStopNode(Node):
    def __init__(self) -> None:
        super().__init__('god_person_stop')

        self.declare_parameter('log_dir', '')
        self.declare_parameter('image_topic', '/camera_node/image_raw')
        self.declare_parameter('use_external_person_topic', False)
        self.declare_parameter('person_topic', '/person_detected')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('person_latched_stops', True)
        self.declare_parameter('mirror_logs_to_terminal', True)

        log_dir = self.get_parameter('log_dir').get_parameter_value().string_value.strip()
        if not log_dir:
            log_dir = default_god_log_dir()
        log_path = make_log_path(log_dir, 'god_person_stop')
        mirror = self.get_parameter('mirror_logs_to_terminal').get_parameter_value().bool_value
        self._glog = GodFileLog(
            log_path,
            ros_logger=self.get_logger(),
            mirror_to_terminal=mirror,
        )

        self._im_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._external = self.get_parameter('use_external_person_topic').get_parameter_value().bool_value
        self._person_topic = self.get_parameter('person_topic').get_parameter_value().string_value
        self._v = float(self.get_parameter('linear_speed').value)
        self._latched = self.get_parameter('person_latched_stops').get_parameter_value().bool_value

        self._bridge = CvBridge()
        self._hog = cv2.HOGDescriptor()
        self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self._person_now = False
        self._halted = False
        self._last_frame_t = None
        self._ticks = 0
        self._no_frame_log_t = 0.0

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image, self._im_topic, self._image_cb, 10)
        if self._external:
            self.create_subscription(Bool, self._person_topic, self._person_cb, 10)
            self._glog.line('event', 'external_person', self._person_topic)
        else:
            self._glog.line('event', 'internal_hog')

        self.create_timer(0.05, self._control)

        self.get_logger().info('GOD person-stop: события дублируются в терминал (mirror_logs_to_terminal)')
        self._glog.line('event', 'start', self._im_topic, self._v)

    def destroy_node(self) -> bool:
        self._glog.line('event', 'shutdown', f'halted={self._halted}')
        self._glog.close()
        self._cmd_pub.publish(Twist())
        self.get_logger().info('GOD person-stop: остановка, cmd_vel=0')
        return super().destroy_node()

    def _person_cb(self, msg: Bool) -> None:
        prev = self._person_now
        self._person_now = bool(msg.data)
        if self._person_now and not prev:
            self._glog.line('person_ext', True)

    def _image_cb(self, msg: Image) -> None:
        self._last_frame_t = self.get_clock().now()
        if self._external:
            return
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
        if detected and not self._person_now:
            self._glog.line('person_hog', len(boxes))
        self._person_now = detected

    def _control(self) -> None:
        cmd = Twist()
        now = self.get_clock().now()

        if not self._external:
            stale = (
                self._last_frame_t is None
                or (now - self._last_frame_t).nanoseconds > int(0.4e9)
            )
            if stale:
                tmono = time.monotonic()
                if tmono - self._no_frame_log_t > 2.0:
                    self.get_logger().warn('Нет кадра камеры')
                    self._no_frame_log_t = tmono
                self._cmd_pub.publish(cmd)
                self._glog.line('warn', 'no_frame')
                return

        if self._person_now:
            if not self._halted:
                self._halted = True
                self.get_logger().warn('Человек — остановка')
                self._glog.line('state', 'HALTED', 'person')
            self._cmd_pub.publish(cmd)
            return

        if self._latched and self._halted:
            self._cmd_pub.publish(cmd)
            self._ticks += 1
        if self._ticks % 20 == 0:
            self._glog.line('state', 'HALTED_LATCHED')
            return

        cmd.linear.x = self._v
        self._cmd_pub.publish(cmd)
        self._ticks += 1
        if self._ticks % 20 == 0:
            self._glog.line('state', 'CRUISE', f'vx={self._v}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GodPersonStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
