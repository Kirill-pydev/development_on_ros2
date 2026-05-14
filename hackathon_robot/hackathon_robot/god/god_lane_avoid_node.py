#!/usr/bin/env python3
"""GOD: разметка (камера) + объезд (лидар).

Записи лога — в файл и в терминал (ROS log), если mirror_logs_to_terminal=true.
"""

from __future__ import annotations

import math
import os
import time
from collections import deque
from datetime import datetime, timezone
from enum import Enum, auto

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


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


class AvoidState(Enum):
    LANE = auto()
    FOLLOW_GAP = auto()
    RECOVER_HEADING = auto()
    BACK_UP = auto()


def _yaw_from_orientation(ox: float, oy: float, oz: float, ow: float) -> float:
    siny_cosp = 2.0 * (ow * oz + ox * oy)
    cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class GodLaneAvoidNode(Node):
    def __init__(self) -> None:
        super().__init__('god_lane_avoid')

        self.declare_parameter('log_dir', '')
        self.declare_parameter('image_topic', '/camera_node/image_raw')

        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('max_angular_lane', 0.65)
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

        self.declare_parameter('rotation_speed', 0.65)
        self.declare_parameter('obstacle_distance', 0.38)
        self.declare_parameter('emergency_distance', 0.14)
        self.declare_parameter('safe_distance', 0.55)
        self.declare_parameter('field_of_view', 70.0)
        self.declare_parameter('gap_search_step_deg', 6.0)
        self.declare_parameter('gap_cone_half_deg', 10.0)
        self.declare_parameter('recover_yaw_gain', 1.35)
        self.declare_parameter('recover_yaw_tol_deg', 12.0)
        self.declare_parameter('recover_min_clear_front', 0.42)
        self.declare_parameter('clear_frames_to_recover', 3)
        self.declare_parameter('stuck_timeout_s', 4.0)
        self.declare_parameter('stuck_move_eps', 0.02)
        self.declare_parameter('backup_duration_s', 0.75)
        self.declare_parameter('backup_speed', -0.12)
        self.declare_parameter('mirror_logs_to_terminal', True)

        log_dir = self.get_parameter('log_dir').get_parameter_value().string_value.strip()
        if not log_dir:
            log_dir = default_god_log_dir()
        log_file = make_log_path(log_dir, 'god_lane_avoid')
        mirror = self.get_parameter('mirror_logs_to_terminal').get_parameter_value().bool_value
        self._glog = GodFileLog(
            log_file,
            ros_logger=self.get_logger(),
            mirror_to_terminal=mirror,
        )

        im_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._linear = float(self.get_parameter('linear_speed').value)
        self._max_ang_lane = float(self.get_parameter('max_angular_lane').value)
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

        self.rotation_speed = float(self.get_parameter('rotation_speed').value)
        self.obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        self.emergency_distance = float(self.get_parameter('emergency_distance').value)
        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.field_of_view = float(self.get_parameter('field_of_view').value)
        self.gap_step = float(self.get_parameter('gap_search_step_deg').value)
        self.gap_cone = float(self.get_parameter('gap_cone_half_deg').value)
        self.recover_k = float(self.get_parameter('recover_yaw_gain').value)
        self.recover_tol = math.radians(float(self.get_parameter('recover_yaw_tol_deg').value))
        self.recover_min_front = float(self.get_parameter('recover_min_clear_front').value)
        self.clear_frames = int(self.get_parameter('clear_frames_to_recover').value)
        self.stuck_timeout = float(self.get_parameter('stuck_timeout_s').value)
        self.stuck_eps = float(self.get_parameter('stuck_move_eps').value)
        self.backup_duration = float(self.get_parameter('backup_duration_s').value)
        self.backup_speed = float(self.get_parameter('backup_speed').value)

        self._bridge = CvBridge()
        self._err_hist: deque[float] = deque(maxlen=5)
        self._last_err = 0.0
        self._have_line = False
        self._last_img_ts = None

        self.state = AvoidState.LANE
        self._target_course_yaw: float | None = None
        self._clear_counter = 0
        self._backup_until = 0.0
        self._stuck_ref_xy = (0.0, 0.0)
        self._stuck_sample_t = time.monotonic()
        self._stuck_initialized = False

        self.latest_scan: LaserScan | None = None
        self.scan_received = False
        self.current_yaw = 0.0
        self.last_position = None

        self.lidar_offset_angle_deg = 0.0
        self.is_calibrated = False
        self._load_calib()

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._state_pub = self.create_publisher(String, '/god_lane_avoid_state', 10)
        self.create_subscription(Image, im_topic, self._image_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self.create_timer(0.05, self._control_loop)
        self._tick_log = 0

        self._glog.line('event', 'start', im_topic)
        self.get_logger().info('GOD lane+avoid: события дублируются в терминал (см. mirror_logs_to_terminal)')

    def destroy_node(self) -> bool:
        self._glog.line('event', 'shutdown')
        self._glog.close()
        self._cmd_pub.publish(Twist())
        self.get_logger().info('GOD lane+avoid: остановка, cmd_vel=0')
        return super().destroy_node()

    def _load_calib(self) -> None:
        calib_path = os.path.join(os.getcwd(), 'params', 'lidar_calibration.yaml')
        try:
            if not os.path.exists(calib_path):
                self.get_logger().warn(f'Нет калибровки: {calib_path}')
                return
            with open(calib_path, 'r', encoding='utf-8') as f:
                d = yaml.safe_load(f)
            self.lidar_offset_angle_deg = float(d.get('lidar_offset_angle_deg', 0.0))
            self.is_calibrated = True
            self._glog.line('calib', self.lidar_offset_angle_deg)
        except (OSError, TypeError, ValueError, yaml.YAMLError) as e:
            self.get_logger().error(f'Ошибка калибровки: {e}')
            self._glog.line('calib_error', str(e))

    def get_corrected_angle_deg(self, raw_angle_deg: float) -> float:
        corrected = raw_angle_deg - self.lidar_offset_angle_deg
        while corrected > 180:
            corrected -= 360
        while corrected < -180:
            corrected += 360
        return corrected

    @staticmethod
    def _raw_angle_deg_from_index(scan: LaserScan, index: int) -> float:
        angle_rad = scan.angle_min + index * scan.angle_increment
        angle_deg = math.degrees(angle_rad)
        if angle_deg > 180:
            angle_deg -= 360
        return angle_deg

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
        mask = self._lane_mask(hsv)
        m = cv2.moments(mask)
        if m['m00'] < self._min_pix:
            self._have_line = False
            self._last_img_ts = self.get_clock().now()
            return
        cx = int(m['m10'] / m['m00'])
        roi_w = roi.shape[1]
        err = (cx - roi_w / 2.0) / max(roi_w / 2.0, 1.0)
        self._last_err = max(-1.0, min(1.0, err))
        self._have_line = True
        self._last_img_ts = self.get_clock().now()

    def _lane_mask(self, hsv):
        mask = None
        if self._use_white:
            mw = cv2.inRange(hsv, self._white_low, self._white_high)
            mask = mw
        if self._use_yellow:
            my = cv2.inRange(hsv, self._yl_low, self._yl_high)
            mask = my if mask is None else cv2.bitwise_or(mask, my)
        if mask is None:
            mask = cv2.inRange(hsv, self._white_low, self._white_high)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, (5, 5), iterations=1)

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.scan_received = True

    def _odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.current_yaw = _yaw_from_orientation(q.x, q.y, q.z, q.w)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_position is None:
            self._stuck_ref_xy = (x, y)
            self._stuck_initialized = True
        self.last_position = (x, y)

    def _front_blocked(self) -> tuple[bool, float]:
        if not self.scan_received or self.latest_scan is None:
            return False, float('inf')
        scan = self.latest_scan
        half_fov = self.field_of_view / 2.0
        min_d = float('inf')
        for i, distance in enumerate(scan.ranges):
            if distance < scan.range_min or distance > scan.range_max:
                continue
            raw_angle = self._raw_angle_deg_from_index(scan, i)
            corrected_angle = self.get_corrected_angle_deg(raw_angle)
            if abs(corrected_angle) <= half_fov and distance < min_d:
                min_d = distance
        blocked = min_d < self.obstacle_distance
        return blocked, min_d

    def _min_range_cone(self, scan: LaserScan, center_deg: float, half_deg: float) -> float:
        dmin = float('inf')
        for i, distance in enumerate(scan.ranges):
            if distance < scan.range_min or distance > scan.range_max:
                continue
            raw_angle = self._raw_angle_deg_from_index(scan, i)
            corrected = self.get_corrected_angle_deg(raw_angle)
            if abs(corrected - center_deg) <= half_deg and distance < dmin:
                dmin = distance
        return dmin

    def _best_gap(self) -> tuple[float, float]:
        if self.latest_scan is None:
            return 0.0, 0.0
        scan = self.latest_scan
        best_score = -1.0
        best_deg = 0.0
        deg = -85.0
        while deg <= 85.0:
            dist = self._min_range_cone(scan, deg, self.gap_cone)
            if dist >= scan.range_max * 0.99:
                dist = scan.range_max * 0.99
            rad = math.radians(deg)
            score = dist * max(0.15, math.cos(rad)) ** 2
            if score > best_score:
                best_score = score
                best_deg = deg
            deg += self.gap_step
        return best_deg, best_score

    def _path_clear_recover(self) -> bool:
        if self.latest_scan is None:
            return False
        d = self._min_range_cone(self.latest_scan, 0.0, self.field_of_view / 2.2)
        return d > self.recover_min_front

    def _check_stuck(self) -> bool:
        if not self._stuck_initialized or self.last_position is None:
            return False
        now = time.monotonic()
        if now - self._stuck_sample_t < self.stuck_timeout:
            return False
        x, y = self.last_position
        moved = math.hypot(x - self._stuck_ref_xy[0], y - self._stuck_ref_xy[1])
        self._stuck_sample_t = now
        self._stuck_ref_xy = (x, y)
        active = self.state in (AvoidState.LANE, AvoidState.FOLLOW_GAP, AvoidState.RECOVER_HEADING)
        return active and moved < self.stuck_eps

    def _lane_twist(self) -> Twist:
        cmd = Twist()
        now = self.get_clock().now()
        if self._last_img_ts is None or (now - self._last_img_ts).nanoseconds > int(0.35e9):
            return cmd
        if not self._have_line:
            cmd.angular.z = 0.12
            return cmd
        self._err_hist.append(self._last_err)
        d_err = (self._err_hist[-1] - self._err_hist[-2]) if len(self._err_hist) >= 2 else 0.0
        w = -(self._kp * self._last_err + self._kd * d_err / 0.05)
        w = max(-self._max_ang_lane, min(self._max_ang_lane, w))
        slow = max(0.35, 1.0 - 0.55 * abs(self._last_err))
        cmd.linear.x = self._linear * slow
        cmd.angular.z = float(w)
        return cmd

    def _control_loop(self) -> None:
        if not self.is_calibrated:
            self._cmd_pub.publish(Twist())
            return

        blocked, front_min = self._front_blocked()

        if front_min < self.emergency_distance and self.state != AvoidState.BACK_UP:
            self.state = AvoidState.BACK_UP
            self._backup_until = time.monotonic() + self.backup_duration
            self._glog.line('state', 'BACK_UP', 'emergency', front_min)
            self.get_logger().warn(f'Аварийно близко: {front_min:.2f} м')
            self._cmd_pub.publish(Twist())
            self._state_pub.publish(String(data=self.state.name))
            self._emit(blocked, front_min, Twist())
            return

        if self._check_stuck() and self.state != AvoidState.BACK_UP:
            self.state = AvoidState.BACK_UP
            self._backup_until = time.monotonic() + self.backup_duration
            self._stuck_sample_t = time.monotonic()
            if self.last_position is not None:
                self._stuck_ref_xy = self.last_position
            self._glog.line('state', 'BACK_UP', 'stuck')

        cmd = Twist()

        if self.state == AvoidState.LANE:
            if blocked:
                self._target_course_yaw = self.current_yaw
                self._clear_counter = 0
                self.state = AvoidState.FOLLOW_GAP
                self._glog.line('state', 'FOLLOW_GAP', 'blocked', front_min)
            else:
                cmd = self._lane_twist()

        elif self.state == AvoidState.FOLLOW_GAP:
            scan = self.latest_scan
            best_deg, score = self._best_gap()
            if scan is not None and blocked and score < self.emergency_distance * 0.35:
                cmd.linear.x = 0.0
                cmd.angular.z = math.copysign(self.rotation_speed, best_deg if best_deg != 0 else 1.0)
            else:
                rad = math.radians(best_deg)
                cmd.angular.z = max(-self.rotation_speed, min(self.rotation_speed, 2.2 * rad))
                speed_scale = min(1.0, max(0.25, score / max(self.safe_distance, 0.1)))
                cmd.linear.x = self._linear * 0.78 * speed_scale
            if not blocked:
                self._clear_counter += 1
            else:
                self._clear_counter = 0
            if self._clear_counter >= self.clear_frames and self._path_clear_recover():
                if self._target_course_yaw is not None:
                    self.state = AvoidState.RECOVER_HEADING
                    self._glog.line('state', 'RECOVER_HEADING')

        elif self.state == AvoidState.RECOVER_HEADING:
            if self._target_course_yaw is None:
                self.state = AvoidState.LANE
            else:
                err = _angle_diff(self._target_course_yaw, self.current_yaw)
                if abs(err) < self.recover_tol:
                    self.state = AvoidState.LANE
                    self._target_course_yaw = None
                    self._glog.line('state', 'LANE', 'recovered')
                elif self._front_blocked()[0]:
                    self.state = AvoidState.FOLLOW_GAP
                    self._clear_counter = 0
                    self._glog.line('state', 'FOLLOW_GAP', 'during_recover')
                else:
                    cmd.linear.x = self._linear * 0.55
                    cmd.angular.z = max(-self.rotation_speed, min(self.rotation_speed, self.recover_k * err))

        elif self.state == AvoidState.BACK_UP:
            if time.monotonic() >= self._backup_until:
                self.state = AvoidState.FOLLOW_GAP
                self._clear_counter = 0
                self._glog.line('state', 'FOLLOW_GAP', 'after_backup')
            else:
                cmd.linear.x = self.backup_speed

        self._state_pub.publish(String(data=self.state.name))
        self._cmd_pub.publish(cmd)
        self._emit(blocked, front_min, cmd)

    def _emit(self, blocked: bool, front_min: float, cmd: Twist) -> None:
        self._tick_log += 1
        if self._tick_log % 8 != 0:
            return
        self._glog.line(
            'tick',
            self.state.name,
            'blocked' if blocked else 'clear',
            f'{front_min:.3f}',
            f'have_line={self._have_line}',
            f'err={self._last_err:.3f}',
            f'vx={cmd.linear.x:.3f}',
            f'wz={cmd.angular.z:.3f}',
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GodLaneAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
