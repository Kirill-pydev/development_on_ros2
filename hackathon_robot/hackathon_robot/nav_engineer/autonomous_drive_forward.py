import math
import os
import time
from enum import Enum

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String


class RobotState(Enum):
    MOVING_FORWARD = 1
    FOLLOW_GAP = 2
    RECOVER_HEADING = 3
    BACK_UP = 4
    STOPPED = 5


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


class AutonomousDrive(Node):
    def __init__(self):
        super().__init__('autonomous_drive')

        self.declare_parameter('linear_speed', 0.2)
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

        self.linear_speed = float(self.get_parameter('linear_speed').value)
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

        self.state = RobotState.MOVING_FORWARD
        self._target_course_yaw: float | None = None
        self._clear_counter = 0
        self._backup_until = 0.0
        self._stuck_ref_xy = (0.0, 0.0)
        self._stuck_sample_t = time.monotonic()
        self._stuck_initialized = False
        self._last_odom_t = time.monotonic()

        self.total_distance = 0.0
        self.start_position = None
        self.last_position = None
        self.movement_start_time = time.time()

        self.latest_scan = None
        self.scan_received = False
        self.current_yaw = 0.0
        self.odom_received = False

        self.lidar_offset_angle_deg = 0.0
        self.lidar_offset_angle_rad = 0.0
        self.is_calibrated = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.distance_pub = self.create_publisher(Float32, '/distance_traveled', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.create_timer(0.1, self.control_loop)
        self.create_timer(2.0, self.check_progress)

        self.load_lidar_calibration()

        self.get_logger().info('=' * 60)
        self.get_logger().info('AUTO DRIVE: вперёд + объезд по «щели» и одометрии курса')
        self.get_logger().info(f'Скорость: {self.linear_speed} м/с | триггер: {self.obstacle_distance} м')
        self.get_logger().info(f'Калибровка лидара: {"OK" if self.is_calibrated else "НЕТ"}')
        self.get_logger().info('=' * 60)

    def load_lidar_calibration(self):
        calib_path = os.path.join(os.getcwd(), 'params', 'lidar_calibration.yaml')
        try:
            if not os.path.exists(calib_path):
                self.get_logger().warn(f'Нет файла калибровки: {calib_path}')
                self.is_calibrated = False
                return
            with open(calib_path, 'r', encoding='utf-8') as f:
                calib_data = yaml.safe_load(f)
            self.lidar_offset_angle_deg = float(calib_data.get('lidar_offset_angle_deg', 0.0))
            self.lidar_offset_angle_rad = math.radians(self.lidar_offset_angle_deg)
            self.is_calibrated = True
            self.get_logger().info(f'Калибровка: смещение {self.lidar_offset_angle_deg:.2f}°')
        except (OSError, TypeError, ValueError, yaml.YAMLError) as e:
            self.get_logger().error(f'Ошибка чтения калибровки: {e}')
            self.is_calibrated = False

    def get_corrected_angle_deg(self, raw_angle_deg: float) -> float:
        if not self.is_calibrated:
            return raw_angle_deg
        corrected = raw_angle_deg - self.lidar_offset_angle_deg
        while corrected > 180:
            corrected -= 360
        while corrected < -180:
            corrected += 360
        return corrected

    @staticmethod
    def get_raw_angle_deg_from_index(scan: LaserScan, index: int) -> float:
        angle_rad = scan.angle_min + index * scan.angle_increment
        angle_deg = math.degrees(angle_rad)
        if angle_deg > 180:
            angle_deg -= 360
        return angle_deg

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.scan_received = True

    def odom_callback(self, msg: Odometry):
        self.odom_received = True
        q = msg.pose.pose.orientation
        self.current_yaw = _yaw_from_orientation(q.x, q.y, q.z, q.w)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._last_odom_t = time.monotonic()
        if self.start_position is None:
            self.start_position = (x, y)
            self.last_position = (x, y)
            self._stuck_ref_xy = (x, y)
            self._stuck_initialized = True
        if self.last_position is not None:
            dx = x - self.last_position[0]
            dy = y - self.last_position[1]
            self.total_distance += math.hypot(dx, dy)
        self.last_position = (x, y)

    def front_obstacle_info(self):
        if not self.scan_received or self.latest_scan is None:
            return False, float('inf'), 0.0, float('inf')

        scan = self.latest_scan
        half_fov = self.field_of_view / 2.0
        min_distance = float('inf')
        min_angle = 0.0
        mean_count = 0
        mean_sum = 0.0

        for i, distance in enumerate(scan.ranges):
            if distance < scan.range_min or distance > scan.range_max:
                continue
            raw_angle = self.get_raw_angle_deg_from_index(scan, i)
            corrected_angle = self.get_corrected_angle_deg(raw_angle)
            if abs(corrected_angle) <= half_fov:
                mean_sum += distance
                mean_count += 1
                if distance < min_distance:
                    min_distance = distance
                    min_angle = corrected_angle

        front_mean = mean_sum / mean_count if mean_count else float('inf')
        critical = min_distance < self.emergency_distance
        blocked = min_distance < self.obstacle_distance
        return blocked, min_distance, min_angle, front_mean

    def min_range_in_cone(self, scan: LaserScan, center_deg: float, half_deg: float) -> float:
        dmin = float('inf')
        for i, distance in enumerate(scan.ranges):
            if distance < scan.range_min or distance > scan.range_max:
                continue
            raw_angle = self.get_raw_angle_deg_from_index(scan, i)
            corrected = self.get_corrected_angle_deg(raw_angle)
            if abs(corrected - center_deg) <= half_deg and distance < dmin:
                dmin = distance
        return dmin

    def best_gap_steering_deg(self) -> tuple[float, float]:
        if not self.scan_received or self.latest_scan is None:
            return 0.0, 0.0

        scan = self.latest_scan
        best_score = -1.0
        best_deg = 0.0
        deg = -85.0
        while deg <= 85.0:
            dist = self.min_range_in_cone(scan, deg, self.gap_cone)
            if dist >= scan.range_max * 0.99:
                dist = scan.range_max * 0.99
            rad = math.radians(deg)
            score = dist * max(0.15, math.cos(rad)) ** 2
            if score > best_score:
                best_score = score
                best_deg = deg
            deg += self.gap_step
        return best_deg, best_score

    def is_path_clear_for_recover(self) -> bool:
        if not self.scan_received or self.latest_scan is None:
            return False
        scan = self.latest_scan
        d = self.min_range_in_cone(scan, 0.0, self.field_of_view / 2.2)
        return d > self.recover_min_front

    def check_stuck(self) -> bool:
        if not self._stuck_initialized or self.last_position is None:
            return False
        now = time.monotonic()
        if now - self._stuck_sample_t < self.stuck_timeout:
            return False
        x, y = self.last_position[0], self.last_position[1]
        moved = math.hypot(x - self._stuck_ref_xy[0], y - self._stuck_ref_xy[1])
        self._stuck_sample_t = now
        self._stuck_ref_xy = (x, y)
        cmd_active = self.state in (
            RobotState.MOVING_FORWARD,
            RobotState.FOLLOW_GAP,
            RobotState.RECOVER_HEADING,
        )
        return cmd_active and moved < self.stuck_eps

    def control_loop(self):
        if not self.is_calibrated:
            self.stop_robot()
            return

        blocked, front_min, _, _ = self.front_obstacle_info()

        if front_min < self.emergency_distance and self.state != RobotState.BACK_UP:
            self.stop_robot()
            self.state = RobotState.BACK_UP
            self._backup_until = time.monotonic() + self.backup_duration
            self.get_logger().warn(f'Критически близко ({front_min:.2f} м) — откат')
            return

        if self.check_stuck() and self.state not in (RobotState.BACK_UP, RobotState.STOPPED):
            self.get_logger().warn('Нет прогресса — откат')
            self.state = RobotState.BACK_UP
            self._backup_until = time.monotonic() + self.backup_duration
            if self.last_position is not None:
                self._stuck_ref_xy = (self.last_position[0], self.last_position[1])
            self._stuck_sample_t = time.monotonic()
            return

        if self.state == RobotState.MOVING_FORWARD:
            self._do_forward(blocked)
        elif self.state == RobotState.FOLLOW_GAP:
            self._do_follow_gap(blocked)
        elif self.state == RobotState.RECOVER_HEADING:
            self._do_recover()
        elif self.state == RobotState.BACK_UP:
            self._do_backup()
        elif self.state == RobotState.STOPPED:
            self.stop_robot()

        self.state_pub.publish(String(data=self.state.name))
        self.distance_pub.publish(Float32(data=self.total_distance))

    def _do_forward(self, blocked: bool):
        if blocked:
            self._target_course_yaw = self.current_yaw
            self._clear_counter = 0
            self.state = RobotState.FOLLOW_GAP
            self.get_logger().info('Препятствие — режим подруливания к щели')
            return
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def _do_follow_gap(self, blocked: bool):
        scan = self.latest_scan
        best_deg, score = self.best_gap_steering_deg()
        cmd = Twist()

        if scan is not None and blocked and score < self.emergency_distance * 0.35:
            cmd.linear.x = 0.0
            cmd.angular.z = math.copysign(self.rotation_speed, best_deg if best_deg != 0 else 1.0)
            self.cmd_pub.publish(cmd)
            return

        rad = math.radians(best_deg)
        cmd.angular.z = max(-self.rotation_speed, min(self.rotation_speed, 2.2 * rad))
        speed_scale = min(1.0, max(0.25, score / max(self.safe_distance, 0.1)))
        cmd.linear.x = self.linear_speed * 0.78 * speed_scale
        self.cmd_pub.publish(cmd)

        if not blocked:
            self._clear_counter += 1
        else:
            self._clear_counter = 0

        if self._clear_counter >= self.clear_frames and self.is_path_clear_for_recover():
            if self._target_course_yaw is not None:
                self.state = RobotState.RECOVER_HEADING
                self.get_logger().info('Фронт свободен — выравнивание по курсу')
            else:
                self.state = RobotState.MOVING_FORWARD

    def _do_recover(self):
        if self._target_course_yaw is None:
            self.state = RobotState.MOVING_FORWARD
            return
        err = _angle_diff(self._target_course_yaw, self.current_yaw)
        if abs(err) < self.recover_tol:
            self.state = RobotState.MOVING_FORWARD
            self._target_course_yaw = None
            self.get_logger().info('Курс восстановлен')
            return

        obstacle_ahead, _, _, _ = self.front_obstacle_info()
        if obstacle_ahead:
            self.state = RobotState.FOLLOW_GAP
            self._clear_counter = 0
            self.get_logger().warn('Снова препятствие при выравнивании')
            return

        cmd = Twist()
        cmd.linear.x = self.linear_speed * 0.55
        cmd.angular.z = max(-self.rotation_speed, min(self.rotation_speed, self.recover_k * err))
        self.cmd_pub.publish(cmd)

    def _do_backup(self):
        if time.monotonic() >= self._backup_until:
            self.state = RobotState.FOLLOW_GAP
            self._clear_counter = 0
            self.get_logger().info('Откат завершён — ищу щель')
            return
        cmd = Twist()
        cmd.linear.x = self.backup_speed
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def check_progress(self):
        if not self.is_calibrated:
            self.get_logger().warn('Жду калибровку лидара...')
            return
        elapsed = time.time() - self.movement_start_time
        hours = int(elapsed // 3600)
        minutes = int((elapsed % 3600) // 60)
        seconds = int(elapsed % 60)
        self.get_logger().info(
            f'Время: {hours:02d}:{minutes:02d}:{seconds:02d} | '
            f'Путь: {self.total_distance:.2f} м | {self.state.name}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Остановка (Ctrl+C)')
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
