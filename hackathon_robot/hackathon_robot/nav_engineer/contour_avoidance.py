import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import time
import os
import yaml
from enum import Enum

class RobotState(Enum):
    MOVING_FORWARD = 1
    AVOIDING = 2
    RETURNING = 3
    FINAL_STRAIGHT = 4
    FINISHED = 5
    STOPPED = 6

class AutonomousDrive(Node):
    def __init__(self):
        super().__init__('autonomous_drive')
        
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('rotation_speed', 0.4)
        self.declare_parameter('obstacle_distance', 0.3)
        self.declare_parameter('safe_side_distance', 0.2)
        self.declare_parameter('final_straight_distance', 0.5)
        self.declare_parameter('angle_tolerance', 10.0)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.safe_side_distance = self.get_parameter('safe_side_distance').value
        self.final_straight = self.get_parameter('final_straight_distance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        
        self.state = RobotState.MOVING_FORWARD
        self.avoid_direction = 0
        self.turn_accumulated = 0.0
        self.last_yaw = None
        self.final_start_time = 0.0
        self.latest_scan = None
        self.current_yaw = None
        
        self.lidar_offset = 0.0
        self.is_calibrated = False
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.create_timer(0.05, self.control_loop)
        
        self.load_calibration()
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('ROBOT - Contour avoidance')
        self.get_logger().info(f'Trigger: {self.obstacle_distance}m | Side: {self.safe_side_distance}m')
        self.get_logger().info(f'Angle tolerance: ±{self.angle_tolerance}°')
        self.get_logger().info('=' * 50)
    
    def load_calibration(self):
        path = os.path.join(os.getcwd(), 'params', 'lidar_calibration.yaml')
        try:
            if os.path.exists(path):
                with open(path, 'r') as f:
                    data = yaml.safe_load(f)
                self.lidar_offset = float(data.get('lidar_offset_angle_deg', 0.0))
                self.is_calibrated = True
                self.get_logger().info(f'Calibration: {self.lidar_offset}°')
        except:
            pass
    
    def correct_angle(self, raw_deg):
        if not self.is_calibrated:
            return raw_deg
        c = raw_deg - self.lidar_offset
        while c > 180: c -= 360
        while c < -180: c += 360
        return c
    
    def get_angle(self, scan, i):
        rad = scan.angle_min + i * scan.angle_increment
        deg = math.degrees(rad)
        if deg > 180: deg -= 360
        return self.correct_angle(deg)
    
    def scan_cb(self, msg):
        self.latest_scan = msg
    
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        
        if self.last_yaw is not None:
            dyaw = yaw - self.last_yaw
            while dyaw > math.pi: dyaw -= 2*math.pi
            while dyaw < -math.pi: dyaw += 2*math.pi
            
            if self.state in [RobotState.AVOIDING, RobotState.RETURNING]:
                self.turn_accumulated += dyaw
        
        self.current_yaw = yaw
        self.last_yaw = yaw
    
    def get_front_distance(self):
        if not self.latest_scan:
            return float('inf')
        scan = self.latest_scan
        min_d = float('inf')
        for i, d in enumerate(scan.ranges):
            if d < scan.range_min or d > scan.range_max:
                continue
            angle = self.get_angle(scan, i)
            if -15 <= angle <= 15:
                if d < min_d:
                    min_d = d
        return min_d
    
    def get_perpendicular_distance(self, direction):
        if not self.latest_scan:
            return float('inf')
        scan = self.latest_scan
        min_d = float('inf')
        
        if direction == 1:
            target_min, target_max = -100, -80
        else:
            target_min, target_max = 80, 100
        
        for i, d in enumerate(scan.ranges):
            if d < scan.range_min or d > scan.range_max:
                continue
            angle = self.get_angle(scan, i)
            if target_min <= angle <= target_max:
                if d < min_d:
                    min_d = d
        return min_d
    
    def control_loop(self):
        if not self.is_calibrated:
            self.stop()
            return
        
        front = self.get_front_distance()
        
        if self.state == RobotState.MOVING_FORWARD:
            self.do_moving_forward(front)
        elif self.state == RobotState.AVOIDING:
            self.do_avoiding(front)
        elif self.state == RobotState.RETURNING:
            self.do_returning(front)
        elif self.state == RobotState.FINAL_STRAIGHT:
            self.do_final_straight()
        elif self.state in [RobotState.FINISHED, RobotState.STOPPED]:
            self.stop()
        
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)
    
    def do_moving_forward(self, front):
        if front < self.obstacle_distance:
            left = self.get_perpendicular_distance(-1)
            right = self.get_perpendicular_distance(1)
            
            if left > right:
                self.avoid_direction = -1
                self.get_logger().info(f'⚠️ Obstacle! LEFT (L={left:.2f} R={right:.2f})')
            else:
                self.avoid_direction = 1
                self.get_logger().info(f'⚠️ Obstacle! RIGHT (L={left:.2f} R={right:.2f})')
            
            self.turn_accumulated = 0.0
            self.state = RobotState.AVOIDING
        else:
            cmd = Twist()
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
    
    def do_avoiding(self, front):
        side_dist = self.get_perpendicular_distance(-self.avoid_direction)
        
        # 🔧 Проверка: сумма углов ≈ 0 (с допуском angle_tolerance)
        angle_diff = abs(math.degrees(self.turn_accumulated))
        
        # Препятствие считается пройденным, если:
        # 1. Спереди свободно
        # 2. Сбоку свободно
        # 3. Робот уже накопил значительный угол (начал поворот)
        # 4. НО главное: возврат будет по симметрии углов в RETURNING
        
        front_clear = front > self.obstacle_distance * 1.3
        side_clear = side_dist > self.obstacle_distance * 1.3
        
        if front_clear and side_clear and angle_diff > 5.0:
            self.get_logger().info(f'✅ Obstacle passed! Accumulated angle: {angle_diff:.1f}°')
            self.state = RobotState.RETURNING
            return
        
        # PID-регулятор
        error = side_dist - self.safe_side_distance
        angular_z = -self.avoid_direction * error * 1.5
        angular_z = max(-self.rotation_speed, min(self.rotation_speed, angular_z))
        
        if front < self.safe_side_distance:
            linear_x = 0.0
        elif front < self.obstacle_distance:
            linear_x = self.linear_speed * 0.3
        else:
            linear_x = self.linear_speed * 0.7
        
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)
        
        if not hasattr(self, '_log_t') or time.time() - self._log_t > 0.5:
            self.get_logger().info(f'🔄 Side={side_dist:.2f}m Front={front:.2f}m Acc={angle_diff:.1f}°')
            self._log_t = time.time()
    
    def do_returning(self, front):
        if front < self.obstacle_distance:
            self.get_logger().info('⚠️ New obstacle!')
            self.state = RobotState.AVOIDING
            self.turn_accumulated = 0.0
            return
        
        # 🔧 Возвращаемся, пока угол не станет близким к 0 (симметрия)
        remaining_deg = math.degrees(self.turn_accumulated)
        
        if abs(remaining_deg) > self.angle_tolerance:
            direction = -1 if self.turn_accumulated > 0 else 1
            angular_z = direction * self.rotation_speed * 0.5
            
            cmd = Twist()
            cmd.linear.x = self.linear_speed * 0.8
            cmd.angular.z = angular_z
            self.cmd_pub.publish(cmd)
            
            reduction = abs(angular_z) * 0.05
            if self.turn_accumulated > 0:
                self.turn_accumulated -= reduction
            else:
                self.turn_accumulated += reduction
            
            self.get_logger().info(f'↩️ Returning: {abs(remaining_deg):.0f}° left')
        else:
            self.get_logger().info(f'✅ Symmetric! Angle diff: {abs(remaining_deg):.1f}° (tol: ±{self.angle_tolerance}°)')
            self.state = RobotState.FINAL_STRAIGHT
            self.final_start_time = time.time()
    
    def do_final_straight(self):
        elapsed = time.time() - self.final_start_time
        time_needed = self.final_straight / self.linear_speed
        
        if elapsed < time_needed:
            cmd = Twist()
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
        else:
            self.get_logger().info('🎉 FINISHED!')
            self.state = RobotState.FINISHED
            self.stop()
    
    def stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
