import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time
import os
import yaml
import threading
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class LidarCalibratorAndObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_calibrator_detector')
        
        # Параметры
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('field_of_view', 60.0)
        self.declare_parameter('calibration_time', 3.0)
        self.declare_parameter('calibration_min_distance', 0.1)
        self.declare_parameter('calibration_max_distance', 0.3)
        self.declare_parameter('calibration_tolerance', 5.0)
        self.declare_parameter('lidar_frame', 'laser_frame')
        self.declare_parameter('base_frame', 'base_link')
        
        scan_topic = self.get_parameter('scan_topic').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.field_of_view = self.get_parameter('field_of_view').value
        self.calibration_time = self.get_parameter('calibration_time').value
        self.calib_min_dist = self.get_parameter('calibration_min_distance').value
        self.calib_max_dist = self.get_parameter('calibration_max_distance').value
        self.calib_tolerance = self.get_parameter('calibration_tolerance').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        self.is_calibrated = False
        self.lidar_offset_angle = 0.0
        self.calibration_data = []
        self.latest_scan = None
        self.scan_received = False
        
        # Издатели
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.distance_pub = self.create_publisher(Float32, '/front_distance', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Подписка на лидар
        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )
        
        # Таймер для проверки препятствий
        self.timer = self.create_timer(0.1, self.check_obstacles)
        
        # Создаём папку params в текущей директории
        self.params_dir = os.path.join(os.getcwd(), 'params')
        os.makedirs(self.params_dir, exist_ok=True)
        self.calibration_file = os.path.join(self.params_dir, 'lidar_calibration.yaml')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('LIDAR CALIBRATOR & DETECTOR')
        self.get_logger().info('=' * 60)
        
        # Проверяем наличие сохраненной калибровки
        if self.load_calibration():
            self.publish_calibration_tf()
        else:
            # Запускаем поток для ожидания нажатия Enter
            self.get_logger().info('')
            self.get_logger().info('Сохраненная калибровка не найдена.')
            self.get_logger().info('')
            
            # Поток для ожидания ввода пользователя
            self.input_thread = threading.Thread(target=self.wait_for_enter, daemon=True)
            self.input_thread.start()
    
    def wait_for_enter(self):
        """Ожидает нажатия Enter для запуска калибровки"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('📋 ИНСТРУКЦИЯ ПО КАЛИБРОВКЕ:')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'1. Поставьте препятствие ПРЯМО ПЕРЕД РОБОТОМ')
        self.get_logger().info(f'2. Расстояние: {self.calib_min_dist}-{self.calib_max_dist} метра')
        self.get_logger().info(f'3. Препятствие должно быть по центру')
        self.get_logger().info('')
        self.get_logger().info('>>> НАЖМИТЕ ENTER ДЛЯ НАЧАЛА КАЛИБРОВКИ <<<')
        self.get_logger().info('')
        
        try:
            input()
            self.get_logger().info('')
            self.get_logger().info('🚀 ЗАПУСК КАЛИБРОВКИ...')
            self.start_calibration()
        except EOFError:
            self.get_logger().warn('Не удалось прочитать ввод. Запустите калибровку через параметр auto_calibrate')
    
    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.scan_received = True
        
        if not self.is_calibrated and hasattr(self, 'calibration_timer'):
            self.collect_calibration_data(msg)
    
    def start_calibration(self):
        """Запускает процесс калибровки"""
        self.get_logger().info('')
        self.get_logger().info('🔧 НАЧАЛО КАЛИБРОВКИ ЛИДАРА 🔧')
        self.get_logger().info(f'Длительность: {self.calibration_time} секунд')
        self.get_logger().info('Не двигайте робота и препятствие...')
        self.get_logger().info('')
        
        self.calibration_data = []
        self.calibration_start_time = time.time()
        self.calibration_timer = self.create_timer(
            self.calibration_time, 
            self.finish_calibration
        )
    
    def collect_calibration_data(self, scan: LaserScan):
        """Собирает данные во время калибровки"""
        best_distance = float('inf')
        best_index = -1
        
        for i, distance in enumerate(scan.ranges):
            if self.calib_min_dist <= distance <= self.calib_max_dist:
                if distance < best_distance:
                    best_distance = distance
                    best_index = i
        
        if best_index != -1:
            angle_rad = scan.angle_min + best_index * scan.angle_increment
            angle_deg = math.degrees(angle_rad)
            if angle_deg > 180:
                angle_deg = angle_deg - 360
            
            self.calibration_data.append({
                'angle': angle_deg,
                'distance': best_distance,
                'index': best_index,
                'timestamp': time.time()
            })
            
            self.get_logger().info(
                f'📏 Измерение: расстояние {best_distance:.3f}м, угол {angle_deg:.1f}° ✅'
            )
        else:
            # Информация для отладки
            min_dist = float('inf')
            for i, distance in enumerate(scan.ranges):
                if scan.range_min < distance < scan.range_max:
                    if distance < min_dist:
                        min_dist = distance
            
            if min_dist != float('inf'):
                if min_dist < self.calib_min_dist:
                    self.get_logger().warn(
                        f'⚠️ Препятствие слишком БЛИЗКО: {min_dist:.3f}м '
                        f'(нужно {self.calib_min_dist}-{self.calib_max_dist}м)'
                    )
                elif min_dist > self.calib_max_dist:
                    self.get_logger().warn(
                        f'⚠️ Препятствие слишком ДАЛЕКО: {min_dist:.3f}м '
                        f'(нужно {self.calib_min_dist}-{self.calib_max_dist}м)'
                    )
            else:
                self.get_logger().warn('⚠️ Нет препятствий в зоне видимости лидара!')
    
    def finish_calibration(self):
        """Завершает калибровку и вычисляет смещение"""
        self.calibration_timer.cancel()
        delattr(self, 'calibration_timer')
        
        if len(self.calibration_data) == 0:
            self.get_logger().error('')
            self.get_logger().error('❌ ОШИБКА КАЛИБРОВКИ!')
            self.get_logger().error('Нет данных для анализа.')
            self.get_logger().error(f'Убедитесь, что препятствие находится на расстоянии {self.calib_min_dist}-{self.calib_max_dist}м')
            self.get_logger().error('')
            self.get_logger().info('>>> НАЖМИТЕ ENTER ДЛЯ ПОВТОРНОЙ ПОПЫТКИ <<<')
            
            # Запускаем новый поток для ожидания Enter
            self.input_thread = threading.Thread(target=self.wait_for_enter_retry, daemon=True)
            self.input_thread.start()
            return
        
        # Анализируем данные
        angles = [d['angle'] for d in self.calibration_data]
        distances = [d['distance'] for d in self.calibration_data]
        
        avg_angle = sum(angles) / len(angles)
        avg_distance = sum(distances) / len(distances)
        min_angle = min(angles)
        max_angle = max(angles)
        angles_sorted = sorted(angles)
        median_angle = angles_sorted[len(angles_sorted) // 2]
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('📊 РЕЗУЛЬТАТЫ КАЛИБРОВКИ')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Собрано измерений: {len(self.calibration_data)}')
        self.get_logger().info(f'Средняя дистанция: {avg_distance:.3f} м')
        self.get_logger().info(f'Диапазон углов: {min_angle:.1f}° ... {max_angle:.1f}°')
        self.get_logger().info(f'Средний угол: {avg_angle:.1f}°')
        self.get_logger().info(f'Медианный угол: {median_angle:.1f}°')
        
        # Проверка на центровку
        if abs(median_angle) > self.calib_tolerance:
            self.get_logger().warn('')
            self.get_logger().warn(
                f'⚠️ Препятствие не по центру! Отклонение: {median_angle:.1f}° '
                f'(допуск ±{self.calib_tolerance}°)'
            )
            self.get_logger().warn('Рекомендуется переставить препятствие и повторить калибровку')
        
        # Вычисляем смещение
        self.lidar_offset_angle = -median_angle
        self.lidar_offset_rad = math.radians(self.lidar_offset_angle)
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('✅ КАЛИБРОВКА ЗАВЕРШЕНА')
        self.get_logger().info(f'Смещение лидара: {self.lidar_offset_angle:.1f}°')
        
        # Оценка качества
        if abs(self.lidar_offset_angle) > 15:
            self.get_logger().error(
                f'❌ КРИТИЧЕСКОЕ СМЕЩЕНИЕ ({self.lidar_offset_angle:.1f}°)!\n'
                f'   Проверьте физическую установку лидара!'
            )
        elif abs(self.lidar_offset_angle) > 5:
            self.get_logger().warn(
                f'⚠️ Значительное смещение ({self.lidar_offset_angle:.1f}°)\n'
                f'   Программная коррекция применена'
            )
        else:
            self.get_logger().info('✅ Лидар настроен отлично!')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        
        self.is_calibrated = True
        self.save_calibration()
        self.publish_calibration_tf()
    
    def wait_for_enter_retry(self):
        """Ожидает нажатия Enter для повторной калибровки"""
        try:
            input()
            self.get_logger().info('🔄 ПОВТОРНАЯ КАЛИБРОВКА...')
            self.start_calibration()
        except EOFError:
            self.get_logger().warn('Не удалось прочитать ввод.')
    
    def save_calibration(self):
        """Сохраняет калибровку в YAML-файл в папке params/"""
        calib_data = {
            'calibration_date': time.ctime(),
            'lidar_frame': self.lidar_frame,
            'base_frame': self.base_frame,
            'lidar_offset_angle_deg': self.lidar_offset_angle,
            'calibration_distance_range': f"{self.calib_min_dist}-{self.calib_max_dist}m",
            'calibration_samples': len(self.calibration_data)
        }
        
        try:
            with open(self.calibration_file, 'w') as f:
                yaml.dump(calib_data, f, default_flow_style=False)
            self.get_logger().info(f'💾 Калибровка сохранена: {self.calibration_file}')
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка сохранения: {e}')
    
    def load_calibration(self):
        """Загружает калибровку из YAML-файла в папке params/"""
        try:
            if os.path.exists(self.calibration_file):
                with open(self.calibration_file, 'r') as f:
                    calib_data = yaml.safe_load(f)
                
                self.lidar_offset_angle = calib_data.get('lidar_offset_angle_deg', 0.0)
                self.is_calibrated = True
                self.get_logger().info(f'✅ Калибровка загружена: {self.calibration_file}')
                self.get_logger().info(f'   Смещение: {self.lidar_offset_angle:.1f}°')
                return True
        except Exception as e:
            self.get_logger().warn(f'⚠️ Ошибка загрузки: {e}')
        return False
    
    def publish_calibration_tf(self):
        """Публикует статическую трансформацию с учетом калибровки"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.lidar_frame
        
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        half_angle = math.radians(self.lidar_offset_angle) / 2.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(half_angle)
        t.transform.rotation.w = math.cos(half_angle)
        
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'📡 Опубликована калибровка TF: {self.base_frame} -> {self.lidar_frame}')
    
    def get_corrected_angle(self, original_angle_rad):
        """Возвращает скорректированный угол"""
        angle_deg = math.degrees(original_angle_rad)
        if angle_deg > 180:
            angle_deg = angle_deg - 360
        
        corrected_deg = angle_deg - self.lidar_offset_angle
        
        if corrected_deg > 180:
            corrected_deg = corrected_deg - 360
        elif corrected_deg < -180:
            corrected_deg = corrected_deg + 360
        
        return math.radians(corrected_deg)
    
    def check_obstacles(self):
        """Проверяет препятствия впереди с учетом калибровки"""
        if not self.is_calibrated or not self.scan_received:
            return
        
        scan = self.latest_scan
        half_fov = self.field_of_view / 2.0
        safety_distance = self.safety_distance
        
        min_distance = float('inf')
        min_angle_deg = 0.0
        
        for i, distance in enumerate(scan.ranges):
            if distance < scan.range_min or distance > scan.range_max:
                continue
            
            original_angle_rad = scan.angle_min + i * scan.angle_increment
            corrected_angle_rad = self.get_corrected_angle(original_angle_rad)
            corrected_angle_deg = math.degrees(corrected_angle_rad)
            
            if abs(corrected_angle_deg) <= half_fov:
                if distance < min_distance:
                    min_distance = distance
                    min_angle_deg = corrected_angle_deg
        
        front_distance_msg = Float32()
        front_distance_msg.data = min_distance if min_distance != float('inf') else scan.range_max
        self.distance_pub.publish(front_distance_msg)
        
        obstacle_detected = min_distance < safety_distance
        obstacle_msg = Bool()
        obstacle_msg.data = obstacle_detected
        self.obstacle_pub.publish(obstacle_msg)
        
        if hasattr(self, 'last_log_time'):
            if time.time() - self.last_log_time > 0.5:
                self.log_obstacle_info(min_distance, min_angle_deg, obstacle_detected)
                self.last_log_time = time.time()
        else:
            self.last_log_time = time.time()
            self.log_obstacle_info(min_distance, min_angle_deg, obstacle_detected)
    
    def log_obstacle_info(self, min_distance, min_angle_deg, obstacle_detected):
        if obstacle_detected:
            self.get_logger().error(f'⚠️ ПРЕПЯТСТВИЕ! {min_distance:.2f}м, {min_angle_deg:.1f}°')
        else:
            if min_distance != float('inf'):
                self.get_logger().info(f'✅ Свободно. Ближайшее: {min_distance:.2f}м')
            else:
                self.get_logger().info(f'✅ Свободно. Нет препятствий')

def main(args=None):
    rclpy.init(args=args)
    node = LidarCalibratorAndObstacleDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n')
        node.get_logger().info('Остановка пользователем')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
