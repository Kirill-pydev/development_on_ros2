#!/usr/bin/env python3
"""ROS 2 нода-шаблон для подготовки датасета и дообучения YOLO на новые классы.

Эта нода не выполняет разметку: кадры сохраняются на диск. Bounding boxes нужно
добавить в LabelImg / CVAT / Roboflow и экспортировать в формат YOLO.

Краткий процесс описан в README.md (раздел «Дообучение YOLO»).

Параметры
---------
image_topic : str
    Топик sensor_msgs/Image (по умолчанию /image_raw).
save_directory : str
    Корень датасета (каталог создаётся при старте).
images_subdir : str
    Подкаталог для кадров относительно save_directory (по умолчанию images/train).
file_prefix : str
    Префикс имени файла.
jpeg_quality : int
    Качество JPEG, 1–100.
auto_capture_period_sec : float
    Если больше нуля, периодически сохранять кадр с этим интервалом (секунды).

Сервисы
-------
~/capture_image (std_srvs/Trigger)
    Сохранить текущий кадр; в message — путь к файлу или текст ошибки.
~/dataset_paths (std_srvs/Trigger)
    Вернуть в message путь к каталогу изображений.

Офлайн обучение выполняется через Ultralytics CLI или Python API — примеры в README.md.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None  # type: ignore[misc, assignment]

try:
    import cv2
except ImportError:
    cv2 = None  # type: ignore[misc, assignment]


class YoloFinetuneNode(Node):
    """Сохранение кадров с камеры ROS 2 для последующего дообучения YOLO."""

    def __init__(self) -> None:
        super().__init__('yolo_finetune_node')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('save_directory', 'yolo_dataset')
        self.declare_parameter('images_subdir', 'images/train')
        self.declare_parameter('file_prefix', 'frame')
        self.declare_parameter('jpeg_quality', 92)
        self.declare_parameter('auto_capture_period_sec', 0.0)

        topic = self.get_parameter('image_topic').get_parameter_value().string_value
        save_dir = self.get_parameter('save_directory').get_parameter_value().string_value
        self._images_subdir = self.get_parameter('images_subdir').get_parameter_value().string_value
        self._prefix = self.get_parameter('file_prefix').get_parameter_value().string_value
        self._jpeg_q = int(self.get_parameter('jpeg_quality').get_parameter_value().integer_value)
        period = float(self.get_parameter('auto_capture_period_sec').get_parameter_value().double_value)

        if cv2 is None:
            self.get_logger().error(
                'Не найден OpenCV. Установите: pip install opencv-python-headless'
            )
            raise RuntimeError('opencv_required')

        if CvBridge is None:
            self.get_logger().error(
                'Не найден cv_bridge. Установите пакет ros-<distro>-cv-bridge.'
            )
            raise RuntimeError('cv_bridge_required')

        self._bridge = CvBridge()
        self._latest: Image | None = None
        self._lock = threading.Lock()

        root = Path(save_dir).expanduser().resolve()
        self._img_dir = root / self._images_subdir
        self._img_dir.mkdir(parents=True, exist_ok=True)

        self.create_subscription(Image, topic, self._image_cb, 10)
        self.create_service(Trigger, '~/capture_image', self._on_capture)
        self.create_service(Trigger, '~/dataset_paths', self._on_paths_info)

        self._counter = 0

        if period > 0.0:
            self.create_timer(period, self._auto_capture)

        self.get_logger().info('=' * 60)
        self.get_logger().info('YOLO: нода сбора датасета')
        self.get_logger().info(f'Подписка: {topic}')
        self.get_logger().info(f'Каталог изображений: {self._img_dir}')
        self.get_logger().info('Сервисы: ~/capture_image, ~/dataset_paths (Trigger)')
        self.get_logger().info('Инструкция: README.md — раздел «Дообучение YOLO»')
        self.get_logger().info('=' * 60)

    def _image_cb(self, msg: Image) -> None:
        with self._lock:
            self._latest = msg

    def _bgr_from_msg(self, msg: Image):
        enc = msg.encoding.lower()
        if enc in ('bgr8', 'rgb8', 'bgra8', 'rgba8'):
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _save_current(self) -> tuple[bool, str]:
        with self._lock:
            msg = self._latest
            if msg is None:
                return False, 'Кадры ещё не приходили с выбранного топика'

            try:
                bgr = self._bgr_from_msg(msg)
            except Exception as e:  # noqa: BLE001 — понятное сообщение в сервисе
                return False, f'cv_bridge: {e}'

            self._counter += 1
            idx = self._counter

        name = f'{self._prefix}_{idx:06d}_{int(time.time() * 1000)}.jpg'
        path = self._img_dir / name
        ok = cv2.imwrite(str(path), bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_q])
        if not ok:
            return False, f'Не удалось записать: {path}'
        self.get_logger().info(f'Сохранено: {path}')
        return True, str(path)

    def _on_capture(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ok, text = self._save_current()
        response.success = ok
        response.message = text
        return response

    def _on_paths_info(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = True
        response.message = f'images_directory={self._img_dir}'
        return response

    def _auto_capture(self) -> None:
        ok, text = self._save_current()
        if not ok:
            self.get_logger().warn(f'auto_capture: {text}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = YoloFinetuneNode()
    except RuntimeError:
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
