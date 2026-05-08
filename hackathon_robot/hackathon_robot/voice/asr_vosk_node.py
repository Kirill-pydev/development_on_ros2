#!/usr/bin/env python3
"""Локальное ASR (Vosk) без облака — аналог потока из учебника, публикация текста в ROS.

Требования:
  pip install vosk sounddevice numpy
  Скачать модель (русский small): https://alphacephei.com/vosk/models
  Параметр model_path — каталог с распакованной моделью.
"""

from __future__ import annotations

import json
import queue
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import numpy as np
    import sounddevice as sd
except ImportError:
    np = None  # type: ignore[misc, assignment]
    sd = None  # type: ignore[misc, assignment]

try:
    from vosk import Model, KaldiRecognizer
except ImportError:
    Model = None  # type: ignore[misc, assignment]
    KaldiRecognizer = None  # type: ignore[misc, assignment]


class AsrVoskNode(Node):
    def __init__(self) -> None:
        super().__init__('asr_vosk_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('publish_topic', 'voice/recognized_text')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_frames', 1024)
        self.declare_parameter('device', -1)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        pub_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self._rate = int(self.get_parameter('sample_rate').get_parameter_value().integer_value)
        chunk = int(self.get_parameter('chunk_frames').get_parameter_value().integer_value)
        device = int(self.get_parameter('device').get_parameter_value().integer_value)

        self._pub = self.create_publisher(String, pub_topic, 10)
        self._audio_q: queue.Queue[bytes] = queue.Queue(maxsize=64)
        self._stop = threading.Event()

        if Model is None or KaldiRecognizer is None:
            self.get_logger().error('Пакет vosk не установлен: pip install vosk')
            raise RuntimeError('vosk_missing')
        if np is None or sd is None:
            self.get_logger().error('Нужны sounddevice и numpy: pip install sounddevice numpy')
            raise RuntimeError('audio_deps_missing')
        if not model_path:
            self.get_logger().error('Задайте параметр model_path (каталог модели Vosk).')
            raise RuntimeError('model_path_empty')

        self._model = Model(model_path)
        self._rec = KaldiRecognizer(self._model, self._rate)
        self._chunk = chunk

        self._capture_thread = threading.Thread(target=self._capture_loop, args=(device,), daemon=True)
        self._recognize_thread = threading.Thread(target=self._recognize_loop, daemon=True)

        self._capture_thread.start()
        self._recognize_thread.start()

        self.get_logger().info(f'Vosk ASR: публикация в «{pub_topic}», {self._rate} Гц')

    def _capture_loop(self, device: int) -> None:
        try:

            def callback(indata, frames, t, status) -> None:  # noqa: ARG001
                if status:
                    self.get_logger().warning(str(status))
                pcm = (indata * 32767).astype(np.int16).tobytes()
                try:
                    self._audio_q.put_nowait(pcm)
                except queue.Full:
                    pass

            with sd.RawInputStream(
                samplerate=self._rate,
                blocksize=self._chunk,
                dtype='float32',
                channels=1,
                callback=callback,
                device=None if device < 0 else device,
            ):
                self._stop.wait()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Микрофон / sounddevice: {e}')

    def _recognize_loop(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            try:
                data = self._audio_q.get(timeout=0.2)
            except queue.Empty:
                continue
            if self._rec.AcceptWaveform(data):
                try:
                    res = json.loads(self._rec.Result())
                except json.JSONDecodeError:
                    continue
                text = (res.get('text') or '').strip()
                if text:
                    msg = String()
                    msg.data = text
                    self._pub.publish(msg)
                    self.get_logger().info(f'Распознано: {text}')
            else:
                partial = self._rec.PartialResult()
                try:
                    pres = json.loads(partial)
                    pt = (pres.get('partial') or '').strip()
                    if pt:
                        self.get_logger().debug(pt)
                except json.JSONDecodeError:
                    pass

    def destroy_node(self) -> None:
        self._stop.set()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = AsrVoskNode()
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
