#!/usr/bin/env python3
"""GOD: распознавание речи (Vosk) + примитивы движения.

Каждая запись лога пишется в файл и дублируется в ROS-лог (терминал), если
mirror_logs_to_terminal=true.
"""

from __future__ import annotations

import json
import queue
import threading
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

import os
import time
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


class MotionMode(Enum):
    IDLE = auto()
    FORWARD = auto()
    BACKWARD = auto()
    TURN_LEFT = auto()
    TURN_RIGHT = auto()


def parse_primitive_command(text: str) -> str | None:
    t = text.lower().strip()
    if not t:
        return None
    if any(k in t for k in ('стоп', 'останов', 'хватит', 'остановись')):
        return 'stop'
    if any(k in t for k in ('назад', 'отойди назад')):
        return 'backward'
    if any(k in t for k in ('влево', 'налево', 'поверни влево', 'руль влево')):
        return 'turn_left'
    if any(k in t for k in ('вправо', 'направо', 'поверни вправо', 'руль вправо')):
        return 'turn_right'
    if any(
        k in t
        for k in (
            'вперёд',
            'вперед',
            'едь',
            'поезжай',
            'иди вперёд',
            'подъезжай',
            'двигайся',
        )
    ):
        return 'forward'
    return None


class GodVoiceAllNode(Node):
    def __init__(self) -> None:
        super().__init__('god_voice_all')

        self.declare_parameter('model_path', '')
        self.declare_parameter('vosk_only', False)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_frames', 1024)
        self.declare_parameter('audio_device', -1)
        self.declare_parameter('log_dir', '')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('tts_topic', 'voice/text_to_speak')
        self.declare_parameter('publish_tts', True)
        self.declare_parameter('forward_linear', 0.2)
        self.declare_parameter('backward_linear', -0.12)
        self.declare_parameter('turn_angular', 0.45)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('mirror_logs_to_terminal', True)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        vosk_only = self.get_parameter('vosk_only').get_parameter_value().bool_value
        self._rate = int(self.get_parameter('sample_rate').get_parameter_value().integer_value)
        chunk = int(self.get_parameter('chunk_frames').get_parameter_value().integer_value)
        audio_dev = int(self.get_parameter('audio_device').get_parameter_value().integer_value)
        log_dir = self.get_parameter('log_dir').get_parameter_value().string_value.strip()
        if not log_dir:
            log_dir = default_god_log_dir()
        log_file = make_log_path(log_dir, 'god_voice_all')
        mirror = self.get_parameter('mirror_logs_to_terminal').get_parameter_value().bool_value
        self._glog = GodFileLog(
            log_file,
            ros_logger=self.get_logger(),
            mirror_to_terminal=mirror,
        )
        self._log_path = log_file

        self._forward_v = float(self.get_parameter('forward_linear').value)
        self._back_v = float(self.get_parameter('backward_linear').value)
        self._turn_w = float(self.get_parameter('turn_angular').value)
        hz = float(self.get_parameter('control_rate_hz').value)
        period = 1.0 / max(hz, 1.0)

        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self._tts_topic = self.get_parameter('tts_topic').get_parameter_value().string_value
        self._publish_tts = self.get_parameter('publish_tts').get_parameter_value().bool_value

        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self._tts_pub = self.create_publisher(String, self._tts_topic, 10)
        self.create_subscription(String, '/voice_cmd', self._on_injected_text, 10)

        self._mode = MotionMode.IDLE
        self._mode_lock = threading.Lock()
        self._audio_q: queue.Queue[bytes] | None = None
        self._stop = threading.Event()
        self._chunk = chunk
        self._rec: object | None = None
        self._cmd_log_counter = 0

        if not vosk_only:
            if Model is None or KaldiRecognizer is None:
                self.get_logger().error(
                    'Нет vosk: pip install vosk — или vosk_only:=true для тестов через /voice_cmd'
                )
                raise RuntimeError('vosk_missing')
            if np is None or sd is None:
                self.get_logger().error('Нужны sounddevice и numpy для микрофона')
                raise RuntimeError('audio_deps_missing')
            if not model_path:
                self.get_logger().error('Задайте model_path или vosk_only:=true')
                raise RuntimeError('model_path_empty')
            model = Model(model_path)
            self._rec = KaldiRecognizer(model, self._rate)
            self._audio_q = queue.Queue(maxsize=64)
            th_cap = threading.Thread(target=self._capture_loop, args=(audio_dev,), daemon=True)
            th_rec = threading.Thread(target=self._recognize_loop, daemon=True)
            th_cap.start()
            th_rec.start()
            self._glog.line('event', 'vosk_started', model_path)
        else:
            self.get_logger().warn('Режим vosk_only: только /voice_cmd, без микрофона')
            self._glog.line('event', 'vosk_only_mode')

        self.create_timer(period, self._control_tick)
        self.get_logger().info(f'GOD voice-all → «{cmd_topic}» (файл + терминал при mirror_logs_to_terminal)')

    def destroy_node(self) -> bool:
        self._stop.set()
        self._glog.line('event', 'shutdown')
        self._glog.close()
        self._cmd_pub.publish(Twist())
        self.get_logger().info('GOD voice-all: остановка, cmd_vel=0')
        return super().destroy_node()

    def _speak(self, phrase: str) -> None:
        if not self._publish_tts:
            return
        self._tts_pub.publish(String(data=phrase))
        self._glog.line('tts', phrase)

    def _on_injected_text(self, msg: String) -> None:
        self._handle_text(msg.data, source='voice_cmd')

    def _capture_loop(self, device: int) -> None:
        assert np is not None and sd is not None
        try:

            def callback(indata, frames, t, status) -> None:  # noqa: ARG001
                if status:
                    self.get_logger().warning(str(status))
                pcm = (indata * 32767).astype(np.int16).tobytes()
                if self._audio_q is not None:
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
            self.get_logger().error(f'Микрофон: {e}')
            self._glog.line('error', 'microphone', str(e))

    def _recognize_loop(self) -> None:
        assert self._rec is not None and self._audio_q is not None
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
                    self._handle_text(text, source='vosk')

    def _handle_text(self, text: str, source: str) -> None:
        self._glog.line('asr', source, text)
        cmd = parse_primitive_command(text)
        if cmd is None:
            self._speak('Не понял команду')
            self._glog.line('parse', 'unknown', text)
            self.get_logger().warning(f'Команда не распознана: «{text}»')
            return

        with self._mode_lock:
            if cmd == 'stop':
                self._mode = MotionMode.IDLE
                self._speak('Останавливаюсь')
            elif cmd == 'forward':
                self._mode = MotionMode.FORWARD
                self._speak('Еду вперёд')
            elif cmd == 'backward':
                self._mode = MotionMode.BACKWARD
                self._speak('Сдаю назад')
            elif cmd == 'turn_left':
                self._mode = MotionMode.TURN_LEFT
                self._speak('Поворачиваю влево')
            elif cmd == 'turn_right':
                self._mode = MotionMode.TURN_RIGHT
                self._speak('Поворачиваю вправо')
            mode_name = self._mode.name
        self._glog.line('motion_set', cmd, mode_name)

    def _control_tick(self) -> None:
        cmd = Twist()
        with self._mode_lock:
            mode = self._mode
        if mode == MotionMode.FORWARD:
            cmd.linear.x = self._forward_v
        elif mode == MotionMode.BACKWARD:
            cmd.linear.x = self._back_v
        elif mode == MotionMode.TURN_LEFT:
            cmd.angular.z = self._turn_w
        elif mode == MotionMode.TURN_RIGHT:
            cmd.angular.z = -self._turn_w
        self._cmd_pub.publish(cmd)
        self._cmd_log_counter += 1
        if self._cmd_log_counter % 10 == 0:
            self._glog.line('cmd_vel', cmd.linear.x, cmd.angular.z, mode.name)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GodVoiceAllNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
