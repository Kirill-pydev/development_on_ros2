#!/usr/bin/env python3
"""Офлайн TTS через espeak-ng (без облачного API).

Подписывается на std_msgs/String `voice/text_to_speak`, синтезирует речь.
Публикует `voice/tts_state`: «playing» | «idle» для синхронизации с FSM.
"""

from __future__ import annotations

import subprocess
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TtsEspeakNode(Node):
    def __init__(self) -> None:
        super().__init__('tts_espeak_node')

        self.declare_parameter('subscribe_topic', 'voice/text_to_speak')
        self.declare_parameter('state_topic', 'voice/tts_state')
        self.declare_parameter('espeak_voice', 'ru')
        self.declare_parameter('espeak_speed', '130')
        self.declare_parameter('espeak_binary', 'espeak-ng')

        sub_topic = self.get_parameter('subscribe_topic').get_parameter_value().string_value
        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        self._voice = self.get_parameter('espeak_voice').get_parameter_value().string_value
        self._speed = self.get_parameter('espeak_speed').get_parameter_value().string_value
        self._binary = self.get_parameter('espeak_binary').get_parameter_value().string_value

        self._queue: deque[str] = deque()
        self._proc: subprocess.Popen | None = None

        self.create_subscription(String, sub_topic, self._speak_cb, 10)
        self._state_pub = self.create_publisher(String, state_topic, 10)
        self.create_timer(0.05, self._tick)
        self._publish_state('idle')

        self.get_logger().info(f'TTS (espeak-ng): in={sub_topic} state={state_topic}')

    def _publish_state(self, state: str) -> None:
        m = String()
        m.data = state
        self._state_pub.publish(m)

    def _speak_cb(self, msg: String) -> None:
        text = (msg.data or '').strip()
        if not text:
            return
        self._queue.append(text)

    def _start_next(self) -> None:
        if not self._queue:
            self._publish_state('idle')
            return
        text = self._queue.popleft()
        try:
            self._proc = subprocess.Popen(
                [self._binary, '-v', self._voice, '-s', self._speed, text],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._publish_state('playing')
            self.get_logger().info(f'espeak-ng: «{text[:80]}…»' if len(text) > 80 else f'espeak-ng: «{text}»')
        except FileNotFoundError:
            self.get_logger().error(
                f'Не найден исполняемый файл «{self._binary}». Установите espeak-ng.'
            )
            self._proc = None
            self._publish_state('idle')

    def _tick(self) -> None:
        if self._proc is not None:
            code = self._proc.poll()
            if code is None:
                return
            self._proc = None
        if self._queue:
            self._start_next()
        else:
            self._publish_state('idle')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TtsEspeakNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
