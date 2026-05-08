#!/usr/bin/env python3
"""Связывает распознанный текст с роботом (учебник: Command Processor).

Подписка: voice/recognized_text и (опционально) /voice_cmd.
Публикации: robot/command (JSON), voice/text_to_speak (ответ пользователю).
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hackathon_robot.voice.command_parser import CommandParser


class VoiceCommandProcessorNode(Node):
    def __init__(self) -> None:
        super().__init__('voice_command_processor')

        self.declare_parameter('recognized_topic', 'voice/recognized_text')
        self.declare_parameter('also_listen_voice_cmd', True)
        self.declare_parameter('command_topic', 'robot/command')
        self.declare_parameter('tts_topic', 'voice/text_to_speak')

        rec_topic = self.get_parameter('recognized_topic').get_parameter_value().string_value
        self._also_voice_cmd = self.get_parameter('also_listen_voice_cmd').get_parameter_value().bool_value
        cmd_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        tts_topic = self.get_parameter('tts_topic').get_parameter_value().string_value

        self._parser = CommandParser()
        self._cmd_pub = self.create_publisher(String, cmd_topic, 10)
        self._tts_pub = self.create_publisher(String, tts_topic, 10)

        self.create_subscription(String, rec_topic, self._on_text, 10)
        if self._also_voice_cmd:
            self.create_subscription(String, '/voice_cmd', self._on_text, 10)

        self.get_logger().info(
            f'Command processor: in«{rec_topic}» + '
            f'{"/voice_cmd " if self._also_voice_cmd else ""}'
            f'→ «{cmd_topic}», TTS «{tts_topic}»'
        )

    def _speak(self, text: str) -> None:
        m = String()
        m.data = text
        self._tts_pub.publish(m)

    def _on_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        try:
            command = self._parser.parse(text)
            if command['action'] == 'unknown':
                self._speak(self._parser.generate_response(command))
                return

            cmd_msg = String()
            cmd_msg.data = json.dumps(command, ensure_ascii=False)
            self._cmd_pub.publish(cmd_msg)
            self._speak(self._parser.generate_response(command))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Ошибка обработки текста: {e}')
            self._speak('Произошла ошибка')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoiceCommandProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
