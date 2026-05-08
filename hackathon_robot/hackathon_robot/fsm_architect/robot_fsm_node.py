#!/usr/bin/env python3
"""FSM + диспетчер приоритетов (занятие 19): батарея > приветствие > исследование.

Коллбэки только обновляют флаги; переходы состояний — в таймере `_dispatch`.
Интеграция с голосом: подписка на `robot/command` (JSON из voice_command_processor).

TTS: публикация в `voice/text_to_speak`; состояние синтеза — `voice/tts_state`
(playing / idle), см. `tts_espeak_node`.
"""

from __future__ import annotations

import json
import math
import queue
import random
import time
from enum import Enum, auto

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String


class State(Enum):
    EXPLORING = auto()
    GOING_HOME = auto()
    GREETING = auto()


class RobotFsmNode(Node):
    def __init__(self) -> None:
        super().__init__('robot_fsm')

        self.declare_parameter('battery_topic', '/hardware/battery')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/icp/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('nav_action', 'navigate_to_pose')
        self.declare_parameter('battery_threshold', 0.20)
        self.declare_parameter('greeting_cooldown_sec', 30.0)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_tolerance', 0.35)
        self.declare_parameter('simulate_battery', False)
        self.declare_parameter('simulate_battery_drop', 0.01)
        self.declare_parameter('simulate_battery_period_sec', 10.0)
        self.declare_parameter('robot_command_topic', 'robot/command')
        self.declare_parameter('tts_topic', 'voice/text_to_speak')
        self.declare_parameter('tts_state_topic', 'voice/tts_state')
        self.declare_parameter('voice_wake_topic', '/voice_cmd')
        self.declare_parameter('listen_wake_word', True)

        self._battery_topic = self.get_parameter('battery_topic').get_parameter_value().string_value
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        nav_action = self.get_parameter('nav_action').get_parameter_value().string_value
        self._battery_threshold = float(self.get_parameter('battery_threshold').get_parameter_value().double_value)
        self._greeting_cooldown_sec = float(self.get_parameter('greeting_cooldown_sec').get_parameter_value().double_value)
        self._home_x = float(self.get_parameter('home_x').get_parameter_value().double_value)
        self._home_y = float(self.get_parameter('home_y').get_parameter_value().double_value)
        self._home_tol = float(self.get_parameter('home_tolerance').get_parameter_value().double_value)
        sim_bat = self.get_parameter('simulate_battery').get_parameter_value().bool_value
        self._sim_drop = float(self.get_parameter('simulate_battery_drop').get_parameter_value().double_value)
        self._sim_period = float(self.get_parameter('simulate_battery_period_sec').get_parameter_value().double_value)

        cmd_topic = self.get_parameter('robot_command_topic').get_parameter_value().string_value
        self._tts_topic = self.get_parameter('tts_topic').get_parameter_value().string_value
        tts_state_topic = self.get_parameter('tts_state_topic').get_parameter_value().string_value
        wake_topic = self.get_parameter('voice_wake_topic').get_parameter_value().string_value
        self._listen_wake = self.get_parameter('listen_wake_word').get_parameter_value().bool_value

        self.state = State.EXPLORING
        self._battery_level = 1.0
        self._battery_low = False
        self._person_detected = False
        self._person_offset_x = 0.0
        self._map: OccupancyGrid | None = None
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._tts_playing = False
        self._last_greeting_time = 0.0
        self._greeting_step = 0
        self._turning = False
        self._turn_start_time = 0.0

        self._cmd_queue: queue.Queue[dict] = queue.Queue()
        self._nav_goal_handle = None
        self._greeting_utterance_sent = False
        self._greeting_heard_playing = False
        self._greeting_step2_started_at = 0.0

        self._cmd_pub_tts = self.create_publisher(String, self._tts_topic, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)

        self.create_subscription(BatteryState, self._battery_topic, self._battery_cb, 10)
        self.create_subscription(Bool, '/person_detected', self._person_cb, 10)
        self.create_subscription(Float32, '/person_offset_x', self._offset_cb, 10)
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, 1)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(String, cmd_topic, self._robot_command_cb, 10)
        self.create_subscription(String, tts_state_topic, self._tts_state_cb, 10)
        if self._listen_wake:
            self.create_subscription(String, wake_topic, self._wake_cb, 10)

        self._nav_client = ActionClient(self, NavigateToPose, nav_action)

        if sim_bat:
            self.create_timer(self._sim_period, self._simulate_battery_tick)

        self.create_timer(0.1, self._dispatch)

        self.get_logger().info('robot_fsm: диспетчер 10 Гц, Nav2 action «%s»' % nav_action)

    # --- Коллбэки: только данные / флаги ---

    def _battery_cb(self, msg: BatteryState) -> None:
        p = msg.percentage
        if p is None or math.isnan(float(p)):
            return
        self._battery_level = float(p)
        if self._battery_level < self._battery_threshold:
            self._battery_low = True

    def _simulate_battery_tick(self) -> None:
        self._battery_level = max(0.0, self._battery_level - self._sim_drop)
        if self._battery_level < self._battery_threshold:
            self._battery_low = True

    def _person_cb(self, msg: Bool) -> None:
        self._person_detected = bool(msg.data)

    def _offset_cb(self, msg: Float32) -> None:
        self._person_offset_x = float(msg.data)

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y

    def _tts_state_cb(self, msg: String) -> None:
        s = msg.data.strip().lower()
        self._tts_playing = s == 'playing'
        if self.state == State.GREETING and self._greeting_step >= 2 and s == 'playing':
            self._greeting_heard_playing = True

    def _robot_command_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._cmd_queue.put(data)
        except json.JSONDecodeError:
            self.get_logger().warn('robot/command: не JSON')

    def _wake_cb(self, msg: String) -> None:
        text = msg.data.lower()
        if 'робот' in text and self.state == State.EXPLORING and not self._battery_low:
            self._speak('Привет друг, я робот. Будущее уже здесь.')

    def _speak(self, text: str) -> None:
        m = String()
        m.data = text
        self._cmd_pub_tts.publish(m)

    # --- Навигация ---

    def _cancel_current_nav(self) -> None:
        if self._nav_goal_handle is not None:
            try:
                self._nav_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            self._nav_goal_handle = None

    def _send_nav_goal(self, x: float, y: float, yaw: float = 0.0) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 недоступен, цель не отправлена')
            return

        send_future = self._nav_client.send_goal_async(goal, feedback_callback=self._nav_feedback)
        send_future.add_done_callback(self._nav_goal_response)

    def _nav_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 отклонил цель')
            self._nav_goal_handle = None
            if self.state == State.EXPLORING:
                self._send_next_exploring_goal()
            return
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future) -> None:
        status = future.result().status
        self._nav_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Цель достигнута')
        else:
            self.get_logger().warn(f'Навигация завершилась со статусом {status}')

        if self.state == State.GOING_HOME and status == GoalStatus.STATUS_SUCCEEDED:
            self._battery_low = False
            self._battery_level = max(self._battery_level, 0.5)
            self._enter_exploring()
            return

        if self.state == State.EXPLORING:
            self._send_next_exploring_goal()

    def _nav_feedback(self, feedback_msg) -> None:
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f'Осталось: {dist:.2f} м')

    def _pick_random_free_goal(self) -> tuple[float, float] | None:
        if self._map is None:
            return None
        data = np.array(self._map.data, dtype=np.int8)
        w = self._map.info.width
        if w <= 0 or len(data) == 0:
            return None
        free_indices = np.where(data == 0)[0]
        if len(free_indices) == 0:
            return None
        idx = int(random.choice(free_indices))
        row, col = divmod(idx, w)
        res = self._map.info.resolution
        ox = self._map.info.origin.position.x
        oy = self._map.info.origin.position.y
        x = ox + col * res + res / 2.0
        y = oy + row * res + res / 2.0
        return x, y

    # --- Переходы ---

    def _enter_going_home(self) -> None:
        self.get_logger().warn('Низкий заряд: еду домой')
        self.state = State.GOING_HOME
        self._cancel_current_nav()
        self._send_nav_goal(self._home_x, self._home_y, 0.0)

    def _enter_greeting(self) -> None:
        self.state = State.GREETING
        self._cancel_current_nav()
        self._greeting_step = 0
        self._greeting_utterance_sent = False
        self._greeting_heard_playing = False
        self._greeting_step2_started_at = 0.0

    def _enter_exploring(self) -> None:
        self.state = State.EXPLORING
        self._person_detected = False
        self._send_next_exploring_goal()

    def _near_home(self) -> bool:
        dx = self._odom_x - self._home_x
        dy = self._odom_y - self._home_y
        return math.hypot(dx, dy) < self._home_tol

    # --- Движение приветствия ---

    def _turn_towards_person(self) -> None:
        self._turning = True
        self._turn_start_time = time.time()
        angular_speed = -0.4 * self._person_offset_x
        angular_speed = max(-0.5, min(0.5, angular_speed))
        t = Twist()
        t.angular.z = angular_speed
        self._cmd_vel_pub.publish(t)

    def _is_turning(self) -> bool:
        if not self._turning:
            return False
        if time.time() - self._turn_start_time > 1.5:
            self._turning = False
            self._cmd_vel_pub.publish(Twist())
            return False
        return True

    def _in_cooldown(self) -> bool:
        return (time.time() - self._last_greeting_time) < self._greeting_cooldown_sec

    # --- Команды из голосового контура ---

    def _apply_voice_command(self, cmd: dict) -> None:
        action = cmd.get('action')
        params = cmd.get('params') or {}
        if action == 'stop':
            self._cancel_current_nav()
            self._cmd_vel_pub.publish(Twist())
            return
        if action == 'find_object':
            obj = params.get('object', 'объект')
            color = params.get('color', '')
            self.get_logger().info(f'Команда поиска: {color} {obj} (заглушка — подключите CV/Nav)')
            return
        if action == 'navigate_to':
            self.get_logger().info('navigate_to: заглушка — задайте цель в Nav2 вручную или расширьте парсер')

    def _drain_commands(self) -> None:
        while True:
            try:
                cmd = self._cmd_queue.get_nowait()
            except queue.Empty:
                break
            self._apply_voice_command(cmd)

    # --- Диспетчер ---

    def _dispatch(self) -> None:
        self._drain_commands()

        if self._battery_low and self.state != State.GOING_HOME:
            self._enter_going_home()
            return

        if self.state == State.GOING_HOME:
            self._tick_going_home()
            return

        if (
            self._person_detected
            and not self._in_cooldown()
            and self.state == State.EXPLORING
            and not self._tts_playing
        ):
            self._enter_greeting()
            return

        if self.state == State.EXPLORING:
            self._tick_exploring()
        elif self.state == State.GREETING:
            self._tick_greeting()

    def _tick_going_home(self) -> None:
        if self._near_home() and self._nav_goal_handle is None:
            self.get_logger().info('Ожидание у базы (одометрия в пределах допуска)')
        # Завершение GOING_HOME при успешном Nav2 обрабатывается в _nav_result

    def _tick_greeting(self) -> None:
        if self._greeting_step == 0:
            self._turn_towards_person()
            self._greeting_step = 1
            return

        if self._greeting_step == 1:
            if self._is_turning():
                return
            if not self._greeting_utterance_sent:
                self._speak('Привет друг, я робот. Меня запрограммировали лучшие инженеры.')
                self._greeting_utterance_sent = True
                self._greeting_step = 2
                self._greeting_step2_started_at = time.time()
            return

        if self._greeting_step == 2:
            timed_out = (time.time() - self._greeting_step2_started_at) > 8.0
            if (self._greeting_heard_playing and not self._tts_playing) or timed_out:
                self._last_greeting_time = time.time()
                self._person_detected = False
                self._enter_exploring()

    def _tick_exploring(self) -> None:
        if self._nav_goal_handle is not None:
            return
        self._send_next_exploring_goal()

    def _send_next_exploring_goal(self) -> None:
        goal = self._pick_random_free_goal()
        if goal is None:
            self.get_logger().warn('Нет свободных клеток на карте — жду /map')
            return
        x, y = goal
        self.get_logger().info(f'Исследование: цель ({x:.2f}, {y:.2f})')
        self._send_nav_goal(x, y, 0.0)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotFsmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
