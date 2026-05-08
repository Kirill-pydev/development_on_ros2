"""Полный стек: голос + HOG + FSM (Nav2, батарея, приветствие)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        'vosk_model_path',
        default_value='',
        description='Каталог модели Vosk.',
    )
    cam_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera_node/image_raw',
        description='Топик камеры для HOG.',
    )
    model = LaunchConfiguration('vosk_model_path')
    cam = LaunchConfiguration('camera_topic')

    tts = Node(
        package='hackathon_robot',
        executable='tts_espeak',
        name='tts_espeak',
        output='screen',
    )

    asr = Node(
        package='hackathon_robot',
        executable='asr_vosk',
        name='asr_vosk',
        output='screen',
        parameters=[{'model_path': model}],
    )

    proc = Node(
        package='hackathon_robot',
        executable='voice_command_processor',
        name='voice_command_processor',
        output='screen',
    )

    hog = Node(
        package='hackathon_robot',
        executable='person_detector',
        name='person_detector',
        output='screen',
        parameters=[{'image_topic': cam}],
    )

    fsm = Node(
        package='hackathon_robot',
        executable='robot_fsm',
        name='robot_fsm',
        output='screen',
        parameters=[
            {
                'simulate_battery': False,
                'listen_wake_word': True,
            }
        ],
    )

    return LaunchDescription([model_arg, cam_arg, tts, asr, proc, hog, fsm])
