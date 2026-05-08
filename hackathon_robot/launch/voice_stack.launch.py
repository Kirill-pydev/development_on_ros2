"""Launch: офлайн голос (Vosk) + espeak-ng + парсер команд (учебник)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        'vosk_model_path',
        default_value='',
        description='Каталог распакованной модели Vosk (русский small/medium).',
    )
    model = LaunchConfiguration('vosk_model_path')

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

    return LaunchDescription([model_arg, tts, asr, proc])
