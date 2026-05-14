from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'hackathon_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML', 'numpy'],
    zip_safe=True,
    maintainer='Hackathon',
    maintainer_email='user@todo',
    description='Калибровка лидара, объезд, разметка, YOLO, голос, FSM, GOD-тестовые ноды.',
    license='Apache-2.0',
    tests_require=['pytest'],
    extras_require={
        'voice': ['vosk', 'sounddevice'],
    },
    entry_points={
        'console_scripts': [
            'autonomous_drive_forward = hackathon_robot.nav_engineer.autonomous_drive_forward:main',
            'lane_follower = hackathon_robot.nav_engineer.lane_follower_node:main',
            'contour_avoidance = hackathon_robot.nav_engineer.contour_avoidance:main',
            'lidar_calibrator = hackathon_robot.sensing.lidar_calibrator:main',
            'yolo_finetune_node = hackathon_robot.cv_engineer.yolo_finetune_node:main',
            'person_detector = hackathon_robot.cv_engineer.person_detector_node:main',
            'robot_fsm = hackathon_robot.fsm_architect.robot_fsm_node:main',
            'tts_espeak = hackathon_robot.voice.tts_espeak_node:main',
            'asr_vosk = hackathon_robot.voice.asr_vosk_node:main',
            'voice_command_processor = hackathon_robot.voice.command_processor_node:main',
            'god_voice_all = hackathon_robot.god.god_voice_all_node:main',
            'god_lane_avoid = hackathon_robot.god.god_lane_avoid_node:main',
            'god_person_stop = hackathon_robot.god.god_person_stop_node:main',
        ],
    },
)
