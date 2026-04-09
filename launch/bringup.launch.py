"""
Launch file for Kinova Real Robot bringup Pick&Place
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    kinova_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kinova_pick_place'),
                'launch',
                'kinova_real_robot.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_ip': '192.168.1.10',
        }.items()
    )

    kinova_vision = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('kinova_vision'),
                        'launch',
                        'kinova_vision.launch.py'
                    ])
                ]),
                launch_arguments={
                    'depth_registration': 'true',
                    'namespace': 'camera',
                    'rviz': 'false'
                }.items()
            )
        ]
    )

    voice_recognition = Node(
        package='kinova_pick_place',
        executable='voice_recognition.py',
        name='voice_recognition',
        output='screen',
    )

    command_interpreter = Node(
        package='kinova_pick_place',
        executable='command_interpreter.py',
        name='command_interpreter',
        output='screen',
    )

    return LaunchDescription([
        kinova_robot,
        kinova_vision,
        voice_recognition,
        command_interpreter
    ])
