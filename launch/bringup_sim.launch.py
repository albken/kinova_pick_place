"""
Launch file for Kinova Simulation bringup Pick&Place
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    kinova_robot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kortex_bringup'),
                'launch',
                'kortex_sim_control.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'launch_rviz': 'false',
            'robot_controller': 'joint_trajectory_controller',
            'gripper': 'robotiq_2f_85',
            'dof': '6',
            'vision': 'true'

        }.items()
    )

    kinova_robot = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('kinova_pick_place'),
                        'launch',
                        'kinova_sim.launch.py'
                    ])
                ]),
            )
        ]
    )

    tf_base_to_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_camera",
        output="screen",
        arguments=[
            "0.25", "0.030", "0.50",
                    "1.57079632679", "3.14159265358979", "0",
                    "base_link", "camera_color_frame",
        ],
    )

    fake_cloud = Node(
        package='kinova_pick_place',
        executable='fake_cloud',
        name='fake_cloud',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}],
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
        tf_base_to_cam,
        kinova_robot_gazebo,
        kinova_robot,
        fake_cloud,
        voice_recognition,
        command_interpreter
    ])
