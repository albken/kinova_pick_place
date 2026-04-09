"""
Launch file for MoveIt Task Constructor Pick&Place and Cube from Point Cloud detector.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder(
        "gen3", package_name="kinova_gen3_6dof_robotiq_2f_85_moveit_config").to_moveit_configs()
    mtc_node = Node(
        package="kinova_pick_place",
        executable="kinova_pick_place",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
        ],
    )

    color_cloud_detector = Node(
        package='kinova_pick_place',
        executable='color_cloud_detector',
        name='color_cloud_detector',
        output='screen'
    )

    return LaunchDescription([color_cloud_detector, mtc_node])
