"""
Launch file for Kinova Simulation
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use simulated clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="true",
            description="Use Gazebo for simulation",
        )
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_gazebo = LaunchConfiguration("sim_gazebo")

    moveit_config = (
        MoveItConfigsBuilder(
            "gen3", package_name="kinova_gen3_6dof_robotiq_2f_85_moveit_config")
        .robot_description(mappings={
            "robot_ip": "xxx.yyy.zzz.www",
            "use_fake_hardware": "false",
            "vision": "true",
            "gripper": "robotiq_2f_85",
            "dof": "6",
            "sim_gazebo": sim_gazebo})
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    octomap_config = {'octomap_frame': 'base_link',
                      'octomap_resolution': 0.02}

    octomap_updater_config = load_yaml(
        'kinova_pick_place', 'config/sensors_3d.yaml')

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"}

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            octomap_config,
            octomap_updater_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory(
            "kinova_pick_place"),
        "rviz",
        "display.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},

        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description,
                    {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(declared_arguments + [move_group_node, robot_state_publisher, rviz_node])
