
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="real_moveit_config").to_moveit_configs()

    # Include the object_detection launch file
    object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('object_detection'),
            '/launch/object_detection_real.launch.py'
        ])
    )

    # MoveItCpp executable
    moveit_cpp_node = Node(
        name="pick_and_place_perception",
        package="moveit2_scripts",
        executable="pick_and_place_perception_real",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': False},
        ],
    )

    delayed_moveit = TimerAction(
        period=3.0,
        actions=[moveit_cpp_node]
    )

    return LaunchDescription(
        [
            object_detection_launch,
            delayed_moveit
        ]
    )