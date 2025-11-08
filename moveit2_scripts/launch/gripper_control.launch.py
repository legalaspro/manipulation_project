"""
Gripper Control launch file (sim/real toggle).

Usage:
  ros2 launch moveit2_scripts gripper_control.launch.py value:=0.05 mode:=width
  ros2 launch moveit2_scripts gripper_control.launch.py value:=0.05 mode:=width use_sim_time:=False
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Whether to use simulation time'
    )

    value_arg = DeclareLaunchArgument(
        'value',
        default_value='0.0',
        description='Gripper value (radians or meters depending on mode)'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='radians',
        description='Control mode: radians or width'
    )

    # Get launch configurations
    use_sim_time_cfg = LaunchConfiguration('use_sim_time')

    # Load MoveIt configs
    moveit_config_sim = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()
    moveit_config_real = MoveItConfigsBuilder("name", package_name="real_moveit_config").to_moveit_configs()

    # Gripper Control node for simulation (when use_sim_time is True)
    gripper_control_node_sim = Node(
        name="gripper_control",
        package="moveit2_scripts",
        executable="gripper_control",
        output="screen",
        condition=IfCondition(use_sim_time_cfg),
        arguments=[
            LaunchConfiguration('value'),
            LaunchConfiguration('mode'),
        ],
        parameters=[
            moveit_config_sim.robot_description,
            moveit_config_sim.robot_description_semantic,
            moveit_config_sim.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    # Gripper Control node for real robot (when use_sim_time is False)
    gripper_control_node_real = Node(
        name="gripper_control",
        package="moveit2_scripts",
        executable="gripper_control",
        output="screen",
        condition=UnlessCondition(use_sim_time_cfg),
        arguments=[
            LaunchConfiguration('value'),
            LaunchConfiguration('mode'),
        ],
        parameters=[
            moveit_config_real.robot_description,
            moveit_config_real.robot_description_semantic,
            moveit_config_real.robot_description_kinematics,
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        value_arg,
        mode_arg,
        gripper_control_node_sim,
        gripper_control_node_real,
    ])

