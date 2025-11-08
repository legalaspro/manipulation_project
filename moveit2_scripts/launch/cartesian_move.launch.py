"""
Cartesian Move launch file (sim/real toggle).

Usage:
  ros2 launch moveit2_scripts cartesian_move.launch.py x:=0.3 y:=0.0 z:=0.2
  ros2 launch moveit2_scripts cartesian_move.launch.py x:=0.3 y:=0.0 z:=0.2 use_sim_time:=False
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # --- Declare Launch Arguments ---
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.25',
        description='Target X position (meters), leave empty to skip'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.25',
        description='Target Y position (meters), leave empty to skip'
    )

    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.25',
        description='Target Z position (meters), leave empty to skip'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Whether simulation or real robot used'
    )
    use_sim_time_cfg = LaunchConfiguration('use_sim_time')

    # Load both MoveIt configurations
    moveit_config_sim = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()
    moveit_config_real = MoveItConfigsBuilder("name", package_name="real_moveit_config").to_moveit_configs()

    # Cartesian Move node for simulation (when use_sim_time is True)
    cartesian_move_node_sim = Node(
        name="cartesian_move",
        package="moveit2_scripts",
        executable="cartesian_move",
        output="screen",
        condition=IfCondition(use_sim_time_cfg),
        parameters=[
            moveit_config_sim.robot_description,
            moveit_config_sim.robot_description_semantic,
            moveit_config_sim.robot_description_kinematics,
            {'use_sim_time': True},
            {'x': LaunchConfiguration('x')},
            {'y': LaunchConfiguration('y')},
            {'z': LaunchConfiguration('z')},
        ],
    )

    # Cartesian Move node for real robot (when use_sim_time is False)
    cartesian_move_node_real = Node(
        name="cartesian_move",
        package="moveit2_scripts",
        executable="cartesian_move",
        output="screen",
        condition=UnlessCondition(use_sim_time_cfg),
        parameters=[
            moveit_config_real.robot_description,
            moveit_config_real.robot_description_semantic,
            moveit_config_real.robot_description_kinematics,
            {'use_sim_time': False},
            {'x': LaunchConfiguration('x')},
            {'y': LaunchConfiguration('y')},
            {'z': LaunchConfiguration('z')},
        ],
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        use_sim_time_arg,
        cartesian_move_node_sim,
        cartesian_move_node_real,
    ])