"""
Move Joints launch file (sim/real toggle).

Usage:
  ros2 launch moveit2_scripts move_joints.launch.py joints:="0.0,-1.5708,0.0,-1.5708,0.0,0.0"
  ros2 launch moveit2_scripts move_joints.launch.py joints:="0.0,-1.5708,0.0,-1.5708,0.0,0.0" use_sim_time:=False
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # --- Declare Launch Arguments ---
    joints_arg = DeclareLaunchArgument(
        'joints',
        default_value='0.0,-1.5708,0.0,-1.5708,0.0,0.0',
        description='Target joint angles as comma-separated values (j0,j1,j2,j3,j4,j5) in radians'
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

    # MoveIt node for simulation (when use_sim_time is True)
    moveit_cpp_node_sim = Node(
        name="move_joints",
        package="moveit2_scripts",
        executable="move_joints",
        output="screen",
        condition=IfCondition(use_sim_time_cfg),
        arguments=[
            LaunchConfiguration('joints'),
        ],
        parameters=[
            moveit_config_sim.robot_description,
            moveit_config_sim.robot_description_semantic,
            moveit_config_sim.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    # MoveIt node for real robot (when use_sim_time is False)
    moveit_cpp_node_real = Node(
        name="move_joints",
        package="moveit2_scripts",
        executable="move_joints",
        output="screen",
        condition=UnlessCondition(use_sim_time_cfg),
        arguments=[
            LaunchConfiguration('joints'),
        ],
        parameters=[
            moveit_config_real.robot_description,
            moveit_config_real.robot_description_semantic,
            moveit_config_real.robot_description_kinematics,
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription([
        joints_arg,
        use_sim_time_arg,
        moveit_cpp_node_sim,
        moveit_cpp_node_real,
    ])

