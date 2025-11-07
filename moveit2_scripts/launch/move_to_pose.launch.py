"""
Move to Pose launch file (sim/real toggle).

Usage:
  ros2 launch moveit2_scripts move_to_pose.launch.py x:=0.3 y:=0.0 z:=0.2
  ros2 launch moveit2_scripts move_to_pose.launch.py x:=0.3 y:=0.0 z:=0.2 use_sim_time:=False
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
        default_value='0.3',
        description='Target X position (meters)'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Target Y position (meters)'
    )

    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.2',
        description='Target Z position (meters)'
    )

    qx_arg = DeclareLaunchArgument(
        'qx',
        default_value='-1.0',
        description='Target orientation quaternion X (default: -1.0 for grasping)'
    )

    qy_arg = DeclareLaunchArgument(
        'qy',
        default_value='0.0',
        description='Target orientation quaternion Y (default: 0.0 for grasping)'
    )

    qz_arg = DeclareLaunchArgument(
        'qz',
        default_value='0.0',
        description='Target orientation quaternion Z (default: 0.0 for grasping)'
    )

    qw_arg = DeclareLaunchArgument(
        'qw',
        default_value='0.0',
        description='Target orientation quaternion W (default: 0.0 for grasping)'
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
        name="move_to_pose",
        package="moveit2_scripts",
        executable="move_to_pose",
        output="screen",
        condition=IfCondition(use_sim_time_cfg),
        arguments=[
            LaunchConfiguration('x'),
            LaunchConfiguration('y'),
            LaunchConfiguration('z'),
            LaunchConfiguration('qx'),
            LaunchConfiguration('qy'),
            LaunchConfiguration('qz'),
            LaunchConfiguration('qw'),
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
        name="move_to_pose",
        package="moveit2_scripts",
        executable="move_to_pose",
        output="screen",
        condition=UnlessCondition(use_sim_time_cfg),
        arguments=[
            LaunchConfiguration('x'),
            LaunchConfiguration('y'),
            LaunchConfiguration('z'),
            LaunchConfiguration('qx'),
            LaunchConfiguration('qy'),
            LaunchConfiguration('qz'),
            LaunchConfiguration('qw'),
        ],
        parameters=[
            moveit_config_real.robot_description,
            moveit_config_real.robot_description_semantic,
            moveit_config_real.robot_description_kinematics,
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        qx_arg,
        qy_arg,
        qz_arg,
        qw_arg,
        use_sim_time_arg,
        moveit_cpp_node_sim,
        moveit_cpp_node_real,
    ])

