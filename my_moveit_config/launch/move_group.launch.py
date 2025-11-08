
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()
    
    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 5.0,},
            {"publish_robot_description_semantic": True},
            {"use_sim_time": True},
        ],
    )

    # Include the scene-adding node
    add_scene_node = Node(
        package='moveit2_scripts', 
        executable='add_ur3_sim_scene',
        name='add_ur3_sim_scene',
        output='screen',
    )

    return LaunchDescription(
        [move_group_node, add_scene_node]
    )