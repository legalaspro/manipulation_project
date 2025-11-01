"""
Static Transform Publisher
Object Detection
RViz2

Usage:
  ros2 launch object_detection object_detection.launch.py
"""

from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # --- Dynamic paths selection ---
    path_pkg_share = FindPackageShare("object_detection")

    rviz_config = PathJoinSubstitution([
        path_pkg_share,
        'rviz',
         'object_detection.rviz'
    ])

    log_rviz_config = LogInfo(msg=["Rviz Config: ", rviz_config])

    return LaunchDescription([    
        log_rviz_config,
        Node(
            package='object_detection',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen'),

        Node(
            package='object_detection',
            executable='object_detection',
            name='object_detection',
            output='screen',
            emulate_tty=True),
        
         TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    parameters=[{"use_sim_time": True}],
                    arguments=[
                        "-d",
                        rviz_config,
                    ],
                ),
            ]
        )
    ])