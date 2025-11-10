"""
Static Transform Publisher
Object Detection
RViz2

Usage:
  ros2 launch object_detection object_detection_real.launch.py
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
         'object_detection_real.rviz'
    ])

    log_rviz_config = LogInfo(msg=["Rviz Config: ", rviz_config])

    return LaunchDescription([    
        log_rviz_config,
        Node(
            package='object_detection',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            parameters=[{
                'header_frame_id': 'base_link', 
                'child_frame_id': 'camera_depth_optical_frame',
                'translation_x': 0.5,
                'translation_y': 0.605,
                'translation_z': 0.1,
                'rotation_x': -0.0,
                'rotation_y': 0.766,
                'rotation_z': -0.643,
                'rotation_w': 0.0
            }]
        ),


        Node(
            package='object_detection',
            executable='object_detection',
            name='object_detection',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'pointcloud_topic': '/camera/depth/color/points',
                'plane_min_y': 0.0,
                'objects_min_y': 0.0
            }]
        ),
        
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    parameters=[{"use_sim_time": False}],
                    arguments=[
                        "-d",
                        rviz_config,
                    ],
                ),
            ]
        )
    ])