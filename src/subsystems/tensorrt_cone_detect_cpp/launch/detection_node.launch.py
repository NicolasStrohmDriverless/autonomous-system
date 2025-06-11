from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tensorrt_cone_detect_cpp',
            executable='detection_node',
            name='detection_node',
            output='screen'
        ),
        Node(
            package='depth_tracking',
            executable='depth_node',
            name='depth_annotation_node',
            output='screen'
        ),
    ])
