from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='onnx_cone_detect',
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
        Node(
            package='onnx_cone_detect',
            executable='imu_viz_node',
            name='imu_viz_node',
            output='screen'
        ),
    ])
