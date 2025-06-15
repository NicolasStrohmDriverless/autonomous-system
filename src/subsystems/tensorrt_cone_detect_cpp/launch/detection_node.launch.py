from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('tensorrt_cone_detect_cpp')
    model_path = os.path.join(pkg_share, 'resource', 'v11n_416x416.onnx')

    return LaunchDescription([
        Node(
            package='tensorrt_cone_detect_cpp',
            executable='detection_node',
            name='detection_node',
            output='screen',
            parameters=[{'onnx_path': model_path}]
        ),
        Node(
            package='depth_tracking',
            executable='depth_node',
            name='depth_annotation_node',
            output='screen'
        ),
    ])
