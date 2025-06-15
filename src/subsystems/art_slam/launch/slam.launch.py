from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='art_slam',
            executable='art_slam_node',
            name='art_slam_node',
            output='screen'
        )
    ])
