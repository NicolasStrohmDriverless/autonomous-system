import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def launch_setup(context, *args, **kwargs):
    urdf_file_name = LaunchConfiguration('urdf_name').perform(context)

    urdf_path = os.path.join(
        get_package_share_directory('descriptions'),
        'urdf',
        urdf_file_name
    )

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF/Xacro file not found: {urdf_path}")

    # Process xacro file here
    doc = xacro.process_file(urdf_path)
    robot_desc = doc.toprettyxml(indent='  ')

    if not robot_desc.strip():
        raise ValueError(f"Processed URDF from {urdf_path} is empty!")

    print(f"[INFO] Successfully processed Xacro file: {urdf_path}")

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc
            }],
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_name',
            default_value='nora11.urdf.xacro',
            description='URDF/Xacro model file name'
        ),
        OpaqueFunction(function=launch_setup)
    ])
