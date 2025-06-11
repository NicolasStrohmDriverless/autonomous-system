from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    ekf = LaunchConfiguration('ekf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ekf',
            default_value='false',
            description='Use Sensor data to perform ekf transformation'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('descriptions'),
                    'launch',
                    'base-link.launch.py'
                ]),
            ]),
            condition = UnlessCondition(ekf)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('descriptions'),
                    'launch',
                    'ekf_localization.launch.py'
                ])
            ]),
            condition = IfCondition(ekf)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('descriptions'),
                    'launch',
                    'odom.launch.py'
                ])
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('descriptions'),
                    'launch',
                    'robot_urdf.launch.py'
                ])
            ]),
            launch_arguments={
                'urdf_name': 'nora11.urdf.xacro'
            }.items()
        ),
        
    ])