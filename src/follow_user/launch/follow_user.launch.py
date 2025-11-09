import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the path to this package
    pkg_dir = get_package_share_directory('follow_user')

    # Declare launch arguments
    params_file = LaunchConfiguration('params_file')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('motion_common'),
            'config',
            'motion_params.yaml'
        ]),
        description='Path to the ROS2 parameters file for motion nodes'
    )

    # Declare the robot_ip launch argument
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.11.1')

    # Include the recording launch file
    record_log_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('motion_common'),
                'launch',
                'record_log.launch.py'
            ])
        ]),
        launch_arguments={'task': 'follow_user', }.items()
    )

    return LaunchDescription([
        robot_ip_arg,
        declare_params_file_arg,
        Node(
            package='follow_user',
            executable='follow_user_motion_node.py',
            name='follow_user_motion_node',
            output='screen',
            arguments=['--robot-ip', LaunchConfiguration('robot_ip')],
            parameters=[params_file]
        ),
        record_log_launch,
    ])
