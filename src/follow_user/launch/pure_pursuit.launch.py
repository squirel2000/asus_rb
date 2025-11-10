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

    # Get the path to the config file
    params_file = os.path.join(pkg_dir, 'config', 'pure_pursuit_params.yaml')

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
        Node(
            package='follow_user',
            executable='path_search_server.py',
            name='path_search_server',
            output='screen',
            arguments=['--robot-ip', LaunchConfiguration('robot_ip')]
        ),
        Node(
            package='follow_user',
            executable='pure_pursuit_controller',
            name='pure_pursuit_controller',
            output='screen',
            parameters=[params_file]
        ),
        record_log_launch,
    ])