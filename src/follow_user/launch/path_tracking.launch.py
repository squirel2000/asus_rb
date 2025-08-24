import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package
    follow_user_pkg_dir = get_package_share_directory('follow_user')

    # Get the path to the config file
    params_file = os.path.join(follow_user_pkg_dir, 'config', 'controller_only.yaml')

    return LaunchDescription([
        
        # Just the controller server without lifecycle manager for testing
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('/cmd_vel', '/cmd_vel_nav'),
                ('/odom', '/slamware_ros_sdk_server_node/odom'),
            ]
        ),
    ])