import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to this package
    pkg_dir = get_package_share_directory('follow_user')
    
    # Get the path to the config file
    params_file = os.path.join(pkg_dir, 'config', 'pure_pursuit_params.yaml')
    
    return LaunchDescription([
        Node(
            package='follow_user',
            executable='path_search_server.py',
            name='path_search_server',
            output='screen'
        ),
        Node(
            package='follow_user',
            executable='pure_pursuit_controller',
            name='pure_pursuit_controller',
            output='screen',
            parameters=[params_file]
        ),
    ])