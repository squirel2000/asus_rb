from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define launch arguments
    ip_address = LaunchConfiguration('ip_address')
    enable_rviz = LaunchConfiguration('enable_rviz')

    launch_file_dir = get_package_share_directory('perception_control_manager')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'ip_address',
            default_value='192.168.12.1',
            description='IP address for the SLAMWARE SDK server'
        ),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            description='Whether to launch RViz for visualization'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'launch', 'slamware_ros_sdk_server_node.launch.py')
            ),
            launch_arguments={
                'ip_address': ip_address,
                'enable_rviz': enable_rviz
            }.items()
        ),
        Node(
            package='perception_control_manager',
            executable='perception_control_manager_node.py',
            name='perception_control_manager_node',
            output='screen',
            parameters=[{'robot_ip': ip_address}]
        ),

    ])