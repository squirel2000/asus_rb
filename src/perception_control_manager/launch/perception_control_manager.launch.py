from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    launch_file_dir = get_package_share_directory('perception_control_manager')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'launch', 'slamware_ros_sdk_server_node.launch.py')
            ),
            launch_arguments={
                'ip_address': '192.168.12.1',
                'port': '1445',
                'enable_rviz':'true'
            }.items()
        ),
        Node(
            package='perception_control_manager',
            executable='perception_control_manager_node.py',
            name='perception_control_manager_node',
            output='screen'
        ),

    ])