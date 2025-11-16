import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get the path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('follow_user'),
        'urdf',
        'robot_head.urdf.xacro'
    )

    # Process the XACRO file to generate the URDF XML
    robot_description_config = xacro.process_file(urdf_file_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Create the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])
