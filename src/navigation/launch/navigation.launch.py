from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation',
            executable='navigation_motion_node.py',
            name='navigation_motion_node',
            output='screen'
        ),
        Node(
            package='navigation',
            executable='coordinator_node.py',
            name='navigation_coordinator',
            output='screen'
        ),
    ])
