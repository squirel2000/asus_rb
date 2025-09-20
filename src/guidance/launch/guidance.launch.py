from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='guidance',
            executable='guidance_motion_node.py',
            name='guidance_motion_node',
            output='screen'
        ),
    ])
