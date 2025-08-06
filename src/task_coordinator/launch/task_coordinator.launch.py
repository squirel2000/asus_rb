from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task_coordinator',
            executable='task_coordinator_node.py',
            name='task_coordinator_node',
            output='screen'
        ),
    ])
