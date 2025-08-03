from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follow_user',
            executable='follow_user_motion_node.py',
            name='follow_user_motion_node',
            output='screen'
        ),
        Node(
            package='follow_user',
            executable='follow_user_vision_node.py',
            name='follow_user_vision_node',
            output='screen'
        ),
        Node(
            package='navigation',
            executable='navigation_motion_node.py',
            name='navigation_motion_node',
            output='screen'
        ),
        Node(
            package='task_coordinator',
            executable='task_coordinator_node',
            name='task_coordinator_node',
            output='screen'
        ),
    ])
