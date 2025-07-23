from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the follow-me-bot project.
    This launch file starts all the necessary nodes for the user following feature.
    """
    return LaunchDescription([
        Node(
            package='follow_me_bot',
            executable='user_tracking_visual_node.py',
            name='user_tracking_visual_node',
            output='screen',
            parameters=[
                # You can override default parameters here if needed
                # {'image_topic': '/camera/color/image_raw'},
                # {'depth_topic': '/camera/depth/image_rect_raw'}
            ]
        ),
        Node(
            package='follow_me_bot',
            executable='user_tracking_controller_node.py',
            name='user_tracking_controller_node',
            output='screen',
            parameters=[
                # {'target_distance': 1.2}
            ]
        ),
        Node(
            package='follow_me_bot',
            executable='behavior_coordinator_node.py',
            name='behavior_coordinator_node',
            output='screen'
        ),
    ])
