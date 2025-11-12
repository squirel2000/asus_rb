#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    params_file = LaunchConfiguration('params_file')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('motion_common'),
            'config',
            'motion_params.yaml'
        ]),
        description='Path to the ROS2 parameters file for motion nodes'
    )

    # Define navigation_motion_node
    navigation_motion_node = Node(
        package='navigation',
        executable='navigation_motion_node.py',
        name='navigation_motion_node',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Define guidance_motion_node
    guidance_motion_node = Node(
        package='guidance',
        executable='guidance_motion_node.py',
        name='guidance_motion_node',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Define guidance_motion_node
    follow_user_motion_node = Node(
        package='follow_user',
        executable='follow_user_motion_node.py',
        name='follow_user_motion_node',
        output='screen',
        parameters=[params_file],
        arguments=[
                   '--robot_ip', '192.168.11.1',
                   '--no_head_control',
                   '--ros-args', '--log-level', 'info',
                   ]
    )

    # Create LaunchDescription
    return LaunchDescription([
        declare_params_file_arg,
        navigation_motion_node,
        guidance_motion_node,
        follow_user_motion_node
    ])