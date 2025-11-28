#!/usr/bin/env python3

import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# Setting log format
os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "{date_time_with_ms} [{name}] [{severity}] {message}"

def generate_launch_description():
    motion_common_dir = get_package_share_directory('motion_common')

    # ==== create log folder ====
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = os.path.join(motion_common_dir, "logs", stamp)
    os.makedirs(log_dir, exist_ok=True)

    # ==== change log output folder ====
    set_log_dir = SetEnvironmentVariable('ROS_LOG_DIR', log_dir)

    # ==== Declare launch arguments ====
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
        output='both',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Define guidance_motion_node
    guidance_motion_node = Node(
        package='guidance',
        executable='guidance_motion_node.py',
        name='guidance_motion_node',
        output='both',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Define guidance_motion_node
    follow_user_motion_node = Node(
        package='follow_user',
        executable='follow_user_motion_node.py',
        name='follow_user_motion_node',
        output='both',
        parameters=[params_file],
        arguments=[
            '--robot_ip', '192.168.11.1',
            #'--no_head_control',
            '--ros-args', '--log-level', 'info',
        ]
    )

    # ==== Return LaunchDescription ====
    return LaunchDescription([
        set_log_dir,
        declare_params_file_arg,
        navigation_motion_node,
        guidance_motion_node,
        follow_user_motion_node
    ])
