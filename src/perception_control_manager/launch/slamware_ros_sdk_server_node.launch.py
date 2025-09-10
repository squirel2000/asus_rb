from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define launch arguments
    ip_address = LaunchConfiguration('ip_address', default='192.168.12.1')
    port = LaunchConfiguration('port', default='1445')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')

    # Get the share directory of slamware_ros_sdk
    slamware_ros_sdk_dir = get_package_share_directory('slamware_ros_sdk')

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

        # SLAMWARE ROS SDK server node
        Node(
            package='slamware_ros_sdk',
            executable='slamware_ros_sdk_server_node',
            name='slamware_ros_sdk_server_node',
            output='both',
            parameters=[
                {'ip_address': ip_address},
                {'robot_port': port},
                {'angle_compensate': True},
                {'fixed_odom_map_tf': True},
                {'raw_ladar_data': False},
                {'robot_frame': 'base_link'},
                {'odom_frame': 'odom'},
                {'laser_frame': 'laser'},
                {'map_frame': 'slamware_map'},
                {'robot_pose_frame': 'robot_pose'},
                {'odometry_pub_period': 0.05},
                {'robot_pose_pub_period': 0.05},
                {'scan_pub_period': 0.1},
                {'map_pub_period': 0.2},
                {'path_pub_period': 0.1},
                {'imu_raw_data_period': 0.1},
                {'virtual_walls_pub_period': 1.0},
                {'virtual_tracks_pub_period': 1.0},
                {'basic_sensors_values_pub_period': 1.0},
                {'vel_control_topic': 'cmd_vel'},
                {'ladar_data_clockwise': True},
                {'pub_accumulate_odometry': False},
                {'robot_pose_topic': 'robot_pose'},
            ],
            remappings=[
                ('scan', 'scan'),
                ('odom', 'odom'),
                ('map', 'slamware_map'),
                ('map_metadata', 'map_metadata'),
                ('global_plan_path', 'global_plan_path'),
            ]
        ),

        # RViz visualization node (launched if enable_rviz is true)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(slamware_ros_sdk_dir, 'rviz', 'slamware_ros_sdk_server_node.rviz')],
            condition=IfCondition(enable_rviz)
        ),

        # Optional: Static transform publishers (uncomment if needed)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='map2odom',
        #     arguments=['0', '0', '0', '0', '0', '0', '1', 'slamware_map', 'odom']
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='map2robotpose',
        #     arguments=['0', '0', '0', '0', '0', '0', '1', 'slamware_map', 'robot_pose']
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base2laser',
        #     arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser']
        # ),
    ])