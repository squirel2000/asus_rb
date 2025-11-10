from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from datetime import datetime
import os

def launch_setup(context, *args, **kwargs):
    task = LaunchConfiguration("task").perform(context)
    label = LaunchConfiguration("label").perform(context)
    output_dir = LaunchConfiguration("output_dir").perform(context)

    final_name = f"test_{task}_{label}" if label else f"test_{task}"

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = f"{timestamp}_{final_name}"

    bag_path = os.path.join(output_dir, bag_name) if output_dir else bag_name

    # Topic lists
    base_topics = ["/tf",
                   "/slamware_ros_sdk_server_node/scan",
                   "/slamware_ros_sdk_server_node/odom",
                   "/robot_pose",
                   "/amr_events",
                   ]
    navigation_topics = base_topics + ["/slamware_ros_sdk_server_node/global_plan_path",
                                       "/remaining_targets",
                                       "/current_max_speed",
                                       ]
    guidance_topics = navigation_topics + ["/human_relative_pose_rear_raw",
                                           "/human_relative_pose_rear",
                                           "/set_max_speed",
                                           ]
    follow_user_topics = base_topics + ["/cmd_vel",
                                        "/clicked_point",
                                        "/human_relative_pose_front",
                                        "/follow_user/human_absolute_pose",
                                        "/follow_user/planned_path",
                                        "/lookahead_point",
                                        ]

    navigation_cmd = ["ros2", "bag", "record", *navigation_topics, "-o", bag_path]
    guidance_cmd = ["ros2", "bag", "record", *guidance_topics, "-o", bag_path]
    follow_user_cmd = ["ros2", "bag", "record", *follow_user_topics, "-o", bag_path]

    # Create the corresponding ExecuteProcess according to the task
    if task == "navigation":
        selected_cmd = navigation_cmd
    elif task == "guidance":
        selected_cmd = guidance_cmd
    elif task == "follow_user":
        selected_cmd = follow_user_cmd
    else:
        raise ValueError(f"Unknown task type: {task}")

    return [
        LogInfo(msg=f"Recording ROS2 bag for task '{task}' → {bag_path}"),
        ExecuteProcess(cmd=selected_cmd, output="screen"),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "task",
            default_value=TextSubstitution(text="guidance"),
            description="Task type: navigation, guidance, or follow_user",
            choices=['navigation', 'guidance', 'follow_user']
        ),
        DeclareLaunchArgument(
            "label",
            default_value=TextSubstitution(text=""),
            description="Optional label appended to bag filename",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value=TextSubstitution(text="testing_log"),
            description="Optional output directory for rosbag files",
        ),
        # OpaqueFunction Allows to get the argument value at runtime using Python
        OpaqueFunction(function=launch_setup),
    ])
