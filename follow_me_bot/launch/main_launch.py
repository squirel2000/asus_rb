import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    This is the main launch file for the follow-me-bot project.
    It launches the following:
    1. The TurtleBot3 Gazebo simulation with the Waffle model in the TurtleBot3 house world.
    2. RViz with a configuration for TurtleBot3.
    3. The three nodes from the follow_me_bot package.
    """
    # Path to the turtlebot3_gazebo package
    turtlebot3_gazebo_pkg_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # Path to this package
    follow_me_bot_pkg_dir = get_package_share_directory('follow_me_bot')

    # --- Gazebo Simulation ---
    # We will include the turtlebot3_house.launch.py launch file
    # and set the model to 'waffle' as it includes a camera.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_pkg_dir, 'launch', 'turtlebot3_house.launch.py')
        ),  
        launch_arguments={'model': 'waffle'}.items(),
    )

    # --- RViz ---
    # Path to the custom RViz configuration file
    rviz_config_file = os.path.join(follow_me_bot_pkg_dir, 'rviz', 'follow_me.rviz')

    # The turtlebot3_bringup package is responsible for launching RViz.
    turtlebot3_bringup_pkg_dir = get_package_share_directory('turtlebot3_bringup')
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_bringup_pkg_dir, 'launch', 'rviz2.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true', 'rviz_config': rviz_config_file}.items(),
    )

    # --- Follow Me Bot Nodes ---
    # Include the launch file for our custom nodes.
    follow_me_bot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(follow_me_bot_pkg_dir, 'launch', 'follow_me_bot.launch.py')
        )
    )

    return LaunchDescription([
        gazebo_launch,
        rviz_launch,
        follow_me_bot_launch
    ])
