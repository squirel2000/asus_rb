import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    This launch file starts the simulation environment.
    - Gazebo with a world
    - RViz
    """
    # Path to the turtlebot3_gazebo package
    turtlebot3_gazebo_pkg_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # Path to this package
    amr_sim_pkg_dir = get_package_share_directory('amr_sim')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='false')

    # --- Gazebo Simulation ---
    # We will include the turtlebot3_house.launch.py launch file
    # and set the model to 'waffle' as it includes a camera.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_pkg_dir, 'launch', 'turtlebot3_house.launch.py')
        ),
        launch_arguments={
            'model': 'waffle',
            'world': os.path.join(amr_sim_pkg_dir, 'worlds', 'amr_sim.world'),
            'x_pose': '-2.5',
            'y_pose': '-2.5',
            'gui': headless
        }.items(),
    )

    # --- RViz ---
    # Path to the custom RViz configuration file
    rviz_config_file = os.path.join(amr_sim_pkg_dir, 'rviz', 'amr_sim.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Whether to run Gazebo in headless mode.'
        ),
        gazebo_launch,
        rviz_node,
    ])