# analyze_rosbag_data.py
import argparse
import matplotlib.pyplot as plt
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore
from rosbags.typesys.stores import Stores

# Usage: python analyze_rosbag_data.py <path_to_rosbag_directory>
# Example:
# python3 src/follow_user/scripts/analyze_rosbag.py testing_log/20251101_205205_test_follow_user_follow_user/

def plot_bag_data(bag_path):
    """
    Reads a ROS2 bag file and plots relevant topics for the follow_user task.
    """
    cmd_vel_data = {'t': [], 'linear_x': [], 'angular_z': []}
    odom_data = {'t': [], 'x': [], 'y': []}
    lookahead_data = {'t': [], 'x': [], 'y': []}

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    print(f"Reading bag file: {bag_path}...")
    try:
        with Reader(bag_path) as reader:
            # Iterate through connections (topics) in the bag
            for connection in reader.connections:
                print(f"  - Found topic: {connection.topic} ({connection.msgtype})")

            # Let the reader deserialize messages automatically
            for connection, timestamp, rawdata in reader.messages():
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                
                if connection.topic == '/cmd_vel':
                    cmd_vel_data['t'].append(timestamp / 1e9)
                    cmd_vel_data['linear_x'].append(msg.linear.x)
                    cmd_vel_data['angular_z'].append(msg.angular.z)
                
                elif connection.topic == '/slamware_ros_sdk_server_node/odom':
                    odom_data['t'].append(timestamp / 1e9)
                    odom_data['x'].append(msg.pose.pose.position.x)
                    odom_data['y'].append(msg.pose.pose.position.y)

                elif connection.topic == '/lookahead_point':
                    lookahead_data['t'].append(timestamp / 1e9)
                    lookahead_data['x'].append(msg.pose.position.x)
                    lookahead_data['y'].append(msg.pose.position.y)

    except Exception as e:
        print(f"Error reading bag file: {e}")
        return

    if not odom_data['t']:
        print("No odometry data found. Cannot create plots.")
        return
        
    print("Data read successfully. Generating plots...")

    # Normalize timestamps to start from 0
    start_time = min(cmd_vel_data['t'][0] if cmd_vel_data['t'] else float('inf'), 
                     odom_data['t'][0] if odom_data['t'] else float('inf'))
    
    if cmd_vel_data['t']:
        cmd_vel_data['t'] = [t - start_time for t in cmd_vel_data['t']]
    if odom_data['t']:
        odom_data['t'] = [t - start_time for t in odom_data['t']]
    if lookahead_data['t']:
        lookahead_data['t'] = [t - start_time for t in lookahead_data['t']]

    # Create plots
    fig, axs = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('Follow-User Bag Analysis', fontsize=16)

    # Plot 1: Velocity Commands
    if cmd_vel_data['t']:
        axs[0].plot(cmd_vel_data['t'], cmd_vel_data['linear_x'], label='Linear Velocity (m/s)', color='b')
        ax0_twin = axs[0].twinx()
        ax0_twin.plot(cmd_vel_data['t'], cmd_vel_data['angular_z'], label='Angular Velocity (rad/s)', color='r', linestyle='--')
        axs[0].set_ylabel('Linear Velocity (m/s)', color='b')
        ax0_twin.set_ylabel('Angular Velocity (rad/s)', color='r')
        axs[0].set_title('Command Velocity (/cmd_vel)')
        axs[0].legend(loc='upper left')
        ax0_twin.legend(loc='upper right')
        axs[0].grid(True)

    # Plot 2: Robot Path and Lookahead Points
    axs[1].plot(odom_data['x'], odom_data['y'], label='Robot Path (/odom)', color='g')
    if lookahead_data['t']:
        axs[1].scatter(lookahead_data['x'], lookahead_data['y'], label='Lookahead Points', color='purple', s=10, alpha=0.6)
    axs[1].set_xlabel('X Position (m)')
    axs[1].set_ylabel('Y Position (m)')
    axs[1].set_title('Robot Trajectory')
    axs[1].legend()
    axs[1].grid(True)
    axs[1].set_aspect('equal', adjustable='box')

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Plot data from a ROS2 bag file.")
    parser.add_argument('bag_path', type=str, help='Path to the directory of the rosbag file (e.g., testing_log/my_bag_dir).')
    args = parser.parse_args()
    
    plot_bag_data(args.bag_path)
