import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore
from rosbags.typesys.stores import Stores
import math

# Usage: python animate_pose_comparison.py <path_to_rosbag_directory>
# Example:
# python3 ./src/follow_user/scripts/animate_pose_comparison.py testing_log/20251101_234225_test_follow_user

def quaternion_to_yaw(q):
    """Convert quaternion to yaw angle (θ in radians)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
    return math.atan2(siny_cosp, cosy_cosp)

def load_data(bag_path):
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    odom_data = {'t': [], 'x': [], 'y': [], 'yaw': []}
    pose_data = {'t': [], 'x': [], 'y': [], 'yaw': []}
    lookahead_data = {'t': [], 'x': [], 'y': []}
    path_data = {'t': [], 'paths': []}
    target_velocity_data = {'t': [], 'v': []}
    current_velocity_data = {'t': [], 'v': []}
    cmd_vel_data = {'t': [], 'x': [], 'y': [], 'yaw': []}

    print(f"Reading bag: {bag_path}")
    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            t = timestamp / 1e9

            if connection.topic == '/slamware_ros_sdk_server_node/odom':
                odom_data['t'].append(t)
                odom_data['x'].append(msg.pose.pose.position.x)
                odom_data['y'].append(msg.pose.pose.position.y)
                odom_data['yaw'].append(quaternion_to_yaw(msg.pose.pose.orientation))
            elif connection.topic == '/robot_pose':
                pose_data['t'].append(t)
                pose_data['x'].append(msg.pose.position.x)
                pose_data['y'].append(msg.pose.position.y)
                pose_data['yaw'].append(quaternion_to_yaw(msg.pose.orientation))
            elif connection.topic == '/lookahead_point':
                lookahead_data['t'].append(t)
                lookahead_data['x'].append(msg.pose.position.x)
                lookahead_data['y'].append(msg.pose.position.y)
            elif connection.topic == '/follow_user/planned_path':
                path_data['t'].append(t)
                path = {'x': [], 'y': []}
                for pose_stamped in msg.poses:
                    path['x'].append(pose_stamped.pose.position.x)
                    path['y'].append(pose_stamped.pose.position.y)
                path_data['paths'].append(path)
            elif connection.topic == '/follow_user/target_velocity':
                target_velocity_data['t'].append(t)
                target_velocity_data['v'].append(msg.data)
            elif connection.topic == '/follow_user/current_velocity':
                current_velocity_data['t'].append(t)
                current_velocity_data['v'].append(msg.data)
            elif connection.topic == '/cmd_vel':
                cmd_vel_data['t'].append(t)
                cmd_vel_data['x'].append(msg.linear.x)
                cmd_vel_data['y'].append(msg.linear.y)
                cmd_vel_data['yaw'].append(msg.angular.z)

    return odom_data, pose_data, lookahead_data, path_data, target_velocity_data, current_velocity_data, cmd_vel_data

def animate_plots(odom, pose, lookahead, path, target_velocity, current_velocity, cmd_vel):
    fig, axs = plt.subplots(2, 2, figsize=(10, 10))
    ax1, ax2, ax3, ax4 = axs.flatten()

    # Subplot 1: Pose Animation
    ax1.set_title('Odom vs Robot Pose')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    all_x = odom['x'] + pose['x'] + lookahead['x']
    all_y = odom['y'] + pose['y'] + lookahead['y']
    for p in path['paths']:
        all_x.extend(p['x'])
        all_y.extend(p['y'])
    ax1.axis('equal')
    if all_x and all_y:
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        ax1.set_xlim(min_x - 1.0, max_x + 1.0)
        ax1.set_ylim(min_y - 1.0, max_y + 1.0)
    ax1.grid(True)
    odom_trace, = ax1.plot([], [], 'g.-', lw=1.5, label='Odom')
    pose_trace, = ax1.plot([], [], 'b.-', lw=1.5, label='Robot Pose')
    path_trace, = ax1.plot([], [], 'y--', lw=1.5, label='Planned Path')
    lookahead_point, = ax1.plot([], [], 'ro', markersize=8, label='Lookahead Point')
    ax1.legend()

    # Subplot 2: X-axis Velocity
    ax2.set_title('X-axis Velocity')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.grid(True)
    target_v_line, = ax2.plot([], [], 'r-', label='Target Velocity')
    current_v_line, = ax2.plot([], [], 'b-', label='Current Velocity')
    cmd_vel_x_line, = ax2.plot([], [], 'g-', label='Cmd Vel X')
    ax2.legend()

    # Subplot 3: Y-axis Velocity
    ax3.set_title('Y-axis Velocity')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity (m/s)')
    ax3.grid(True)
    cmd_vel_y_line, = ax3.plot([], [], 'g-', label='Cmd Vel Y')
    # Also show target and current velocity on Y-axis for comparison
    target_v_line_y, = ax3.plot([], [], 'r-', label='Target Velocity')
    current_v_line_y, = ax3.plot([], [], 'b-', label='Current Velocity')
    ax3.legend()

    # Subplot 4: Yaw Velocity
    ax4.set_title('Yaw Velocity')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Velocity (rad/s)')
    ax4.grid(True)
    cmd_vel_yaw_line, = ax4.plot([], [], 'g-', label='Cmd Vel Yaw')
    # Also show target and current velocity on Yaw for comparison
    target_v_line_yaw, = ax4.plot([], [], 'r-', label='Target Velocity')
    current_v_line_yaw, = ax4.plot([], [], 'b-', label='Current Velocity')
    ax4.legend()

    def init():
        # Init for Subplot 1
        odom_trace.set_data([], [])
        pose_trace.set_data([], [])
        path_trace.set_data([], [])
        lookahead_point.set_data([], [])
        # Init for Subplot 2
        target_v_line.set_data([], [])
        current_v_line.set_data([], [])
        cmd_vel_x_line.set_data([], [])
        # Init for Subplot 3
        cmd_vel_y_line.set_data([], [])
        target_v_line_y.set_data([], [])
        current_v_line_y.set_data([], [])
        # Init for Subplot 4
        cmd_vel_yaw_line.set_data([], [])
        target_v_line_yaw.set_data([], [])
        current_v_line_yaw.set_data([], [])
        return (odom_trace, pose_trace, path_trace, lookahead_point,
            target_v_line, current_v_line, cmd_vel_x_line,
            cmd_vel_y_line, target_v_line_y, current_v_line_y,
            cmd_vel_yaw_line, target_v_line_yaw, current_v_line_yaw)

    def update(i):
        current_time = odom['t'][i]

        # Update Subplot 1
        ax1.patches.clear()  # Clear previous arrows
        odom_trace.set_data(odom['x'][:i+1], odom['y'][:i+1])
        pose_trace.set_data(pose['x'][:i+1], pose['y'][:i+1])

        # Add odom arrow
        ox, oy, oyaw = odom['x'][i], odom['y'][i], odom['yaw'][i]
        ax1.arrow(ox, oy, 0.1*math.cos(oyaw), 0.1*math.sin(oyaw), head_width=0.02, fc='g', ec='g')

        # Add pose arrow
        px, py, pyaw = pose['x'][i], pose['y'][i], pose['yaw'][i]
        ax1.arrow(px, py, 0.1*math.cos(pyaw), 0.1*math.sin(pyaw), head_width=0.02, fc='b', ec='b')

        lookahead_idx = np.searchsorted(lookahead['t'], current_time, side='right') - 1
        if lookahead_idx >= 0:
            lookahead_point.set_data(lookahead['x'][lookahead_idx], lookahead['y'][lookahead_idx])
        path_idx = np.searchsorted(path['t'], current_time, side='right') - 1
        if path_idx >= 0:
            path_trace.set_data(path['paths'][path_idx]['x'], path['paths'][path_idx]['y'])

        # Update Subplot 2
        target_v_idx = np.searchsorted(target_velocity['t'], current_time, side='right')
        current_v_idx = np.searchsorted(current_velocity['t'], current_time, side='right')
        cmd_vel_idx = np.searchsorted(cmd_vel['t'], current_time, side='right')
        target_v_line.set_data(target_velocity['t'][:target_v_idx], target_velocity['v'][:target_v_idx])
        current_v_line.set_data(current_velocity['t'][:current_v_idx], current_velocity['v'][:current_v_idx])
        cmd_vel_x_line.set_data(cmd_vel['t'][:cmd_vel_idx], cmd_vel['x'][:cmd_vel_idx])
        ax2.relim()
        ax2.autoscale_view()

        # Update Subplot 3 (Y-axis)
        cmd_vel_y_line.set_data(cmd_vel['t'][:cmd_vel_idx], cmd_vel['y'][:cmd_vel_idx])
        target_v_line_y.set_data(target_velocity['t'][:target_v_idx], target_velocity['v'][:target_v_idx])
        current_v_line_y.set_data(current_velocity['t'][:current_v_idx], current_velocity['v'][:current_v_idx])
        ax3.relim()
        ax3.autoscale_view()

        # Update Subplot 4 (Yaw)
        cmd_vel_yaw_line.set_data(cmd_vel['t'][:cmd_vel_idx], cmd_vel['yaw'][:cmd_vel_idx])
        target_v_line_yaw.set_data(target_velocity['t'][:target_v_idx], target_velocity['v'][:target_v_idx])
        current_v_line_yaw.set_data(current_velocity['t'][:current_v_idx], current_velocity['v'][:current_v_idx])
        ax4.relim()
        ax4.autoscale_view()

        return (odom_trace, pose_trace, path_trace, lookahead_point,
            target_v_line, current_v_line, cmd_vel_x_line,
            cmd_vel_y_line, target_v_line_y, current_v_line_y,
            cmd_vel_yaw_line, target_v_line_yaw, current_v_line_yaw)

    num_frames = min(len(odom['t']), len(pose['t']))
    ani = FuncAnimation(fig, update, frames=num_frames, init_func=init,
                        interval=50, blit=False, repeat=False)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Animate pose and velocity data from a ROS2 bag file.")
    parser.add_argument('bag_path', type=str, help='Path to the rosbag2 directory')
    args = parser.parse_args()

    (odom_data, pose_data, lookahead_data, path_data,
     target_velocity_data, current_velocity_data, cmd_vel_data) = load_data(args.bag_path)

    if not odom_data['t'] or not pose_data['t']:
        print("Error: missing /odom or /robot_pose data.")
    else:
        animate_plots(odom_data, pose_data, lookahead_data, path_data,
                      target_velocity_data, current_velocity_data, cmd_vel_data)
