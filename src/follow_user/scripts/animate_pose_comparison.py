import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore
from rosbags.typesys.stores import Stores
import math

def quaternion_to_yaw(q):
    """Convert quaternion to yaw angle (θ in radians)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
    return math.atan2(siny_cosp, cosy_cosp)

def load_pose_data(bag_path):
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    odom_data = {'t': [], 'x': [], 'y': [], 'yaw': []}
    pose_data = {'t': [], 'x': [], 'y': [], 'yaw': []}

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

    return odom_data, pose_data


def animate_pose(odom, pose):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title('Odom vs Robot Pose (x-y-θ)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.axis('equal')
    ax.grid(True)

    # plot traces
    odom_trace, = ax.plot([], [], 'g-', lw=1.5, label='Odom')
    pose_trace, = ax.plot([], [], 'b-', lw=1.5, label='Robot Pose')

    # heading arrows
    odom_arrow = ax.arrow(0, 0, 0, 0, head_width=0.1, fc='g', ec='g')
    pose_arrow = ax.arrow(0, 0, 0, 0, head_width=0.1, fc='b', ec='b')

    ax.legend()

    def init():
        odom_trace.set_data([], [])
        pose_trace.set_data([], [])
        return odom_trace, pose_trace, odom_arrow, pose_arrow

    def update(i):
        ax.patches.clear()  # remove old arrows

        # update odom trajectory and heading
        odom_trace.set_data(odom['x'][:i], odom['y'][:i])
        ox, oy, oyaw = odom['x'][i], odom['y'][i], odom['yaw'][i]
        odom_arrow = ax.arrow(ox, oy, 0.3*math.cos(oyaw), 0.3*math.sin(oyaw), 
                              head_width=0.1, fc='g', ec='g')

        # update robot_pose trajectory and heading
        pose_trace.set_data(pose['x'][:i], pose['y'][:i])
        px, py, pyaw = pose['x'][i], pose['y'][i], pose['yaw'][i]
        pose_arrow = ax.arrow(px, py, 0.3*math.cos(pyaw), 0.3*math.sin(pyaw),
                              head_width=0.1, fc='b', ec='b')

        return odom_trace, pose_trace, odom_arrow, pose_arrow

    num_frames = min(len(odom['x']), len(pose['x']))
    ani = FuncAnimation(fig, update, frames=num_frames, init_func=init,
                        interval=100, blit=False, repeat=False)
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Animate /odom and /robot_pose from a ROS2 bag file.")
    parser.add_argument('bag_path', type=str, help='Path to the rosbag2 directory (e.g., ./my_bag/)')
    args = parser.parse_args()

    odom_data, pose_data = load_pose_data(args.bag_path)
    if not odom_data['x'] or not pose_data['x']:
        print("Error: missing /odom or /robot_pose data.")
    else:
        animate_pose(odom_data, pose_data)
