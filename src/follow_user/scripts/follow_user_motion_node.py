#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import requests
import argparse
import math
import numpy as np
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
from utils.head_control import HeadController
from follow_user.action import FollowUser

# Constants
CAMERA_OFFSET = np.array([0.0, 0.0, 0.0]) # Camera position (x,y,z) in base_link frame

class FollowUserMotionNode(Node):
    """
    This node controls the robot's motion to follow a user using an action server.
    It receives relative user pose, controls the head, calculates the user's absolute pose,
    searches for a path, and publishes it.
    """
    def __init__(self, robot_ip, control_head=False):
        super().__init__('follow_user_motion_node')
        self.get_logger().info('Follow User Motion Node has been started.')

        self.robot_ip = robot_ip
        self.control_head = control_head

        # Action Server
        self._action_server = ActionServer(
            self,
            FollowUser,
            'follow_user',
            self.execute_callback)
        
        # Publishers
        self.path_publisher = self.create_publisher(Path, 'follow_user/planned_path', 10)

        # Subscribers
        self.robot_pose_sub = self.create_subscription(PoseStamped, '/robot_pose', self.robot_pose_callback, 10)
        self.relative_pose_sub = self.create_subscription(PoseStamped, '/human_relative_pose_front', self.relative_pose_callback, 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Head Control
        if self.control_head:
            self.head_controller = HeadController(self.get_logger())
        else:
            self.head_controller = None

        # State
        self.robot_pose = None
        self.goal_handle = None

    def robot_pose_callback(self, msg):
        self.robot_pose = msg

    def relative_pose_callback(self, msg):
        
        # Print relative_pose_sub message
        self.get_logger().info(f'goal_handle: {self.goal_handle} and is_active: {self.goal_handle.is_active if self.goal_handle else "N/A"}')
        self.get_logger().info(f'Relative User Pose: {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}')


        if self.goal_handle is None or not self.goal_handle.is_active:
            return

        self._update_head_tracking(msg)

        # Check if the user is within the following distance
        x_rel, y_rel = msg.pose.position.x, msg.pose.position.y
        current_distance = math.sqrt(x_rel**2 + y_rel**2)
        following_distance = self.goal_handle.request.following_distance

        # TODO: Make the robot purely rotational if within following distance
        if current_distance < following_distance:
            self.get_logger().info(f"User is within following distance ({current_distance:.2f}m < {following_distance:.2f}m). Stopping motion.")
            # Publish an empty path to make the robot stop
            empty_path = Path()
            empty_path.header.stamp = self.get_clock().now().to_msg()
            empty_path.header.frame_id = 'slamware_map'
            self.path_publisher.publish(empty_path)
            return

        if self.robot_pose is None:
            self.get_logger().warn("No robot pose available to calculate absolute user pose.")
            return

        try:
            # To avoid extrapolation errors, we transform the pose at the latest available time
            # by setting the timestamp in the header to zero.
            msg.header.stamp = rclpy.time.Time().to_msg()
            absolute_user_pose = self.tf_buffer.transform(
                msg,
                'slamware_map',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            x = absolute_user_pose.pose.position.x
            y = absolute_user_pose.pose.position.y
            path_points = self.search_path(x, y)

            if path_points:
                path_msg = self.convert_points_to_path(path_points, 'slamware_map')
                self.path_publisher.publish(path_msg)
                self.get_logger().info(f"Published path with {len(path_msg.poses)} points to ({x:.2f}, {y:.2f})")

        except tf2_ros.TransformException as e:
            self.get_logger().error(f'Could not transform pose for path planning: {e}')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal for user: {goal_handle.request.user_id}')
        self.goal_handle = goal_handle

        # Print the goal handle information
        self.get_logger().info(f'goal_handle: {self.goal_handle} and is_active: {self.goal_handle.is_active if self.goal_handle else "N/A"}')

        while rclpy.ok() and self.goal_handle.is_active:
            if self.goal_handle.is_cancel_requested:
                self.goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                self.goal_handle = None
                return FollowUser.Result(final_status='Goal canceled')
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_handle.succeed()
        self.goal_handle = None
        return FollowUser.Result(final_status='Follow task finished.')

    def _update_head_tracking(self, relative_pose_msg):
        x, y, z = relative_pose_msg.pose.position.x, relative_pose_msg.pose.position.y, relative_pose_msg.pose.position.z
        point_vec = np.array([x, y, z])
        vec_from_camera = point_vec - CAMERA_OFFSET
        x_cam, y_cam, z_cam = vec_from_camera
        distance = math.sqrt(x_cam**2 + y_cam**2)
        yaw = math.atan2(y_cam, x_cam)
        pitch = math.atan2(z_cam, math.sqrt(x_cam**2 + y_cam**2))

        self.get_logger().info(f"Head tracking -> Dist: {distance:.2f}m, Yaw: {math.degrees(yaw):.2f}°, Pitch: {math.degrees(pitch):.2f}°")

        feedback_msg = FollowUser.Feedback()
        feedback_msg.current_user_distance = distance
        feedback_msg.status = "Following user..."
        if self.robot_pose:
            feedback_msg.current_pose = self.robot_pose

        if self.goal_handle and self.goal_handle.is_active:
            self.goal_handle.publish_feedback(feedback_msg)

        if self.head_controller:
            self.head_controller.control_head(yaw, pitch)

    def search_path(self, x, y, timeout=100):
        url = f"http://{self.robot_ip}:1448/api/core/motion/v1/:search_path"
        payload = {"target": {"x": x, "y": y}, "timeout": timeout}
        headers = {"Content-Type": "application/json"}
        try:
            response = requests.post(url, headers=headers, json=payload, timeout=timeout/1000 + 5)
            response.raise_for_status()
            return response.json().get("path_points")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error calling search_path: {e}")
            return None

    def convert_points_to_path(self, points, frame_id):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        for point in points:
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            path.poses.append(pose)
        return path

    def destroy_node(self):
        if self.head_controller:
            self.head_controller.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Follow user motion node with path planning and head control.")
    parser.add_argument("--robot-ip", required=True, help="The IP address of the robot.")
    parser.add_argument("--no-head-control", action="store_true", help="Disable head control.")
    args, _ = parser.parse_known_args()
    
    node = FollowUserMotionNode(robot_ip=args.robot_ip, control_head=not args.no_head_control)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
