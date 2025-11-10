#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path, Odometry
import requests
import argparse
import math
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from utils.head_control import HeadController

# Constants
CAMERA_OFFSET = np.array([0.0, 0.0, 0.0]) # Camera position (x,y,z) in base_link frame

class PathSearchServerNode(Node):
    def __init__(self, robot_ip, control_head=False):
        super().__init__('path_search_server_node')
        self.robot_ip = robot_ip
        self.control_head = control_head
        self.subscription = self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.path_publisher = self.create_publisher(Path, 'follow_user/planned_path', 10)

        # New subscribers
        self.robot_pose_sub = self.create_subscription(PoseStamped, '/robot_pose', self.robot_pose_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/slamware_ros_sdk_server_node/odom', self.odom_callback, 10)

        self.robot_pose = None
        self.clicked_point = None

        # For head control
        if self.control_head:
            self.head_controller = HeadController(self.get_logger())
        else:
            self.head_controller = None
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def robot_pose_callback(self, msg):
        """
        Callback for receiving the robot's pose. Stores the pose and triggers head tracking.
        """
        self.robot_pose = msg
        # self.get_logger().info(f"Received /robot_pose: {msg.pose.position.x}, {msg.pose.position.y}")
        
        # If a target point from the user exists, update the head tracking logic.
        if self.clicked_point is not None:
            self._update_head_tracking()


    def odom_callback(self, msg):
        # self.get_logger().info(f"Received /slamware_ros_sdk_server_node/odom: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")
        pass

    def clicked_point_callback(self, msg):
        self.clicked_point = msg
        x = msg.point.x
        y = msg.point.y
        self.get_logger().info(f"Received /clicked_point: (x={x}, y={y})")
        path_points = self.search_path(x, y)
        if path_points:
            path_msg = self.convert_points_to_path(path_points, msg.header.frame_id)
            self.path_publisher.publish(path_msg)
            self.get_logger().info(f"Published follow_user/planned_path with {len(path_msg.poses)} points")

    def search_path(self, x, y, timeout=100):
        """
        Calls the /api/core/motion/v1/:search_path endpoint of the SLAMTEC API.
        """
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
            pose.pose.position.z = 0.0  # Assuming 2D path
            path.poses.append(pose)
        return path

    def _update_head_tracking(self):
        """
        Transforms the user's point to the robot's frame, calculates the required
        yaw and pitch for the camera, and sends control commands.
        """
        
        try:
            # Wait for the transform to be available from the user's frame to the robot's base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # Target frame
                self.clicked_point.header.frame_id,  # Source frame
                rclpy.time.Time()
            )

            # Transform the clicked point into the robot's base_link frame
            point_in_base_frame = do_transform_point(self.clicked_point, transform)

            # Calculate the vector from the camera's position to the target point
            point_vec = np.array([point_in_base_frame.point.x, point_in_base_frame.point.y, point_in_base_frame.point.z])
            vec_from_camera = point_vec - CAMERA_OFFSET

            # Calculate distance, yaw, and pitch from the camera's perspective
            x, y, z = vec_from_camera
            distance = math.sqrt(x**2 + y**2)
            yaw = math.atan2(y, x)
            pitch = math.atan2(z, math.sqrt(x**2 + y**2))

            self.get_logger().info(f"Rel. pose to user: (x={x:.2f}, y={y:.2f}, z={z:.2f}) -> Dist.: {distance:.2f}m, Yaw: {math.degrees(yaw):.2f}°, Pitch: {math.degrees(pitch):.2f}°")
            
            # If head control is enabled, send the command
            if self.head_controller:
                self.head_controller.control_head(yaw, pitch)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform point for head tracking: {e}')

    def destroy_node(self):
        if self.head_controller:
            self.head_controller.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Call the SLAMTEC search_path API and control robot head.")
    parser.add_argument("--robot-ip", required=True, help="The IP address of the robot.")
    parser.add_argument("--no-head-control", action="store_true", help="Disable head control.")
    args, _ = parser.parse_known_args()
    
    path_search_server_node = PathSearchServerNode(robot_ip=args.robot_ip, control_head=not args.no_head_control)
    rclpy.spin(path_search_server_node)
    path_search_server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
