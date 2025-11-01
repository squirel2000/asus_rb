#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
import requests
import argparse

class PathSearchServerNode(Node):
    def __init__(self, robot_ip):
        super().__init__('path_search_server_node')
        self.robot_ip = robot_ip
        self.subscription = self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.path_publisher = self.create_publisher(Path, 'follow_user/planned_path', 10)

    def clicked_point_callback(self, msg):
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

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Call the SLAMTEC search_path API.")
    parser.add_argument("--robot-ip", required=True, help="The IP address of the robot.")
    args, _ = parser.parse_known_args()
    
    path_search_server_node = PathSearchServerNode(robot_ip=args.robot_ip)
    rclpy.spin(path_search_server_node)
    path_search_server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()