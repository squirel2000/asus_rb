import math
import numpy as np
import requests
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class MotionUtils:
    def __init__(self, node):
        self._node = node
        self.last_angular_vel = 0.0
        self.last_time = self._node.get_clock().now()

    def search_path(self, robot_ip, x, y, timeout=100):
        """
        Calls the external API to find a path to the target coordinates.
        """
        url = f"http://{robot_ip}:1448/api/core/motion/v1/:search_path"
        payload = {"target": {"x": x, "y": y}, "timeout": timeout}
        headers = {"Content-Type": "application/json"}
        try:
            response = requests.post(url, headers=headers, json=payload, timeout=timeout/1000 + 5)
            response.raise_for_status()
            return response.json().get("path_points")
        except requests.exceptions.RequestException as e:
            self._node.get_logger().error(f"Error calling search_path: {e}")
            return None

    def convert_points_to_path(self, points, frame_id):
        """
        Converts a list of points from the API into a nav_msgs/Path message.
        """
        path = Path()
        path.header.stamp = self._node.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        for point in points:
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            path.poses.append(pose)
        return path
