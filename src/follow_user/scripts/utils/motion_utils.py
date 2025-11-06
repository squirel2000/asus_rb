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

    def calculate_smooth_angular_velocity(self, angle_to_user, params):
        """
        Calculates a smooth, rate-limited angular velocity to face the user.
        """
        # 1. Implement deadband to prevent jitter
        if abs(angle_to_user) < math.radians(5.0):  # +/- 5 degrees
            target_angular_vel = 0.0
        else:
            # Simple proportional controller for target velocity
            target_angular_vel = 0.8 * angle_to_user

        # 2. Apply acceleration and velocity limits
        current_time = self._node.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Calculate the change in velocity based on acceleration limits
        max_vel_change = params['angular_acceleration'] * dt
        min_vel_change = -params['angular_deceleration'] * dt

        # Smoothly ramp the velocity
        if target_angular_vel > self.last_angular_vel:
            new_angular_vel = self.last_angular_vel + max_vel_change
            if new_angular_vel > target_angular_vel:
                new_angular_vel = target_angular_vel
        else:
            new_angular_vel = self.last_angular_vel + min_vel_change
            if new_angular_vel < target_angular_vel:
                new_angular_vel = target_angular_vel
        
        # Clamp to the absolute maximum velocity
        final_angular_vel = np.clip(new_angular_vel, -params['max_angular_vel'], params['max_angular_vel'])
        self.last_angular_vel = final_angular_vel
        
        return final_angular_vel

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
