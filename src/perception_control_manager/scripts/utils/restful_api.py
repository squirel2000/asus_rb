#!/usr/bin/env python3
import requests
from geometry_msgs.msg import PoseStamped

class RestfulAPI:
    def __init__(self, robot_ip, logger):
        self.base_url = f"http://{robot_ip}/api/core"
        self.logger = logger

    def create_navigation_action(self, pose: PoseStamped):
        """Sends a POST request to create a navigation action."""
        url = f"{self.base_url}/motion/v1/actions"
        payload = {
            "name": "move_to",
            "params": {
                "target": {
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y,
                    "z": 0.0  # Assuming 2D navigation
                },
                "movement_options": {
                    "final_orientation": {
                        "w": pose.pose.orientation.w,
                        "x": pose.pose.orientation.x,
                        "y": pose.pose.orientation.y,
                        "z": pose.pose.orientation.z
                    }
                }
            }
        }
        try:
            response = requests.post(url, json=payload, timeout=5)
            response.raise_for_status()
            return response.json().get("id")
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Error creating navigation action: {e}")
            return None

    def get_action_status(self, action_id):
        """Gets the status of a specific action by its ID."""
        url = f"{self.base_url}/motion/v1/actions/{action_id}"
        try:
            response = requests.get(url, timeout=2)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Error getting action status: {e}")
            return None

    def get_current_pose(self, clock):
        """Gets the current robot pose from the API."""
        url = f"{self.base_url}/slam/v1/localization/pose"
        try:
            response = requests.get(url, timeout=2)
            response.raise_for_status()
            data = response.json()

            pose_msg = PoseStamped()
            pose_msg.header.stamp = clock.now().to_msg()
            pose_msg.header.frame_id = "map"  # Or your relevant frame
            pose_msg.pose.position.x = data['x']
            pose_msg.pose.position.y = data['y']
            pose_msg.pose.position.z = data['z']
            pose_msg.pose.orientation.w = data['orientation']['w']
            pose_msg.pose.orientation.x = data['orientation']['x']
            pose_msg.pose.orientation.y = data['orientation']['y']
            pose_msg.pose.orientation.z = data['orientation']['z']
            return pose_msg
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Error getting current pose: {e}")
            return None
        except KeyError as e:
            self.logger.error(f"Malformed pose data received: {e}")
            return None

    def cancel_current_action(self):
        """Sends a DELETE request to cancel the current action."""
        url = f"{self.base_url}/motion/v1/actions/:current"
        try:
            response = requests.delete(url, timeout=5)
            response.raise_for_status()
            self.logger.info("Successfully sent cancel request to robot.")
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Error canceling action: {e}")
