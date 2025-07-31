#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import requests
import time
import threading

from geometry_msgs.msg import PoseStamped
from navigation.action import Navigate

# Configuration
ROBOT_IP = "192.168.11.1"  # IMPORTANT: Change this to your robot's IP address
BASE_URL = f"http://{ROBOT_IP}/api/core"

class NavigateActionServer(Node):
    """
    Action server to navigate the robot to a target pose.
    """
    def __init__(self):
        super().__init__('navigation_motion_node')
        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info("Navigate to Pose Action Server has been started.")

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """
        A new goal has been accepted.
        This function is called in the main thread, so we need to start a new thread for the execution.
        """
        self.get_logger().info('Goal accepted, starting execution in a new thread.')
        thread = threading.Thread(target=goal_handle.execute)
        thread.start()

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Executes the navigation action."""
        self.get_logger().info('Executing goal...')

        target_pose = goal_handle.request.target_pose
        feedback_msg = Navigate.Feedback()
        result = Navigate.Result()

        # 1. Create a navigation action via API
        action_id = self._create_navigation_action(target_pose)
        if not action_id:
            self.get_logger().error("Failed to create navigation action.")
            goal_handle.abort()
            result.success = False
            return result

        self.get_logger().info(f"Navigation action created with ID: {action_id}")

        # 2. Monitor the action status
        while rclpy.ok():
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self._cancel_current_action()
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
                result.success = False
                return result

            # Publish feedback (current pose)
            current_pose = self._get_current_pose()
            if current_pose:
                feedback_msg.current_pose = current_pose
                goal_handle.publish_feedback(feedback_msg)

            # Check action status
            status = self._get_action_status(action_id)
            if status is None:
                self.get_logger().warn("Could not get action status. Retrying...")
                time.sleep(1)
                continue
            
            self.get_logger().info(f"Current action status: {status['state']}")

            if status['state'] == 'succeeded':
                self.get_logger().info('Goal succeeded!')
                goal_handle.succeed()
                result.success = True
                return result
            elif status['state'] in ['failed', 'error', 'aborted']:
                self.get_logger().error(f"Goal failed with status: {status['state']}")
                goal_handle.abort()
                result.success = False
                return result

            time.sleep(0.5) # Poll every 500ms

        # If rclpy is shut down
        self.get_logger().info("RCLPY shutdown, aborting goal.")
        self._cancel_current_action()
        goal_handle.abort()
        result.success = False
        return result

    def _create_navigation_action(self, pose: PoseStamped):
        """Sends a POST request to create a navigation action."""
        url = f"{BASE_URL}/motion/v1/actions"
        payload = {
            "name": "move_to",
            "params": {
                "target": {
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y,
                    "z": 0.0 # Assuming 2D navigation
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
            self.get_logger().error(f"Error creating navigation action: {e}")
            return None

    def _get_action_status(self, action_id):
        """Gets the status of a specific action by its ID."""
        url = f"{BASE_URL}/motion/v1/actions/{action_id}"
        try:
            response = requests.get(url, timeout=2)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error getting action status: {e}")
            return None

    def _get_current_pose(self):
        """Gets the current robot pose from the API."""
        url = f"{BASE_URL}/slam/v1/localization/pose"
        try:
            response = requests.get(url, timeout=2)
            response.raise_for_status()
            data = response.json()
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map" # Or your relevant frame
            pose_msg.pose.position.x = data['x']
            pose_msg.pose.position.y = data['y']
            pose_msg.pose.position.z = data['z']
            pose_msg.pose.orientation.w = data['orientation']['w']
            pose_msg.pose.orientation.x = data['orientation']['x']
            pose_msg.pose.orientation.y = data['orientation']['y']
            pose_msg.pose.orientation.z = data['orientation']['z']
            return pose_msg
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error getting current pose: {e}")
            return None
        except KeyError as e:
            self.get_logger().error(f"Malformed pose data received: {e}")
            return None

    def _cancel_current_action(self):
        """Sends a DELETE request to cancel the current action."""
        url = f"{BASE_URL}/motion/v1/actions/:current"
        try:
            response = requests.delete(url, timeout=5)
            response.raise_for_status()
            self.get_logger().info("Successfully sent cancel request to robot.")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error canceling action: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    action_server = NavigateActionServer()
    
    # Use a MultiThreadedExecutor to handle multiple callbacks concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)
    
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
