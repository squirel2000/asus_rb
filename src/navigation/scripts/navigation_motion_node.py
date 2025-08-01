#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import time
import threading

from geometry_msgs.msg import PoseStamped
from navigation.action import Navigate
from utils.restful_api import RestfulAPI

# Configuration
ROBOT_IP = "192.168.11.1"  # IMPORTANT: Change this to your robot's IP address

class NavigateActionServer(Node):
    """
    Action server to navigate the robot to a target pose.
    """
    def __init__(self):
        super().__init__('navigation_motion_node')
        self.api = RestfulAPI(ROBOT_IP, self.get_logger())
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
        action_id = self.api.create_navigation_action(target_pose)
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
                self.api.cancel_current_action()
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
                result.success = False
                return result

            # Publish feedback (current pose)
            current_pose = self.api.get_current_pose(self.get_clock())
            if current_pose:
                feedback_msg.current_pose = current_pose
                goal_handle.publish_feedback(feedback_msg)

            # Check action status
            status = self.api.get_action_status(action_id)
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
        self.api.cancel_current_action()
        goal_handle.abort()
        result.success = False
        return result


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
