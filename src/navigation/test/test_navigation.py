#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from navigation.action import Navigate
import math

def create_pose_stamped(node: Node, x, y, yaw):
    """Helper function to create a PoseStamped message."""
    pose = PoseStamped()
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    
    # Convert yaw to quaternion
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(0)
    sp = math.sin(0)
    cr = math.cos(0)
    sr = math.sin(0)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    pose.pose.orientation = q
    
    return pose

class NavigateActionClient(Node):
    """
    Action client to test the navigation action server.
    """
    def __init__(self):
        super().__init__('navigate_action_client')
        self._action_client = ActionClient(self, Navigate, 'navigate_to_pose')

    def send_goal(self, pose):
        """Sends a navigation goal to the action server."""
        goal_msg = Navigate.Goal()
        goal_msg.target_pose = pose

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback for when the goal is accepted or rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback for when the action is finished."""
        result = future.result().result
        self.get_logger().info(f'Result: {{success: {result.success}}}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """Callback for receiving feedback from the action server."""
        pose = feedback_msg.feedback.current_pose
        self.get_logger().info(
            f'Received feedback: Current Pose: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    
    action_client = NavigateActionClient()
    
    # Create a sample goal pose (e.g., x=1.0, y=2.0, yaw=90 degrees)
    # IMPORTANT: Change these coordinates to a valid location in your map
    target_pose = create_pose_stamped(action_client, 1.0, 2.0, math.pi / 2.0)
    
    action_client.send_goal(target_pose)
    
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
