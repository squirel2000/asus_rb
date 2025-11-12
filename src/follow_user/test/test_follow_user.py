#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from motion_common.action import FollowUser
import argparse
import sys
import signal


class FollowUserActionClient(Node):
    """Action client to test the follow_user action server."""
    def __init__(self, user_id="test", following_dis=1.5, cancel_after=0.0):
        super().__init__('follow_user_action_client')
        self._action_client = ActionClient(self, FollowUser, 'follow_user')
        self._user_id = user_id
        self._following_dis = following_dis
        self._cancel_after = cancel_after
        self._goal_handle = None
        self._cancel_requested = False

    def send_goal(self):
        goal_msg = FollowUser.Goal()
        goal_msg.user_id = self._user_id
        goal_msg.following_distance = self._following_dis

        self.get_logger().info(
            f'Sending goal: user_id={self._user_id}, following_distance={self._following_dis}'
        )

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted :)')
        self._goal_handle = goal_handle

        # Schedule automatic cancel after N seconds
        if self._cancel_after > 0:
            self.get_logger().info(f'Scheduling auto-cancel in {self._cancel_after} seconds...')
            self.create_timer(self._cancel_after, self.cancel_goal)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {{message: {result.message}}}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        pose = feedback_msg.feedback.current_pose
        status = feedback_msg.feedback.status
        current_distance = feedback_msg.feedback.current_user_distance
        self.get_logger().info(
            f'Feedback: Pose(x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}), '
            f'Status: {status}, Distance: {current_distance:.2f}'
        )

    def cancel_goal(self):
        """Attempt to cancel the active goal."""
        if self._goal_handle is None:
            self.get_logger().warn('No active goal to cancel.')
            return

        if self._cancel_requested:
            return  # Prevent duplicate cancel requests

        self._cancel_requested = True
        self.get_logger().info('Cancel request sent to action server...')
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled.')
        else:
            self.get_logger().warn('Failed to cancel goal or goal already finished.')

def main(args=None):
    parser = argparse.ArgumentParser(description='Send a follow_user goal.')
    parser.add_argument('--user_id', type=str, default='test', help='User ID string')
    parser.add_argument('--follow_dis', type=float, default=1.5, help='Maintaining following distance')
    parser.add_argument('--cancel_after', type=float, default=0.0, help='Auto-cancel after N seconds (default=5)')

    parsed_args, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=args)
    action_client = FollowUserActionClient(
        user_id=parsed_args.user_id,
        following_dis=parsed_args.follow_dis,
        cancel_after=parsed_args.cancel_after
    )

    # --- Ctrl+C handler ---
    def signal_handler(sig, frame):
        if rclpy.ok():
            action_client.get_logger().info('Ctrl+C detected, attempting to cancel goal...')
            action_client.cancel_goal()
        else:
            rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
