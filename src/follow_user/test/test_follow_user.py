#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from motion_common.action import FollowUser
import math
import argparse
import sys


class FollowUserActionClient(Node):
    """Action client to test the guidance action server."""
    def __init__(self, user_id="test", following_dis=1.5):
        super().__init__('follow_user_action_client')
        self._action_client = ActionClient(self, FollowUser, 'follow_user')
        self._user_id = user_id
        self._following_dis = following_dis

    def send_goal(self):
        goal_msg = FollowUser.Goal()
        goal_msg.user_id = self._user_id
        goal_msg.following_distance = self._following_dis


        self.get_logger().info(
            f'Sending goal: user_id={self._user_id}, '
            f'following_distance="{self._following_dis}"'
        )

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal request...')
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
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {{success: {result.success}, message: {result.message}}}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        pose = feedback_msg.feedback.current_pose
        status = feedback_msg.feedback.status
        current_distance = feedback_msg.feedback.current_user_distance
        self.get_logger().info(
            f'Feedback: Pose(x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}), Status: {status}, Current Distance: {current_distance}'
        )


def main(args=None):
    parser = argparse.ArgumentParser(description='Send a guidance goal.')
    parser.add_argument('--user_id', type=str, default='test', help='User ID string')
    parser.add_argument('--follow_dis', type=float, default=1.5, help='Maintaining following distance')

    parsed_args, unknown = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=args)
    action_client = FollowUserActionClient(
        user_id=parsed_args.user_id,
        following_dis=parsed_args.follow_dis
    )

    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
