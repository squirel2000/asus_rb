#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from motion_common.action import Guidance
import math
import argparse
import sys
import signal

def create_pose_stamped(node: Node, x, y, yaw_deg=None):
    """Helper function to create a PoseStamped message."""
    pose = PoseStamped()
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y

    q = Quaternion()

    if yaw_deg is not None:
        # Convert degrees to radians
        yaw = math.radians(yaw_deg)

        # Convert yaw to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(0)
        sp = math.sin(0)
        cr = math.cos(0)
        sr = math.sin(0)

        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
    else:
        # Default orientation
        q.w = 1.0
        q.x = q.y = q.z = 0.0

    pose.pose.orientation = q
    return pose


class GuidanceActionClient(Node):
    """Action client to test the guidance action server."""
    def __init__(self, align_yaw=False, nav_mode='', user_id="test", speed_ratio=1.0):
        super().__init__('guidance_action_client')
        self._action_client = ActionClient(self, Guidance, 'guide_user_to_pose')
        self._align_yaw = align_yaw
        self._nav_mode = nav_mode
        self._user_id = user_id
        self._speed_ratio = speed_ratio

        self._goal_handle = None
        self._cancel_requested = False

    def send_goal(self, pose):
        goal_msg = Guidance.Goal()
        goal_msg.target_pose = pose
        goal_msg.speed_ratio = self._speed_ratio
        goal_msg.user_id = self._user_id
        goal_msg.align_yaw = self._align_yaw
        goal_msg.nav_mode = self._nav_mode

        self.get_logger().info(
            f'Sending goal: align_yaw={self._align_yaw}, nav_mode={self._nav_mode}, '
            f'user_id="{self._user_id}", speed_ratio={self._speed_ratio}'
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
        self._goal_handle = goal_handle

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {{success: {result.success}, message: {result.message}}}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        pose = feedback_msg.feedback.current_pose
        status = feedback_msg.feedback.status
        self.get_logger().info(
            f'Feedback: Pose(x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}), Status: {status}'
        )

    def cancel_goal(self):
        """Attempt to cancel the active goal."""
        if self._cancel_requested:
            return
        self._cancel_requested = True

        if self._goal_handle is None:
            self.get_logger().warn('Goal handle not available yet — cannot cancel.')
            return

        self.get_logger().info('Attempting to cancel goal...')
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info('Goal successfully canceled.')
            else:
                self.get_logger().warn('Failed to cancel goal or goal already finished.')
        except Exception as e:
            self.get_logger().error(f'Cancel request failed: {e}')
        finally:
            self.get_logger().info('Shutting down...')
            rclpy.shutdown()

def main(args=None):
    parser = argparse.ArgumentParser(description='Send a guidance goal.')
    parser.add_argument('x', type=float, nargs='?', default=0.0, help='Target X coordinate')
    parser.add_argument('y', type=float, nargs='?', default=0.0, help='Target Y coordinate')
    parser.add_argument('--yaw', type=float, help='Yaw angle in degrees (optional)')
    parser.add_argument('--nav_mode', default='FREE',choices=['FREE','TRACK','HYBRID'], help='Determine navigation mode')
    parser.add_argument('--user_id', type=str, default='test', help='User ID string')
    parser.add_argument('--speed_ratio', type=float, default=1.0, help='Speed ratio (0.0~1.0)')

    parsed_args, _ = parser.parse_known_args(sys.argv[1:])

    align_yaw = parsed_args.yaw is not None
    nav_mode = parsed_args.nav_mode

    rclpy.init(args=args)
    action_client = GuidanceActionClient(
        align_yaw=align_yaw,
        nav_mode=nav_mode,
        user_id=parsed_args.user_id,
        speed_ratio=parsed_args.speed_ratio
    )

    # --- Ctrl+C handler ---
    def signal_handler(sig, frame):
        if rclpy.ok():
            action_client.get_logger().info('Ctrl+C detected, attempting to cancel goal...')
            action_client.cancel_goal()
        else:
            rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    target_pose = create_pose_stamped(
        action_client,
        parsed_args.x,
        parsed_args.y,
        parsed_args.yaw
    )

    action_client.send_goal(target_pose)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
