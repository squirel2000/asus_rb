#!/usr/bin/env python3
import rclpy
import argparse
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from task_coordinator.action import Task
from follow_user.action import FollowUser

class TaskClientNode(Node):
    def __init__(self, args):
        super().__init__('task_client_node')
        self.args = args
        self.get_result_future = None # Initialize the future
        if self.args.action == 'navigate':
            self.client = ActionClient(self, Task, 'task_server')
        elif self.args.action == 'follow':
            self.client = ActionClient(self, FollowUser, 'follow_user_task')

    def send_goal(self):
        if self.args.action == 'navigate':
            goal_msg = Task.Goal()
            goal_msg.target_pose = PoseStamped()
            goal_msg.target_pose.header.frame_id = "map"
            goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.target_pose.pose.position.x = self.args.x
            goal_msg.target_pose.pose.position.y = self.args.y
            goal_msg.target_pose.pose.orientation.w = self.args.w
        elif self.args.action == 'follow':
            goal_msg = FollowUser.Goal()
            goal_msg.user_id = self.args.user_id

        self.client.wait_for_server()

        self.send_goal_future = self.client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if self.args.action == 'navigate':
            self.get_logger().info(f'Result: {result.success}')
        elif self.args.action == 'follow':
            self.get_logger().info(f'Result: {result.final_status}')
        self.destroy_node()
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        if self.args.action == 'navigate':
            feedback = feedback_msg.feedback
            self.get_logger().info(f'Received feedback: {feedback.current_pose.pose.position.x}, {feedback.current_pose.pose.position.y}')
        elif self.args.action == 'follow':
            feedback = feedback_msg.feedback
            self.get_logger().info(f'Received feedback: {feedback.current_distance}')

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('action', type=str, default='navigate', help='Action to perform', choices=['navigate', 'follow'])
    parser.add_argument('--x', type=float, default=1.0, help='x position for navigation goal')
    parser.add_argument('--y', type=float, default=1.0, help='y position for navigation goal')
    parser.add_argument('--w', type=float, default=1.0, help='w orientation for navigation goal')
    parser.add_argument('--user_id', type=str, default='user_1', help='user id for follow goal')
    args = parser.parse_args()

    node = TaskClientNode(args)
    node.send_goal()
    rclpy.spin(node)

    # The node will be destroyed and rclpy shut down in get_result_callback

if __name__ == '__main__':
    main()
