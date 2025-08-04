#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from geometry_msgs.msg import PoseStamped
from follow_user.action import FollowUser
from navigation.action import Navigate
from task_coordinator.action import Task

class TaskCoordinatorNode(Node):
    """
    Coordinates between following a user and navigating to a pose.
    """
    def __init__(self):
        super().__init__('task_coordinator_node')

        # Parameters
        self.declare_parameter('user_pose_topic', '/user_tracker/target_pose')

        # Action Clients
        self.follow_user_client = ActionClient(self, FollowUser, 'follow_user')
        self.navigate_client = ActionClient(self, Navigate, 'navigate_to_pose')

        # Action Servers
        self.task_server = ActionServer(self, Task, 'task_server', self.execute_task_callback)
        self.follow_user_server = ActionServer(self, FollowUser, 'follow_user_task', self.execute_follow_user_callback)

        # Subscribers
        user_pose_topic = self.get_parameter('user_pose_topic').get_parameter_value().string_value
        self.user_pose_subscription = self.create_subscription(
            PoseStamped,
            user_pose_topic,
            self.user_pose_callback,
            10)
        
        self.follow_user_goal_handle = None
        self.navigate_goal_handle = None
        self.current_user_pose = None
        self.is_follow_mode = True  # Default to follow mode

        self.get_logger().info('Task Coordinator Node has been started.')

    def user_pose_callback(self, msg):
        self.current_user_pose = msg
        if self.is_follow_mode and self.follow_user_goal_handle and self.follow_user_goal_handle.is_active:
            self.send_follow_user_goal(self.current_user_pose)

    def execute_task_callback(self, goal_handle):
        self.get_logger().info('Executing navigation task...')
        self.is_follow_mode = False
        if self.follow_user_goal_handle and self.follow_user_goal_handle.is_active:
            self.get_logger().info('Canceling follow user goal.')
            self.follow_user_goal_handle.cancel_goal_async()

        self.send_navigation_goal(goal_handle)

    def execute_follow_user_callback(self, goal_handle):
        self.get_logger().info('Executing follow user task...')
        self.is_follow_mode = True
        if self.navigate_goal_handle and self.navigate_goal_handle.is_active:
            self.get_logger().info('Canceling navigation goal.')
            self.navigate_goal_handle.cancel_goal_async()

        if self.current_user_pose is None:
            self.get_logger().info('User pose not available, waiting...')
            goal_handle.abort()
            return FollowUser.Result()

        self.follow_user_goal_handle = goal_handle
        self.send_follow_user_goal(self.current_user_pose)
        
        # Keep the goal active
        while rclpy.ok() and self.follow_user_goal_handle.is_active:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return FollowUser.Result()


    def send_follow_user_goal(self, pose):
        if not self.follow_user_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Follow User action server not available, waiting...')
            return

        goal_msg = FollowUser.Goal()
        goal_msg.user_id = "user_1"  # This can be dynamic based on the goal
        goal_msg.user_pose = pose

        self.get_logger().info('Sending goal to Follow User action server...')
        send_goal_future = self.follow_user_client.send_goal_async(
            goal_msg,
            feedback_callback=self.follow_user_feedback_callback)
        send_goal_future.add_done_callback(self.follow_user_goal_response_callback)

    def send_navigation_goal(self, goal_handle):
        if not self.navigate_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Navigate action server not available, waiting...')
            goal_handle.abort()
            return Task.Result()

        goal_msg = Navigate.Goal()
        goal_msg.target_pose = goal_handle.request.target_pose

        self.get_logger().info('Sending goal to Navigate action server...')
        send_goal_future = self.navigate_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_feedback_callback)
        send_goal_future.add_done_callback(lambda future: self.navigate_goal_response_callback(future, goal_handle))

    # Follow User Callbacks
    def follow_user_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Follow User goal rejected')
            if self.follow_user_goal_handle:
                self.follow_user_goal_handle.abort()
            return
        self.get_logger().info('Follow User goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.follow_user_result_callback)

    def follow_user_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Follow User result: {result.final_status}')
        if self.follow_user_goal_handle:
            if result.final_status == "Success":
                self.follow_user_goal_handle.succeed()
            else:
                self.follow_user_goal_handle.abort()
        self.follow_user_goal_handle = None

    def follow_user_feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Follow User feedback: Distance = {feedback_msg.feedback.current_distance:.2f}m')
        if self.follow_user_goal_handle:
            feedback = FollowUser.Feedback()
            feedback.current_distance = feedback_msg.feedback.current_distance
            self.follow_user_goal_handle.publish_feedback(feedback)


    # Navigate Callbacks
    def navigate_goal_response_callback(self, future, goal_handle):
        self.navigate_goal_handle = future.result()
        if not self.navigate_goal_handle.accepted:
            self.get_logger().info('Navigate goal rejected')
            goal_handle.abort()
            return
        self.get_logger().info('Navigate goal accepted')
        get_result_future = self.navigate_goal_handle.get_result_async()
        get_result_future.add_done_callback(lambda future: self.navigate_result_callback(future, goal_handle))

    def navigate_result_callback(self, future, goal_handle):
        result = future.result().result
        self.get_logger().info(f'Navigate result: {result.success}')
        self.navigate_goal_handle = None
        result_msg = Task.Result()
        result_msg.success = result.success
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

    def navigate_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigate feedback: Current pose is {feedback.current_pose.pose.position.x}, {feedback.current_pose.pose.position.y}')
        feedback_message = Task.Feedback()
        feedback_message.current_pose = feedback.current_pose
        # goal_handle.publish_feedback(feedback_message)

def main(args=None):
    rclpy.init(args=args)
    node = TaskCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
