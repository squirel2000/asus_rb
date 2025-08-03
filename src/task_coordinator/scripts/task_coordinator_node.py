#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from follow_user.action import FollowUser
from navigation.action import Navigate
from std_srvs.srv import SetBool

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

        # Service to switch modes
        self.mode_service = self.create_service(SetBool, 'set_follow_mode', self.set_mode_callback)
        self.is_follow_mode = True  # Default to follow mode

        # Subscribers
        user_pose_topic = self.get_parameter('user_pose_topic').get_parameter_value().string_value
        self.user_pose_subscription = self.create_subscription(
            PoseStamped,
            user_pose_topic,
            self.user_pose_callback,
            10)
        
        self.follow_user_goal_handle = None
        self.navigate_goal_handle = None

        self.get_logger().info('Task Coordinator Node has been started.')

    def set_mode_callback(self, request, response):
        self.is_follow_mode = request.data
        response.success = True
        if self.is_follow_mode:
            response.message = "Switched to Follow User mode"
            self.get_logger().info("Switched to Follow User mode")
            if self.navigate_goal_handle:
                self.get_logger().info('Canceling navigation goal.')
                self.navigate_goal_handle.cancel_goal_async()
        else:
            response.message = "Switched to Navigate mode"
            self.get_logger().info("Switched to Navigate mode")
            if self.follow_user_goal_handle:
                self.get_logger().info('Canceling follow user goal.')
                self.follow_user_goal_handle.cancel_goal_async()
            self.send_navigation_goal()
        return response

    def user_pose_callback(self, msg):
        if self.is_follow_mode:
            self.send_follow_user_goal(msg)

    def send_follow_user_goal(self, pose):
        if not self.follow_user_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Follow User action server not available, waiting...')
            return

        if self.follow_user_goal_handle is not None and self.follow_user_goal_handle.is_active:
            self.get_logger().info('Follow User action is still active, not sending new goal.')
            return

        goal_msg = FollowUser.Goal()
        goal_msg.user_id = "user_1"
        goal_msg.user_pose = pose

        self.get_logger().info('Sending goal to Follow User action server...')
        send_goal_future = self.follow_user_client.send_goal_async(
            goal_msg,
            feedback_callback=self.follow_user_feedback_callback)
        send_goal_future.add_done_callback(self.follow_user_goal_response_callback)

    def send_navigation_goal(self):
        if not self.navigate_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Navigate action server not available, waiting...')
            return

        goal_msg = Navigate.Goal()
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.header.frame_id = "map"
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target_pose.pose.position.x = 1.0
        goal_msg.target_pose.pose.position.y = 1.0
        goal_msg.target_pose.pose.orientation.w = 1.0

        self.get_logger().info('Sending goal to Navigate action server...')
        send_goal_future = self.navigate_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_feedback_callback)
        send_goal_future.add_done_callback(self.navigate_goal_response_callback)

    # Follow User Callbacks
    def follow_user_goal_response_callback(self, future):
        self.follow_user_goal_handle = future.result()
        if not self.follow_user_goal_handle.accepted:
            self.get_logger().info('Follow User goal rejected')
            return
        self.get_logger().info('Follow User goal accepted')
        get_result_future = self.follow_user_goal_handle.get_result_async()
        get_result_future.add_done_callback(self.follow_user_result_callback)

    def follow_user_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Follow User result: {result.final_status}')
        self.follow_user_goal_handle = None

    def follow_user_feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Follow User feedback: Distance = {feedback_msg.feedback.current_distance:.2f}m')

    # Navigate Callbacks
    def navigate_goal_response_callback(self, future):
        self.navigate_goal_handle = future.result()
        if not self.navigate_goal_handle.accepted:
            self.get_logger().info('Navigate goal rejected')
            return
        self.get_logger().info('Navigate goal accepted')
        get_result_future = self.navigate_goal_handle.get_result_async()
        get_result_future.add_done_callback(self.navigate_result_callback)

    def navigate_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigate result: {result.success}')
        self.navigate_goal_handle = None

    def navigate_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigate feedback: Current pose is {feedback.current_pose.pose.position.x}, {feedback.current_pose.pose.position.y}')

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
