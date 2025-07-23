#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from follow_user.action import FollowUser

class BehaviorCoordinatorNode(Node):
    """
    This node coordinates the user-following behavior by acting as an action client.
    It subscribes to the user's pose and sends a goal to the action server.
    """
    def __init__(self):
        super().__init__('behavior_coordinator_node')
        self.get_logger().info('Behavior Coordinator Node has been started.')

        # Parameters
        self.declare_parameter('raw_pose_topic', '/user_tracker/target_pose')

        # Action Client
        self._action_client = ActionClient(self, FollowUser, 'follow_user')

        # Subscribers
        raw_pose_topic = self.get_parameter('raw_pose_topic').get_parameter_value().string_value
        self.raw_pose_subscription = self.create_subscription(
            PoseStamped,
            raw_pose_topic,
            self.raw_pose_callback,
            10)
        
        self.goal_handle = None

    def raw_pose_callback(self, msg):
        """
        Callback for the raw pose. Sends a goal to the action server.
        """
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action server not available, waiting...')
            return

        # If we already have a goal, do nothing
        if self.goal_handle is not None and self.goal_handle.is_active:
            self.get_logger().info('Action is still active, not sending new goal.')
            return

        goal_msg = FollowUser.Goal()
        goal_msg.user_id = "user_1"  # Placeholder
        goal_msg.user_pose = msg

        self.get_logger().info('Sending goal to action server...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.final_status}')
        self.goal_handle = None

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: Distance = {feedback_msg.feedback.current_distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
