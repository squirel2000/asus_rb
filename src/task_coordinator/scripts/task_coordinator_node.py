#!/usr/bin/env python3
import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
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

        # Action Clients
        self.follow_user_client = ActionClient(self, FollowUser, 'follow_user')
        self.navigate_client = ActionClient(self, Navigate, 'navigate_to_pose')

        # Action Servers
        self.task_server = ActionServer(self, Task, 'task_server', self.execute_task_callback)
        self.follow_user_server = ActionServer(self, FollowUser, 'follow_user_task', self.execute_follow_user_callback)

        # Publishers
        self.human_relative_pose_publisher = self.create_publisher(PoseStamped, '/human_relative_pose_front', 10)

        # Subscribers
        self.user_pose_subscription = self.create_subscription(PoseStamped, '/human_relative_pose_front_raw', self.human_relative_pose_callback, 10)

        # State variables        
        self.follow_user_goal_handle = None
        self.navigate_goal_handle = None
        self.current_user_pose = None
        self.is_follow_mode = True  # Default to follow mode
        self.follow_user_goal_accepted = False

        # Threading event to signal completion of navigation
        self.nav_done_event = threading.Event()
        self.user_pose_event = threading.Event()
        self.follow_user_done_event = threading.Event()
        self.follow_user_result = FollowUser.Result()
        self.nav_success_result = False

        # Keep a reference to the future for getting the result
        self.navigate_get_result_future = None
        self.follow_user_get_result_future = None

        self.get_logger().info('Task Coordinator Node has been started.')

    def human_relative_pose_callback(self, msg):
        # TODO: Add any processing if needed
        if self.is_follow_mode and self.follow_user_goal_accepted:
            self.human_relative_pose_publisher.publish(msg)
        self.current_user_pose = msg
        self.user_pose_event.set()  # Signal that a new pose has been received

    def execute_task_callback(self, goal_handle):
        self.get_logger().info('Executing navigation task...')
        self.is_follow_mode = False
        if self.follow_user_goal_handle and self.follow_user_goal_handle.is_active:
            self.get_logger().info('Canceling follow user goal.')
            cancel_future = self.follow_user_goal_handle.cancel_goal_async()

        # Reset the event and result before starting a new navigation task
        self.nav_done_event.clear()
        self.nav_success_result = False

        self.send_navigation_goal(goal_handle)

        # Wait until the navigation action is complete
        self.get_logger().info('Waiting for navigation to complete...')
        self.nav_done_event.wait()
        self.get_logger().info('Navigation completed.')

        # Now that navigation is done, we can set the result for the task goal
        result = Task.Result()
        result.success = self.nav_success_result
        
        if result.success:
            self.get_logger().info('Attempting to succeed task goal.')
            goal_handle.succeed()
            self.get_logger().info('Task goal succeeded called.')
        else:
            self.get_logger().info('Attempting to abort task goal.')
            goal_handle.abort()
            self.get_logger().info('Task goal aborted called.')
            
        # # Add a small delay to allow the result message to be sent
        # import time
        # time.sleep(0.1)
        self.get_logger().info('Returning from execute_task_callback.')
        return result

    def execute_follow_user_callback(self, goal_handle):
        self.get_logger().info('Executing follow user task...')
        self.is_follow_mode = True
        if self.navigate_goal_handle and self.navigate_goal_handle.is_active:
            self.get_logger().info('Canceling navigation goal.')
            self.navigate_goal_handle.cancel_goal_async()

        if self.current_user_pose is None:
            self.get_logger().info('User pose not available, waiting...')
            self.user_pose_event.clear()
            self.user_pose_event.wait()

        self.follow_user_goal_handle = goal_handle
        
        # Extract parameters from the incoming goal request
        user_id = goal_handle.request.user_id
        following_distance = goal_handle.request.following_distance
        self.send_follow_user_goal(user_id, following_distance)
        
        # Wait for the goal to be done
        self.follow_user_done_event.clear()
        self.follow_user_done_event.wait()

        return self.follow_user_result


    def send_follow_user_goal(self, user_id, following_distance):
        if not self.follow_user_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Follow User action server not available, waiting...')
            return

        goal_msg = FollowUser.Goal()
        goal_msg.user_id = user_id
        goal_msg.following_distance = following_distance

        self.get_logger().info(f'Sending goal to Follow User action server for user: {user_id} at {following_distance:.2f}m...')
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
        goal_msg.speed_ratio = 1.0
        self.get_logger().info('Sending goal to Navigate action server...')
        send_goal_future = self.navigate_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg: self.navigate_feedback_callback(feedback_msg, goal_handle))
        # The lambda passes the original task goal handle to the response callback
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
        self.follow_user_goal_accepted = True
        self.follow_user_get_result_future = goal_handle.get_result_async()
        self.follow_user_get_result_future.add_done_callback(self.follow_user_result_callback)

    def follow_user_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Follow User result: {result.final_status}')
        self.follow_user_result = result
        self.follow_user_done_event.set()
        self.follow_user_goal_accepted = False
        if self.follow_user_goal_handle:
            if result.final_status == "Success":
                self.follow_user_goal_handle.succeed()
            else:
                self.follow_user_goal_handle.abort()
        self.follow_user_goal_handle = None

    def follow_user_feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Follow User feedback: Distance = {feedback_msg.feedback.current_user_distance:.2f}m')
        if self.follow_user_goal_handle:
            feedback = FollowUser.Feedback()
            feedback.current_user_distance = feedback_msg.feedback.current_user_distance
            self.follow_user_goal_handle.publish_feedback(feedback)


    # Navigate Callbacks
    def navigate_goal_response_callback(self, future, goal_handle):
        self.navigate_goal_handle = future.result()
        if not self.navigate_goal_handle.accepted:
            self.get_logger().info('Navigate goal rejected')
            # Set the event to unblock the execute callback
            self.nav_success_result = False
            self.nav_done_event.set()
            return
        self.get_logger().info('Navigate goal accepted')
        self.navigate_get_result_future = self.navigate_goal_handle.get_result_async()
        # The lambda passes the original task goal handle to the result callback
        self.navigate_get_result_future.add_done_callback(lambda future: self.navigate_result_callback(future, goal_handle))

    def navigate_result_callback(self, future, goal_handle):
        result = future.result().result
        self.get_logger().info(f'Navigate result: {result.success}')
        self.navigate_goal_handle = None
        
        # Store the result and set the event to unblock the execute callback
        self.nav_success_result = result.success
        self.get_logger().info(f'Setting nav_done_event for result: {self.nav_success_result}')
        self.nav_done_event.set()

    def navigate_feedback_callback(self, feedback_msg, goal_handle):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigate feedback: Current pose is {feedback.current_pose.pose.position.x}, {feedback.current_pose.pose.position.y}')
        # Publish feedback to the original task goal handle
        feedback_message = Task.Feedback()
        feedback_message.current_pose = feedback.current_pose
        goal_handle.publish_feedback(feedback_message) # This needs the original goal_handle

def main(args=None):
    rclpy.init(args=args)
    node = TaskCoordinatorNode()
    # Use a MultiThreadedExecutor to handle callbacks in parallel, which is crucial
    # for waiting for action results without blocking the entire system.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Important to shutdown the executor and destroy the node
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
