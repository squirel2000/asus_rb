#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.task import Future

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from navigation.action import Navigate
from perception_control_manager.srv import CreateNavigation, GetActionStatus

class NavigateActionServer(Node):
    """
    Action server to navigate the robot to a target pose.
    It communicates with the perception_control_manager node to execute the navigation.
    """
    def __init__(self):
        super().__init__('navigation_motion_node')
        
        # Use a reentrant callback group to allow for nested service calls and callbacks
        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        # Service clients to communicate with the perception_control_manager
        self.create_nav_client = self.create_client(CreateNavigation, 'create_navigation', callback_group=self.callback_group)
        self.get_status_client = self.create_client(GetActionStatus, 'get_action_status', callback_group=self.callback_group)
        self.cancel_action_client = self.create_client(Trigger, 'cancel_action', callback_group=self.callback_group)

        # Subscription to the current pose for feedback
        self.current_pose = None
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("Navigate to Pose Action Server has been started.")

    async def ros_async_sleep(self, seconds: float):
        """
        A ROS-compatible asynchronous sleep function that uses a one-shot timer.
        This is necessary because asyncio.sleep() requires an asyncio event loop.
        """
        future = Future()
        # Create a one-shot timer that sets the future's result when it expires.
        self.create_timer(seconds, lambda: future.set_result(None))
        # Await the future to be completed by the timer.
        await future
        
    def pose_callback(self, msg):
        """Callback to store the current pose."""
        self.current_pose = msg

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        # Service checks are still a good idea
        if not self.create_nav_client.wait_for_service(timeout_sec=1.0) or \
           not self.get_status_client.wait_for_service(timeout_sec=1.0) or \
           not self.cancel_action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('One or more required services are not available, rejecting goal.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """A new goal has been accepted."""
        self.get_logger().info('Goal accepted, starting execution.')
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Executes the navigation action by calling services on the perception_control_manager."""
        self.get_logger().info('Executing goal...')

        target_pose = goal_handle.request.target_pose
        feedback_msg = Navigate.Feedback()
        result = Navigate.Result()

        # 1. Create a navigation action via service
        create_req = CreateNavigation.Request()
        create_req.pose = target_pose
        try:
            response = await self.create_nav_client.call_async(create_req)
            if not response.success:
                self.get_logger().error(f"Failed to create navigation action. Reason: {response.action_id}")
                goal_handle.abort()
                result.success = False
                return result
        except Exception as e:
            self.get_logger().error(f"Service call to create_navigation failed: {e}")
            goal_handle.abort()
            result.success = False
            return result
        
        action_id = response.action_id
        self.get_logger().info(f"Navigation action created with ID: {action_id}")

        # 2. Monitor the action status
        status_req = GetActionStatus.Request()
        status_req.action_id = action_id

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                await self.cancel_action_client.call_async(Trigger.Request())
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
                result.success = False
                return result

            if self.current_pose:
                feedback_msg.current_pose = self.current_pose
                goal_handle.publish_feedback(feedback_msg)

            try:
                status_response = await self.get_status_client.call_async(status_req)
                if not status_response:
                    self.get_logger().warn("Could not get action status. Retrying...")
                    await self.ros_async_sleep(1.0) # USE ROS-NATIVE SLEEP
                    continue
            except Exception as e:
                self.get_logger().error(f"Service call to get_action_status failed: {e}")
                await self.ros_async_sleep(1.0) # USE ROS-NATIVE SLEEP
                continue
            
            current_state = status_response.status
            self.get_logger().info(f"Current action status: {current_state}")

            if current_state == 'succeeded':
                self.get_logger().info('Goal succeeded!')
                goal_handle.succeed()
                result.success = True
                return result
            elif current_state in ['failed', 'error', 'aborted']:
                self.get_logger().error(f"Goal failed with status: {current_state}")
                goal_handle.abort()
                result.success = False
                return result

            # Use non-blocking ROS-native sleep
            await self.ros_async_sleep(0.5)

        self.get_logger().info("RCLPY shutdown, aborting goal.")
        await self.cancel_action_client.call_async(Trigger.Request())
        goal_handle.abort()
        result.success = False
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = NavigateActionServer()
    # The executor needs to be added to the node and spun correctly.
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()