#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.task import Future

from geometry_msgs.msg import PoseStamped
from navigation.action import Navigate
from slamware_ros_sdk.msg import MoveToRequest, CancelActionRequest
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Path

import ast

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

        # Subscription to the current pose for feedback
        self.current_pose = None
        self.pose_subscriber = self.create_subscription(PoseStamped, '/robot_pose', self.current_pose_callback, 10, callback_group=self.callback_group)
        
        # Subscription to the AMR's remaining target points
        self.remaining_targets: list[list] = []
        self.remaining_targets_subscriber = self.create_subscription(Float32MultiArray, '/remaining_targets', self.remaining_targets_callback, 10, callback_group=self.callback_group)
            
        # Subscription to the AMR's global planning path
        self.global_path: list[PoseStamped] = []
        self.global_path_subscriber = self.create_subscription(Path, '/slamware_ros_sdk_server_node/global_plan_path', self.global_path_callback, 10, callback_group=self.callback_group)
        
        # Subscription to AMR events
        self.amr_events: list[dict] = []
        self.events_subscriber = self.create_subscription(
            String, '/amr_events', self.amr_events_callback, 10, callback_group=self.callback_group)
        
        # Publishers to communicate with the slamware_ros_sdk
        self.publisher_move_to = self.create_publisher(
            MoveToRequest,
            '/slamware_ros_sdk_server_node/move_to',
            10,
            callback_group=self.callback_group
        )
        self.publisher_cancel = self.create_publisher(
            CancelActionRequest,
            '/slamware_ros_sdk_server_node/cancel_action',
            10,
            callback_group=self.callback_group
        )

        # Declare ROS parameters with default values
        self.declare_parameter('success_distance_threshold', 0.1)  # Position distance threshold (meters)
        self.declare_parameter('success_yaw_threshold', 0.1)       # Yaw angle threshold (radians, ~5.7 degrees)
        self.declare_parameter('stuck_timeout_sec', 20.0)          # Stuck timeout duration (seconds)
        self.declare_parameter('stuck_distance_threshold', 0.05)   # Stuck detection distance threshold (meters)

        # Get parameter values
        self.success_distance_threshold = self.get_parameter('success_distance_threshold').get_parameter_value().double_value
        self.success_yaw_threshold = self.get_parameter('success_yaw_threshold').get_parameter_value().double_value
        self.stuck_timeout_sec = self.get_parameter('stuck_timeout_sec').get_parameter_value().double_value
        self.stuck_distance_threshold = self.get_parameter('stuck_distance_threshold').get_parameter_value().double_value

        self.get_logger().info("Navigate to Pose Action Server has been started.")
        self.get_logger().info(
            f"Parameters: success_distance_threshold={self.success_distance_threshold:.2f}, "
            f"success_yaw_threshold={self.success_yaw_threshold:.2f}, "
            f"stuck_timeout_sec={self.stuck_timeout_sec:.2f}, "
            f"stuck_distance_threshold={self.stuck_distance_threshold:.2f}"
        )

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
        
    def current_pose_callback(self, msg):
        """Callback to store the current pose."""
        self.current_pose = msg

    def remaining_targets_callback(self, msg):
        """Callback to store the AMR's remaining target points."""

        # Get dimension information from layout
        num_points = msg.layout.dim[0].size  # Number of points
        num_coords = msg.layout.dim[1].size  # Number of coordinates per point

        # Reconstruct 2D array from flattened data
        points = [
            list(msg.data[i:i + num_coords])
            for i in range(0, len(msg.data), num_coords)
        ]
        
        self.remaining_targets = points
        self.get_logger().debug(f"Updated remaining_targets: {self.remaining_targets}")

    def global_path_callback(self, msg):
        """Callback to store the AMR's global planning path."""

        self.global_path = msg.poses
        self.get_logger().debug(f"Updated global planning path: {self.global_path}")

    def amr_events_callback(self, msg):
        """Callback to store the AMR's events."""
        self.amr_events = ast.literal_eval(msg.data)
        self.get_logger().debug(f"AMR's events : {self.amr_events}")

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('|----------------------------------------|')
        self.get_logger().info('Received goal request')
        
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """A new goal has been accepted."""
        self.get_logger().info('Goal accepted, starting execution.')
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request.')
        return CancelResponse.ACCEPT

    def publish_move_to(self, pose: PoseStamped, speed_ratio: float):
        """Publish a MoveToRequest message with the given pose."""
        msg = MoveToRequest()
        msg.location.x = pose.pose.position.x
        msg.location.y = pose.pose.position.y
        msg.location.z = pose.pose.position.z
        quaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        msg.yaw = yaw
        msg.options.opt_flags.flags = 48  # 16+32, MoveOptionFlag: [16:'PRECISE', 32:'WITH_YAW']
        msg.options.speed_ratio.is_valid = True
        msg.options.speed_ratio.value = speed_ratio
        self.publisher_move_to.publish(msg)
        self.get_logger().info(
            f'Published MoveToRequest: location=(%.2f, %.2f, %.2f), yaw=%.2f, speed_ratio=%.2f' % 
            (msg.location.x, msg.location.y, msg.location.z, msg.yaw, msg.options.speed_ratio.value)
        )

    def publish_cancel(self):
        """Publish a CancelActionRequest message."""
        msg = CancelActionRequest()
        self.publisher_cancel.publish(msg)
        self.get_logger().info(f'Published CancelActionRequest.')

    def _is_goal_reached(self, current_pose: PoseStamped, target_pose: PoseStamped, with_yaw= True) -> bool:
        """Check if the robot has reached the target pose (position and yaw)."""
        if current_pose is None:
            return False
        # Calculate position distance
        dx = current_pose.pose.position.x - target_pose.pose.position.x
        dy = current_pose.pose.position.y - target_pose.pose.position.y
        distance = (dx**2 + dy**2)**0.5
        # Extract yaw angles
        q1 = (current_pose.pose.orientation.x, current_pose.pose.orientation.y, 
              current_pose.pose.orientation.z, current_pose.pose.orientation.w)
        q2 = (target_pose.pose.orientation.x, target_pose.pose.orientation.y, 
              target_pose.pose.orientation.z, target_pose.pose.orientation.w)
        _, _, yaw1 = euler_from_quaternion(q1)
        _, _, yaw2 = euler_from_quaternion(q2)
        yaw_diff = abs(yaw1 - yaw2)
        # Ensure yaw difference is in [0, π]
        if yaw_diff > 3.1415926535:
            yaw_diff = 2 * 3.1415926535 - yaw_diff
            
        self.get_logger().debug(f"Current distance: {distance:.2f} m, yaw difference: {yaw_diff:.2f} rad")
        # Check if goal is reached
        if with_yaw:
            reached = distance < self.success_distance_threshold and yaw_diff < self.success_yaw_threshold
        else:
            reached = distance < self.success_distance_threshold

        return reached

    def _is_stuck(self, current_pose: PoseStamped, last_pose: PoseStamped) -> bool:
        """Check if the robot is stuck based on position change."""
        if current_pose is None or last_pose is None:
            return False
        dx = current_pose.pose.position.x - last_pose.pose.position.x
        dy = current_pose.pose.position.y - last_pose.pose.position.y
        distance_moved = (dx**2 + dy**2)**0.5

        q1 = (current_pose.pose.orientation.x, current_pose.pose.orientation.y, 
            current_pose.pose.orientation.z, current_pose.pose.orientation.w)
        q2 = (last_pose.pose.orientation.x, last_pose.pose.orientation.y, 
            last_pose.pose.orientation.z, last_pose.pose.orientation.w)
        _, _, yaw1 = euler_from_quaternion(q1)
        _, _, yaw2 = euler_from_quaternion(q2)
        yaw_diff = abs(yaw1 - yaw2)
        if yaw_diff > 3.1415926535:
            yaw_diff = 2 * 3.1415926535 - yaw_diff

        motionless = distance_moved < self.stuck_distance_threshold and yaw_diff < 0.1

        no_path = self._get_status(self.amr_events) == "NO_VALID_PATH_FOUND"

        return no_path or motionless

    def _get_status(self, events: list[dict]) -> str:
        """Determine the current status based on AMR events."""
        if any(e['type'] in ['DEVICE_ERROR'] for e in events):
            return "DEVICE_ERROR_DETECTED"
        elif any(e['type'] in ['BUMPER_TRIGGERED'] for e in events):
            return "COLLISION_DETECTED_BY_BUMPER"
        elif any(e['type'] in ['CLIFF_DETECTED'] for e in events):
            return "CLIFF_DETECTED"
        elif any(e['type'] in ['WAIT_PLANNING_FAILED', 'PATH_FINDER_FAILED', 'SEARCH_LOCAL_PATH_FAILED'] for e in events):
            return "NO_VALID_PATH_FOUND"
        elif any(e['type'] in ['CURRENT_POSE_OCCUPIED'] for e in events):
            return "TARGET_POSE_IS_OCCUPIED"
        elif any(e['type'] in ['PATH_OCCUPIED'] for e in events):
            return "DETOURING_TO_AVOID_OBSTACLE"
        else:
            return "NAVIGATING_TO_TARGET"

    def _abort_goal(self, goal_handle, message: str):
        """Helper method to abort the goal with a specific message."""
        self.publish_cancel()
        goal_handle.abort()
        result = Navigate.Result()
        result.success = False
        result.message = message
        self.get_logger().info(result.message)
        return result
    
    async def execute_callback(self, goal_handle):
        """Executes the navigation action by publishing to slamware_ros_sdk topics."""
        target_pose = goal_handle.request.target_pose
        speed_ratio = goal_handle.request.speed_ratio
        feedback_msg = Navigate.Feedback()
        result = Navigate.Result()

        # Create a navigation action via publisher
        self.publish_move_to(target_pose, speed_ratio)

        # Confirm the AMR has received the action and the target point exists
        _waiting_timeout = self.stuck_timeout_sec / 2
        _last_move_time = self.get_clock().now()
        _last_pose = self.current_pose
        
        while rclpy.ok():

            current_state = self._get_status(self.amr_events)
            # Publish feedback
            if self.current_pose:
                feedback_msg.current_pose = self.current_pose
            feedback_msg.status = current_state
            goal_handle.publish_feedback(feedback_msg)
            
            if current_state == "DEVICE_ERROR_DETECTED":
                self.get_logger().error('Device error detected on the AMR!')
                timeout_message = f"Failed to dismiss the AMR device error warning within {_waiting_timeout} seconds."
                result.message = "Goal aborted due to a device error on the AMR."

                self.publish_move_to(target_pose, speed_ratio) # trying to publish again
            elif not self.remaining_targets:
                self.get_logger().warn('Waiting for the AMR remaining target points.')
                timeout_message = f"Waiting for the AMR remaining target points for {_waiting_timeout} seconds."
                result.message = "Goal aborted because no valid target points exist."

                self.publish_move_to(target_pose, speed_ratio) # trying to publish again
            else:
                if not self.global_path and not self._is_goal_reached(self.current_pose, target_pose, with_yaw=False):
                    self.get_logger().warn('Try to find a path to the target pose.')
                    timeout_message = f"Failed to find a valid path to the target within {_waiting_timeout} seconds."
                    result.message = "Goal aborted because the target pose is unreachable."
                else:
                    self.get_logger().info('MoveTo action published successfully.')
                    break
            
            if (self.get_clock().now() - _last_move_time).nanoseconds / 1e9 > _waiting_timeout:
                self.get_logger().error(timeout_message)
                result = self._abort_goal(goal_handle, result.message)
                return result
            
            await self.ros_async_sleep(0.2)

        self.get_logger().info('Executing goal...')
        # Monitor the action status
        _last_move_time = self.get_clock().now()
        _last_pose = self.current_pose

        while rclpy.ok():

            current_state = self._get_status(self.amr_events)

            """Publish feedback if current pose is available"""
            if self.current_pose:
                feedback_msg.current_pose = self.current_pose
                feedback_msg.status = current_state
                goal_handle.publish_feedback(feedback_msg)

            """Safety prevention"""
            if current_state == "DEVICE_ERROR_DETECTED":
                self.get_logger().error(f"Device error detected on the AMR!")
                result = self._abort_goal(goal_handle, "Goal aborted due to a device error on the AMR.")
                return result
            
            elif current_state == "COLLISION_DETECTED_BY_BUMPER":
                self.get_logger().warn(f"collision detected by the bumper!")
                result = self._abort_goal(goal_handle, "Goal aborted due to a collision detected by the AMR's bumper.")
                return result


            """Check if robot is stuck"""
            if self._is_stuck(self.current_pose, _last_pose):
                if (self.get_clock().now() - _last_move_time).nanoseconds / 1e9 > self.stuck_timeout_sec:
                    self.get_logger().error(
                        f"Robot stuck for {self.stuck_timeout_sec} seconds at position "
                        f"(x={self.current_pose.pose.position.x:.2f}, y={self.current_pose.pose.position.y:.2f})"
                    )

                    result = self._abort_goal(goal_handle, "Goal aborted because the AMR failed to find a valid path and got stuck.")
                    return result
            else:
                _last_move_time = self.get_clock().now()
                _last_pose = self.current_pose

            """Check for cancel request"""
            if goal_handle.is_cancel_requested:
                self.publish_cancel()
                goal_handle.canceled()
                result.success = False
                result.message = "Goal canceled by the client."
                self.get_logger().info(result.message)
                return result

            """Check whether the goal is completed"""
            if not self.remaining_targets: # No more target points
                # Check if the robot has reached the target pose
                if self._is_goal_reached(self.current_pose, target_pose):
                    goal_handle.succeed()
                    result.success = True
                    result.message = "Goal achieved successfully."
                    self.get_logger().info(result.message)
                    return result
                else: # AMR action is done but did not meet threshold criteria
                    result = self._abort_goal(goal_handle, "Goal aborted because the MoveTo action completed but the target pose was not reached.")
                    return result
            
            # Reset pose to none until new msg is posted
            self.current_pose = None

            # Use non-blocking ROS-native sleep
            await self.ros_async_sleep(0.1)

        self.get_logger().info("RCLPY shutdown, aborting goal.")
        return await self._abort_goal(goal_handle, "Goal aborted due to RCLPY shutdown.")

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