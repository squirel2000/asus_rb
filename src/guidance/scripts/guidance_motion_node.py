#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from geometry_msgs.msg import PoseStamped
from guidance.action import Guidance
from slamware_ros_sdk.msg import MoveToRequest, CancelActionRequest
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray, String
from perception_control_manager.srv import SetMaxSpeed
from nav_msgs.msg import Path
import ast

class GuidanceActionServer(Node):
    """
    Action server to guide the user to a target pose.
    It communicates with the perception_control_manager node to execute the navigation.
    """
    def __init__(self):
        super().__init__('guidance_motion_node')
        
        # Use a reentrant callback group to allow for nested service calls and callbacks
        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Guidance,
            'guide_user_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        # Service clients to communicate with the perception_control_manager
        self.set_max_speed_client = self.create_client(SetMaxSpeed, 'set_max_speed', callback_group=self.callback_group)
        while not self.set_max_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_max_speed service...')

        # Subscription to the current pose for feedback
        self.current_pose = None
        self.pose_subscriber = self.create_subscription(
            PoseStamped, '/robot_pose', self.current_pose_callback, 10, callback_group=self.callback_group)
        
        # Subscription to the AMR's remaining target points
        self.remaining_targets: list[list] = []
        self.remaining_targets_subscriber = self.create_subscription(
            Float32MultiArray, '/remaining_targets', self.remaining_targets_callback, 10, callback_group=self.callback_group)
        
        # Subscription to human relative pose
        self.human_relative_pose = None
        self.human_pose_subscriber = self.create_subscription(
            PoseStamped, '/human_relative_pose', self.human_pose_callback, 10, callback_group=self.callback_group)
        
        # Subscription to AMR events
        self.amr_events: list[dict] = []
        self.events_subscriber = self.create_subscription(
            String, '/amr_events', self.amr_events_callback, 10, callback_group=self.callback_group)
        
        # Subscription to the AMR's global planning path
        self.global_path: list[PoseStamped] = []
        self.global_path_subscriber = self.create_subscription(
            Path, '/slamware_ros_sdk_server_node/global_plan_path', self.global_path_callback, 10, callback_group=self.callback_group)
        
        # Publishers to communicate with the slamware_ros_sdk
        self.publisher_move_to = self.create_publisher(
            MoveToRequest, '/slamware_ros_sdk_server_node/move_to', 10, callback_group=self.callback_group)
        self.publisher_cancel = self.create_publisher(
            CancelActionRequest, '/slamware_ros_sdk_server_node/cancel_action', 10, callback_group=self.callback_group)

        # Declare ROS parameters with default values
        self.declare_parameter('success_distance_threshold', 0.1)  # Position distance threshold (meters)
        self.declare_parameter('success_yaw_threshold', 0.1)       # Yaw angle threshold (radians)
        self.declare_parameter('stuck_timeout_sec', 30.0)          # Stuck timeout duration (seconds)
        self.declare_parameter('stuck_distance_threshold', 0.05)   # Stuck detection distance threshold (meters)
        self.declare_parameter('normal_distance_min', 0.8)         # Min distance for normal following (meters)
        self.declare_parameter('normal_distance_max', 2.0)         # Max distance for normal following (meters)
        self.declare_parameter('lost_distance_threshold', 3.0)     # Distance threshold for lost (meters)
        self.declare_parameter('lost_timeout_sec', 10.0)           # Timeout for waiting after lost (seconds)
        self.declare_parameter('max_moving_speed', 1.0)            # Max linear speed of AMR (m/s)
        self.declare_parameter('max_angular_speed', 1.2)           # Max angular speed of AMR (rad/s)

        # Get parameter values
        self.success_distance_threshold = self.get_parameter('success_distance_threshold').get_parameter_value().double_value
        self.success_yaw_threshold = self.get_parameter('success_yaw_threshold').get_parameter_value().double_value
        self.stuck_timeout_sec = self.get_parameter('stuck_timeout_sec').get_parameter_value().double_value
        self.stuck_distance_threshold = self.get_parameter('stuck_distance_threshold').get_parameter_value().double_value
        self.normal_distance_min = self.get_parameter('normal_distance_min').get_parameter_value().double_value
        self.normal_distance_max = self.get_parameter('normal_distance_max').get_parameter_value().double_value
        self.lost_distance_threshold = self.get_parameter('lost_distance_threshold').get_parameter_value().double_value
        self.lost_timeout_sec = self.get_parameter('lost_timeout_sec').get_parameter_value().double_value
        self.max_moving_speed = self.get_parameter('max_moving_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        # State variables for speed adjustment and human tracking
        self._last_speed_change_time = None
        self._is_human_lost = False
        self._human_lost_start_time = None
        self._last_max_moving_speed = None
        self._speed_ratio = None

        self.get_logger().info("Guide User to Pose Action Server has been started.")
        self.get_logger().info(
            f"Parameters: success_distance_threshold={self.success_distance_threshold:.2f}, "
            f"success_yaw_threshold={self.success_yaw_threshold:.2f}, "
            f"stuck_timeout_sec={self.stuck_timeout_sec:.2f}, "
            f"stuck_distance_threshold={self.stuck_distance_threshold:.2f}, "
            f"normal_distance_min={self.normal_distance_min:.2f}, "
            f"normal_distance_max={self.normal_distance_max:.2f}, "
            f"lost_distance_threshold={self.lost_distance_threshold:.2f}, "
            f"lost_timeout_sec={self.lost_timeout_sec:.2f}, "
            f"max_moving_speed={self.max_moving_speed:.2f}, "
            f"max_angular_speed={self.max_angular_speed:.2f}"
        )

    async def ros_async_sleep(self, seconds: float):
        """A ROS-compatible asynchronous sleep function that uses a one-shot timer."""
        future = Future()
        self.create_timer(seconds, lambda: future.set_result(None))
        await future

    def current_pose_callback(self, msg):
        """Callback to store the current pose."""
        self.current_pose = msg

    def human_pose_callback(self, msg):
        """Callback to store the human's relative pose."""
        self.human_relative_pose = msg

    def remaining_targets_callback(self, msg):
        """Callback to store the AMR's remaining target points."""
        num_points = msg.layout.dim[0].size
        num_coords = msg.layout.dim[1].size
        points = [list(msg.data[i:i + num_coords]) for i in range(0, len(msg.data), num_coords)]
        self.remaining_targets = points
        self.get_logger().debug(f"Updated remaining_targets: {self.remaining_targets}")

    def amr_events_callback(self, msg):
        """Callback to store the AMR's events."""
        self.amr_events = ast.literal_eval(msg.data)
        self.get_logger().debug(f"AMR's events: {self.amr_events}")

    def global_path_callback(self, msg):
        """Callback to store the AMR's global planning path."""
        self.global_path = msg.poses
        self.get_logger().debug(f"Updated global planning path: {self.global_path}")

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

    async def publish_cancel(self):
        """Publish a CancelActionRequest message."""
        msg = CancelActionRequest()
        self.publisher_cancel.publish(msg)
        self.get_logger().info('Published CancelActionRequest')
        # Reset to default speed before aborting
        await self.set_max_speed(self._speed_ratio * self.max_moving_speed, 
                            self._speed_ratio * self.max_angular_speed)

    async def set_max_speed(self, max_moving_speed: float, max_angular_speed: float):
        """Call service to set max moving and angular speed with retry on failure."""
        request = SetMaxSpeed.Request()
        request.max_moving_speed = max_moving_speed
        request.max_angular_speed = max_angular_speed
        for attempt in range(3):  # Retry up to 3 times
            future = await self.set_max_speed_client.call_async(request)
            if future.success:
                self.get_logger().debug(
                    f'Set max speed: max_moving_speed={max_moving_speed:.2f}, max_angular_speed={max_angular_speed:.2f}')
                return True
            self.get_logger().warn(f'Failed to set max speed, attempt {attempt + 1}/3')
            await self.ros_async_sleep(0.1)
        self.get_logger().error('Failed to set max speed after 3 attempts')
        return False

    def _is_goal_reached(self, current_pose: PoseStamped, target_pose: PoseStamped) -> bool:
        """Check if the robot has reached the target pose (position and yaw)."""
        if current_pose is None:
            return False
        dx = current_pose.pose.position.x - target_pose.pose.position.x
        dy = current_pose.pose.position.y - target_pose.pose.position.y
        distance = (dx**2 + dy**2)**0.5
        q1 = (current_pose.pose.orientation.x, current_pose.pose.orientation.y, 
              current_pose.pose.orientation.z, current_pose.pose.orientation.w)
        q2 = (target_pose.pose.orientation.x, target_pose.pose.orientation.y, 
              target_pose.pose.orientation.z, target_pose.pose.orientation.w)
        _, _, yaw1 = euler_from_quaternion(q1)
        _, _, yaw2 = euler_from_quaternion(q2)
        yaw_diff = abs(yaw1 - yaw2)
        if yaw_diff > 3.1415926535:
            yaw_diff = 2 * 3.1415926535 - yaw_diff
        self.get_logger().debug(f"Current distance: {distance:.2f} m, yaw difference: {yaw_diff:.2f} rad")
        return distance < self.success_distance_threshold and yaw_diff < self.success_yaw_threshold

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
        no_path = self._get_status(self.amr_events) == "NO_VALID_PATH_FOUND"
        motionless = distance_moved < self.stuck_distance_threshold and yaw_diff < 0.1
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

    def _get_human_distance(self, human_relative_pose) -> float:
        """Calculate the distance to the human from human_relative_pose."""
        if human_relative_pose is None:
            self.get_logger().debug("Human relative pose does not exist.")
            return float('inf')
        dx = human_relative_pose.pose.position.x
        dy = human_relative_pose.pose.position.y
        distance = (dx**2 + dy**2)**0.5
        self.get_logger().debug(f"Human relative distance: {distance:.2f} m")
        return distance

    async def _adjust_speed_and_handle_lost(self, human_distance: float):
        """Adjust robot speed based on human distance and handle lost scenarios."""
        current_time = self.get_clock().now()
        
        # Speed adjustment logic
        status = "Following"
        if (self._last_speed_change_time is None or 
            (current_time - self._last_speed_change_time).nanoseconds / 1e9 > 1.0):  # Adjust speed every 1 second
            if human_distance < self.normal_distance_min:
                # Human too close, accelerate
                new_max_moving_speed = 1.3 * self._speed_ratio * self.max_moving_speed
                new_max_angular_speed = 1.3 * self._speed_ratio * self.max_angular_speed
                status = "TooClose"
                self._is_human_lost = False
                self._human_lost_start_time = None
                self.get_logger().info(f"TooClose  ---> Distance:{human_distance:.2f}, moving_speed:{new_max_moving_speed:.2f}, angular_speed:{new_max_angular_speed:.2f}")
            elif human_distance > self.normal_distance_max:
                if human_distance > self.lost_distance_threshold:
                    # Human lost
                    new_max_moving_speed = 0.05
                    new_max_angular_speed = 0.1
                    status = "Lost"
                    self.get_logger().warn(f"Lost      ---> Distance:{human_distance:.2f}, moving_speed:{new_max_moving_speed:.2f}, angular_speed:{new_max_angular_speed:.2f}")
                    if not self._is_human_lost:
                        self._is_human_lost = True
                        self._human_lost_start_time = current_time
                    elif (current_time - self._human_lost_start_time).nanoseconds / 1e9 > self.lost_timeout_sec:
                        self.get_logger().error(f"Human lost for {self.lost_timeout_sec} seconds, aborting action")
                        status = "ABORTING"
                        return status
                else:
                    # Human lagging, smooth deceleration
                    k = (self._speed_ratio * self.max_moving_speed - 0.05) / (self.lost_distance_threshold - self.normal_distance_max)
                    new_max_moving_speed = self._speed_ratio * self.max_moving_speed - k * (human_distance - self.normal_distance_max)
                    new_max_angular_speed = self._speed_ratio * self.max_angular_speed - k * (human_distance - self.normal_distance_max)
                    status = "Lagging"
                    self._is_human_lost = False
                    self._human_lost_start_time = None
                    self.get_logger().info(f"Lagging   ---> Distance:{human_distance:.2f}, moving_speed:{new_max_moving_speed:.2f}, angular_speed:{new_max_angular_speed:.2f}")
            else:
                # Normal following
                new_max_moving_speed = self._speed_ratio * self.max_moving_speed
                new_max_angular_speed = self._speed_ratio * self.max_angular_speed
                status = "Following"
                self._is_human_lost = False
                self._human_lost_start_time = None
                self.get_logger().info(f"Following ---> Distance:{human_distance:.2f}, moving_speed:{new_max_moving_speed:.2f}, angular_speed:{new_max_angular_speed:.2f}")

            # Update speed if changed
            if self._last_max_moving_speed is None or abs(new_max_moving_speed - self._last_max_moving_speed) > 0.01:
                success = await self.set_max_speed(new_max_moving_speed, new_max_angular_speed)
                if not success:
                    self.get_logger().error("Speed adjustment failed, service failure")
                    return status
                
                self._last_max_moving_speed = new_max_moving_speed
                self._last_speed_change_time = current_time

        return status

    async def _resume_navigation(self, target_pose: PoseStamped, speed_ratio: float):
        """Resume navigation when human is re-detected."""
        self.get_logger().info("Human re-detected, resuming navigation")
        await self.publish_cancel()
        self.publish_move_to(target_pose, speed_ratio)

    async def _abort_goal(self, goal_handle, message: str):
        """Helper method to abort the goal with a specific message."""
        await self.publish_cancel()
        goal_handle.abort()
        result = Guidance.Result()
        result.success = False
        result.message = message
        self.get_logger().info(result.message)
        return result
    
    async def execute_callback(self, goal_handle):
        """Execute the guidance action with human following."""
        target_pose = goal_handle.request.target_pose
        self._speed_ratio = goal_handle.request.speed_ratio
        user_id = goal_handle.request.user_id
        feedback_msg = Guidance.Feedback()
        result = Guidance.Result()

        self.publish_move_to(target_pose, self._speed_ratio)
        
        # Confirm the AMR has received the action and the target point exists
        _waiting_timeout = self.stuck_timeout_sec / 2
        _last_move_time = self.get_clock().now()
        _last_pose = self.current_pose
        
        while rclpy.ok():

            current_state = self._get_status(self.amr_events)
            if current_state == "DEVICE_ERROR_DETECTED":
                self.get_logger().error('Device error detected on the AMR!')
                timeout_message = f"Failed to dismiss the AMR device error warning within {_waiting_timeout} seconds."
                result.message = "Goal aborted due to a device error on the AMR."
                self.publish_move_to(target_pose, self._speed_ratio)  # Try to publish again
            elif not self.remaining_targets:
                self.get_logger().warn('Waiting for the AMR remaining target points.')
                timeout_message = f"Waiting for the AMR remaining target points for {_waiting_timeout} seconds."
                result.message = "Goal aborted because no valid target points exist."
                self.publish_move_to(target_pose, self._speed_ratio)  # Try to publish again
            else:
                if not self.global_path and not self._is_goal_reached(self.current_pose, target_pose):
                    self.get_logger().warn('Try to find a path to the target pose.')
                    timeout_message = f"Failed to find a valid path to the target within {_waiting_timeout} seconds."
                    result.message = "Goal aborted because the target pose is unreachable."
                else:
                    self.get_logger().info('MoveTo action published successfully.')
                    break
            
            if (self.get_clock().now() - _last_move_time).nanoseconds / 1e9 > _waiting_timeout:
                self.get_logger().error(timeout_message)
                result = await self._abort_goal(goal_handle, result.message)
                return result
            
            await self.ros_async_sleep(0.2)

        self.get_logger().info('Executing guidance goal...')

        # Initialize state
        self._last_speed_change_time = self.get_clock().now()
        self._is_human_lost = False
        self._human_lost_start_time = None
        self._last_max_moving_speed = None
        _last_move_time = self.get_clock().now()
        _last_pose = self.current_pose

        while rclpy.ok():

            # Check safety conditions
            current_state = self._get_status(self.amr_events)

            """Safety prevention"""
            if current_state == "DEVICE_ERROR_DETECTED":
                self.get_logger().error("Device error detected on the AMR!")
                result = await self._abort_goal(goal_handle, "Goal aborted due to a device error on the AMR.")
                return result

            elif current_state == "COLLISION_DETECTED_BY_BUMPER":
                self.get_logger().warn("Collision detected by the bumper!")
                result = await self._abort_goal(goal_handle, "Goal aborted due to a collision detected by the AMR's bumper.")
                return result
            
            """Check if robot is stuck"""
            if self.current_pose and _last_pose:
                if self._is_stuck(self.current_pose, _last_pose):
                    if (self.get_clock().now() - _last_move_time).nanoseconds / 1e9 > self.stuck_timeout_sec:
                        self.get_logger().error(
                            f"Robot stuck for {self.stuck_timeout_sec} seconds at position "
                            f"(x={self.current_pose.pose.position.x:.2f}, y={self.current_pose.pose.position.y:.2f})"
                        )
                        result = await self._abort_goal(goal_handle, "Goal aborted because the AMR failed to find a valid path and got stuck.")
                        return result
                else:
                    _last_move_time = self.get_clock().now()
                    _last_pose = self.current_pose

            """Check for cancel request"""
            if goal_handle.is_cancel_requested:
                await self.publish_cancel()
                goal_handle.canceled()
                result.success = False
                result.message = "Goal canceled by the client."
                self.get_logger().info(result.message)
                return result

            """Calculate human distance and publish feedback"""
            human_distance = self._get_human_distance(self.human_relative_pose)
            guiding_stage = await self._adjust_speed_and_handle_lost(human_distance)

            if guiding_stage == "ABORTING":
                result = await self._abort_goal(goal_handle, "Goal aborted due to human lost.")
                return result

            """Check if human is lost and needs to resume navigation"""
            if self._is_human_lost and self._human_lost_start_time and \
               (self.get_clock().now() - self._human_lost_start_time).nanoseconds / 1e9 > self.lost_timeout_sec:
                if human_distance <= self.lost_distance_threshold:
                    await self._resume_navigation(target_pose, self._speed_ratio)
                    self._is_human_lost = False
                    self._human_lost_start_time = None

            """Publish feedback if current pose is available"""
            if self.current_pose:
                feedback_msg.current_pose = self.current_pose
                feedback_msg.status = f"{current_state} - {guiding_stage}"
                goal_handle.publish_feedback(feedback_msg)

            """Check whether the goal is completed"""
            if self.current_pose and not self.remaining_targets:
                if self._is_goal_reached(self.current_pose, target_pose):
                    await self.set_max_speed(self._speed_ratio * self.max_moving_speed, 
                                            self._speed_ratio * self.max_angular_speed)
                    goal_handle.succeed()
                    result.success = True
                    result.message = "Goal achieved successfully."
                    self.get_logger().info(result.message)
                    return result
                else:
                    result = await self._abort_goal(goal_handle, "Goal aborted because the MoveTo action completed but the target pose was not reached.")
                    return result

            # Reset pose to none until new msg is posted
            self.human_relative_pose = None 
            self.current_pose = None

            await self.ros_async_sleep(0.1)

        self.get_logger().info("RCLPY shutdown, aborting goal.")
        return await self._abort_goal(goal_handle, "Goal aborted due to RCLPY shutdown.")

def main(args=None):
    rclpy.init(args=args)
    action_server = GuidanceActionServer()
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