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
        self.remaining_targets: list[list] = None
        self.remaining_targets_subscriber = self.create_subscription(
            Float32MultiArray, '/remaining_targets', self.remaining_targets_callback, 10, callback_group=self.callback_group)
        
        # Subscription to human relative pose
        self.human_relative_pose = None
        self.human_pose_subscriber = self.create_subscription(
            PoseStamped, '/human_relative_pose', self.human_pose_callback, 10, callback_group=self.callback_group)
        
        # Subscription to AMR health
        self.info_subscriber = self.create_subscription(
            String, '/amr_health', self.info_callback, 10, callback_group=self.callback_group)
        
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
        self.declare_parameter('normal_distance_min', 1.0)         # Min distance for normal following (meters)
        self.declare_parameter('normal_distance_max', 2.0)         # Max distance for normal following (meters)
        self.declare_parameter('lag_distance_threshold', 2.0)      # Distance threshold for lagging (meters)
        self.declare_parameter('lost_distance_threshold', 5.0)     # Distance threshold for lost (meters)
        self.declare_parameter('lost_timeout_sec', 10.0)           # Timeout for waiting after lost (seconds)
        self.declare_parameter('abort_timeout_sec', 30.0)          # Timeout for aborting after lost (seconds)
        self.declare_parameter('max_moving_speed', 1.0)            # Max linear speed of AMR (m/s)
        self.declare_parameter('max_angular_speed', 1.0)           # Max angular speed of AMR (rad/s)

        # Get parameter values
        self.success_distance_threshold = self.get_parameter('success_distance_threshold').get_parameter_value().double_value
        self.success_yaw_threshold = self.get_parameter('success_yaw_threshold').get_parameter_value().double_value
        self.stuck_timeout_sec = self.get_parameter('stuck_timeout_sec').get_parameter_value().double_value
        self.stuck_distance_threshold = self.get_parameter('stuck_distance_threshold').get_parameter_value().double_value
        self.normal_distance_min = self.get_parameter('normal_distance_min').get_parameter_value().double_value
        self.normal_distance_max = self.get_parameter('normal_distance_max').get_parameter_value().double_value
        self.lag_distance_threshold = self.get_parameter('lag_distance_threshold').get_parameter_value().double_value
        self.lost_distance_threshold = self.get_parameter('lost_distance_threshold').get_parameter_value().double_value
        self.lost_timeout_sec = self.get_parameter('lost_timeout_sec').get_parameter_value().double_value
        self.abort_timeout_sec = self.get_parameter('abort_timeout_sec').get_parameter_value().double_value
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
            f"lag_distance_threshold={self.lag_distance_threshold:.2f}, "
            f"lost_distance_threshold={self.lost_distance_threshold:.2f}, "
            f"lost_timeout_sec={self.lost_timeout_sec:.2f}, "
            f"abort_timeout_sec={self.abort_timeout_sec:.2f}, "
            f"max_moving_speed={self.max_moving_speed:.2f}, "
            f"max_angular_speed={self.max_angular_speed:.2f}"
        )

    async def ros_async_sleep(self, seconds: float):
        """A ROS-compatible asynchronous sleep function that uses a one-shot timer."""
        future = Future()
        self.create_timer(seconds, lambda: future.set_result(None))
        await future

    def info_callback(self, msg):
        """Callback to store AMR health information."""
        self.get_logger().debug(f"The AMR health information: {msg}")

    def current_pose_callback(self, msg):
        """Callback to store the current pose."""
        self.current_pose = msg

    def human_pose_callback(self, msg):
        """Callback to store the human's relative pose."""
        self.human_relative_pose = msg
        distance = self._get_human_distance()
        self.get_logger().debug(f"Human relative distance: {distance:.2f} m")

    def remaining_targets_callback(self, msg):
        """Callback to store the AMR's remaining target points."""
        num_points = msg.layout.dim[0].size
        num_coords = msg.layout.dim[1].size
        points = [list(msg.data[i:i + num_coords]) for i in range(0, len(msg.data), num_coords)]
        self.remaining_targets = points
        self.get_logger().debug(f"Updated remaining_targets: {self.remaining_targets}")

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
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
        self.get_logger().info('Published CancelActionRequest')

    async def set_max_speed(self, max_moving_speed: float, max_angular_speed: float):
        """Call service to set max moving and angular speed with retry on failure."""
        request = SetMaxSpeed.Request()
        request.max_moving_speed = max_moving_speed
        request.max_angular_speed = max_angular_speed
        for attempt in range(3):  # Retry up to 3 times
            future = await self.set_max_speed_client.call_async(request)
            if future.result() is not None and future.result().success:
                self.get_logger().info(
                    f'Set max speed: max_moving_speed={max_moving_speed:.2f}, max_angular_speed={max_angular_speed:.2f}')
                return True
            self.get_logger().warn(f'Failed to set max speed, attempt {attempt + 1}/3')
            await self.ros_async_sleep(0.1)
        self.get_logger().error('Failed to set max speed after 3 attempts')
        return False

    def _is_goal_reached(self, current_pose: PoseStamped, target_pose: PoseStamped) -> bool:
        """Check if the robot has reached the target pose (position and yaw)."""
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
        return distance_moved < self.stuck_distance_threshold and yaw_diff < 0.1

    def _get_human_distance(self) -> float:
        """Calculate the distance to the human from human_relative_pose."""
        if self.human_relative_pose is None:
            return float('inf')
        dx = self.human_relative_pose.pose.position.x
        dy = self.human_relative_pose.pose.position.y
        return (dx**2 + dy**2)**0.5

    async def _adjust_speed_and_handle_lost(self, human_distance: float):
        """Adjust robot speed based on human distance and handle lost scenarios."""
        current_time = self.get_clock().now()
        
        # Speed adjustment logic
        status = "Following"
        if (current_time - self._last_speed_change_time).nanoseconds / 1e9 > 1.0:  # Adjust speed every 1 second
            if human_distance < self.normal_distance_min or human_distance > self.lost_distance_threshold:
                # Human too close or lost
                new_max_moving_speed = 0.05
                new_max_angular_speed = 0.1

                if human_distance > self.lost_distance_threshold:
                    status = "Lost"
                    if not self._is_human_lost:
                        self._is_human_lost = True
                        self._human_lost_start_time = current_time
                    elif (current_time - self._human_lost_start_time).nanoseconds / 1e9 > self.abort_timeout_sec:
                        self.get_logger().error(f"Human lost for {self.abort_timeout_sec} seconds, aborting action")
                        return status
                else:
                    status = "TooClose"

            elif human_distance > self.lag_distance_threshold:
                # Human lagging
                new_max_moving_speed = 0.1
                new_max_angular_speed = 0.2
                status = "Lagging"
                self._is_human_lost = False
                self._human_lost_start_time = None
            else:
                # Normal following
                new_max_moving_speed = self._speed_ratio * self.max_moving_speed
                new_max_angular_speed = self._speed_ratio * self.max_angular_speed
                status = "Following"
                self._is_human_lost = False
                self._human_lost_start_time = None

            # Update speed if changed
            if self._last_max_moving_speed is None or abs(new_max_moving_speed - self._last_max_moving_speed) > 0.01:
                success = await self.set_max_speed(new_max_moving_speed, new_max_angular_speed)
                if not success:
                    self.get_logger().error(f"Speed adjustment failed, service failure")
                    return status  # Signal service failure
                
                self._last_max_moving_speed = new_max_moving_speed
                self._last_speed_change_time = current_time

        return status

    async def _resume_navigation(self, target_pose: PoseStamped, speed_ratio: float):
        """Resume navigation when human is re-detected."""
        self.get_logger().info("Human re-detected, resuming navigation")
        self.publish_cancel()
        self.publish_move_to(target_pose, speed_ratio)

    async def execute_callback(self, goal_handle):
        """Execute the guidance action with human following."""

        # Wait for initial pose
        while rclpy.ok() and self.current_pose is None:
            self.get_logger().info('Waiting for initial robot pose.')
            await self.ros_async_sleep(0.1)

        target_pose = goal_handle.request.target_pose
        self._speed_ratio = goal_handle.request.speed_ratio
        user_id = goal_handle.request.user_id

        self.publish_move_to(target_pose, self._speed_ratio)
        
        # Confirm the AMR has received the action and the target point exists
        while rclpy.ok() and not self.remaining_targets:
            self.get_logger().info('Waiting for get AMR remaining target points.')
            await self.ros_async_sleep(0.1)
        self.get_logger().info('Executing guidance goal...')

        # Initialize state
        feedback_msg = Guidance.Feedback()
        result = Guidance.Result()

        self._last_speed_change_time = self.get_clock().now()
        self._is_human_lost = False
        self._human_lost_start_time = None
        self._last_max_moving_speed = None
        _last_move_time = self.get_clock().now()
        _last_pose = self.current_pose

        while rclpy.ok():
            # Check if robot is stuck
            if self.current_pose and _last_pose:
                if self._is_stuck(self.current_pose, _last_pose):
                    if (self.get_clock().now() - _last_move_time).nanoseconds / 1e9 > self.stuck_timeout_sec:
                        self.get_logger().error(
                            f"Robot stuck for {self.stuck_timeout_sec} seconds at position "
                            f"(x={self.current_pose.pose.position.x:.2f}, y={self.current_pose.pose.position.y:.2f})"
                        )
                        self.get_logger().info("Goal aborted due to robot being stuck")
                        self.publish_cancel()
                        goal_handle.abort()
                        result.success = False
                        return result
                else:
                    _last_move_time = self.get_clock().now()
                    _last_pose = self.current_pose

            # Check for cancel request
            if goal_handle.is_cancel_requested:
                self.publish_cancel()
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
                result.success = False
                return result

            # Calculate human distance and publish feedback
            human_distance = self._get_human_distance()
            
            # Adjust speed and handle human lost
            guiding_stage = await self._adjust_speed_and_handle_lost(human_distance)

            feedback_msg.current_pose = self.current_pose
            feedback_msg.status = guiding_stage
            goal_handle.publish_feedback(feedback_msg)
            
            if guiding_stage == "Lost":
                self.publish_cancel()
                goal_handle.abort()
                result.success = False
                return result


            # Check if human is lost and needs to resume navigation
            if self._is_human_lost and self._human_lost_start_time and \
               (self.get_clock().now() - self._human_lost_start_time).nanoseconds / 1e9 > self.lost_timeout_sec:
                if human_distance <= self.lost_distance_threshold:
                    await self._resume_navigation(target_pose, self._speed_ratio)
                    self._is_human_lost = False
                    self._human_lost_start_time = None



            # Check if goal is reached
            if self.current_pose and not self.remaining_targets:
                if self._is_goal_reached(self.current_pose, target_pose):
                    self.get_logger().info('Goal succeeded!')
                    goal_handle.succeed()
                    result.success = True
                    return result
                else:
                    self.get_logger().info('Goal aborted! AMR action is done but did not reach the expected target pose.')
                    self.publish_cancel()
                    goal_handle.abort()
                    result.success = False
                    return result

            await self.ros_async_sleep(0.1)

        self.get_logger().info("RCLPY shutdown, aborting goal.")
        self.publish_cancel()
        goal_handle.abort()
        result.success = False
        return result

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