#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
import argparse
import math
import time
import numpy as np
import tf2_ros
from utils.head_control import HeadController
from utils.head_tracker import HeadTracker
from utils.motion_utils import MotionUtils
from utils.pure_pursuit_controller import PurePursuitController
from motion_common.action import FollowUser
from copy import deepcopy

# Constants
FOLLOW_USER_OFFSET = 0.25 # Move the target point closer to avoid path not found issues
STATIC_NECK_ANGLE = 0.0
STATIC_NECK_PITCH_DEG = 20.0  # Fixed neck pitch angle to face horizontally

class FollowUserMotionNode(Node):
    """
    This node controls the robot's motion to follow a user using an action server.
    It receives relative user pose, controls the head, calculates the user's absolute pose,
    searches for a path, and publishes it.
    """
    def __init__(self, robot_ip, control_head=False):
        super().__init__('follow_user_motion_node')

        self.robot_ip = robot_ip
        self.control_head = control_head

        # Action Server
        self._action_server = ActionServer(
            self,
            FollowUser,
            'follow_user',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            )

        # Subscribers
        self.robot_pose_sub = self.create_subscription(PoseStamped, '/robot_pose', self.robot_pose_callback, 10)
        self.human_relative_pose_sub = self.create_subscription(PoseStamped, '/human_relative_pose_front', self.human_relative_pose_front_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/slamware_ros_sdk_server_node/odom', self.odom_callback, 10)
                
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_publisher = self.create_publisher(Path, '/follow_user/planned_path', 10)
        self.human_absolute_pose_publisher = self.create_publisher(PoseStamped, '/follow_user/human_absolute_pose', 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Head Control
        self.head_controller = None
        if self.control_head:
            self._init_head_controller()

        # State
        self._active_goal_handle = None
        self.robot_pose = None
        self.current_velocity = Twist()
        self.path_msg = None
        self.last_path_search_time = self.get_clock().now()
        self.neck_yaw, self.neck_pitch = 0.0, 0.0
        self.human_absolute_pose = None
        self.human_last_update_time = None
        self.robot_last_update_time = None
        self.current_user_distance = None
        self.follow_state = ""

        # Parameters for pure pursuit
        self.declare_parameter("min_lookahead_dist", 0.5)
        self.declare_parameter("max_lookahead_dist", 1.5)
        self.declare_parameter("lookahead_time", 1.5)
        self.declare_parameter("max_linear_vel", 0.20)
        self.declare_parameter("linear_acceleration", 0.3)
        self.declare_parameter("linear_deceleration", 0.6)
        self.declare_parameter('max_angular_vel', 1.2)
        self.declare_parameter("max_angular_acceleration", 1.0)
        self.declare_parameter("heading_error_for_pure_rotation", 1.57)
        self.declare_parameter("min_heading_error_for_motion", 0.35)
        self.declare_parameter("min_approach_linear_velocity", 0.05)
        self.declare_parameter("approach_velocity_scaling_dist", 0.6)
        self.declare_parameter("goal_dist_buf", 0.15)
        self.declare_parameter("goal_dist_tol", 0.075)

        self.declare_parameter("lagging_dist_thres", 1.5)
        self.declare_parameter("lost_user_timeout", 0.5)
        self.lagging_dist_thres = self.get_parameter("lagging_dist_thres").get_parameter_value().double_value
        self.lost_user_timeout = self.get_parameter("lost_user_timeout").get_parameter_value().double_value

        # Utility classes
        self.motion_utils = MotionUtils(self)
        self.pure_pursuit_controller = PurePursuitController(self)
        self.head_tracker = HeadTracker(self.get_logger())

        self.get_logger().info('Follow User Motion Node has been started.')
        
        self.get_logger().info(
            f"Parameters: "
            f"control_head={self.control_head}, "
            f"lagging_dist_thres={self.lagging_dist_thres:.2f}, "
            f"lost_user_timeout={self.lost_user_timeout:.2f}, "
        )

    def _init_head_controller(self, max_retries=5, retry_delay=0.5):
        """
        Initializes or re-initializes the head controller with a retry mechanism.
        """
        self.get_logger().info("Initializing head controller...")
        if self.head_controller:
            self.head_controller.destroy()
            self.head_controller = None

        for attempt in range(max_retries):
            self.get_logger().info(f"Head controller initialization attempt {attempt + 1}/{max_retries}...")
            try:
                # Create a new instance
                self.head_controller = HeadController(self.get_logger())
                
                if not self.head_controller.serial_port or not self.head_controller.serial_port.is_open:
                    raise ConnectionError("Serial port could not be opened.")

                self.head_controller.start_listening()
                
                if not self.head_controller._running:
                     raise ConnectionError("Head controller listener thread failed to start.")

                version_info = self.head_controller.get_firmware_version(timeout=2)
                if "error" in version_info:
                    raise ConnectionError(f"Failed to get firmware version: {version_info['error']}")

                self.head_controller.control_head(yaw_deg=STATIC_NECK_ANGLE, pitch_deg=STATIC_NECK_PITCH_DEG)
                self.get_logger().info("Head controller initialized successfully.")
                return True

            except Exception as e:
                self.get_logger().error(f"Attempt {attempt + 1} failed: {e}")
                if self.head_controller:
                    self.head_controller.destroy()
                self.head_controller = None

                if attempt < max_retries - 1:
                    self.get_logger().info(f"Retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
                else:
                    self.get_logger().error("Max retries reached. Failed to initialize head controller.")
                    return False
        return False

    async def ros_async_sleep(self, seconds: float):
        """A ROS-compatible asynchronous sleep function that uses a one-shot timer."""
        future = Future()
        self.create_timer(seconds, lambda: future.set_result(None))
        await future

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist

    def robot_pose_callback(self, msg):
        """ 
        Update the robot's command velocity based on the current path and human absolute pose.
        """
        self.robot_pose = msg  
        self.robot_last_update_time = self.get_clock().now()

        # If no active goal, nothing to do
        if not getattr(self._active_goal_handle, 'is_active', False):
            return        
        
        # Determine status
        _human_updated = (self.human_last_update_time is not None and
                 (self.get_clock().now() - self.human_last_update_time).nanoseconds / 1e9 < self.lost_user_timeout)
        
        if _human_updated:
            if self.current_user_distance < self.lagging_dist_thres:
                self.follow_state = "FOLLOWING_USER"
            else:
                self.follow_state = "LAGGING_BEHIND_USER"
        else:
            self.follow_state = "LOST_USER"
            self.get_logger().warn("LOST_USER: No relative pose update recently")

        if self.human_absolute_pose:
            now = self.get_clock().now()
            if (now - self.last_path_search_time).nanoseconds > 0.5 * 1e9:
                self._update_path(self.human_absolute_pose)
                self.last_path_search_time = now
            
            cmd_vel = self.pure_pursuit_controller.compute_velocity_commands(self.robot_pose, self.current_velocity)
            self.cmd_vel_publisher.publish(cmd_vel)
            self.get_logger().info(f"Following path. Vel: {cmd_vel.linear.x:.2f}, Ang: {cmd_vel.angular.z:.2f}")
            # self.get_logger().info(f"Following path. Vel: {cmd_vel.linear.x:.2f}, Ang: {cmd_vel.angular.z:.2f}, human_abs: ({self.human_absolute_pose.pose.position.x:.2f}, {self.human_absolute_pose.pose.position.y:.2f}), last_path_point: ({self.pure_pursuit_controller.path_.poses[-1].pose.position.x:.2f}, {self.pure_pursuit_controller.path_.poses[-1].pose.position.y:.2f})")
            
            # Update head tracking
            try:
                self._update_head_tracking(self.robot_pose, self.human_absolute_pose)
            except Exception:
                pass
            
    def human_relative_pose_front_callback(self, msg):
        """
        Apply the shrinking in 3D (x, y, z): move the target point closer to the camera by `FOLLOW_USER_OFFSET`
        along the vector from the camera origin. Guards against near-zero distances.
        """
        processed_pose = deepcopy(msg)

        # 3D distance from camera origin to the detected human point
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        dist = math.sqrt(x * x + y * y + z * z)

        # Avoid division by zero and only shrink if farther than the configured offset
        if dist > FOLLOW_USER_OFFSET:
            scale = (dist - FOLLOW_USER_OFFSET) / dist
            processed_pose.pose.position.x = x * scale
            processed_pose.pose.position.y = y * scale
            processed_pose.pose.position.z = z * scale
        else:
            processed_pose.pose.position.x = 0.0
            processed_pose.pose.position.y = 0.0
            processed_pose.pose.position.z = 0.0

        self.current_user_distance = dist
        self.human_last_update_time = self.get_clock().now()

        # Update the human absolute pose
        self._update_human_absolute_pose(processed_pose)

    def _update_human_absolute_pose(self, human_relative_pose_offset):
        """
        Update the human's absolute pose based on the human relative pose from the front camera.
        Note: Fix the neck pitch angle to STATIC_NECK_PITCH_DEG (15 degrees) to face horizontally.
        """
        try:
            if not self.robot_pose: return

            # Create a PoseStamped message for the human in the camera frame
            human_in_camera_frame = deepcopy(human_relative_pose_offset)
            x_cam = human_in_camera_frame.pose.position.x
            y_cam = human_in_camera_frame.pose.position.y
            z_cam = human_in_camera_frame.pose.position.z

            # Update neck angles from actual encoder feedback to avoid unstable feedback loop
            self.neck_yaw = math.radians(self.head_controller.current_neck_yaw_deg) if self.control_head else STATIC_NECK_ANGLE
            self.neck_pitch = math.radians(self.head_controller.current_neck_pitch_deg) if self.control_head else STATIC_NECK_ANGLE

            # # Rotation around Y-axis (pitch) - commented out to keep fixed pitch
            # x_p = x_cam * math.cos(self.neck_pitch) + z_cam * math.sin(self.neck_pitch)
            # y_p = y_cam
            # z_p = -x_cam * math.sin(self.neck_pitch) + z_cam * math.cos(self.neck_pitch)

            # Rotation around Z-axis (yaw) - Replace x_p, y_p, z_p with x_cam, y_cam, z_cam
            x_b = x_cam * math.cos(self.neck_yaw) - y_cam * math.sin(self.neck_yaw)
            y_b = x_cam * math.sin(self.neck_yaw) + y_cam * math.cos(self.neck_yaw)
            z_b = z_cam

            # Manually transform from base_link to the map frame
            robot_yaw = self._get_yaw_from_quaternion(self.robot_pose.pose.orientation)
            x_robot = self.robot_pose.pose.position.x
            y_robot = self.robot_pose.pose.position.y

            x_map = x_robot + (x_b * math.cos(robot_yaw) - y_b * math.sin(robot_yaw))
            y_map = y_robot + (x_b * math.sin(robot_yaw) + y_b * math.cos(robot_yaw))
            
            # Create the absolute pose message
            self.human_absolute_pose = PoseStamped()
            self.human_absolute_pose.header.frame_id = 'slamware_map'
            self.human_absolute_pose.header.stamp = self.get_clock().now().to_msg()
            self.human_absolute_pose.pose.position.x = x_map
            self.human_absolute_pose.pose.position.y = y_map
            self.human_absolute_pose.pose.position.z = self.robot_pose.pose.position.z + z_b # Approximate height
            self.human_absolute_pose.pose.orientation = self.robot_pose.pose.orientation # Orientation is not critical here
            
            self.human_absolute_pose_publisher.publish(self.human_absolute_pose)

        except Exception as e:
            self.get_logger().error(f'Could not calculate human absolute pose: {e}')
            self.human_absolute_pose = None
            return

    def _get_yaw_from_quaternion(self, q):
        # Conversion from quaternion to yaw (rotation around z-axis)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _update_path(self, human_absolute_pose):
        if self.robot_pose is None or human_absolute_pose is None:
            self.get_logger().warn("Robot or human absolute pose is not available, skipping path update.")
            return

        try:
            target_x = human_absolute_pose.pose.position.x
            target_y = human_absolute_pose.pose.position.y
            path_points = self.motion_utils.search_path(self.robot_ip, target_x, target_y)

            if path_points:
                self.path_msg = self.motion_utils.convert_points_to_path(path_points, 'slamware_map')
                self.pure_pursuit_controller.set_path(self.path_msg)
                self.path_publisher.publish(self.path_msg)
                self.get_logger().info(f"Update {len(self.path_msg.poses)}-point path with the last point: ({self.path_msg.poses[-1].pose.position.x:.2f}, {self.path_msg.poses[-1].pose.position.y:.2f})")
            else:
                self.pure_pursuit_controller.set_path(None)

        except Exception as e:
            self.get_logger().error(f'Error in path update: {e}')
            self.pure_pursuit_controller.set_path(None)

    def _update_head_tracking(self, robot_pose, human_absolute_pose):
        # Calculate the vector from robot to human in the map frame
        dx = human_absolute_pose.pose.position.x - robot_pose.pose.position.x
        dy = human_absolute_pose.pose.position.y - robot_pose.pose.position.y
        dz = human_absolute_pose.pose.position.z - robot_pose.pose.position.z

        # Distance to the human
        distance = math.sqrt(dx**2 + dy**2) # Use 2D distance for yaw tracking logic

        # Get robot's yaw
        robot_yaw = self._get_yaw_from_quaternion(robot_pose.pose.orientation)

        # To transform the vector from map frame to robot's base_link frame,
        # we rotate it by the inverse of the robot's yaw.
        x_base = dx * math.cos(-robot_yaw) - dy * math.sin(-robot_yaw)
        y_base = dx * math.sin(-robot_yaw) + dy * math.cos(-robot_yaw)

        # Calculate target yaw in radians relative to the robot's base_link frame
        target_yaw_rad = math.atan2(y_base, x_base)

        if self.control_head and self.head_controller:
            # Get current state from head_controller
            current_neck_yaw_rad = math.radians(self.head_controller.current_neck_yaw_deg)
            current_neck_pitch_rad = math.radians(self.head_controller.current_neck_pitch_deg)
            current_neck_yaw_vel_rps = math.radians(self.head_controller.current_neck_yaw_vel_dps)
            base_angular_vel_rps = self.current_velocity.angular.z

            # Calculate desired neck velocities using the new tracker
            yaw_vel_dps, pitch_vel_dps = self.head_tracker.calculate_velocities(
                target_yaw_rad=target_yaw_rad,
                target_dist=distance,
                current_neck_yaw_rad=current_neck_yaw_rad,
                current_neck_yaw_vel_rps=current_neck_yaw_vel_rps,
                base_angular_vel_rps=base_angular_vel_rps,
                current_neck_pitch_rad=current_neck_pitch_rad
            )

            self.get_logger().info(
                f"Head tracking: TargetYaw: {math.degrees(target_yaw_rad):.1f}°, "
                f"CurrentYaw: {math.degrees(current_neck_yaw_rad):.1f}°, "
                f"Dist: {distance:.2f}m | "
                f"Vels (dps): Yaw={yaw_vel_dps:.2f}, Pitch={pitch_vel_dps:.2f}"
            )

            # Send velocity command to head
            self.head_controller.control_head_velocity(yaw_vel_dps, pitch_vel_dps)

    def _reset_robot_status(self):
        self._active_goal_handle = None
        self.human_absolute_pose = None
        self.path_msg = None
        self.follow_state = ""
        self.cmd_vel_publisher.publish(Twist())
        self.path_publisher.publish(Path())

        if self.control_head and self.head_controller:
            # Set neck to default pose
            self.head_controller.control_head(yaw_deg=STATIC_NECK_ANGLE,pitch_deg=STATIC_NECK_PITCH_DEG)

    # ---------- ActionServer callbacks ----------
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('|----------------------------------------|')
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """A new goal has been accepted."""
        self.get_logger().info('Goal accepted, starting execution.')
        goal_handle.execute()

    async def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request.')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """
        This loop will honor cancel requests via goal_handle.is_cancel_requested (call if necessary).
        """
        self._active_goal_handle = goal_handle
        self.get_logger().info(f'Executing goal for user: {goal_handle.request.user_id}')
        try:
            while rclpy.ok() and goal_handle.is_active:
                if self.control_head:
                    # Check if head controller is still alive, if not, re-initialize
                    is_connected = False
                    if self.head_controller:
                        # A quick check; get_firmware_version also handles listener thread status
                        version_info = self.head_controller.get_firmware_version(timeout=1)
                        if "error" not in version_info:
                            is_connected = True

                    if not is_connected:
                        self.get_logger().warn("Head controller connection lost or not initialized. Attempting to (re)initialize...")
                        self._init_head_controller()

                feedback_msg = FollowUser.Feedback()
                # Quick checks
                _robot_updated = (self.robot_last_update_time and
                                (self.get_clock().now() - self.robot_last_update_time).nanoseconds / 1e9 < 0.2)

                if not _robot_updated:
                    self.follow_state = "WAITING_ROBOT_POSE"
                    self.get_logger().warn("Robot pose is unavailable. Please check the connection with the AMR.")
                else:
                    feedback_msg.current_pose = self.robot_pose

                if not self.human_last_update_time:
                    self.get_logger().warn("Human pose is unavailable. Please check the CV service.")
                else:
                    feedback_msg.current_user_distance = self.current_user_distance

                # Check for cancel request
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    
                    # Stop the robot and reset
                    self._reset_robot_status()
                    self.get_logger().info('Goal canceled by the client.')

                    return FollowUser.Result(message='Goal canceled by the client.')

                feedback_msg.status = self.follow_state
                goal_handle.publish_feedback(feedback_msg)

                await self.ros_async_sleep(0.1)

            # If the loop exits because the goal is no longer active (but not cancelled), succeed it.
            goal_handle.succeed()
            self._active_goal_handle = None
            return FollowUser.Result(message='Follow task finished.')
        
        finally:
            # ensure robot stopped on any exit path
            try:
                # Stop the robot and reset
                self._reset_robot_status()
            except Exception:
                pass

    def destroy_node(self):
        if self.head_controller:
            self.head_controller.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Follow user motion node with path planning and head control.")
    parser.add_argument("--robot_ip", default='192.168.11.1', help="The IP address of the robot.")
    parser.add_argument("--no_head_control", action="store_true", help="Disable head control.")
    args, _ = parser.parse_known_args()
    
    node = FollowUserMotionNode(robot_ip=args.robot_ip, control_head=not args.no_head_control)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
