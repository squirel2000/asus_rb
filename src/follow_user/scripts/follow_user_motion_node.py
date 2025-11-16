#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
import argparse
import math
import numpy as np
import tf2_ros
from utils.head_control import HeadController
from utils.motion_utils import MotionUtils
from utils.pure_pursuit_controller import PurePursuitController
from motion_common.action import FollowUser
from copy import deepcopy

# Constants
CAMERA_OFFSET = np.array([0.0, 0.0, 0.0]) # Camera position (x,y,z) in base_link frame
FOLLOW_USER_OFFSET = 0.25 # Move the target point closer to avoid path not found issues
STATIC_NECK_ANGLE = 0.0
STATIC_NECK_PITCH_DEG = 15.0  # Fixed neck pitch angle to face horizontally

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
            self.head_controller = HeadController(self.get_logger())
            self.head_controller.start_listening()
            # Use get_firmware_version to activate the automatic update for the neck angles
            self.head_controller.get_firmware_version(timeout=3)

        # State
        self._active_goal_handle = None
        self.robot_pose = None
        self.current_velocity = Twist()
        self.path_msg = None
        self.last_path_search_time = self.get_clock().now()
        self.neck_yaw, self.neck_pitch = 0.0, 0.0
        self.human_absolute_pose = None
        self.human_last_update_time = None
        self.current_user_distance = None

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

        self.get_logger().info('Follow User Motion Node has been started.')
        
        self.get_logger().info(
            f"Parameters: "
            f"control_head={self.control_head}, "
            f"lagging_dist_thres={self.lagging_dist_thres:.2f}, "
            f"lost_user_timeout={self.lost_user_timeout:.2f}, "
        )

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist

    def robot_pose_callback(self, msg):
        """ 
        Update the robot's command velocity based on the current path and human absolute pose.
        """
        self.robot_pose = msg  

        # If no active goal, nothing to do
        if not getattr(self._active_goal_handle, 'is_active', False):
            return        
        
        # Determine status
        _human_updated = (self.human_last_update_time is not None and
                 (self.get_clock().now() - self.human_last_update_time).nanoseconds / 1e9 < self.lost_user_timeout)
        
        feedback_msg = FollowUser.Feedback()
        feedback_msg.current_pose = self.robot_pose

        if _human_updated:
            if self.current_user_distance < self.lagging_dist_thres:
                feedback_msg.status = "FOLLOWING_USER"
            else:
                feedback_msg.status = "LAGGING_BEHIND_USER"
        else:
            feedback_msg.status = "LOST_USER"
            self.get_logger().warn("LOST_USER: No relative pose update recently")

        if self.human_absolute_pose:
            feedback_msg.current_user_distance = self.current_user_distance
            now = self.get_clock().now()
            if (now - self.last_path_search_time).nanoseconds > 0.5 * 1e9:
                self._update_path(self.human_absolute_pose)
                self.last_path_search_time = now
            
            cmd_vel = self.pure_pursuit_controller.compute_velocity_commands(self.robot_pose, self.current_velocity)
            self.cmd_vel_publisher.publish(cmd_vel)
            self.get_logger().info(f"Following path. Vel: {cmd_vel.linear.x:.2f}, Ang: {cmd_vel.angular.z:.2f}")
            # self.get_logger().info(f"Following path. Vel: {cmd_vel.linear.x:.2f}, Ang: {cmd_vel.angular.z:.2f}, human_abs: ({self.human_absolute_pose.pose.position.x:.2f}, {self.human_absolute_pose.pose.position.y:.2f}), last_path_point: ({self.pure_pursuit_controller.path_.poses[-1].pose.position.x:.2f}, {self.pure_pursuit_controller.path_.poses[-1].pose.position.y:.2f})")
            
            # Update head tracking
            self._update_head_tracking(self.robot_pose, self.human_absolute_pose)

        # Publish feedback to action client
        self._active_goal_handle.publish_feedback(feedback_msg)

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
        Update the human's absolute pose by transforming the relative pose from the
        camera's frame to the map frame using TF2. This is the correct, robust way to handle
        the coordinate transformations and avoids feedback-induced oscillations.
        """
        try:
            # The vision node should publish the relative pose with the frame_id
            # 'front_camera_color_optical_frame'.
            # We want to transform it to the 'slamware_map' frame.
            # TF2 will handle all intermediate transforms (camera -> neck -> base_link -> odom -> map)
            # using the timestamp from the human pose message, which prevents the feedback loop.
            self.human_absolute_pose = self.tf_buffer.transform(
                human_relative_pose_offset,
                'slamware_map',
                timeout=rclpy.duration.Duration(seconds=0.1) # Short timeout
            )
            self.human_absolute_pose_publisher.publish(self.human_absolute_pose)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform human pose to map frame: {e}')
            self.human_absolute_pose = None

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
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        # Get robot's yaw
        robot_yaw = self._get_yaw_from_quaternion(robot_pose.pose.orientation)

        # To transform the vector from map frame to robot's base_link frame,
        # we rotate it by the inverse of the robot's yaw.
        x_base = dx * math.cos(-robot_yaw) - dy * math.sin(-robot_yaw)
        y_base = dx * math.sin(-robot_yaw) + dy * math.cos(-robot_yaw)
        z_base = dz

        # Calculate yaw and pitch in radians relative to the robot's base_link frame
        yaw = math.atan2(y_base, x_base)
        # Replace "pitch = math.atan2(-z_base, math.sqrt(x_base**2 + y_base**2))" with fixed pitch angle
        pitch  = math.radians(STATIC_NECK_PITCH_DEG)  # Use fixed pitch angle

        # Convert to degrees for the head controller
        yaw_deg = math.degrees(yaw)
        pitch_deg = math.degrees(pitch)

        self.get_logger().info(f"Head tracking: ({math.degrees(self.neck_yaw)}°, {math.degrees(self.neck_pitch)}°) -> ({yaw_deg:.2f}°, {pitch_deg:.2f}°), Dist: {distance:.2f}m")
        if self.control_head and self.head_controller:
            self.head_controller.control_head(yaw_deg, pitch_deg)

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

    def cancel_callback(self, goal_handle):
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
                
                # Quick checks & feedback
                feedback_msg = FollowUser.Feedback(status="CHECKING_EXECUTION_STATUS")
                if not self.robot_pose:
                    self.get_logger().warn("Robot pose is unavailable. Please check the connection with the AMR.")
                    goal_handle.publish_feedback(feedback_msg)
                if not self.human_last_update_time:
                    self.get_logger().warn("Human pose is unavailable. Please check the CV service.")
                    goal_handle.publish_feedback(feedback_msg)

                # Check for cancel request
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._active_goal_handle = None
                    self.get_logger().info('Goal canceled by the client.')

                    # Stop the robot and clear the path
                    self.human_absolute_pose = None
                    self.cmd_vel_publisher.publish(Twist())
                    self.path_publisher.publish(Path())

                    return FollowUser.Result(message='Goal canceled by the client.')

                rclpy.spin_once(self, timeout_sec=0.1)

            # If the loop exits because the goal is no longer active (but not cancelled), succeed it.
            goal_handle.succeed()
            self._active_goal_handle = None
            return FollowUser.Result(message='Follow task finished.')
        
        finally:
            # ensure robot stopped on any exit path
            try:
                self._active_goal_handle = None
                self.human_absolute_pose = None
                self.cmd_vel_publisher.publish(Twist())
                self.path_publisher.publish(Path())
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
