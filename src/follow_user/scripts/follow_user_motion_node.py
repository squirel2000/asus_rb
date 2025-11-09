#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
import argparse
import math
import numpy as np
import tf2_ros
from utils.head_control import HeadController
from utils.motion_utils import MotionUtils
from utils.pure_pursuit_controller import PurePursuitController
from follow_user.action import FollowUser
from copy import deepcopy

# Constants
CAMERA_OFFSET = np.array([0.0, 0.0, 0.0]) # Camera position (x,y,z) in base_link frame
FOLLOW_USER_OFFSET = 0.25 # Move the target point closer to avoid path not found issues

class FollowUserMotionNode(Node):
    """
    This node controls the robot's motion to follow a user using an action server.
    It receives relative user pose, controls the head, calculates the user's absolute pose,
    searches for a path, and publishes it.
    """
    def __init__(self, robot_ip, control_head=False):
        super().__init__('follow_user_motion_node')
        self.get_logger().info('Follow User Motion Node has been started.')

        self.robot_ip = robot_ip
        self.control_head = control_head

        # Action Server
        self._action_server = ActionServer(
            self,
            FollowUser,
            'follow_user',
            self.execute_callback)

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
        if self.control_head:
            self.head_controller = HeadController(self.get_logger())
        else:
            self.head_controller = None

        # State
        self.goal_handle = None
        self.robot_pose = None
        self.current_velocity = Twist()
        self.path_msg = None
        self.last_path_search_time = self.get_clock().now()
        self.neck_yaw, self.neck_pitch = 0.0 , 0.0
        self.human_relative_pose, self.human_absolute_pose = None, None

        # Parameters for smooth rotation
        self.declare_parameter('angular_acceleration', 0.20)
        self.declare_parameter('angular_deceleration', 0.40)

        # Parameters for pure pursuit
        self.declare_parameter("lookahead_dist", 1.0)
        self.declare_parameter("min_lookahead_dist", 0.5)
        self.declare_parameter("max_lookahead_dist", 1.5)
        self.declare_parameter("lookahead_time", 1.5)
        self.declare_parameter("desired_linear_vel", 0.5)
        self.declare_parameter("max_linear_vel", 0.20)
        self.declare_parameter('max_angular_vel', 1.2)
        self.declare_parameter("max_angular_acceleration", 1.0)
        self.declare_parameter("heading_error_for_pure_rotation", 1.57)
        self.declare_parameter("min_heading_error_for_motion", 0.35)
        self.declare_parameter("min_approach_linear_velocity", 0.05)
        self.declare_parameter("approach_velocity_scaling_dist", 0.6)
        self.declare_parameter("goal_dist_buf", 0.15)
        self.declare_parameter("goal_dist_tol", 0.075)
        self.declare_parameter("linear_acceleration", 0.3)
        self.declare_parameter("linear_deceleration", 0.6)
        self.declare_parameter("target_velocity_ema_alpha", 0.2)
        
        self.motion_params = {
            'max_angular_vel': self.get_parameter('max_angular_vel').get_parameter_value().double_value,
            'angular_acceleration': self.get_parameter('angular_acceleration').get_parameter_value().double_value,
            'angular_deceleration': self.get_parameter('angular_deceleration').get_parameter_value().double_value
        }

        # Utility classes
        self.motion_utils = MotionUtils(self)
        self.pure_pursuit_controller = PurePursuitController(self)

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist

    def robot_pose_callback(self, msg):
        """ Update the robot's command velocity based on the current path and human absolute pose.
        """
        self.robot_pose = msg        
        if self.goal_handle is None or not self.goal_handle.is_active:
            return

        if self.human_absolute_pose:
            now = self.get_clock().now()
            if (now - self.last_path_search_time).nanoseconds > 0.5 * 1e9:
                self._update_path(self.human_absolute_pose)
                self.last_path_search_time = now
            
            cmd_vel = self.pure_pursuit_controller.compute_velocity_commands(self.robot_pose, self.current_velocity)
            self.cmd_vel_publisher.publish(cmd_vel)
            # self.get_logger().info(f"Following path. Vel: {cmd_vel.linear.x:.2f}, Ang: {cmd_vel.angular.z:.2f}, human_abs: ({self.human_absolute_pose.pose.position.x:.2f}, {self.human_absolute_pose.pose.position.y:.2f}), last_path_point: ({self.pure_pursuit_controller.path_.poses[-1].pose.position.x:.2f}, {self.pure_pursuit_controller.path_.poses[-1].pose.position.y:.2f})")

    def human_relative_pose_front_callback(self, msg):
        """Apply the shrinking in 3D (x, y, z): move the target point closer to the camera by `FOLLOW_USER_OFFSET`
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
        self.human_relative_pose = processed_pose
        
        # Update the human absolute pose
        self._update_human_absolute_pose(self.human_relative_pose)

    def _update_human_absolute_pose(self, human_relative_pose):
        """Update the human's absolute pose based on the human relative pose from the front camera.
        """
        try:
            # Create a PoseStamped message for the human in the camera frame
            human_in_camera_frame = deepcopy(human_relative_pose)
            x_cam = human_in_camera_frame.pose.position.x
            y_cam = human_in_camera_frame.pose.position.y
            z_cam = human_in_camera_frame.pose.position.z

            # Rotation around Y-axis (pitch)
            x_p = x_cam * math.cos(self.neck_pitch) + z_cam * math.sin(self.neck_pitch)
            y_p = y_cam
            z_p = -x_cam * math.sin(self.neck_pitch) + z_cam * math.cos(self.neck_pitch)

            # Rotation around Z-axis (yaw)
            x_b = x_p * math.cos(self.neck_yaw) - y_p * math.sin(self.neck_yaw)
            y_b = x_p * math.sin(self.neck_yaw) + y_p * math.cos(self.neck_yaw)
            z_b = z_p

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


    def get_head_angles_to_human(self, relative_pose_msg):
        # TODO: The calculated angle is relative to camera, but the neck yaw/pitch angles are relative to the robot base_link frame.
        x, y, z = relative_pose_msg.pose.position.x, relative_pose_msg.pose.position.y, relative_pose_msg.pose.position.z
        point_vec = np.array([x, y, z])
        vec_from_camera = point_vec - CAMERA_OFFSET
        x_cam, y_cam, z_cam = vec_from_camera
        distance = math.sqrt(x_cam**2 + y_cam**2)
        yaw = math.atan2(y_cam, x_cam)
        pitch = math.atan2(z_cam, math.sqrt(x_cam**2 + y_cam**2))
        return yaw, pitch, distance

    def _update_head_tracking(self, relative_pose_msg):
        yaw, pitch, distance = self.get_head_angles_to_human(relative_pose_msg)
        
        # Store neck angles
        self.neck_yaw = yaw
        self.neck_pitch = pitch

        self.get_logger().info(f"Head tracking -> Dist: {distance:.2f}m, Yaw: {math.degrees(yaw):.2f}°, Pitch: {math.degrees(pitch):.2f}°")
        if self.head_controller:
            self.head_controller.control_head(yaw, pitch)
        
        feedback_msg = FollowUser.Feedback()
        feedback_msg.current_user_distance = distance
        feedback_msg.status = "Following user..."
        feedback_msg.current_pose = self.robot_pose

        if self.goal_handle and self.goal_handle.is_active:
            self.goal_handle.publish_feedback(feedback_msg)


    def execute_callback(self, goal_handle):
        self.goal_handle = goal_handle
        self.get_logger().info(f'Executing goal for user: {self.goal_handle.request.user_id}')

        while rclpy.ok() and self.goal_handle.is_active:
            if self.goal_handle.is_cancel_requested:
                self.goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                # Stop the robot and clear the path
                self.cmd_vel_publisher.publish(Twist())
                self.path_publisher.publish(Path())
                self.goal_handle = None
                return FollowUser.Result(final_status='Goal canceled')
            rclpy.spin_once(self, timeout_sec=0.1)

        # If the loop exits because the goal is no longer active (but not cancelled), succeed it.
        self.goal_handle.succeed()
        self.goal_handle = None
        return FollowUser.Result(final_status='Follow task finished.')
    
    def destroy_node(self):
        if self.head_controller:
            self.head_controller.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Follow user motion node with path planning and head control.")
    parser.add_argument("--robot-ip", required=True, help="The IP address of the robot.")
    parser.add_argument("--no-head-control", action="store_true", help="Disable head control.")
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
