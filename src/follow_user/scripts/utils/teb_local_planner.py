#!/usr/bin/env python3
import math
import time
from threading import Lock

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class TebLocalPlanner:
    """
    Minimal TEB-like local planner implemented in pure Python.
    - consumes LaserScan and Odometry
    - computes obstacle-aware, smoothed velocity commands to follow a dynamic goal (user)
    This is not the full TEB optimizer; it's a reactive, smooth local planner that
    mimics TEB behavior suitable for follow-user with LiDAR.
    """
    def __init__(self, node,
                 max_linear_vel=0.20,
                 max_angular_vel=1.2,
                 linear_accel=0.3,
                 angular_accel=1.0,
                 close_goal_dist=0.75,
                 obstacle_influence=1.0,
                 obstacle_clearance=0.45,
                 k_heading=1.8,
                 k_rotate=1.4,
                 k_obs=0.9):
        self.node = node
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.linear_accel = linear_accel
        self.angular_accel = angular_accel
        self.close_goal_dist = close_goal_dist
        self.obstacle_influence = obstacle_influence
        self.obstacle_clearance = obstacle_clearance
        self.k_heading = k_heading
        self.k_rotate = k_rotate
        self.k_obs = k_obs

        self.scan = None
        self.scan_time = None
        self.odom = None
        self.odom_time = None

        self.last_cmd = Twist()
        self.last_time = time.time()
        self.lock = Lock()

    def update_scan(self, scan_msg: LaserScan):
        with self.lock:
            self.scan = scan_msg
            self.scan_time = time.time()

    def update_odom(self, odom_msg: Odometry):
        with self.lock:
            self.odom = odom_msg
            self.odom_time = time.time()

    def _get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _limit(self, value, min_v, max_v):
        if value > max_v: return max_v
        if value < min_v: return min_v
        return value

    def _normalize_angle(self, ang):
        while ang > math.pi: ang -= 2*math.pi
        while ang < -math.pi: ang += 2*math.pi
        return ang

    def _compute_min_front_distance(self, front_ang_width=math.radians(60)):
        """
        Compute minimum distance in the front sector (±front_ang_width/2).
        Returns large number if no scan available.
        """
        with self.lock:
            scan = self.scan
        if scan is None:
            return float('inf')

        angle = scan.angle_min
        min_dist = float('inf')
        half = front_ang_width / 2.0
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min < r < scan.range_max:
                if -half <= angle <= half:
                    if r < min_dist:
                        min_dist = r
            angle += scan.angle_increment
        return min_dist

    def _compute_obstacle_steer(self, front_ang_width=math.radians(120)):
        """
        Compute an angular bias to steer away from nearby obstacles using the laser scan.
        Positive returned value means steer left (positive angular), negative steer right.
        """
        with self.lock:
            scan = self.scan
        if scan is None:
            return 0.0

        angle = scan.angle_min
        steer = 0.0
        total_weight = 0.0
        half = front_ang_width / 2.0
        for r in scan.ranges:
            if not math.isfinite(r):
                angle += scan.angle_increment
                continue
            if r <= scan.range_min or r >= scan.range_max:
                angle += scan.angle_increment
                continue
            if -half <= angle <= half:
                if r < self.obstacle_influence:
                    # weight closer obstacles heavier, and lateral component influences steering
                    weight = (self.obstacle_influence - r) / (self.obstacle_influence + 1e-6)
                    steer += weight * math.sin(angle) / (r + 1e-3)
                    total_weight += abs(weight)
            angle += scan.angle_increment
        if total_weight == 0:
            return 0.0
        # steer is sum of lateral influences; scale with k_obs
        return self.k_obs * steer / (total_weight + 1e-6)

    def compute_velocity_commands(self, robot_pose: PoseStamped, current_twist, goal_pose: PoseStamped):
        """
        Compute and return a geometry_msgs/Twist command.
        - robot_pose: current pose in map frame
        - current_twist: nav_msgs/Twist (from odom) containing current velocities
        - goal_pose: PoseStamped (user position) in same frame as robot_pose
        """
        now = time.time()
        dt = now - self.last_time if self.last_time is not None else 1.0 / 20.0
        dt = max(1e-3, dt)
        self.last_time = now

        # Extract positions & heading
        rx = robot_pose.pose.position.x
        ry = robot_pose.pose.position.y
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y

        robot_yaw = self._get_yaw_from_quaternion(robot_pose.pose.orientation)
        dx = gx - rx
        dy = gy - ry
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        heading_error = self._normalize_angle(target_yaw - robot_yaw)

        # Obstacle info
        min_front = self._compute_min_front_distance()
        obs_bias = self._compute_obstacle_steer()

        desired_linear = 0.0
        desired_angular = 0.0

        if distance < self.close_goal_dist:
            # Close to target: rotate in place to face the user
            desired_linear = 0.0
            # Use rotate gain
            desired_angular = self._limit(self.k_rotate * heading_error,
                                          -self.max_angular_vel, self.max_angular_vel)
        else:
            # Move towards the user with heading correction
            # Reduce linear velocity when heading error large
            heading_scale = max(0.0, math.cos(heading_error))
            base_speed = self.max_linear_vel * heading_scale
            # Scale by clearance factor
            if min_front == float('inf'):
                clearance_scale = 1.0
            else:
                clearance_scale = (min_front - self.obstacle_clearance) / (self.obstacle_influence - self.obstacle_clearance + 1e-6)
                clearance_scale = max(0.0, min(1.0, clearance_scale))
            desired_linear = base_speed * clearance_scale

            # Angular velocity: heading controller plus obstacle bias
            desired_angular = self.k_heading * heading_error + obs_bias
            desired_angular = self._limit(desired_angular, -self.max_angular_vel, self.max_angular_vel)

            # If obstacle very close in front, reduce forward speed strongly and add stronger bias
            if min_front < self.obstacle_clearance:
                desired_linear = 0.0
                desired_angular += self.k_obs * (1.0 / (min_front + 1e-3)) * (1.0 if obs_bias >= 0 else -1.0)
                desired_angular = self._limit(desired_angular, -self.max_angular_vel, self.max_angular_vel)

        # Smooth using acceleration limits (respect provided odom if available)
        last_lin = self.last_cmd.linear.x
        last_ang = self.last_cmd.angular.z

        max_lin_delta = self.linear_accel * dt
        max_ang_delta = self.angular_accel * dt

        lin_delta = desired_linear - last_lin
        if lin_delta > max_lin_delta:
            desired_linear = last_lin + max_lin_delta
        elif lin_delta < -max_lin_delta:
            desired_linear = last_lin - max_lin_delta

        ang_delta = desired_angular - last_ang
        if ang_delta > max_ang_delta:
            desired_angular = last_ang + max_ang_delta
        elif ang_delta < -max_ang_delta:
            desired_angular = last_ang - max_ang_delta

        # Respect absolute limits
        desired_linear = self._limit(desired_linear, -self.max_linear_vel, self.max_linear_vel)
        desired_angular = self._limit(desired_angular, -self.max_angular_vel, self.max_angular_vel)

        cmd = Twist()
        cmd.linear.x = desired_linear
        cmd.angular.z = desired_angular

        # Save last cmd
        self.last_cmd = cmd

        # Debug logging occasionally
        if self.node:
            if (int(now * 10) % 20) == 0:
                self.node.get_logger().debug(
                    f"TebLocalPlanner: dist={distance:.2f}, heading_err={math.degrees(heading_error):.1f}°, "
                    f"min_front={min_front:.2f}, obs_bias={obs_bias:.3f}, cmd_lin={cmd.linear.x:.2f}, cmd_ang={cmd.angular.z:.2f}"
                )
        return cmd