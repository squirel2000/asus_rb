#!/usr/bin/env python3
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan

class DWAPlanner:
    """
    A lightweight Python implementation of the Dynamic Window Approach (DWA).
    Designed for local goal following (user position) with 360deg LiDAR input.
    """

    def __init__(self, node):
        self.node = node

        # Planner parameters (can be tuned)
        self.dt = 0.05  # control interval (s), ~20Hz
        self.predict_horizon = 1.0  # seconds
        self.steps = int(self.predict_horizon / self.dt)
        self.v_samples = 5
        self.w_samples = 7

        # robot limits
        self.max_v = node.get_parameter("max_linear_vel").get_parameter_value().double_value
        self.max_w = node.get_parameter("max_angular_vel").get_parameter_value().double_value
        self.max_acc = node.get_parameter("linear_acceleration").get_parameter_value().double_value
        self.max_ang_acc = node.get_parameter("max_angular_acceleration").get_parameter_value().double_value

        # scoring weights
        self.weight_goal = 1.0
        self.weight_clearance = 1.2
        self.weight_velocity = 0.1

        # robot footprint radius (safety)
        self.robot_radius = 0.35

        # minimum clearance to consider (m)
        self.min_allowed_clearance = 0.05

        # Pure rotation control parameters
        self.node.declare_parameter("rotate_only_dist", 0.75)
        self.node.declare_parameter("rotation_gain_k", 4.0)
        self.node.declare_parameter("smoothing_alpha", 0.8)
        self.rotate_only_dist = self.node.get_parameter("rotate_only_dist").get_parameter_value().double_value
        self.rotation_gain_k = self.node.get_parameter("rotation_gain_k").get_parameter_value().double_value
        self.smoothing_alpha = self.node.get_parameter("smoothing_alpha").get_parameter_value().double_value
        self.last_angular_vel = 0.0

    def _scan_to_points_global(self, scan: LaserScan, robot_pose):
        """Convert scan ranges to list of (x,y) points in map frame using robot_pose (PoseStamped)."""
        if scan is None or robot_pose is None:
            return np.empty((0,2))

        ranges = np.array(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        if not np.any(valid):
            return np.empty((0,2))

        angles = angles[valid]
        ranges = ranges[valid]

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        # transform from robot base frame to map frame using robot_pose yaw + pos
        q = robot_pose.pose.orientation
        robot_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        cos_r = math.cos(robot_yaw)
        sin_r = math.sin(robot_yaw)
        x_map = robot_pose.pose.position.x + (xs * cos_r - ys * sin_r)
        y_map = robot_pose.pose.position.y + (xs * sin_r + ys * cos_r)
        return np.vstack((x_map, y_map)).T

    def _simulate_trajectory(self, v, w, x0, y0, yaw0):
        """
        Simulate robot trajectory applying constant v and w over the prediction horizon.
        Returns array of poses shape (N,3).
        """
        traj = np.zeros((self.steps, 3))
        x, y, yaw = x0, y0, yaw0
        for i in range(self.steps):
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt
            traj[i, :] = [x, y, yaw]
        return traj

    def _clearance_cost(self, traj, obstacles):
        """
        Compute min clearance from trajectory to obstacles (both in map frame).
        Return a clearance score (larger is better).
        """
        if obstacles.size == 0:
            return 1.0  # no obstacles: excellent clearance

        min_d = float('inf')
        for (x, y, _) in traj:
            # compute distances to all obstacles, but break early if very close
            dists = np.hypot(obstacles[:,0] - x, obstacles[:,1] - y)
            dmin = dists.min() if dists.size>0 else float('inf')
            if dmin < min_d:
                min_d = dmin
            if min_d <= 0.0:
                return 0.0
            if min_d < self.robot_radius + self.min_allowed_clearance:
                # immediate reject
                return 0.0
        # normalize clearance into (0,1]
        return min(1.0, max(0.0, (min_d - self.robot_radius) / 2.0))

    def _goal_cost(self, traj, goal):
        """
        cost based on final pose distance to goal (smaller cost for closer).
        """
        last = traj[-1]
        dx = goal.pose.position.x - last[0]
        dy = goal.pose.position.y - last[1]
        dist = math.hypot(dx, dy)
        return dist

    def compute_velocity_commands(self, robot_pose, current_velocity: Twist, scan, local_goal: PoseStamped):
        """
        Main planning function. Returns a Twist with the chosen (v,w).
        The logic is as follows:
        1. If the robot is very close to the goal (e.g., within 0.75m), it will only rotate to face the user.
        2. Otherwise, it will generate a set of candidate velocities (v,w) within a dynamic window that respects the robot's acceleration limits.
        3. For each candidate velocity, it will simulate a trajectory over a short time horizon.
        4. Each trajectory is evaluated based on three criteria:
            - Goal Cost: How close the trajectory gets to the user.
            - Clearance Cost: The closest distance to any obstacle detected by the LiDAR.
            - Velocity Score: A small bonus for maintaining forward speed.
        5. The velocity pair that results in the best-scoring trajectory is selected.
        6. If no valid trajectory is found (e.g., all paths lead to a collision), the robot will stop and rotate towards the user as a fallback.
        7. The final command is clamped to ensure it respects acceleration limits.
        """
        cmd = Twist()
        if robot_pose is None or current_velocity is None or scan is None or local_goal is None:
            return cmd  # zero

        # distance to local goal (2D)
        dxg = local_goal.pose.position.x - robot_pose.pose.position.x
        dyg = local_goal.pose.position.y - robot_pose.pose.position.y
        dist_to_goal = math.hypot(dxg, dyg)
        angle_to_goal = math.atan2(dyg, dxg)
        # robot yaw
        q = robot_pose.pose.orientation
        robot_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

        # if close, only rotate to face user
        if dist_to_goal < self.rotate_only_dist:
            yaw_err = self._angle_diff(angle_to_goal, robot_yaw)
            # Use non-linear gain for faster response and smooth stop
            desired_w = self.max_w * math.tanh(self.rotation_gain_k * yaw_err)

            # apply angular acceleration limit relative to last angular vel
            max_dw = self.max_ang_acc * self.dt
            dw = desired_w - self.last_angular_vel
            if abs(dw) > max_dw:
                desired_w = self.last_angular_vel + math.copysign(max_dw, dw)

            # smoothing filter
            final_w = self.smoothing_alpha * desired_w + (1 - self.smoothing_alpha) * self.last_angular_vel
            self.last_angular_vel = final_w
            cmd.angular.z = final_w
            cmd.linear.x = 0.0
            return cmd

        # Build obstacle points in map frame
        obstacles = self._scan_to_points_global(scan, robot_pose)

        # current velocities from Twist
        v0 = getattr(current_velocity, 'linear', None).x if current_velocity is not None else 0.0
        w0 = getattr(current_velocity, 'angular', None).z if current_velocity is not None else 0.0

        # dynamic window: feasible v,w ranges respecting acceleration limits
        v_min = max(0.0, v0 - self.max_acc * self.dt)
        v_max = min(self.max_v, v0 + self.max_acc * self.dt)
        w_min = max(-self.max_w, w0 - self.max_ang_acc * self.dt)
        w_max = min(self.max_w, w0 + self.max_ang_acc * self.dt)

        # sample velocities
        v_samples = np.linspace(v_min, v_max, self.v_samples)
        w_samples = np.linspace(w_min, w_max, self.w_samples)

        best_score = -float('inf')
        best_v, best_w = 0.0, 0.0

        for v in v_samples:
            for w in w_samples:
                traj = self._simulate_trajectory(v, w, robot_pose.pose.position.x, robot_pose.pose.position.y, robot_yaw)
                clearance_score = self._clearance_cost(traj, obstacles)
                if clearance_score <= 0.0:
                    continue  # collision or too close

                g_cost = self._goal_cost(traj, local_goal)
                # score: higher is better -> invert g_cost
                score = (-self.weight_goal * g_cost) + (self.weight_clearance * clearance_score) + (self.weight_velocity * v)
                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w

        # fallback: if no candidate found, stop and rotate towards goal
        if best_score == -float('inf'):
            yaw_err = self._angle_diff(angle_to_goal, robot_yaw)
            w_des = max(-self.max_w, min(self.max_w, 0.8 * yaw_err))

            # clamp angular acceleration relative to current angular velocity
            dw = w_des - w0
            max_dw = self.max_ang_acc * self.dt
            if abs(dw) > max_dw:
                w_des = w0 + math.copysign(max_dw, dw)

            cmd.angular.z = w_des
            cmd.linear.x = 0.0
            self.last_angular_vel = cmd.angular.z
            return cmd

        # Apply smoothing / acceleration clamp relative to current velocities
        dv = best_v - v0
        max_dv = self.max_acc * self.dt
        if abs(dv) > max_dv:
            best_v = v0 + math.copysign(max_dv, dv)
        dw = best_w - w0
        max_dw = self.max_ang_acc * self.dt
        if abs(dw) > max_dw:
            best_w = w0 + math.copysign(max_dw, dw)

        cmd.linear.x = float(best_v)
        cmd.angular.z = float(best_w)
        self.last_angular_vel = cmd.angular.z
        return cmd

    @staticmethod
    def _angle_diff(a, b):
        d = a - b
        while d > math.pi:
            d -= 2*math.pi
        while d < -math.pi:
            d += 2*math.pi
        return d