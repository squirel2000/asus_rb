#!/usr/bin/env python3
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from typing import Tuple

class DWAPlanner:
    """
    A lightweight Python implementation of the Dynamic Window Approach (DWA).
    Designed for local goal following (user position) with 360deg LiDAR input.
    """

    def __init__(self, node):
        self.node = node

        # Planner parameters
        self.dt = 0.05  # control interval (s), ~20Hz
        self.predict_horizon = 1.0  # seconds
        self.steps = int(self.predict_horizon / self.dt)
        self.v_samples = 5
        self.w_samples = 7

        # Robot limits
        self.max_v = self.node.get_parameter("max_linear_vel").get_parameter_value().double_value
        self.max_w = self.node.get_parameter("max_angular_vel").get_parameter_value().double_value
        self.max_acc = self.node.get_parameter("linear_acceleration").get_parameter_value().double_value
        self.max_ang_acc = self.node.get_parameter("max_angular_acceleration").get_parameter_value().double_value

        # Scoring weights
        self.weight_goal = 1.2
        self.weight_clearance = 1.5
        self.weight_velocity = 0.2
        self.weight_heading = 0.8

        # Robot footprint
        self.robot_radius = 0.35
        self.min_allowed_clearance = 0.05

        # Pure rotation control parameters
        self.node.declare_parameter("rotate_only_dist", 0.75)
        self.node.declare_parameter("rotation_gain_k", 4.0)
        self.node.declare_parameter("smoothing_alpha", 0.8)
        self.rotate_only_dist = self.node.get_parameter("rotate_only_dist").get_parameter_value().double_value
        self.rotation_gain_k = self.node.get_parameter("rotation_gain_k").get_parameter_value().double_value
        self.smoothing_alpha = self.node.get_parameter("smoothing_alpha").get_parameter_value().double_value
        
        # Planner state
        self.last_angular_vel = 0.0

    def _compute_goal_vector_and_yaw(self, robot_pose: PoseStamped, local_goal: PoseStamped) -> Tuple[float, float, float, float, float]:
        """
        Compute the vector and distance to the goal, the angle to the goal, and the robot's current yaw.
        Returns (dxg, dyg, dist_to_goal, angle_to_goal, robot_yaw)
        """
        dxg = local_goal.pose.position.x - robot_pose.pose.position.x
        dyg = local_goal.pose.position.y - robot_pose.pose.position.y
        dist_to_goal = math.hypot(dxg, dyg)
        angle_to_goal = math.atan2(dyg, dxg)
        q = robot_pose.pose.orientation
        robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return dxg, dyg, dist_to_goal, angle_to_goal, robot_yaw

    def _smooth_rotation_command(self, angle_to_goal: float, robot_yaw: float, cmd: Twist) -> Twist:
        """
        Compute a smoothed pure rotation command to face the goal.
        Modifies and returns the given Twist cmd.
        """
        yaw_err = self._angle_diff(angle_to_goal, robot_yaw)
        desired_w = self.max_w * math.tanh(self.rotation_gain_k * yaw_err)
        max_dw = self.max_ang_acc * self.dt
        dw = desired_w - self.last_angular_vel
        if abs(dw) > max_dw:
            desired_w = self.last_angular_vel + math.copysign(max_dw, dw)
        final_w = self.smoothing_alpha * desired_w + (1 - self.smoothing_alpha) * self.last_angular_vel
        self.last_angular_vel = final_w
        cmd.angular.z = final_w
        return cmd

    def _apply_acceleration_limits(self, best_v: float, best_w: float, current_velocity: Twist) -> Tuple[float, float]:
        """
        Apply acceleration limits to the best found velocities.
        Returns (limited_v, limited_w)
        """
        v0, w0 = current_velocity.linear.x, current_velocity.angular.z
        dv = best_v - v0
        max_dv = self.max_acc * self.dt
        if abs(dv) > max_dv:
            best_v = v0 + math.copysign(max_dv, dv)
        dw = best_w - w0
        max_dw = self.max_ang_acc * self.dt
        if abs(dw) > max_dw:
            best_w = w0 + math.copysign(max_dw, dw)
        return float(best_v), float(best_w)

    def _scan_to_points_global(self, scan: LaserScan, robot_pose: PoseStamped) -> np.ndarray:
        """
        Convert scan ranges to a list of (x, y) points in the map frame using robot_pose (PoseStamped).
        Returns an Nx2 numpy array of obstacle points.
        """
        if scan is None or robot_pose is None:
            return np.empty((0, 2))

        ranges = np.array(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        if not np.any(valid):
            return np.empty((0, 2))

        angles = angles[valid]
        ranges = ranges[valid]

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        q = robot_pose.pose.orientation
        robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_r = math.cos(robot_yaw)
        sin_r = math.sin(robot_yaw)
        x_map = robot_pose.pose.position.x + (xs * cos_r - ys * sin_r)
        y_map = robot_pose.pose.position.y + (xs * sin_r + ys * cos_r)
        return np.vstack((x_map, y_map)).T

    def _simulate_trajectory(self, v: float, w: float, x0: float, y0: float, yaw0: float) -> np.ndarray:
        """
        Simulate robot trajectory by applying constant linear (v) and angular (w) velocities
        over the prediction horizon. Returns an array of poses with shape (N, 3).
        """
        traj = np.zeros((self.steps, 3))
        x, y, yaw = x0, y0, yaw0
        for i in range(self.steps):
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt
            traj[i, :] = [x, y, yaw]
        return traj

    def _get_min_clearance(self, traj: np.ndarray, obstacles: np.ndarray) -> float:
        """
        Compute the minimum clearance from the trajectory to all obstacles (both in map frame).
        Returns the minimum distance found.
        """
        if obstacles.size == 0:
            return float('inf')

        traj_points = traj[:, :2]  # N x 2 array of (x, y)
        expanded_traj = traj_points[:, np.newaxis, :]  # N x 1 x 2
        expanded_obs = obstacles[np.newaxis, :, :]  # 1 x M x 2
        
        # Calculate squared distances, shape will be N x M
        dist_sq = np.sum((expanded_traj - expanded_obs)**2, axis=2)
        
        # Find the minimum squared distance, then take the square root
        min_dist_sq = np.min(dist_sq)
        return math.sqrt(min_dist_sq)

    def _goal_metrics(self, traj: np.ndarray, goal: PoseStamped) -> Tuple[float, float]:
        """
        Calculate distance and heading error from the trajectory's end-point to the goal.
        Returns (distance_error, heading_error).
        """
        last_x, last_y, last_yaw = traj[-1]
        goal_x, goal_y = goal.pose.position.x, goal.pose.position.y

        dist_err = math.hypot(goal_x - last_x, goal_y - last_y)
        angle_to_goal = math.atan2(goal_y - last_y, goal_x - last_x)
        heading_err = self._angle_diff(angle_to_goal, last_yaw)

        return dist_err, heading_err

    def _filter_human_from_obstacles(self, obstacles: np.ndarray, robot_pose: PoseStamped, human_pose: PoseStamped, human_radius: float) -> np.ndarray:
        """
        Remove points from the obstacles array that are likely to be the tracked human.
        """
        if human_pose is None or obstacles.size == 0:
            return obstacles

        hx, hy = human_pose.pose.position.x, human_pose.pose.position.y
        rx, ry = robot_pose.pose.position.x, robot_pose.pose.position.y
        
        dist_h = math.hypot(hx - rx, hy - ry)
        angle_h_map = math.atan2(hy - ry, hx - rx)

        obs_vectors = obstacles - np.array([rx, ry])
        obs_dists_map = np.linalg.norm(obs_vectors, axis=1)
        obs_angles_map = np.arctan2(obs_vectors[:, 1], obs_vectors[:, 0])

        ang_diff = np.abs(self._angle_diff(obs_angles_map, angle_h_map))

        margin = 0.1
        ang_thresh = math.atan2(human_radius + margin, max(dist_h, 1e-6))
        ang_thresh = max(ang_thresh, math.radians(15.0))
        dist_window = human_radius + margin

        human_mask = (ang_diff <= ang_thresh) & (np.abs(obs_dists_map - dist_h) <= dist_window)
        return obstacles[~human_mask]

    def _calculate_dynamic_window(self, current_velocity: Twist) -> Tuple[float, float, float, float]:
        """
        Calculate the dynamic window for velocity sampling based on current velocity and acceleration limits.
        Returns (v_min, v_max, w_min, w_max).
        """
        v0, w0 = current_velocity.linear.x, current_velocity.angular.z
        v_min = max(0.0, v0 - self.max_acc * self.dt)
        v_max = min(self.max_v, v0 + self.max_acc * self.dt)
        w_min = max(-self.max_w, w0 - self.max_ang_acc * self.dt)
        w_max = min(self.max_w, w0 + self.max_ang_acc * self.dt)
        return v_min, v_max, w_min, w_max

    def _evaluate_trajectories(self, robot_pose: PoseStamped, robot_yaw: float, local_goal: PoseStamped, obstacles: np.ndarray, v_samples: np.ndarray, w_samples: np.ndarray) -> Tuple[float, float, float]:
        """
        Evaluate all sampled (v, w) pairs and score their simulated trajectories.
        Returns the best (v, w) and its score.
        """
        best_score = -float('inf')
        best_v, best_w = 0.0, 0.0
        max_possible_clearance = 3.0 * self.robot_radius

        for v in v_samples:
            for w in w_samples:
                traj = self._simulate_trajectory(v, w, robot_pose.pose.position.x, robot_pose.pose.position.y, robot_yaw)
                min_clearance = self._get_min_clearance(traj, obstacles)
                if min_clearance < self.robot_radius:
                    continue

                goal_dist, heading_err = self._goal_metrics(traj, local_goal)
                clearance_score = min(1.0, min_clearance / max_possible_clearance)
                goal_dist_score = 1.0 / (1.0 + 2.0 * goal_dist)
                heading_score = (math.pi - abs(heading_err)) / math.pi
                velocity_score = v / self.max_v if self.max_v > 0 else 0.0

                score = (self.weight_goal * goal_dist_score) + \
                        (self.weight_clearance * clearance_score) + \
                        (self.weight_heading * heading_score) + \
                        (self.weight_velocity * velocity_score)

                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w
        return best_v, best_w, best_score

    def _get_fallback_command(self, angle_to_goal: float, robot_yaw: float, w0: float) -> Twist:
        """
        Generate a fallback command: pure rotation towards the goal if no valid trajectory is found.
        """
        cmd = Twist()
        yaw_err = self._angle_diff(angle_to_goal, robot_yaw)
        w_des = max(-self.max_w, min(self.max_w, 0.8 * yaw_err))

        dw = w_des - w0
        max_dw = self.max_ang_acc * self.dt
        if abs(dw) > max_dw:
            w_des = w0 + math.copysign(max_dw, dw)

        cmd.angular.z = w_des
        self.last_angular_vel = cmd.angular.z
        return cmd

    def compute_velocity_commands(
        self,
        robot_pose: PoseStamped,
        current_velocity: Twist,
        scan: LaserScan,
        local_goal: PoseStamped,
        human_pose: PoseStamped = None,
        human_radius: float = 0.25
    ) -> Twist:
        """
        Main entry point: Compute the velocity command (Twist) for the robot to follow the local goal.
        """
        if not all([robot_pose, current_velocity, scan, local_goal]):
            return Twist()

        _, _, dist_to_goal, angle_to_goal, robot_yaw = self._compute_goal_vector_and_yaw(robot_pose, local_goal)

        if dist_to_goal < self.rotate_only_dist:
            return self._smooth_rotation_command(angle_to_goal, robot_yaw, Twist())

        obstacles = self._scan_to_points_global(scan, robot_pose)
        obstacles = self._filter_human_from_obstacles(obstacles, robot_pose, human_pose, human_radius)

        v_min, v_max, w_min, w_max = self._calculate_dynamic_window(current_velocity)
        v_samples = np.linspace(v_min, v_max, self.v_samples)
        w_samples = np.linspace(w_min, w_max, self.w_samples)

        best_v, best_w, best_score = self._evaluate_trajectories(
            robot_pose, robot_yaw, local_goal, obstacles, v_samples, w_samples
        )

        if best_score == -float('inf'):
            return self._get_fallback_command(angle_to_goal, robot_yaw, current_velocity.angular.z)

        cmd = Twist()
        cmd.linear.x, cmd.angular.z = self._apply_acceleration_limits(best_v, best_w, current_velocity)
        self.last_angular_vel = cmd.angular.z
        return cmd

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """
        Compute the difference between two angles, result wrapped to [-pi, pi].
        """
        d = a - b
        d = (d + math.pi) % (2 * math.pi) - math.pi
        return d
