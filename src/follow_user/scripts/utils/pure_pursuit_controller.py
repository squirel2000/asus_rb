import math
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path
import angles

class PurePursuitController:
    def __init__(self, node):
        self._node = node
        # Parameters are declared in the main node and loaded from yaml
        self.lookahead_dist_ = self._node.get_parameter("lookahead_dist").get_parameter_value().double_value
        self.min_lookahead_dist_ = self._node.get_parameter("min_lookahead_dist").get_parameter_value().double_value
        self.max_lookahead_dist_ = self._node.get_parameter("max_lookahead_dist").get_parameter_value().double_value
        self.lookahead_time_ = self._node.get_parameter("lookahead_time").get_parameter_value().double_value
        self.desired_linear_vel_ = self._node.get_parameter("desired_linear_vel").get_parameter_value().double_value
        self.max_linear_vel_ = self._node.get_parameter("max_linear_vel").get_parameter_value().double_value
        # max_angular_vel is already declared in the main motion node
        self.max_angular_vel_ = self._node.get_parameter("max_angular_vel").get_parameter_value().double_value
        self.heading_error_for_pure_rotation_ = self._node.get_parameter("heading_error_for_pure_rotation").get_parameter_value().double_value
        self.min_heading_error_for_motion_ = self._node.get_parameter("min_heading_error_for_motion").get_parameter_value().double_value
        self.min_approach_linear_velocity_ = self._node.get_parameter("min_approach_linear_velocity").get_parameter_value().double_value
        self.approach_velocity_scaling_dist_ = self._node.get_parameter("approach_velocity_scaling_dist").get_parameter_value().double_value
        self.goal_dist_buf_ = self._node.get_parameter("goal_dist_buf").get_parameter_value().double_value
        self.goal_dist_tol_ = self._node.get_parameter("goal_dist_tol").get_parameter_value().double_value
        self.linear_acceleration_ = self._node.get_parameter("linear_acceleration").get_parameter_value().double_value
        self.linear_deceleration_ = self._node.get_parameter("linear_deceleration").get_parameter_value().double_value
        self.target_velocity_ema_alpha_ = self._node.get_parameter("target_velocity_ema_alpha").get_parameter_value().double_value

        self.commanded_velocity_ = 0.0
        self.target_velocity_ema_ = None
        self.last_path_segment_idx_ = 0

    def compute_velocity_commands(self, robot_pose, path):
        cmd_vel = Twist()
        if not path.poses:
            return cmd_vel

        carrot_pose = self.get_lookahead_point(robot_pose, path)
        if carrot_pose is None:
            self._node.get_logger().warn("Could not find a lookahead point. Stopping.")
            return cmd_vel

        # Get the robot's current yaw
        robot_yaw = self.get_yaw_from_quaternion(robot_pose.pose.orientation)

        # Calculate the angle to the lookahead point (carrot)
        angle_to_carrot_global = math.atan2(
            carrot_pose.pose.position.y - robot_pose.pose.position.y,
            carrot_pose.pose.position.x - robot_pose.pose.position.x)

        # Calculate the heading error
        heading_error = angles.normalize_angle(angle_to_carrot_global - robot_yaw)

        # Implement smooth, scaled turning
        speed_scale = 1.0
        if abs(heading_error) > self.heading_error_for_pure_rotation_:
            speed_scale = 0.0  # Error is too large, pure rotation
        elif abs(heading_error) > self.min_heading_error_for_motion_:
            speed_scale = (self.heading_error_for_pure_rotation_ - abs(heading_error)) / \
                          (self.heading_error_for_pure_rotation_ - self.min_heading_error_for_motion_)

        # Determine the target velocity based on distance to goal
        dist_to_goal = math.hypot(
            robot_pose.pose.position.x - path.poses[-1].pose.position.x,
            robot_pose.pose.position.y - path.poses[-1].pose.position.y)

        if dist_to_goal > self.approach_velocity_scaling_dist_:
            goal_approach_target_vel = self.desired_linear_vel_
        elif dist_to_goal > self.goal_dist_buf_:
            range_ = self.approach_velocity_scaling_dist_ - self.goal_dist_buf_
            scale = (dist_to_goal - self.goal_dist_buf_) / max(range_, 1e-4)
            goal_approach_target_vel = self.min_approach_linear_velocity_ + scale * (self.desired_linear_vel_ - self.min_approach_linear_velocity_)
        elif dist_to_goal > self.goal_dist_tol_:
            final_crawl_vel = 0.025
            range_ = self.goal_dist_buf_ - self.goal_dist_tol_
            scale = (dist_to_goal - self.goal_dist_tol_) / max(range_, 1e-4)
            goal_approach_target_vel = final_crawl_vel + scale * (self.min_approach_linear_velocity_ - final_crawl_vel)
        else:
            final_crawl_vel = 0.025
            range_ = self.goal_dist_tol_
            scale = dist_to_goal / max(range_, 1e-4)
            goal_approach_target_vel = scale * final_crawl_vel
        
        goal_approach_target_vel = np.clip(goal_approach_target_vel, 0.0, self.desired_linear_vel_)
        final_target_velocity = goal_approach_target_vel * speed_scale
        linear_vel = self.calculate_speed(final_target_velocity)

        # Pure pursuit logic for angular velocity
        angle_to_carrot_robot_frame = heading_error # Simplified for this context
        
        pure_rotation_w = np.sign(heading_error) * 0.7 * self.max_angular_vel_
        
        lookahead_dist_for_curve = math.hypot(carrot_pose.pose.position.x - robot_pose.pose.position.x,
                                              carrot_pose.pose.position.y - robot_pose.pose.position.y)
        lookahead_dist_for_curve = max(lookahead_dist_for_curve, 0.01)
        
        pure_pursuit_curvature = 2.0 * math.sin(angle_to_carrot_robot_frame) / lookahead_dist_for_curve
        pure_pursuit_w = linear_vel * pure_pursuit_curvature
        
        cmd_vel.angular.z = (1.0 - speed_scale) * pure_rotation_w + speed_scale * pure_pursuit_w
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = np.clip(cmd_vel.angular.z, -self.max_angular_vel_, self.max_angular_vel_)

        return cmd_vel

    def calculate_speed(self, target_velocity):
        if self.target_velocity_ema_ is None:
            self.target_velocity_ema_ = target_velocity
        else:
            self.target_velocity_ema_ = self.target_velocity_ema_ * (1.0 - self.target_velocity_ema_alpha_) + target_velocity * self.target_velocity_ema_alpha_

        velocity_error = self.target_velocity_ema_ - self.commanded_velocity_
        dt = 0.05 # Assuming 20Hz control loop from the motion node
        
        if velocity_error > 0:
            new_velocity = min(self.target_velocity_ema_, self.commanded_velocity_ + abs(self.linear_acceleration_) * dt)
        else:
            new_velocity = max(self.target_velocity_ema_, self.commanded_velocity_ - abs(self.linear_deceleration_) * dt)
            
        self.commanded_velocity_ = np.clip(new_velocity, 0.0, self.max_linear_vel_)
        return self.commanded_velocity_

    def get_lookahead_point(self, robot_pose, path):
        vel_for_lookahead = self.commanded_velocity_
        lookahead_dist = np.clip(self.lookahead_time_ * vel_for_lookahead, self.min_lookahead_dist_, self.max_lookahead_dist_)
        
        closest_segment_idx = self.find_closest_path_segment(robot_pose, path, self.last_path_segment_idx_)
        self.last_path_segment_idx_ = closest_segment_idx

        for i in range(closest_segment_idx, len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i+1].pose.position
            intersection = self.find_intersection(p1, p2, robot_pose.pose.position, lookahead_dist)

            if intersection:
                lookahead_point = Point()
                lookahead_point.x = intersection[0]
                lookahead_point.y = intersection[1]
                
                carrot_pose = robot_pose
                carrot_pose.pose.position = lookahead_point
                return carrot_pose

        # If no intersection, use the last point if it's close enough
        dist_to_last_point = math.hypot(
            robot_pose.pose.position.x - path.poses[-1].pose.position.x,
            robot_pose.pose.position.y - path.poses[-1].pose.position.y)
        if dist_to_last_point <= lookahead_dist + self.goal_dist_tol_:
            return path.poses[-1]
            
        return None

    def find_closest_path_segment(self, robot_pose, path, start_idx):
        min_dist_sq = float('inf')
        closest_idx = start_idx

        for i in range(start_idx, len(path.poses)):
            dx = path.poses[i].pose.position.x - robot_pose.pose.position.x
            dy = path.poses[i].pose.position.y - robot_pose.pose.position.y
            dist_sq = dx*dx + dy*dy
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_idx = i
        
        return max(0, closest_idx - 1)

    def find_intersection(self, p1, p2, robot_pos, L):
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        d_sq = dx*dx + dy*dy
        if d_sq == 0: return None

        L_sq = L*L
        a = d_sq
        b = 2 * (dx * (p1.x - robot_pos.x) + dy * (p1.y - robot_pos.y))
        c = (p1.x - robot_pos.x)**2 + (p1.y - robot_pos.y)**2 - L_sq
        
        discriminant = b*b - 4*a*c
        if discriminant < 0: return None

        t1 = (-b + math.sqrt(discriminant)) / (2*a)
        if 0 <= t1 <= 1:
            return p1.x + t1 * dx, p1.y + t1 * dy

        t2 = (-b - math.sqrt(discriminant)) / (2*a)
        if 0 <= t2 <= 1:
            return p1.x + t2 * dx, p1.y + t2 * dy
            
        return None

    def get_yaw_from_quaternion(self, q):
        # Conversion from quaternion to yaw (rotation around z-axis)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
