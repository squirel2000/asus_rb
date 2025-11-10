import math
import numpy as np
from geometry_msgs.msg import Twist, Point
import angles

# Constants
DT = 0.05  # Assuming 20Hz control loop from the motion node
HEADING_DEADZONE = math.radians(5.0)  # Apply a small dead zone to avoid jitter around zero (±5.0 degrees)

class PurePursuitController:
    def __init__(self, node):
        self._node = node
        # Parameters are declared in the main node and loaded from yaml
        self.min_lookahead_dist_ = self._node.get_parameter("min_lookahead_dist").get_parameter_value().double_value
        self.max_lookahead_dist_ = self._node.get_parameter("max_lookahead_dist").get_parameter_value().double_value
        self.lookahead_time_ = self._node.get_parameter("lookahead_time").get_parameter_value().double_value
        self.max_linear_vel_ = self._node.get_parameter("max_linear_vel").get_parameter_value().double_value
        self.linear_acceleration_ = self._node.get_parameter("linear_acceleration").get_parameter_value().double_value
        self.linear_deceleration_ = self._node.get_parameter("linear_deceleration").get_parameter_value().double_value
        self.max_angular_vel_ = self._node.get_parameter("max_angular_vel").get_parameter_value().double_value
        self.max_angular_acceleration_ = self._node.get_parameter("max_angular_acceleration").get_parameter_value().double_value
        self.heading_error_for_pure_rotation_ = self._node.get_parameter("heading_error_for_pure_rotation").get_parameter_value().double_value
        self.min_heading_error_for_motion_ = self._node.get_parameter("min_heading_error_for_motion").get_parameter_value().double_value
        self.min_approach_linear_velocity_ = self._node.get_parameter("min_approach_linear_velocity").get_parameter_value().double_value
        self.approach_velocity_scaling_dist_ = self._node.get_parameter("approach_velocity_scaling_dist").get_parameter_value().double_value
        self.goal_dist_buf_ = self._node.get_parameter("goal_dist_buf").get_parameter_value().double_value
        self.goal_dist_tol_ = self._node.get_parameter("goal_dist_tol").get_parameter_value().double_value        

        self.lookahead_point_pub_ = self._node.create_publisher(Point, 'lookahead_point', 10)
        self._node.get_logger().info('Publishing lookahead point on topic "lookahead_point"')
        self.last_path_segment_idx_ = 0
        self.path_ = None

    def set_path(self, path):
        self.path_ = path
        self.last_path_segment_idx_ = 0

    def compute_velocity_commands(self, robot_pose, current_velocity):
        # High-level dispatcher: choose short-path rotation or full pure-pursuit
        if not self.path_ or not self.path_.poses:
            return self._rectify_velocity(0.0, 0.0, current_velocity)

        return self._pure_pursuit_control(robot_pose, current_velocity)

    def _pure_pursuit_control(self, robot_pose, current_velocity):
        """Compute linear and angular commands using the pure pursuit algorithm."""
        # Calculate the angle to the lookahead point (carrot)
        lookahead_point = self._get_lookahead_point(robot_pose, self.path_, current_velocity.linear.x)
        if lookahead_point is None:
            self._node.get_logger().warn("Could not find a lookahead point. Stopping.")
            return self._rectify_velocity(0.0, 0.0, current_velocity)

        self.lookahead_point_pub_.publish(lookahead_point)
        robot_yaw = self.get_yaw_from_quaternion(robot_pose.pose.orientation)

        # Determine the target for heading calculation. Use the final goal point when close,
        # otherwise use the lookahead point. This helps with final alignment.
        dist_to_goal = math.hypot(
            robot_pose.pose.position.x - self.path_.poses[-1].pose.position.x,
            robot_pose.pose.position.y - self.path_.poses[-1].pose.position.y)

        heading_target_point = self.path_.poses[-1].pose.position if dist_to_goal < self.approach_velocity_scaling_dist_ else lookahead_point

        angle_to_carrot_global = math.atan2(
            heading_target_point.y - robot_pose.pose.position.y,
            heading_target_point.x - robot_pose.pose.position.x)

        # Calculate the heading error
        heading_error = angles.normalize_angle(angle_to_carrot_global - robot_yaw)
        
        # Implement smooth, scaled turning
        speed_scale = 1.0
        if abs(heading_error) > self.heading_error_for_pure_rotation_:
            speed_scale = 0.0  # Error is too large, pure rotation
        elif abs(heading_error) > self.min_heading_error_for_motion_:
            speed_scale = (self.heading_error_for_pure_rotation_ - abs(heading_error)) / \
                          (self.heading_error_for_pure_rotation_ - self.min_heading_error_for_motion_)

        if dist_to_goal > self.approach_velocity_scaling_dist_:
            goal_approach_target_vel = self.max_linear_vel_
        elif dist_to_goal > self.goal_dist_buf_:
            range_ = self.approach_velocity_scaling_dist_ - self.goal_dist_buf_
            scale = (dist_to_goal - self.goal_dist_buf_) / max(range_, 1e-4)
            goal_approach_target_vel = self.min_approach_linear_velocity_ + scale * (self.max_linear_vel_ - self.min_approach_linear_velocity_)
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

        goal_approach_target_vel = np.clip(goal_approach_target_vel, 0.0, self.max_linear_vel_)
        target_vel_lin_x = goal_approach_target_vel * speed_scale
        
        # Pure pursuit logic for angular velocity
        pure_rotation_w = np.sign(heading_error) * 0.7 * self.max_angular_vel_
        lookahead_dist_for_curve = math.hypot(lookahead_point.x - robot_pose.pose.position.x,
                                              lookahead_point.y - robot_pose.pose.position.y)
        lookahead_dist_for_curve = max(lookahead_dist_for_curve, 0.01)

        pure_pursuit_curvature = 2.0 * math.sin(heading_error) / lookahead_dist_for_curve
        pure_pursuit_w = target_vel_lin_x * pure_pursuit_curvature
        target_vel_ang_z = (1.0 - speed_scale) * pure_rotation_w + speed_scale * pure_pursuit_w
        
        cmd_vel = self._rectify_velocity(target_vel_lin_x, target_vel_ang_z, current_velocity)
        
        # Print debug info
        self._node.get_logger().info(f"_pure_pursuit_control: lookahead_point({lookahead_point.x:.2f}, {lookahead_point.y:.2f}), Robot({robot_pose.pose.position.x:.2f}, {robot_pose.pose.position.y:.2f}, {math.degrees(self.get_yaw_from_quaternion(robot_pose.pose.orientation)):.2f} deg), Heading error: {math.degrees(heading_error):.2f} deg, target_vel({target_vel_lin_x:.2f}, {target_vel_ang_z:.2f}), cmd_vel({cmd_vel.linear.x:.2f}, {cmd_vel.angular.z:.2f})")
        
        return cmd_vel


    def _rectify_velocity(self, target_vel_lin_x, target_vel_ang_z, current_velocity):
        """Refine velocity commands based on acceleration/deceleration limits.
        """
        cmd_vel = Twist()

        vel_lin_x_error = target_vel_lin_x - current_velocity.linear.x
        if vel_lin_x_error > 0:
            stepped_vel_lin_x = min(target_vel_lin_x, current_velocity.linear.x + abs(self.linear_acceleration_) * DT)
        else:
            stepped_vel_lin_x = max(target_vel_lin_x, current_velocity.linear.x - abs(self.linear_deceleration_) * DT)
        cmd_vel.linear.x = np.clip(stepped_vel_lin_x, 0.0, self.max_linear_vel_)

        vel_ang_z_error = target_vel_ang_z - current_velocity.angular.z
        if vel_ang_z_error > 0:
            stepped_vel_ang_z = min(target_vel_ang_z, current_velocity.angular.z + abs(self.max_angular_acceleration_) * DT)
        else:
            stepped_vel_ang_z = max(target_vel_ang_z, current_velocity.angular.z - abs(self.max_angular_acceleration_) * DT)
        cmd_vel.angular.z = np.clip(stepped_vel_ang_z, -self.max_angular_vel_, self.max_angular_vel_)
        
        return cmd_vel

    def _get_lookahead_point(self, robot_pose, path, current_velocity):
        vel_for_lookahead = current_velocity
        lookahead_dist = np.clip(self.lookahead_time_ * vel_for_lookahead, self.min_lookahead_dist_, self.max_lookahead_dist_)
        
        closest_segment_idx = self._find_closest_path_segment(robot_pose, path, self.last_path_segment_idx_)
        self.last_path_segment_idx_ = closest_segment_idx

        for i in range(closest_segment_idx, len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i+1].pose.position
            intersection = self._find_intersection(p1, p2, robot_pose.pose.position, lookahead_dist)

            if intersection:
                lookahead_point = Point()
                lookahead_point.x = intersection[0]
                lookahead_point.y = intersection[1]
                return lookahead_point

        # If no intersection, use the last point if it's close enough
        dist_to_last_point = math.hypot(
            robot_pose.pose.position.x - path.poses[-1].pose.position.x,
            robot_pose.pose.position.y - path.poses[-1].pose.position.y)
        if dist_to_last_point <= lookahead_dist + self.goal_dist_tol_:
            return path.poses[-1].pose.position
        
        return None

    def _find_closest_path_segment(self, robot_pose, path, start_idx):
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

    def _find_intersection(self, p1, p2, robot_pos, L):
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
