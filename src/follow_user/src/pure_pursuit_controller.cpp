#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>
#include <limits>
#include <mutex>
#include <optional>
#include <angles/angles.h>

class PurePursuitController : public rclcpp::Node
{
public:
    PurePursuitController() : Node("pure_pursuit_controller"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // Parameters
        this->declare_parameter("lookahead_dist", 1.0);
        this->declare_parameter("min_lookahead_dist", 0.5);
        this->declare_parameter("max_lookahead_dist", 1.5);
        this->declare_parameter("lookahead_time", 1.5);
        this->declare_parameter("desired_linear_vel", 0.5);
        this->declare_parameter("max_linear_vel", 0.20); 
        this->declare_parameter("max_angular_vel", 1.2);
        this->declare_parameter("heading_error_for_pure_rotation", 1.57); // 90 degrees
        this->declare_parameter("min_heading_error_for_motion", 0.35); // 20 degrees
        this->declare_parameter("min_approach_linear_velocity", 0.05);
        this->declare_parameter("approach_velocity_scaling_dist", 0.6);
        this->declare_parameter("goal_dist_tol", 0.25);
        this->declare_parameter("path_topic", "/follow_user/planned_path");
        this->declare_parameter("odom_topic", "/slamware_ros_sdk_server_node/odom");
        this->declare_parameter("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("global_frame", "odom");  // Change to slamware_map
        this->declare_parameter("controller_frequency", 20.0);

        // Get parameters
        lookahead_dist_ = this->get_parameter("lookahead_dist").as_double();
        min_lookahead_dist_ = this->get_parameter("min_lookahead_dist").as_double();
        max_lookahead_dist_ = this->get_parameter("max_lookahead_dist").as_double();
        lookahead_time_ = this->get_parameter("lookahead_time").as_double();
        desired_linear_vel_ = this->get_parameter("desired_linear_vel").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        heading_error_for_pure_rotation_ = this->get_parameter("heading_error_for_pure_rotation").as_double();
        min_heading_error_for_motion_ = this->get_parameter("min_heading_error_for_motion").as_double();
        min_approach_linear_velocity_ = this->get_parameter("min_approach_linear_velocity").as_double();
        approach_velocity_scaling_dist_ = this->get_parameter("approach_velocity_scaling_dist").as_double();
        goal_dist_tol_ = this->get_parameter("goal_dist_tol").as_double();
        path_topic_ = this->get_parameter("path_topic").as_string();
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        global_frame_ = this->get_parameter("global_frame").as_string();
        double controller_frequency = this->get_parameter("controller_frequency").as_double();

        // Publishers and subscribers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic_, 10,
            std::bind(&PurePursuitController::pathCallback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, 
            std::bind(&PurePursuitController::odomCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        carrot_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("lookahead_point", 10);

        // Timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / controller_frequency)),
            std::bind(&PurePursuitController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Regulated Pure Pursuit Controller initialized");
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty path. Ignoring.");
            return;
        }
        current_path_ = *msg;
        path_received_ = true;
        goal_reached_ = false;
        last_path_segment_idx_ = 0; 
        RCLCPP_INFO(this->get_logger(), "Received new path with %zu points", current_path_.poses.size());
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
    current_velocity_ = msg->twist.twist.linear.x;
    // Cache latest odometry pose for fallback when TF lookup fails
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = *msg;
    }

    void controlLoop()
    {
        if (!path_received_ || current_path_.poses.empty() || goal_reached_) {
            if (is_moving_) {
                publishZeroVelocity();
                is_moving_ = false;
            }
            return;
        }
        
        geometry_msgs::msg::PoseStamped robot_pose;
        if (!getRobotPose(robot_pose)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get robot pose, stopping.");
            publishZeroVelocity();
            return;
        }

        if (isGoalReached(robot_pose)) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            goal_reached_ = true;
            path_received_ = false;
            current_path_.poses.clear();
            publishZeroVelocity();
            is_moving_ = false;
            return;
        }
        
        auto cmd_vel = computeVelocityCommands(robot_pose);
        cmd_vel_pub_->publish(cmd_vel);
        is_moving_ = true;
    }

    geometry_msgs::msg::Twist computeVelocityCommands(const geometry_msgs::msg::PoseStamped& robot_pose)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;

        geometry_msgs::msg::PoseStamped carrot_pose;
        if (!getLookaheadPoint(robot_pose, carrot_pose)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not find a lookahead point. Stopping.");
            return cmd_vel; 
        }

        // Implement "turn-then-go" logic
        // Calculate the angle to the lookahead point (carrot)
        double angle_to_carrot_global = atan2(
            carrot_pose.pose.position.y - robot_pose.pose.position.y,
            carrot_pose.pose.position.x - robot_pose.pose.position.x);

        // Get the robot's current yaw
        double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
        
        // Calculate the heading error
        double heading_error = angles::normalize_angle(angle_to_carrot_global - robot_yaw);
        
        // Implement smooth, scaled turning
        double linear_vel = calculateSpeed(robot_pose);
        double speed_scale = 1.0;
        if (std::abs(heading_error) > heading_error_for_pure_rotation_) {
            speed_scale = 0.0; // Error is too large, pure rotation
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Large heading error. Pure rotation.");
        } else if (std::abs(heading_error) > min_heading_error_for_motion_) {
            // Scale speed linearly between the two thresholds
            speed_scale = (heading_error_for_pure_rotation_ - std::abs(heading_error)) / 
                          (heading_error_for_pure_rotation_ - min_heading_error_for_motion_);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Aligning with scaled speed. Scale: %.2f", speed_scale);
        }
        
        linear_vel *= speed_scale;
        
        // The rest of the pure pursuit logic for calculating angular velocity
        double angle_to_carrot_robot_frame = 0.0;
        try {
            geometry_msgs::msg::PoseStamped carrot_in_robot_frame;
            if (!transformPose(base_frame_, carrot_pose, carrot_in_robot_frame)) {
                RCLCPP_WARN(this->get_logger(), "Could not transform lookahead point to robot frame. Stopping.");
                return cmd_vel; // zero velocity
            }
            angle_to_carrot_robot_frame = atan2(carrot_in_robot_frame.pose.position.y, carrot_in_robot_frame.pose.position.x);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform lookahead point: %s", ex.what());
            return cmd_vel; // zero velocity
        }

        // If we are only rotating, use a simple proportional controller on heading error
        if (speed_scale == 0.0) {
            cmd_vel.angular.z = std::copysign(0.7 * max_angular_vel_, heading_error);
        } else {
             double lookahead_dist = std::hypot(carrot_pose.pose.position.x - robot_pose.pose.position.x,
                                               carrot_pose.pose.position.y - robot_pose.pose.position.y);
            // Avoid division by zero if lookahead distance is tiny
            if (std::abs(lookahead_dist) < 0.01) {
                lookahead_dist = 0.01;
            }
            double curvature = 2.0 * sin(angle_to_carrot_robot_frame) / lookahead_dist;
            cmd_vel.angular.z = linear_vel * curvature;
        }

        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -max_angular_vel_, max_angular_vel_);
        
        return cmd_vel;
    }

    double calculateSpeed(const geometry_msgs::msg::PoseStamped & robot_pose)
    {
        double dist_to_goal = std::hypot(
            robot_pose.pose.position.x - current_path_.poses.back().pose.position.x,
            robot_pose.pose.position.y - current_path_.poses.back().pose.position.y);

        double commanded_vel;
        if (dist_to_goal < approach_velocity_scaling_dist_) {
            double scale = dist_to_goal / approach_velocity_scaling_dist_;
            commanded_vel = std::max(min_approach_linear_velocity_, scale * desired_linear_vel_);
        } else {
            commanded_vel = desired_linear_vel_;
        }
        
        return std::min(commanded_vel, max_linear_vel_);
    }

    bool getLookaheadPoint(const geometry_msgs::msg::PoseStamped& robot_pose,
                           geometry_msgs::msg::PoseStamped& lookahead_point)
    {
        double lookahead_dist = std::clamp(lookahead_time_ * current_velocity_, min_lookahead_dist_, max_lookahead_dist_);
        
        size_t closest_segment_idx = findClosestPathSegment(robot_pose, last_path_segment_idx_);
        last_path_segment_idx_ = closest_segment_idx;

        bool found_lookahead = false;
        for (size_t i = closest_segment_idx; i < current_path_.poses.size() - 1; ++i) {
            auto& p1 = current_path_.poses[i].pose.position;
            auto& p2 = current_path_.poses[i+1].pose.position;
            Point intersection = findIntersection(p1, p2, robot_pose.pose.position, lookahead_dist);

            if (intersection.is_valid) {
                lookahead_point.header.frame_id = current_path_.header.frame_id;
                lookahead_point.header.stamp = this->get_clock()->now();
                lookahead_point.pose.position.x = intersection.x;
                lookahead_point.pose.position.y = intersection.y;
                lookahead_point.pose.orientation.w = 1.0;
                lookahead_point.pose.orientation.x = 0.0;
                lookahead_point.pose.orientation.y = 0.0;
                lookahead_point.pose.orientation.z = 0.0;
                found_lookahead = true;
                carrot_pub_->publish(lookahead_point);
                return true;
            }
        }

        if (!found_lookahead) {
            double dist_to_last_point = std::hypot(
                robot_pose.pose.position.x - current_path_.poses.back().pose.position.x,
                robot_pose.pose.position.y - current_path_.poses.back().pose.position.y
            );
            if (dist_to_last_point <= lookahead_dist + goal_dist_tol_) {
                lookahead_point.pose = current_path_.poses.back().pose;
                found_lookahead = true;
            }
        }
        
        if (found_lookahead) {
            lookahead_point.header.frame_id = current_path_.header.frame_id;
            lookahead_point.header.stamp = this->get_clock()->now();
            carrot_pub_->publish(lookahead_point);
            return true;
        }

        return false;
    }

    struct Point { double x, y; bool is_valid = false; };

    size_t findClosestPathSegment(const geometry_msgs::msg::PoseStamped& robot_pose, size_t start_idx) {
        double min_dist_sq = std::numeric_limits<double>::max();
        size_t closest_idx = start_idx;

        for (size_t i = start_idx; i < current_path_.poses.size(); ++i) {
            double dx = current_path_.poses[i].pose.position.x - robot_pose.pose.position.x;
            double dy = current_path_.poses[i].pose.position.y - robot_pose.pose.position.y;
            double dist_sq = dx * dx + dy * dy;

            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_idx = i;
            }
        }
        return (closest_idx > 0) ? closest_idx - 1 : 0;
    }

    Point findIntersection(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2,
                           const geometry_msgs::msg::Point& robot_pos, double L)
    {
        Point intersection;
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double d_sq = dx * dx + dy * dy;

        if (d_sq == 0.0) return intersection;

        // FIX START: Remove unused variable 't' to resolve compiler warning
        // double t = ((robot_pos.x - p1.x) * dx + (robot_pos.y - p1.y) * dy) / d_sq;
        // FIX END
        double L_sq = L * L;

        double a = d_sq;
        double b = 2 * (dx * (p1.x - robot_pos.x) + dy * (p1.y - robot_pos.y));
        double c = (p1.x - robot_pos.x) * (p1.x - robot_pos.x) + 
                   (p1.y - robot_pos.y) * (p1.y - robot_pos.y) - L_sq;
        
        double discriminant = b*b - 4*a*c;
        if (discriminant < 0) return intersection;

        double t1 = (-b + sqrt(discriminant)) / (2*a);
        if (t1 >= 0 && t1 <= 1) {
            intersection.x = p1.x + t1 * dx;
            intersection.y = p1.y + t1 * dy;
            intersection.is_valid = true;
            return intersection;
        }

        double t2 = (-b - sqrt(discriminant)) / (2*a);
        if (t2 >= 0 && t2 <= 1) {
            intersection.x = p1.x + t2 * dx;
            intersection.y = p1.y + t2 * dy;
            intersection.is_valid = true;
            return intersection;
        }
        
        return intersection;
    }
    
    bool getRobotPose(geometry_msgs::msg::PoseStamped& robot_pose)
    {
        geometry_msgs::msg::TransformStamped transform;
        try {
            // Request the latest transform available. This may throw if the buffer
            // does not have any transform for the requested frames/time.
            transform = tf_buffer_.lookupTransform(
                global_frame_, base_frame_, tf2::TimePointZero, std::chrono::milliseconds(500));

            robot_pose.header.stamp = transform.header.stamp;
            robot_pose.header.frame_id = global_frame_;
            robot_pose.pose.position.x = transform.transform.translation.x;
            robot_pose.pose.position.y = transform.transform.translation.y;
            robot_pose.pose.position.z = transform.transform.translation.z;
            robot_pose.pose.orientation = transform.transform.rotation;
            return true;
        } catch (tf2::TransformException& ex) {
            // If the error is extrapolation (timestamps), try to fallback to latest
            // transform available in the buffer by asking for the latest common time.
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Could not get robot pose from TF: %s -- attempting odom fallback", ex.what());

            // Try to use the most recent odometry message we have received as a fallback.
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (latest_odom_.has_value()) {
                const auto& odom = latest_odom_.value();
                robot_pose.header.stamp = odom.header.stamp;
                robot_pose.header.frame_id = odom.header.frame_id;
                robot_pose.pose = odom.pose.pose;
                return true;
            }

            return false;
        }
    }

    bool transformPose(const std::string& target_frame, const geometry_msgs::msg::PoseStamped& in_pose,
                        geometry_msgs::msg::PoseStamped& out_pose)
    {
        if (in_pose.header.frame_id == target_frame) {
            out_pose = in_pose;
            return true;
        }
        try {
            // Ask for the LATEST available transform
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                target_frame, in_pose.header.frame_id, tf2::TimePointZero, std::chrono::milliseconds(500));
            tf2::doTransform(in_pose, out_pose, transform);
            out_pose.header.stamp = transform.header.stamp; // Ensure timestamp is consistent
            return true;

        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Exception in transformPose: %s", ex.what());
        }
        return false;
    }
    
    bool isGoalReached(const geometry_msgs::msg::PoseStamped& robot_pose)
    {
        if (current_path_.poses.empty()) return true;
        const auto& goal = current_path_.poses.back();
        double dx = goal.pose.position.x - robot_pose.pose.position.x;
        double dy = goal.pose.position.y - robot_pose.pose.position.y;
        return std::hypot(dx, dy) < goal_dist_tol_;
    }
    
    void publishZeroVelocity()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }
    
    // Parameters
    double lookahead_dist_;
    double min_lookahead_dist_;
    double max_lookahead_dist_;
    double lookahead_time_;
    double desired_linear_vel_;
    double max_linear_vel_;
    double max_angular_vel_;
    double min_approach_linear_velocity_;
    double approach_velocity_scaling_dist_;
    double goal_dist_tol_;
    std::string path_topic_;
    std::string odom_topic_;
    std::string cmd_vel_topic_;
    std::string base_frame_;
    std::string global_frame_;
    double heading_error_for_pure_rotation_;
    double min_heading_error_for_motion_;
    
    // ROS components
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // State
    nav_msgs::msg::Path current_path_;
    bool path_received_ = false;
    bool goal_reached_ = true;
    double current_velocity_ = 0.0;
    size_t last_path_segment_idx_ = 0;
    bool is_moving_ = false;
    // Odom cache for TF fallback
    std::mutex odom_mutex_;
    std::optional<nav_msgs::msg::Odometry> latest_odom_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}