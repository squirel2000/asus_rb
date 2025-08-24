#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
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
        this->declare_parameter("max_angular_vel", 1.2);
        this->declare_parameter("min_approach_linear_velocity", 0.1);
        this->declare_parameter("approach_velocity_scaling_dist", 0.6);
        this->declare_parameter("goal_dist_tol", 0.25);
        this->declare_parameter("path_topic", "/follow_user/planned_path");
        this->declare_parameter("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("global_frame", "odom");
        this->declare_parameter("controller_frequency", 20.0);

        // Get parameters
        lookahead_dist_ = this->get_parameter("lookahead_dist").as_double();
        min_lookahead_dist_ = this->get_parameter("min_lookahead_dist").as_double();
        max_lookahead_dist_ = this->get_parameter("max_lookahead_dist").as_double();
        lookahead_time_ = this->get_parameter("lookahead_time").as_double();
        desired_linear_vel_ = this->get_parameter("desired_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        min_approach_linear_velocity_ = this->get_parameter("min_approach_linear_velocity").as_double();
        approach_velocity_scaling_dist_ = this->get_parameter("approach_velocity_scaling_dist").as_double();
        goal_dist_tol_ = this->get_parameter("goal_dist_tol").as_double();
        path_topic_ = this->get_parameter("path_topic").as_string();
        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        global_frame_ = this->get_parameter("global_frame").as_string();
        double controller_frequency = this->get_parameter("controller_frequency").as_double();

        // Publishers and subscribers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic_, 10,
            std::bind(&PurePursuitController::pathCallback, this, std::placeholders::_1));
            
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
            RCLCPP_WARN(this->get_logger(), "Received an empty path.");
            return;
        }
        current_path_ = *msg;
        path_received_ = true;
        goal_reached_ = false;
        RCLCPP_INFO(this->get_logger(), "Received path with %zu points", current_path_.poses.size());
    }
    
    void controlLoop()
    {
        if (!path_received_ || current_path_.poses.empty() || goal_reached_) {
            publishZeroVelocity();
            return;
        }
        
        geometry_msgs::msg::PoseStamped robot_pose;
        if (!getRobotPose(robot_pose)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get robot pose");
            publishZeroVelocity();
            return;
        }

        if (isGoalReached(robot_pose)) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            goal_reached_ = true;
            publishZeroVelocity();
            path_received_ = false; 
            return;
        }
        
        auto cmd_vel = computeVelocityCommands(robot_pose);
        cmd_vel_pub_->publish(cmd_vel);
    }

    geometry_msgs::msg::Twist computeVelocityCommands(const geometry_msgs::msg::PoseStamped& robot_pose)
    {
        geometry_msgs::msg::Twist cmd_vel;

        // Find lookahead point
        geometry_msgs::msg::PoseStamped carrot_pose;
        if (!getLookaheadPoint(robot_pose, carrot_pose)) {
            RCLCPP_WARN(this->get_logger(), "Could not find a lookahead point. Stopping.");
            return cmd_vel;
        }
        
        double lookahead_dist = std::hypot(
            carrot_pose.pose.position.x - robot_pose.pose.position.x,
            carrot_pose.pose.position.y - robot_pose.pose.position.y);

        // Calculate curvature
        double curvature = 0.0;
        double linear_vel = desired_linear_vel_;

        // Transform lookahead point to robot's frame
        geometry_msgs::msg::PoseStamped carrot_in_robot_frame;
        if(!transformPose(base_frame_, carrot_pose, carrot_in_robot_frame))
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform lookahead point to robot frame. Stopping.");
            return cmd_vel;
        }

        // The lookahead point is already transformed to the robot frame, so we can simply use atan2
        double angle_to_carrot = atan2(carrot_in_robot_frame.pose.position.y, carrot_in_robot_frame.pose.position.x);
        curvature = 2.0 * sin(angle_to_carrot) / lookahead_dist;
        
        // Regulate linear velocity
        linear_vel = calculateSpeed(robot_pose);

        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = linear_vel * curvature;

        // Apply constraints
        if (std::abs(cmd_vel.angular.z) > max_angular_vel_) {
            cmd_vel.angular.z = std::copysign(max_angular_vel_, cmd_vel.angular.z);
        }

        return cmd_vel;
    }

    double calculateSpeed(const geometry_msgs::msg::PoseStamped & robot_pose)
    {
        double dist_to_goal = std::hypot(
            robot_pose.pose.position.x - current_path_.poses.back().pose.position.x,
            robot_pose.pose.position.y - current_path_.poses.back().pose.position.y);

        if (dist_to_goal < approach_velocity_scaling_dist_) {
            double velocity = std::max(min_approach_linear_velocity_, 
                                     (dist_to_goal / approach_velocity_scaling_dist_) * desired_linear_vel_);
            return velocity;
        }
        
        return desired_linear_vel_;
    }

    bool getLookaheadPoint(const geometry_msgs::msg::PoseStamped& robot_pose,
                           geometry_msgs::msg::PoseStamped& lookahead_point)
    {
        if (current_path_.poses.empty()) return false;

        double current_speed = desired_linear_vel_; // Simplified for now
        double lookahead_dist = std::clamp(lookahead_dist_ + lookahead_time_ * current_speed, 
                                           min_lookahead_dist_, max_lookahead_dist_);

        // Find the closest point on the path to the robot
        auto closest_it = std::min_element(current_path_.poses.begin(), current_path_.poses.end(),
            [&](const auto& p1, const auto& p2){
                return std::hypot(p1.pose.position.x - robot_pose.pose.position.x, p1.pose.position.y - robot_pose.pose.position.y) <
                       std::hypot(p2.pose.position.x - robot_pose.pose.position.x, p2.pose.position.y - robot_pose.pose.position.y);
            });

        // From the closest point, find the first point that is beyond the lookahead distance
        for (auto it = closest_it; it != current_path_.poses.end(); ++it) {
            double dist = std::hypot(it->pose.position.x - robot_pose.pose.position.x,
                                     it->pose.position.y - robot_pose.pose.position.y);
            if (dist >= lookahead_dist) {
                lookahead_point = *it;
                carrot_pub_->publish(lookahead_point);
                return true;
            }
        }

        // If no point is far enough, take the last point of the path
        double dist_to_goal = std::hypot(
            robot_pose.pose.position.x - current_path_.poses.back().pose.position.x,
            robot_pose.pose.position.y - current_path_.poses.back().pose.position.y);
        
        if (dist_to_goal > goal_dist_tol_) {
            lookahead_point = current_path_.poses.back();
            carrot_pub_->publish(lookahead_point);
            return true;
        }

        return false;
    }
    
    bool getRobotPose(geometry_msgs::msg::PoseStamped& robot_pose)
    {
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                global_frame_, base_frame_, tf2::TimePointZero, std::chrono::nanoseconds(100000000));
            
            robot_pose.header.stamp = this->now();
            robot_pose.header.frame_id = global_frame_;
            robot_pose.pose.position.x = transform.transform.translation.x;
            robot_pose.pose.position.y = transform.transform.translation.y;
            robot_pose.pose.position.z = transform.transform.translation.z;
            robot_pose.pose.orientation = transform.transform.rotation;
            
            return true;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Transform lookup failed: %s", ex.what());
            return false;
        }
    }

    bool transformPose(const std::string& frame, const geometry_msgs::msg::PoseStamped& in_pose,
                       geometry_msgs::msg::PoseStamped& out_pose) const
    {
        if (in_pose.header.frame_id == frame) {
            out_pose = in_pose;
            return true;
        }

        try {
            tf_buffer_.transform(in_pose, out_pose, frame);
            return true;
        } catch (tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Exception in transformPose: %s", ex.what());
        }
        return false;
    }
    
    bool isGoalReached(const geometry_msgs::msg::PoseStamped& robot_pose)
    {
        if (current_path_.poses.empty()) return false;
        
        const auto& goal = current_path_.poses.back();
        double dx = goal.pose.position.x - robot_pose.pose.position.x;
        double dy = goal.pose.position.y - robot_pose.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        return distance < goal_dist_tol_;
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
    double max_angular_vel_;
    double min_approach_linear_velocity_;
    double approach_velocity_scaling_dist_;
    double goal_dist_tol_;
    std::string path_topic_;
    std::string cmd_vel_topic_;
    std::string base_frame_;
    std::string global_frame_;
    
    // ROS components
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // State
    nav_msgs::msg::Path current_path_;
    bool path_received_ = false;
    bool goal_reached_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
