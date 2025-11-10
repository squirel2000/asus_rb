#ifndef PURE_PURSUIT_CONTROLLER_HPP_
#define PURE_PURSUIT_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
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
    PurePursuitController();

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();
    geometry_msgs::msg::Twist computeVelocityCommands(const geometry_msgs::msg::PoseStamped& robot_pose);
    double calculateSpeed(double target_velocity);
    bool getLookaheadPoint(const geometry_msgs::msg::PoseStamped& robot_pose,
                           geometry_msgs::msg::PoseStamped& lookahead_point);
    struct Point { double x, y; bool is_valid = false; };
    size_t findClosestPathSegment(const geometry_msgs::msg::PoseStamped& robot_pose, size_t start_idx);
    Point findIntersection(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2,
                           const geometry_msgs::msg::Point& robot_pos, double L);
    bool getRobotPose(geometry_msgs::msg::PoseStamped& robot_pose);
    bool transformPose(const std::string& target_frame, const geometry_msgs::msg::PoseStamped& in_pose,
                        geometry_msgs::msg::PoseStamped& out_pose);
    bool isGoalReached(const geometry_msgs::msg::PoseStamped& robot_pose);
    void publishZeroVelocity();
    
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
    double goal_dist_buf_;
    double goal_dist_tol_;
    std::string path_topic_;
    std::string odom_topic_;
    std::string cmd_vel_topic_;
    std::string base_frame_;
    std::string global_frame_;
    double heading_error_for_pure_rotation_;
    double min_heading_error_for_motion_;
    double controller_frequency_;
    double linear_acceleration_;
    double linear_deceleration_;
    double min_dt_ = 0.01;
    double max_dt_ = 0.5;
    rclcpp::Time last_odom_stamp_;
    std::optional<double> last_odom_dt_;
    bool have_last_odom_stamp_ = false;
    double odom_timeout_ = 1.0;
    // Diagnostics for odom dt
    std::optional<double> odom_dt_ema_;
    double odom_dt_ema_alpha_ = 0.2;
    int max_dt_consec_threshold_ = 3;
    int consec_max_dt_count_ = 0;

    // Filtered target velocity (EMA) used for lookahead and speed smoothing
    std::optional<double> target_velocity_ema_;
    double target_velocity_ema_alpha_ = 0.2;

    // ROS components
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr odom_dt_pub_;
    // Debug publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_target_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_current_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_speed_scale_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // State
    nav_msgs::msg::Path current_path_;
    bool path_received_ = false;
    bool goal_reached_ = true;
    double commanded_velocity_ = 0.0;
    size_t last_path_segment_idx_ = 0;
    bool is_moving_ = false;
    // Odom cache for TF fallback
    std::mutex odom_mutex_;
    std::optional<nav_msgs::msg::Odometry> latest_odom_;
};

#endif // PURE_PURSUIT_CONTROLLER_HPP_
