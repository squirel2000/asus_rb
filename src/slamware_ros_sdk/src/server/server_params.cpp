
#include "server_params.h"

#include <cmath>

namespace slamware_ros_sdk {

    //////////////////////////////////////////////////////////////////////////

    const float C_FLT_PI = ((float)M_PI);
    const float C_FLT_2PI = (C_FLT_PI * 2);

    //////////////////////////////////////////////////////////////////////////

    ServerParams::ServerParams()
    : Node("server_params")
    {
        resetToDefault();
    }

    void ServerParams::resetToDefault()
    {
        this->declare_parameter<std::string>("ip_address", "192.168.11.1");
        this->declare_parameter<int>("robot_port", 1445);
        this->declare_parameter<int>("reconn_wait_ms", 1000 * 3);

        this->declare_parameter<bool>("angle_compensate", true);
        this->declare_parameter<bool>("fixed_odom_map_tf", true);
        this->declare_parameter<bool>("raw_ladar_data", false);
        this->declare_parameter<bool>("ladar_data_clockwise", true);
        this->declare_parameter<bool>("pub_accumulate_odometry", false);

        this->declare_parameter<std::string>("robot_frame", "base_link");
        this->declare_parameter<std::string>("laser_frame", "laser");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("robot_pose_frame", "robot_pose");

        this->declare_parameter<float>("odometry_pub_period", 0.05f);
        this->declare_parameter<float>("robot_pose_pub_period", 0.05f);
        this->declare_parameter<float>("scan_pub_period", 0.1f);
        this->declare_parameter<float>("map_update_period", 0.2f);
        this->declare_parameter<float>("map_pub_period", 0.2f);
        this->declare_parameter<float>("basic_sensors_info_update_period", 7.0f);
        this->declare_parameter<float>("basic_sensors_values_pub_period", 0.05f);
        this->declare_parameter<float>("path_pub_period", 0.05f);
        this->declare_parameter<float>("robot_basic_state_pub_period", 1.0f);
        this->declare_parameter<float>("virtual_walls_pub_period", 0.5f);
        this->declare_parameter<float>("virtual_tracks_pub_period", 0.5f);

        this->declare_parameter<float>("map_sync_once_get_max_wh", 100.f);
        this->declare_parameter<float>("map_update_near_robot_half_wh", 8.0f);

        this->declare_parameter<float>("imu_raw_data_period", 0.05f);

        this->declare_parameter<std::string>("scan_topic", "/slamware_ros_sdk_server_node/scan");
        this->declare_parameter<std::string>("odom_topic", "/slamware_ros_sdk_server_node/odom");
        this->declare_parameter<std::string>("robot_pose_topic", "/slamware_ros_sdk_server_node/robot_pose");
        this->declare_parameter<std::string>("map_topic", "/slamware_ros_sdk_server_node/map");
        this->declare_parameter<std::string>("map_info_topic", "/slamware_ros_sdk_server_node/map_metadata");
        this->declare_parameter<std::string>("basic_sensors_info_topic", "/slamware_ros_sdk_server_node/basic_sensors_info");
        this->declare_parameter<std::string>("basic_sensors_values_topic", "/slamware_ros_sdk_server_node/basic_sensors_values");
        this->declare_parameter<std::string>("path_topic", "/slamware_ros_sdk_server_node/global_plan_path");

        this->declare_parameter<std::string>("vel_control_topic", "/cmd_vel");
        this->declare_parameter<std::string>("goal_topic", "/move_base_simple/goal");

        this->declare_parameter<std::string>("imu_raw_data_topic", "/slamware_ros_sdk_server_node/imu_raw_data");
        this->declare_parameter<std::string>("imu_raw_mag_data_topic", "/slamware_ros_sdk_server_node/imu_raw_mag_data");
    }

    void ServerParams::setBy(const std::shared_ptr<rclcpp::Node> nhRos)
    {
        std::string strVal;
        bool bVal;
        int iVal;
        float fVal;
        if (nhRos->has_parameter("ip_address")) {
            nhRos->declare_parameter<std::string>("ip_address", strVal);
        }
        if (nhRos->has_parameter("robot_port")) {
            nhRos->declare_parameter<int>("robot_port", iVal);
        }
        if (nhRos->has_parameter("reconn_wait_ms")) {
            nhRos->declare_parameter<int>("reconn_wait_ms", iVal);
        }
        if (nhRos->has_parameter("angle_compensate")) {
            nhRos->declare_parameter<bool>("angle_compensate", bVal);
        }
        if (nhRos->has_parameter("fixed_odom_map_tf")) {
            nhRos->declare_parameter<bool>("fixed_odom_map_tf", bVal);
        }
        if (nhRos->has_parameter("raw_ladar_data")) {
            nhRos->declare_parameter<bool>("raw_ladar_data", bVal);
        }
        if (nhRos->has_parameter("ladar_data_clockwise")) {
            nhRos->declare_parameter<bool>("ladar_data_clockwise", bVal);
        }
        if (nhRos->has_parameter("pub_accumulate_odometry")) {
            nhRos->declare_parameter<bool>("pub_accumulate_odometry", bVal);
        }

        if (nhRos->has_parameter("robot_frame")) {
            nhRos->declare_parameter<std::string>("robot_frame", strVal);
        }
        if (nhRos->has_parameter("laser_frame")) {
            nhRos->declare_parameter<std::string>("laser_frame", strVal);
        }
        if (nhRos->has_parameter("map_frame")) {
            nhRos->declare_parameter<std::string>("map_frame", strVal);
        }
        if (nhRos->has_parameter("odom_frame")) {
            nhRos->declare_parameter<std::string>("odom_frame", strVal);
        }
        if (nhRos->has_parameter("robot_pose_frame")) {
            nhRos->declare_parameter<std::string>("robot_pose_frame", strVal);
        }
        if (nhRos->has_parameter("odometry_pub_period")) {
            nhRos->declare_parameter<float>("odometry_pub_period", fVal);
        }
        if (nhRos->has_parameter("robot_pose_pub_period")) {
            nhRos->declare_parameter<float>("robot_pose_pub_period", fVal);
        }
        if (nhRos->has_parameter("scan_pub_period")) {
            nhRos->declare_parameter<float>("scan_pub_period", fVal);
        }
        if (nhRos->has_parameter("map_update_period")) {
            nhRos->declare_parameter<float>("map_update_period", fVal);
        }
        if (nhRos->has_parameter("map_pub_period")) {
            nhRos->declare_parameter<float>("map_pub_period", fVal);
        }
        if (nhRos->has_parameter("basic_sensors_info_update_period")) {
            nhRos->declare_parameter<float>("basic_sensors_info_update_period", fVal);
        }
        if (nhRos->has_parameter("basic_sensors_values_pub_period")) {
            nhRos->declare_parameter<float>("basic_sensors_values_pub_period", fVal);
        }
        if (nhRos->has_parameter("path_pub_period")) {
            nhRos->declare_parameter<float>("path_pub_period", fVal);
        }
        if (nhRos->has_parameter("robot_basic_state_pub_period")) {
            nhRos->declare_parameter<float>("robot_basic_state_pub_period", fVal);
        }
        if (nhRos->has_parameter("virtual_walls_pub_period")) {
            nhRos->declare_parameter<float>("virtual_walls_pub_period", fVal);
        }
        if (nhRos->has_parameter("virtual_tracks_pub_period")) {
            nhRos->declare_parameter<float>("virtual_tracks_pub_period", fVal);
        }
        if (nhRos->has_parameter("map_sync_once_get_max_wh")) {
            nhRos->declare_parameter<float>("map_sync_once_get_max_wh", fVal);
        }
        if (nhRos->has_parameter("map_update_near_robot_half_wh")) {
            nhRos->declare_parameter<float>("map_update_near_robot_half_wh", fVal);
        }
        if (nhRos->has_parameter("scan_topic")) {
            nhRos->declare_parameter<std::string>("scan_topic", strVal);
        }
        if (nhRos->has_parameter("odom_topic")) {
            nhRos->declare_parameter<std::string>("odom_topic", strVal);
        }
        if (nhRos->has_parameter("robot_pose_topic")) {
            nhRos->declare_parameter<std::string>("robot_pose_topic", strVal);
        }
        if (nhRos->has_parameter("map_topic")) {
            nhRos->declare_parameter<std::string>("map_topic", strVal);
        }
        if (nhRos->has_parameter("map_info_topic")) {
            nhRos->declare_parameter<std::string>("map_info_topic", strVal);
        }
        if (nhRos->has_parameter("basic_sensors_info_topic")) {
            nhRos->declare_parameter<std::string>("basic_sensors_info_topic", strVal);
        }
        if (nhRos->has_parameter("basic_sensors_values_topic")) {
            nhRos->declare_parameter<std::string>("basic_sensors_values_topic", strVal);
        }
        if (nhRos->has_parameter("path_topic")) {
            nhRos->declare_parameter<std::string>("path_topic", strVal);
        }
        if (nhRos->has_parameter("vel_control_topic")) {
            nhRos->declare_parameter<std::string>("vel_control_topic", strVal);
        }
        if (nhRos->has_parameter("goal_topic")) {
            nhRos->declare_parameter<std::string>("goal_topic", strVal);
        }
        if (nhRos->has_parameter("imu_raw_data_topic")) {
            nhRos->declare_parameter<std::string>("imu_raw_data_topic", strVal);
        }
        if (nhRos->has_parameter("imu_raw_mag_data_topic")) {
            nhRos->declare_parameter<std::string>("imu_raw_mag_data_topic", strVal);
        }
        if (nhRos->has_parameter("imu_raw_data_period")) {
            nhRos->declare_parameter<float>("imu_raw_data_period", fVal);
        }
    }

    //////////////////////////////////////////////////////////////////////////
    
}
