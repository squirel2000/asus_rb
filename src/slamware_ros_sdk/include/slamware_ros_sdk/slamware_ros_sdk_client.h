
#pragma once

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/message_filter.h>

#include <geometry_msgs/msg/point_stamped.hpp>

#include "utils.h"

#include <slamware_ros_sdk/msg/line2_d_flt32_array.hpp>
#include <slamware_ros_sdk/msg/rect_int32.hpp>
#include <slamware_ros_sdk/msg/robot_device_info.hpp>
#include <slamware_ros_sdk/msg/basic_sensor_info_array.hpp>
#include <slamware_ros_sdk/msg/basic_sensor_value_data_array.hpp>
#include <slamware_ros_sdk/msg/robot_basic_state.hpp>
#include <slamware_ros_sdk/msg/sync_map_request.hpp>
#include <slamware_ros_sdk/msg/move_by_direction_request.hpp>
#include <slamware_ros_sdk/msg/move_by_theta_request.hpp>
#include <slamware_ros_sdk/msg/move_to_request.hpp>
#include <slamware_ros_sdk/msg/move_to_locations_request.hpp>
#include <slamware_ros_sdk/msg/rotate_to_request.hpp>
#include <slamware_ros_sdk/msg/rotate_request.hpp>
#include <slamware_ros_sdk/msg/recover_localization_request.hpp>
#include <slamware_ros_sdk/msg/clear_map_request.hpp>
#include <slamware_ros_sdk/msg/set_map_update_request.hpp>
#include <slamware_ros_sdk/msg/set_map_localization_request.hpp>
#include <slamware_ros_sdk/msg/go_home_request.hpp>
#include <slamware_ros_sdk/msg/cancel_action_request.hpp>
#include <slamware_ros_sdk/msg/add_line_request.hpp>
#include <slamware_ros_sdk/msg/add_lines_request.hpp>
#include <slamware_ros_sdk/msg/remove_line_request.hpp>
#include <slamware_ros_sdk/msg/clear_lines_request.hpp>
#include <slamware_ros_sdk/msg/move_line_request.hpp>
#include <slamware_ros_sdk/msg/move_lines_request.hpp>

#include <slamware_ros_sdk/srv/sync_get_stcm.hpp>
#include <slamware_ros_sdk/srv/sync_set_stcm.hpp>

#include <boost/filesystem/path.hpp>
#include <memory>

namespace slamware_ros_sdk {

    class SlamwareRosSdkClient
    {
    public:
        typedef boost::filesystem::path             fs_path_t;

    public:
        explicit SlamwareRosSdkClient(std::shared_ptr<rclcpp::Node> nhRos
            , const char* serverNodeName = nullptr
            , const char* msgNamePrefix = nullptr
            );
        ~SlamwareRosSdkClient();

    public:
        //////////////////////////////////////////////////////////////////////////

        void syncMap(const slamware_ros_sdk::msg::SyncMapRequest& msg) { return pubSyncMap_->publish(msg); }
        void setPose(const geometry_msgs::msg::Pose& msg) { pubSetPose_->publish(msg); }

        void recoverLocalization(const slamware_ros_sdk::msg::RecoverLocalizationRequest& msg) { pubRecoverLocalization_->publish(msg); }
        void clearMap(const slamware_ros_sdk::msg::ClearMapRequest& msg) { pubClearMap_->publish(msg); }
        void setMapUpdate(const slamware_ros_sdk::msg::SetMapUpdateRequest& msg) { pubSetMapUpdate_->publish(msg); }
        void setMapLocalization(const slamware_ros_sdk::msg::SetMapLocalizationRequest& msg) { pubSetMapLocalization_->publish(msg); }

        void moveBy(const slamware_ros_sdk::msg::MoveByDirectionRequest& msg) { pubMoveByDirection_->publish(msg); }
        void moveBy(const slamware_ros_sdk::msg::MoveByThetaRequest& msg) { pubMoveByTheta_->publish(msg); }
        void moveTo(const slamware_ros_sdk::msg::MoveToRequest& msg) { pubMoveTo_->publish(msg); }
        void moveTo(const slamware_ros_sdk::msg::MoveToLocationsRequest& msg) { pubMoveToLocations_->publish(msg); }
        void rotateTo(const slamware_ros_sdk::msg::RotateToRequest& msg) { pubRotateTo_->publish(msg); }
        void rotate(const slamware_ros_sdk::msg::RotateRequest& msg) { pubRotate_->publish(msg); }

        void goHome(const slamware_ros_sdk::msg::GoHomeRequest& msg) { pubGoHome_->publish(msg); }
        void cancelAction(const slamware_ros_sdk::msg::CancelActionRequest& msg) { pubCancelAction_->publish(msg); }

        void addLine(const slamware_ros_sdk::msg::AddLineRequest& msg) { pubAddLine_->publish(msg); }
        void addLines(const slamware_ros_sdk::msg::AddLinesRequest& msg) { pubAddLines_->publish(msg); }
        void removeLine(const slamware_ros_sdk::msg::RemoveLineRequest& msg) { pubRemoveLine_->publish(msg); }
        void clearLines(const slamware_ros_sdk::msg::ClearLinesRequest& msg) { pubClearLines_->publish(msg); }
        void moveLine(const slamware_ros_sdk::msg::MoveLineRequest& msg) { pubMoveLine_->publish(msg); }
        void moveLines(const slamware_ros_sdk::msg::MoveLinesRequest& msg) { pubMoveLines_->publish(msg); }

        //////////////////////////////////////////////////////////////////////////

        bool syncGetStcm(std::shared_ptr<slamware_ros_sdk::srv::SyncGetStcm::Request> srvMsg, std::shared_ptr<slamware_ros_sdk::srv::SyncGetStcm::Response> srvMsgResp) 
        { 
            auto result = scSyncGetStcm_->async_send_request(srvMsg); 
            if (rclcpp::spin_until_future_complete(nh_, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                srvMsgResp->raw_stcm = result.get()->raw_stcm;
                return true;
            } 
            return false;
        }
        // get stcm and write to filePath.
        bool syncGetStcm(std::string& errMsg
            , const fs_path_t& filePath
            );

        bool syncSetStcm(std::shared_ptr<slamware_ros_sdk::srv::SyncSetStcm::Request> srvMsg) 
        { 
            auto result = scSyncSetStcm_->async_send_request(srvMsg); 
            if (rclcpp::spin_until_future_complete(nh_, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                return true;
            } 
            return false;
        }
        // load stcm from filePath, and upload to slamware.
        bool syncSetStcm(const fs_path_t& filePath
            , const geometry_msgs::msg::Pose& robotPose
            , std::string& errMsg
            );

        //////////////////////////////////////////////////////////////////////////

    private:
        std::string genTopicFullName_(const std::string& strName) const { return msgNamePrefix_ + strName; }

    private:
        std::shared_ptr<rclcpp::Node> nh_;
        std::string sdkServerNodeName_;
        std::string msgNamePrefix_;

        rclcpp::Publisher<slamware_ros_sdk::msg::SyncMapRequest>::SharedPtr pubSyncMap_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pubSetPose_;

        rclcpp::Publisher<slamware_ros_sdk::msg::RecoverLocalizationRequest>::SharedPtr pubRecoverLocalization_;
        rclcpp::Publisher<slamware_ros_sdk::msg::ClearMapRequest>::SharedPtr pubClearMap_;
        rclcpp::Publisher<slamware_ros_sdk::msg::SetMapUpdateRequest>::SharedPtr pubSetMapUpdate_;
        rclcpp::Publisher<slamware_ros_sdk::msg::SetMapLocalizationRequest>::SharedPtr pubSetMapLocalization_;

        rclcpp::Publisher<slamware_ros_sdk::msg::MoveByDirectionRequest>::SharedPtr pubMoveByDirection_;
        rclcpp::Publisher<slamware_ros_sdk::msg::MoveByThetaRequest>::SharedPtr pubMoveByTheta_;
        rclcpp::Publisher<slamware_ros_sdk::msg::MoveToRequest>::SharedPtr pubMoveTo_;
        rclcpp::Publisher<slamware_ros_sdk::msg::MoveToLocationsRequest>::SharedPtr pubMoveToLocations_;
        rclcpp::Publisher<slamware_ros_sdk::msg::RotateToRequest>::SharedPtr pubRotateTo_;
        rclcpp::Publisher<slamware_ros_sdk::msg::RotateRequest>::SharedPtr pubRotate_;

        rclcpp::Publisher<slamware_ros_sdk::msg::GoHomeRequest>::SharedPtr pubGoHome_;
        rclcpp::Publisher<slamware_ros_sdk::msg::CancelActionRequest>::SharedPtr pubCancelAction_;

        rclcpp::Publisher<slamware_ros_sdk::msg::AddLineRequest>::SharedPtr pubAddLine_;
        rclcpp::Publisher<slamware_ros_sdk::msg::AddLinesRequest>::SharedPtr pubAddLines_;
        rclcpp::Publisher<slamware_ros_sdk::msg::RemoveLineRequest>::SharedPtr pubRemoveLine_;
        rclcpp::Publisher<slamware_ros_sdk::msg::ClearLinesRequest>::SharedPtr pubClearLines_;
        rclcpp::Publisher<slamware_ros_sdk::msg::MoveLineRequest>::SharedPtr pubMoveLine_;
        rclcpp::Publisher<slamware_ros_sdk::msg::MoveLinesRequest>::SharedPtr pubMoveLines_;

        rclcpp::Client<slamware_ros_sdk::srv::SyncGetStcm>::SharedPtr scSyncGetStcm_;
        rclcpp::Client<slamware_ros_sdk::srv::SyncSetStcm>::SharedPtr scSyncSetStcm_;
    };

}
