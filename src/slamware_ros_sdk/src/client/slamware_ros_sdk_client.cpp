
#include <slamware_ros_sdk/slamware_ros_sdk_client.h>

#include <boost/assert.hpp>
#include <boost/filesystem/fstream.hpp>

namespace slamware_ros_sdk {
    
    SlamwareRosSdkClient::SlamwareRosSdkClient(std::shared_ptr<rclcpp::Node> nhRos
        , const char* serverNodeName
        , const char* msgNamePrefix
        )
        : nh_(nhRos)
    {
        if (nullptr != serverNodeName)
            sdkServerNodeName_ = serverNodeName;
        else
            sdkServerNodeName_ = "slamware_ros_sdk_server_node";

        if (nullptr != msgNamePrefix)
        {
            msgNamePrefix_ = msgNamePrefix;
        }
        else if (!sdkServerNodeName_.empty())
        {
            if ('/' != sdkServerNodeName_.front())
                msgNamePrefix_ = "/";
            msgNamePrefix_ += sdkServerNodeName_;
            if ('/' != msgNamePrefix_.back())
                msgNamePrefix_ += "/";
        }

        // initialize publishers
        {
            pubSyncMap_ = nh_->create_publisher<slamware_ros_sdk::msg::SyncMapRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/sync_map"), 1);
            pubSetPose_ = nh_->create_publisher<geometry_msgs::msg::Pose>(genTopicFullName_("/slamware_ros_sdk_server_node/set_pose"), 1);
            
            pubRecoverLocalization_ = nh_->create_publisher<slamware_ros_sdk::msg::RecoverLocalizationRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/recover_localization"), 1);
            pubClearMap_ = nh_->create_publisher<slamware_ros_sdk::msg::ClearMapRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/clear_map"), 1);
            pubSetMapUpdate_ = nh_->create_publisher<slamware_ros_sdk::msg::SetMapUpdateRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/set_map_update"), 1);
            pubSetMapLocalization_ = nh_->create_publisher<slamware_ros_sdk::msg::SetMapLocalizationRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/set_map_localization"), 1);

            pubMoveByDirection_ = nh_->create_publisher<slamware_ros_sdk::msg::MoveByDirectionRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/move_by_direction"), 1);
            pubMoveByTheta_ = nh_->create_publisher<slamware_ros_sdk::msg::MoveByThetaRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/move_by_theta"), 1);
            pubMoveTo_ = nh_->create_publisher<slamware_ros_sdk::msg::MoveToRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/move_to"), 1);
            pubMoveToLocations_ = nh_->create_publisher<slamware_ros_sdk::msg::MoveToLocationsRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/move_to_locations"), 1);
            pubRotateTo_ = nh_->create_publisher<slamware_ros_sdk::msg::RotateToRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/rotate_to"), 1);
            pubRotate_ = nh_->create_publisher<slamware_ros_sdk::msg::RotateRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/rotate"), 1);

            pubGoHome_ = nh_->create_publisher<slamware_ros_sdk::msg::GoHomeRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/go_home"), 1);
            pubCancelAction_ = nh_->create_publisher<slamware_ros_sdk::msg::CancelActionRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/cancel_action"), 1);

            pubAddLine_ = nh_->create_publisher<slamware_ros_sdk::msg::AddLineRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/add_line"), 1);
            pubAddLines_ = nh_->create_publisher<slamware_ros_sdk::msg::AddLinesRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/add_lines"), 1);
            pubRemoveLine_ = nh_->create_publisher<slamware_ros_sdk::msg::RemoveLineRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/remove_line"), 1);
            pubClearLines_ = nh_->create_publisher<slamware_ros_sdk::msg::ClearLinesRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/clear_lines"), 1);
            pubMoveLine_ = nh_->create_publisher<slamware_ros_sdk::msg::MoveLineRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/move_line"), 1);
            pubMoveLines_ = nh_->create_publisher<slamware_ros_sdk::msg::MoveLinesRequest>(genTopicFullName_("/slamware_ros_sdk_server_node/move_lines"), 1);
        }

        // initialize service clients
        {
            scSyncGetStcm_ = nh_->create_client<slamware_ros_sdk::srv::SyncGetStcm>(genTopicFullName_("sync_get_stcm"));
            scSyncSetStcm_ = nh_->create_client<slamware_ros_sdk::srv::SyncSetStcm>(genTopicFullName_("sync_set_stcm"));
        }
    }

    SlamwareRosSdkClient::~SlamwareRosSdkClient()
    {
        //
    }

    bool SlamwareRosSdkClient::syncGetStcm(std::string& errMsg
        , const fs_path_t& filePath
        )
    {
        errMsg.clear();

        auto srvMsg = std::make_shared<slamware_ros_sdk::srv::SyncGetStcm::Request>();
        auto srvMsgResp = std::make_shared<slamware_ros_sdk::srv::SyncGetStcm::Response>();
        if (!syncGetStcm(srvMsg, srvMsgResp))
        {
            errMsg = "failed to call syncGetStcm().";
            return false;
        }

        {
            boost::filesystem::ofstream ofs(filePath, (std::ios_base::out | std::ios_base::trunc | std::ios_base::binary));
            if (!ofs.is_open())
            {
                errMsg = "failed to open file.";
                return false;
            }
            ofs.write((const char*)srvMsgResp->raw_stcm.data(), srvMsgResp->raw_stcm.size());
            if (ofs.fail())
            {
                errMsg = "failed to write file";
                return false;
            }
        }
        return true;
    }

    bool SlamwareRosSdkClient::syncSetStcm(const fs_path_t& filePath
        , const geometry_msgs::msg::Pose& robotPose
        , std::string& errMsg
        )
    {
        errMsg.clear();

        auto srvMsg = std::make_shared<slamware_ros_sdk::srv::SyncSetStcm::Request>();
        srvMsg->robot_pose = robotPose;
        {
            boost::filesystem::ifstream ifs(filePath, (std::ios_base::in | std::ios_base::binary | std::ios_base::ate));
            if (!ifs.is_open())
            {
                errMsg = "failed to open file";
                return false;
            }
            const auto szDat = ifs.tellg();
            if (boost::filesystem::ifstream::pos_type(-1) == szDat)
            {
                errMsg = "failed to get file size.";
                return false;
            }
            ifs.seekg(0);

            srvMsg->raw_stcm.resize(szDat);
            ifs.read((char*)srvMsg->raw_stcm.data(), szDat);
            if (ifs.gcount() != szDat)
            {
                errMsg = "failed to read file data.";
                return false;
            }
        }

        if (!syncSetStcm(srvMsg))
        {
            errMsg = "failed to call syncSetStcm().";
            return false;
        }
        return true;
    }

}
