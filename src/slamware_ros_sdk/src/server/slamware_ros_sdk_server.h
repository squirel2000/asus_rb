/**
 kint.zhao  huasheng_zyh@163.com
   2017.0721

  modified by yun.li@slamtec.com, 2019.
*/

#pragma once

#include "server_workers.h"

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include <slamware_ros_sdk/srv/sync_get_stcm.hpp>
#include <slamware_ros_sdk/srv/sync_set_stcm.hpp>

#include <message_filters/subscriber.h>

#include <boost/atomic.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

#include <boost/function.hpp>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace slamware_ros_sdk {

    class SlamwareRosSdkServer : public rclcpp::Node
    {
    private:
        friend class ServerWorkerBase;

    public:
        SlamwareRosSdkServer();
        ~SlamwareRosSdkServer();

        bool startRun(std::string& errMsg);
        void requestStop();
        void waitUntilStopped(); // not thread-safe

        const std::shared_ptr<rclcpp::Node> getRosNodeHandle() const { return nh_; }

    public:
        void requestSyncMap();

    private:
        enum ServerState
        {
            ServerStateNotInit
            , ServerStateRunning
            , ServerStateStopped
        };

        typedef rpos::robot_platforms::SlamwareCorePlatform     slamware_platform_t;

        template<class MsgT>
        struct msg_cb_help_t
        {
            typedef MsgT                                msg_t;
            typedef typename msg_t::SharedPtr           msg_shared_ptr;
            typedef void (SlamwareRosSdkServer::*msg_cb_perform_fun_t)(slamware_platform_t&,  msg_shared_ptr);
            typedef boost::function< void( msg_shared_ptr) >        ros_cb_fun_t; // callback function for ROS.
        };

        template<class SrvMsgT>
        struct srv_cb_help_t
        {
            typedef SrvMsgT                             srv_msg_t;
            typedef typename srv_msg_t::Request::SharedPtr         request_t;
            typedef typename srv_msg_t::Response::SharedPtr        response_t;
            typedef bool (SlamwareRosSdkServer::*srv_cb_perform_fun_t)(slamware_platform_t&, request_t, response_t);
            typedef boost::function< bool(request_t, response_t) >            ros_cb_fun_t; // callback function for ROS.
        };

    private:
        static boost::chrono::milliseconds sfConvFloatSecToBoostMs_(float fSec);

        bool isRunning_() const { return ServerStateRunning == state_.load(); }
        bool shouldContinueRunning_() const;

        const std::shared_ptr<rclcpp::Node> rosNodeHandle_() const { return nh_; }
        std::shared_ptr<rclcpp::Node> rosNodeHandle_() { return nh_; }

        const ServerParams& serverParams_() const { return params_; }
        
        const std::unique_ptr<tf2_ros::TransformBroadcaster>& tfBroadcaster_() const { return tfBrdcstr_; }
        std::unique_ptr<tf2_ros::TransformBroadcaster>& tfBroadcaster_() { return tfBrdcstr_; }

        ServerWorkData_ConstPtr safeGetWorkData_() const;
        ServerWorkData_Ptr safeGetMutableWorkData_();

        bool safeIsSlamwarePlatformConnected_() const;
        slamware_platform_t safeGetSlamwarePlatform_() const;
        void safeSetSlamwarePlatform_(const slamware_platform_t& pltfm);
        void safeReleaseSlamwarePlatform_();
        slamware_platform_t connectSlamwarePlatform_(const std::string& ip, int port) const;
        void disconnectSlamwarePlatform_(slamware_platform_t& pltfm) const;

        bool init_(std::string& errMsg);
        void cleanup_();

        void workThreadFun_();

        void roughSleepWait_(std::uint32_t maxSleepMs, std::uint32_t onceSleepMs);
        void loopTryConnectToSlamwarePlatform_();

        bool reinitWorkLoop_(slamware_platform_t& pltfm);
        void loopWork_();

        //////////////////////////////////////////////////////////////////////////
        // subscribed messages
        //////////////////////////////////////////////////////////////////////////

        template<class MsgT>
        void msgCbWrapperFun_T_(typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform
            , const std::string& msgTopic
            ,  typename msg_cb_help_t<MsgT>::msg_shared_ptr  msg
            );
        template<class MsgT>
        typename rclcpp::Subscription<MsgT>::SharedPtr subscribe_T_(const std::string& msgTopic
            , std::uint32_t queueSize
            , typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform
            );

        void msgCbRobotControl_(slamware_platform_t& pltfm, const geometry_msgs::msg::Twist::SharedPtr msg);
        void msgCbRobotControlNoLimit_(slamware_platform_t& pltfm, const geometry_msgs::msg::Twist::SharedPtr msg);
        void msgCbMoveToGoal_(slamware_platform_t& pltfm, const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        
        void msgCbSyncMap_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::SyncMapRequest::SharedPtr msg);
        void msgCbSetPose_(slamware_platform_t& pltfm, const geometry_msgs::msg::Pose::SharedPtr msg);

        void msgCbRecoverLocalization_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::RecoverLocalizationRequest::SharedPtr msg);
        void msgCbClearMap_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::ClearMapRequest::SharedPtr msg);
        void msgCbSetMapUpdate_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::SetMapUpdateRequest::SharedPtr msg);
        void msgCbSetMapLocalization_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::SetMapLocalizationRequest::SharedPtr msg);

        void msgCbMoveByDirection_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::MoveByDirectionRequest::SharedPtr msg);
        void msgCbMoveByTheta_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::MoveByThetaRequest::SharedPtr msg);
        void msgCbMoveTo_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::MoveToRequest::SharedPtr msg);
        void msgCbMoveToLocations_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::MoveToLocationsRequest::SharedPtr msg);
        void msgCbRotateTo_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::RotateToRequest::SharedPtr msg);
        void msgCbRotate_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::RotateRequest::SharedPtr msg);

        void msgCbGoHome_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::GoHomeRequest::SharedPtr msg);
        void msgCbCancelAction_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::CancelActionRequest::SharedPtr msg);

        void msgCbAddLine_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::AddLineRequest::SharedPtr msg);
        void msgCbAddLines_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::AddLinesRequest::SharedPtr msg);
        void msgCbRemoveLine_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::RemoveLineRequest::SharedPtr msg);
        void msgCbClearLines_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::ClearLinesRequest::SharedPtr msg);
        void msgCbMoveLine_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::MoveLineRequest::SharedPtr msg);
        void msgCbMoveLines_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::MoveLinesRequest::SharedPtr msg);

        //////////////////////////////////////////////////////////////////////////
        // advertised services
        //////////////////////////////////////////////////////////////////////////

        template<class SrvMsgT>
        bool srvCbWrapperFun_T_(typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform
            , const std::string& srvMsgTopic
            , typename srv_cb_help_t<SrvMsgT>::request_t req
            , typename srv_cb_help_t<SrvMsgT>::response_t resp
            );
        template<class SrvMsgT>
        typename rclcpp::Service<SrvMsgT>::SharedPtr advertiseService_T_(const std::string& srvMsgTopic
            , typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform
            );

        bool srvCbSyncGetStcm_(slamware_platform_t& pltfm, slamware_ros_sdk::srv::SyncGetStcm::Request::SharedPtr req, slamware_ros_sdk::srv::SyncGetStcm::Response::SharedPtr resp);
        bool srvCbSyncSetStcm_(slamware_platform_t& pltfm, slamware_ros_sdk::srv::SyncSetStcm::Request::SharedPtr req, slamware_ros_sdk::srv::SyncSetStcm::Response::SharedPtr resp);

        //////////////////////////////////////////////////////////////////////////

    private:
        boost::atomic<ServerState> state_;
        boost::atomic<bool> isStopRequested_;
        
        std::shared_ptr<rclcpp::Node> nh_;
        ServerParams params_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tfBrdcstr_;

        mutable boost::mutex workDatLock_;
        ServerWorkData_Ptr workDat_;

        std::vector<ServerWorkerBase_Ptr> serverWorkers_;

        // subscriptions
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subRobotControl_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subRobotControlNoLimit_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subMoveToGoal_;
        
        rclcpp::Subscription<slamware_ros_sdk::msg::SyncMapRequest>::SharedPtr subSyncMap_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subSetPose_;

        rclcpp::Subscription<slamware_ros_sdk::msg::RecoverLocalizationRequest>::SharedPtr subRecoverLocalization_;
        rclcpp::Subscription<slamware_ros_sdk::msg::ClearMapRequest>::SharedPtr subClearMap_;
        rclcpp::Subscription<slamware_ros_sdk::msg::SetMapUpdateRequest>::SharedPtr subSetMapUpdate_;
        rclcpp::Subscription<slamware_ros_sdk::msg::SetMapLocalizationRequest>::SharedPtr subSetMapLocalization_;

        rclcpp::Subscription<slamware_ros_sdk::msg::MoveByDirectionRequest>::SharedPtr subMoveByDirection_;
        rclcpp::Subscription<slamware_ros_sdk::msg::MoveByThetaRequest>::SharedPtr subMoveByTheta_;
        rclcpp::Subscription<slamware_ros_sdk::msg::MoveToRequest>::SharedPtr subMoveTo_;
        rclcpp::Subscription<slamware_ros_sdk::msg::MoveToLocationsRequest>::SharedPtr subMoveToLocations_;
        rclcpp::Subscription<slamware_ros_sdk::msg::RotateToRequest>::SharedPtr subRotateTo_;
        rclcpp::Subscription<slamware_ros_sdk::msg::RotateRequest>::SharedPtr subRotate_;

        rclcpp::Subscription<slamware_ros_sdk::msg::GoHomeRequest>::SharedPtr subGoHome_;
        rclcpp::Subscription<slamware_ros_sdk::msg::CancelActionRequest>::SharedPtr subCancelAction_;

        rclcpp::Subscription<slamware_ros_sdk::msg::AddLineRequest>::SharedPtr subAddLine_;
        rclcpp::Subscription<slamware_ros_sdk::msg::AddLinesRequest>::SharedPtr subAddLines_;
        rclcpp::Subscription<slamware_ros_sdk::msg::RemoveLineRequest>::SharedPtr subRemoveLine_;
        rclcpp::Subscription<slamware_ros_sdk::msg::ClearLinesRequest>::SharedPtr subClearLines_;
        rclcpp::Subscription<slamware_ros_sdk::msg::MoveLineRequest>::SharedPtr subMoveLine_;
        rclcpp::Subscription<slamware_ros_sdk::msg::MoveLinesRequest>::SharedPtr subMoveLines_;
        
        rpos::actions::VelocityControlMoveAction velocityControllAction_;
        
        // services
        rclcpp::Service<slamware_ros_sdk::srv::SyncGetStcm>::SharedPtr srvSyncGetStcm_;
        rclcpp::Service<slamware_ros_sdk::srv::SyncSetStcm>::SharedPtr srvSyncSetStcm_;

        boost::thread workThread_;

        mutable boost::mutex slamwarePltfmLock_;
        slamware_platform_t slamwarePltfm_;

    };
    
}
