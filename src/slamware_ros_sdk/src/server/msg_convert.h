
#pragma once

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/message_filter.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/imu.hpp>

#include <slamware_ros_sdk/utils.h>

#include <slamware_ros_sdk/msg/vec2_d_int32.hpp>
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

#include <rpos/core/geometry.h>
#include <rpos/features/artifact_provider.h>
#include <rpos/features/location_provider.h>
#include <rpos/features/motion_planner.h>
#include <rpos/features/system_resource.h>
#include <rpos/features/impact_sensor_feature.h>

#include <vector>
#include <map>

#define M_PI       3.14159265358979323846   // pi

namespace slamware_ros_sdk {

    //////////////////////////////////////////////////////////////////////////
    // Important Notes:
    //      Generally, MsgConvert just overwrites known fields;
    //          unknown fields, which are new added and their codes are
    //          not added into MsgConvert, will be unchanged.
    //////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////
    
    template<class RosMsgT, class SltcValT>
    struct MsgConvert;

    template<class RosMsgT, class SltcValT>
    inline void sltcToRosMsg(const SltcValT& sltcVal, RosMsgT& rosMsg)
    {
        MsgConvert<RosMsgT, SltcValT>::toRos(sltcVal, rosMsg);
    }
    template<class RosMsgT, class SltcValT>
    inline void rosMsgToSltc(const RosMsgT& rosMsg, SltcValT& sltcVal)
    {
        MsgConvert<RosMsgT, SltcValT>::toSltc(rosMsg, sltcVal);
    }

    //////////////////////////////////////////////////////////////////////////

    template<class ValT, class RosMsgT>
    inline void toRosOptionalMsg(const boost::optional<ValT> & optVal, RosMsgT& rosMsg)
    {
        if (optVal)
        {
            rosMsg.is_valid = true;
            sltcToRosMsg(*optVal, rosMsg.value);
        }
        else
        {
            rosMsg = RosMsgT();
        }
    }

    template<class ValT, class RosMsgT>
    inline void fromRosOptionalMsg(const RosMsgT& rosMsg, boost::optional<ValT> & optVal)
    {
        if (rosMsg.is_valid)
        {
            optVal = ValT();
            rosMsgToSltc(rosMsg.value, *optVal);
        }
        else
        {
            optVal.reset();
        }
    }

    //////////////////////////////////////////////////////////////////////////

    template<>
    struct MsgConvert<slamware_ros_sdk::msg::MapKind, rpos::features::location_provider::MapKind>
    {
    public:
        typedef slamware_ros_sdk::msg::MapKind                                        ros_msg_t;
        typedef rpos::features::location_provider::MapKind     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<slamware_ros_sdk::msg::ArtifactUsage, rpos::features::artifact_provider::ArtifactUsage>
    {
    public:
        typedef slamware_ros_sdk::msg::ArtifactUsage                                         ros_msg_t;
        typedef rpos::features::artifact_provider::ArtifactUsage      sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<slamware_ros_sdk::msg::SensorType, rpos::core::SensorType>
    {
    public:
        typedef slamware_ros_sdk::msg::SensorType                  ros_msg_t;
        typedef rpos::core::SensorType      sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<slamware_ros_sdk::msg::ImpactType, rpos::features::impact_sensor::ImpactSensorType>
    {
    public:
        typedef slamware_ros_sdk::msg::ImpactType                                          ros_msg_t;
        typedef rpos::features::impact_sensor::ImpactSensorType     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    // TODO: wait to be uncommented and debug
    template<>
    struct MsgConvert<slamware_ros_sdk::msg::BasicSensorInfo, rpos::features::impact_sensor::ImpactSensorInfo>
    {
    public:
        typedef slamware_ros_sdk::msg::BasicSensorInfo                                     ros_msg_t;
        typedef rpos::features::impact_sensor::ImpactSensorInfo     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<slamware_ros_sdk::msg::ActionDirection, rpos::core::ACTION_DIRECTION>
    {
    public:
        typedef slamware_ros_sdk::msg::ActionDirection                     ros_msg_t;
        typedef rpos::core::ACTION_DIRECTION        sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<slamware_ros_sdk::msg::MoveOptionFlag, rpos::features::motion_planner::MoveOptionFlag>
    {
    public:
        typedef slamware_ros_sdk::msg::MoveOptionFlag                                  ros_msg_t;
        typedef rpos::features::motion_planner::MoveOptionFlag  sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<slamware_ros_sdk::msg::MoveOptions, rpos::features::motion_planner::MoveOptions>
    {
    public:
        typedef slamware_ros_sdk::msg::MoveOptions                                     ros_msg_t;
        typedef rpos::features::motion_planner::MoveOptions     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<geometry_msgs::msg::Point, rpos::core::Location>
    {
    public:
        typedef geometry_msgs::msg::Point                ros_msg_t;
        typedef rpos::core::Location                sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<tf2::Quaternion, rpos::core::Rotation>
    {
    public:
        typedef tf2::Quaternion           ros_msg_t;
        typedef rpos::core::Rotation                sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<geometry_msgs::msg::Quaternion, rpos::core::Rotation>
    {
    public:
        typedef geometry_msgs::msg::Quaternion           ros_msg_t;
        typedef rpos::core::Rotation                sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    // TODO: wait to be uncommented and debug
    template<>
    struct MsgConvert<geometry_msgs::msg::Pose, rpos::core::Pose>
    {
    public:
        typedef geometry_msgs::msg::Pose                 ros_msg_t;
        typedef rpos::core::Pose                    sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<slamware_ros_sdk::msg::Vec2DInt32, rpos::core::Vector2i>
    {
    public:
        typedef slamware_ros_sdk::msg::Vec2DInt32                        ros_msg_t;
        typedef rpos::core::Vector2i              sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    // Vec2DFlt32 <<======>> rpos::core::Vector2f
    template<>
    struct MsgConvert<slamware_ros_sdk::msg::Vec2DFlt32, rpos::core::Vector2f>
    {
    public:
        typedef slamware_ros_sdk::msg::Vec2DFlt32                        ros_msg_t;
        typedef rpos::core::Vector2f              sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };
    // Vec2DFlt32 <<======>> rpos::core::Point
    template<>
    struct MsgConvert<slamware_ros_sdk::msg::Vec2DFlt32, rpos::core::Point>
    {
    public:
        typedef slamware_ros_sdk::msg::Vec2DFlt32                        ros_msg_t;
        typedef rpos::core::Point                 sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<slamware_ros_sdk::msg::Line2DFlt32, rpos::core::Line>
    {
    public:
        typedef slamware_ros_sdk::msg::Line2DFlt32                       ros_msg_t;
        typedef rpos::core::Line                  sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<slamware_ros_sdk::msg::RectInt32, rpos::core::RectangleI>
    {
    public:
        typedef slamware_ros_sdk::msg::RectInt32                           ros_msg_t;
        typedef rpos::core::RectangleI              sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<slamware_ros_sdk::msg::RectFlt32, rpos::core::RectangleF>
    {
    public:
        typedef slamware_ros_sdk::msg::RectFlt32                           ros_msg_t;
        typedef rpos::core::RectangleF              sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<slamware_ros_sdk::msg::LocalizationMovement, rpos::features::motion_planner::RecoverLocalizationMovement>
    {
    public:
        typedef slamware_ros_sdk::msg::LocalizationMovement                                            ros_msg_t;
        typedef rpos::features::motion_planner::RecoverLocalizationMovement     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    template<>
    struct MsgConvert<slamware_ros_sdk::msg::LocalizationOptions, rpos::features::motion_planner::RecoverLocalizationOptions>
    {
    public:
        typedef slamware_ros_sdk::msg::LocalizationOptions                                            ros_msg_t;
        typedef rpos::features::motion_planner::RecoverLocalizationOptions     sltc_val_t;
    public:
        static void toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg);
        static void toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal);
    };

    //////////////////////////////////////////////////////////////////////////

    template<class RosMsgT, class SltcValT, class RosVecAllocT, class SltcVecAllocT>
    struct MsgConvert< std::vector<RosMsgT, RosVecAllocT> , std::vector<SltcValT, SltcVecAllocT> >
    {
    public:
        typedef std::vector<RosMsgT, RosVecAllocT>              ros_msg_t;
        typedef std::vector<SltcValT, SltcVecAllocT>            sltc_val_t;

    public:
        static void toRos(const sltc_val_t& vSltcVal, ros_msg_t& vRosMsg)
        {
            const size_t szCnt = vSltcVal.size();
            vRosMsg.resize(szCnt);
            for (size_t t = 0; t < szCnt; ++t)
            {
                sltcToRosMsg(vSltcVal[t], vRosMsg[t]);
            }
        }

        static void toSltc(const ros_msg_t& vRosMsg, sltc_val_t& vSltcVal)
        {
            const size_t szCnt = vRosMsg.size();
            vSltcVal.resize(szCnt);
            for (size_t t = 0; t < szCnt; ++t)
            {
                rosMsgToSltc(vRosMsg[t], vSltcVal[t]);
            }
        }
    };

    template<class RosKeyT, class SltcKeyT, class RosMsgT, class SltcValT, class RosCmpT, class SltcCmpT, class RosAllocT, class SltcAllocT>
    struct MsgConvert< std::map<RosKeyT, RosMsgT, RosCmpT, RosAllocT>, std::map<SltcKeyT, SltcValT, SltcCmpT, SltcAllocT> >
    {
    public:
        typedef std::map<RosKeyT, RosMsgT, RosCmpT, RosAllocT>              ros_msg_t;
        typedef std::map<SltcKeyT, SltcValT, SltcCmpT, SltcAllocT>          sltc_val_t;

    public:
        static void toRos(const sltc_val_t& mapSltcVal, ros_msg_t& mapRosMsg)
        {
            mapRosMsg.clear();
            for (auto cit = mapSltcVal.cbegin(), citEnd = mapSltcVal.cend(); citEnd != cit; ++cit)
            {
                sltcToRosMsg(cit->second, mapRosMsg[cit->first]);
            }
        }

        static void toSltc(const ros_msg_t& mapRosMsg, sltc_val_t& mapSltcVal)
        {
            mapSltcVal.clear();
            for (auto cit = mapRosMsg.cbegin(), citEnd = mapRosMsg.cend(); citEnd != cit; ++cit)
            {
                rosMsgToSltc(cit->second, mapSltcVal[cit->first]);
            }
        }
    };

    //////////////////////////////////////////////////////////////////////////
    
}
