/**
 kint.zhao  huasheng_zyh@163.com
   2017.0721

  modified by yun.li@slamtec.com, 2019.
*/

#include "rclcpp/rclcpp.hpp"

#include  "slamware_ros_sdk_server.h"

int main(int argc, char** argv)
{
  std::string errMsg;
  rclcpp::init(argc, argv);
  
  {
    slamware_ros_sdk::SlamwareRosSdkServer rosSdkServer;
    if (!rosSdkServer.startRun(errMsg))
    {
      RCLCPP_ERROR(rclcpp::get_logger("server node"), "failed to start slamware ros sdk server: %s.", errMsg.c_str());
      return -1;
    }

    rclcpp::spin(rosSdkServer.getRosNodeHandle());
    RCLCPP_INFO(rclcpp::get_logger("server node"), "node spin over.");

    rosSdkServer.requestStop();
    rosSdkServer.waitUntilStopped();
  }
  return 0;
}
