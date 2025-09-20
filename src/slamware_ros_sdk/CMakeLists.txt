cmake_minimum_required(VERSION 3.8)
project(slamware_ros_sdk)

# 設置 C/C++ 標準
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 尋找 Slamtec SDK 路徑
find_path(slamware_sdk_INCLUDE_DIR rpos/rpos.h ${PROJECT_SOURCE_DIR}/../slamware_sdk/include)
find_path(slamware_sdk_LIBRARY librpos_framework.a ${PROJECT_SOURCE_DIR}/../slamware_sdk/lib)
if(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
  set(SLTC_SDK_INC_DIR "${slamware_sdk_INCLUDE_DIR}")
  set(SLTC_SDK_LIB_DIR "${slamware_sdk_LIBRARY}")
else()
  message(FATAL_ERROR "Slamtec SDK not found. Ensure ../slamware_sdk exists.")
endif()

# 尋找依賴
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Threads REQUIRED)

# 生成消息和服務
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/OptionalBool.msg
  msg/OptionalInt8.msg
  msg/OptionalInt16.msg
  msg/OptionalInt32.msg
  msg/OptionalInt64.msg
  msg/OptionalUInt8.msg
  msg/OptionalUInt16.msg
  msg/OptionalUInt32.msg
  msg/OptionalUInt64.msg
  msg/OptionalFlt32.msg
  msg/OptionalFlt64.msg
  msg/Vec2DInt32.msg
  msg/Vec2DFlt32.msg
  msg/Line2DFlt32.msg
  msg/Line2DFlt32Array.msg
  msg/RectInt32.msg
  msg/RectFlt32.msg
  msg/RobotDeviceInfo.msg
  msg/MapKind.msg
  msg/ArtifactUsage.msg
  msg/SensorType.msg
  msg/ImpactType.msg
  msg/BasicSensorInfo.msg
  msg/BasicSensorInfoArray.msg
  msg/BasicSensorValue.msg
  msg/BasicSensorValueData.msg
  msg/BasicSensorValueDataArray.msg
  msg/ActionDirection.msg
  msg/RobotBasicState.msg
  msg/SyncMapRequest.msg
  msg/MoveOptionFlag.msg
  msg/MoveOptions.msg
  msg/MoveByDirectionRequest.msg
  msg/MoveByThetaRequest.msg
  msg/MoveToRequest.msg
  msg/MoveToLocationsRequest.msg
  msg/RotateToRequest.msg
  msg/RotateRequest.msg
  msg/LocalizationMovement.msg
  msg/OptionalLocalizationMovement.msg
  msg/LocalizationOptions.msg
  msg/RecoverLocalizationRequest.msg
  msg/ClearMapRequest.msg
  msg/SetMapUpdateRequest.msg
  msg/SetMapLocalizationRequest.msg
  msg/GoHomeRequest.msg
  msg/CancelActionRequest.msg
  msg/AddLineRequest.msg
  msg/AddLinesRequest.msg
  msg/RemoveLineRequest.msg
  msg/ClearLinesRequest.msg
  msg/MoveLineRequest.msg
  msg/MoveLinesRequest.msg
  srv/SyncGetStcm.srv
  srv/SyncSetStcm.srv
  DEPENDENCIES nav_msgs sensor_msgs std_msgs geometry_msgs tf2_msgs
)

# 包含頭文件路徑
include_directories(
  include
  ${SLTC_SDK_INC_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# 定義客戶端庫
add_library(slamware_ros_sdk_client STATIC
  src/client/slamware_ros_sdk_client.cpp
)
ament_target_dependencies(slamware_ros_sdk_client rclcpp std_msgs tf2_ros)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(slamware_ros_sdk_client ${cpp_typesupport_target})

# 定義服務器節點
add_executable(slamware_ros_sdk_server_node
  src/server/msg_convert.cpp
  src/server/server_params.cpp
  src/server/server_map_holder.cpp
  src/server/server_work_data.cpp
  src/server/server_worker_base.cpp
  src/server/server_workers.cpp
  src/server/slamware_ros_sdk_server_node.cpp
  src/server/slamware_ros_sdk_server.cpp
)
ament_target_dependencies(slamware_ros_sdk_server_node rclcpp std_msgs std_srvs tf2_ros tf2_geometry_msgs)
rosidl_get_typesupport_target(cpp_server_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(slamware_ros_sdk_server_node ${cpp_server_typesupport_target})

target_include_directories(slamware_ros_sdk_server_node
  PRIVATE ${SLTC_SDK_INC_DIR}
)
target_compile_options(slamware_ros_sdk_server_node
  PRIVATE -Wno-deprecated-declarations
)

target_link_libraries(slamware_ros_sdk_server_node 
  ${SLTC_SDK_LIB_DIR}/librpos_robotplatforms_rpslamware.a
  ${SLTC_SDK_LIB_DIR}/librpos_framework.a
  ${SLTC_SDK_LIB_DIR}/libbase64.a
  ${SLTC_SDK_LIB_DIR}/librlelib.a
  ${SLTC_SDK_LIB_DIR}/libjsoncpp.a
  ${SLTC_SDK_LIB_DIR}/libcurl.a
  ${SLTC_SDK_LIB_DIR}/libcares.a
  ${SLTC_SDK_LIB_DIR}/libssl.a
  ${SLTC_SDK_LIB_DIR}/libcrypto.a
  ${SLTC_SDK_LIB_DIR}/libboost_atomic.a
  ${SLTC_SDK_LIB_DIR}/libboost_chrono.a
  ${SLTC_SDK_LIB_DIR}/libboost_date_time.a
  ${SLTC_SDK_LIB_DIR}/libboost_regex.a 
  ${SLTC_SDK_LIB_DIR}/libboost_filesystem.a
  ${SLTC_SDK_LIB_DIR}/libboost_system.a
  ${SLTC_SDK_LIB_DIR}/libboost_thread.a
  ${SLTC_SDK_LIB_DIR}/libboost_random.a
  ${SLTC_SDK_LIB_DIR}/libz.a
  Threads::Threads
  dl
  rt
)

# 安裝目標
install(TARGETS slamware_ros_sdk_server_node slamware_ros_sdk_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安裝頭文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
ament_export_include_directories(include/${PROJECT_NAME})

# 安裝啟動文件和 RViz 配置
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# 測試
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()