cmake_minimum_required(VERSION 3.5)
project(slamware_ros_sdk)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_path(slamware_sdk_INCLUDE_DIR rpos/rpos.h ${PROJECT_SOURCE_DIR}/../slamware_sdk/include)
find_path(slamware_sdk_LIBRARY librpos_framework.a ${PROJECT_SOURCE_DIR}/../slamware_sdk/lib)

if(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
  set(SLTC_SDK_INC_DIR "${slamware_sdk_INCLUDE_DIR}")
  set(SLTC_SDK_LIB_DIR "${slamware_sdk_LIBRARY}")
else(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
  message( FATAL_ERROR "project dir not exit" )
endif(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
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

find_package(Threads) 

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
 DEPENDENCIES nav_msgs sensor_msgs std_msgs
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(message_runtime)
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)
include_directories(${AMENT_PREFIX_PATH})
include_directories(~/ros2_humble/ros2-linux/include)
include_directories($ENV{AMENT_PREFIX_PATH}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/build/slamware_ros_sdk/rosidl_generator_cpp)

add_library(${PROJECT_NAME}_client STATIC
  src/client/slamware_ros_sdk_client.cpp
)
ament_target_dependencies(${PROJECT_NAME}_client rclcpp std_msgs tf2_ros)
rosidl_target_interfaces(${PROJECT_NAME}_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

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
rosidl_target_interfaces(slamware_ros_sdk_server_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

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
  pthread
  dl
  rt
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME}_server_node 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
ament_export_include_directories(include/${PROJECT_NAME})

install(FILES
  launch/slamware_ros_sdk_server_node.xml
  launch/view_slamware_ros_sdk_server_node.xml
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY 
  rviz
	DESTINATION share/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
