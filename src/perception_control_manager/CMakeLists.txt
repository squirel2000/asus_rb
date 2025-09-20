cmake_minimum_required(VERSION 3.8)
project(perception_control_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(slamware_ros_sdk REQUIRED)
find_package(rclpy REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetActionStatus.srv"
  "srv/CreateMoveTo.srv"
  "srv/SetMaxSpeed.srv"
  DEPENDENCIES slamware_ros_sdk
)

install(PROGRAMS
  scripts/perception_control_manager_node.py
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  scripts/utils
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
