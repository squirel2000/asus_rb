cmake_minimum_required(VERSION 3.8)
project(follow_user)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)

# Find packages for generating interfaces (actions, services, messages)
find_package(rosidl_default_generators REQUIRED)

# --- Handle Action/Service/Message generation ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowUser.action"
  DEPENDENCIES action_msgs geometry_msgs
)


# --- Python Configuration ---
# This installs the Python modules located in the "follow_user" directory
# ament_python_install_package(${PROJECT_NAME})


# This installs the Python scripts located in the "scripts" directory as executables
install(PROGRAMS
  scripts/follow_user_motion_node.py
  scripts/follow_user_vision_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Launch File Installation ---
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# --- Finalize the package ---
ament_package()