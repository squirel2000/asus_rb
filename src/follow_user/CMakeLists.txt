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
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(perception_control_manager REQUIRED)

# Find packages for generating interfaces (actions, services, messages)
find_package(rosidl_default_generators REQUIRED)


# # Add executable targets
# add_executable(pure_pursuit_controller src/pure_pursuit_controller.cpp)

# target_include_directories(pure_pursuit_controller PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# # Specify libraries to link a library or executable target against
# ament_target_dependencies(pure_pursuit_controller
#   rclcpp
#   nav_msgs
#   geometry_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   angles
# )

# # Install targets
# install(TARGETS
#   pure_pursuit_controller
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install headers
# install(DIRECTORY include/
#   DESTINATION include
# )

# # Export include directories
# ament_export_include_directories(include)



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
  scripts/path_search_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/utils/
  DESTINATION lib/${PROJECT_NAME}/utils
)

# --- Launch File Installation ---
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# --- Config File Installation ---
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# --- Finalize the package ---
ament_package()
