# Navigation Package

This package provides a ROS2 action server to navigate the robot to a specified pose using the Slamtec RESTful API.

## How to Build and Run
1. Navigate to your ROS2 workspace directory.

2. Build the package using colcon and source the workspace:
```bash
colcon build --packages-select navigation
source install/setup.bash
```

3. Open a terminal and run the navigation action server.
```bash
ros2 run navigation navigation_motion_node.py
```

4. In another terminal, run the test client to send a goal:
```bash
ros2 run navigation test_navigation.py
```