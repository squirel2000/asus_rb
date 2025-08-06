# Navigation Package

This package provides a ROS2 action server to navigate the robot to a specified pose using the Slamtec RESTful API.

## How to Run

1.  Navigate to your ROS2 workspace directory, and build the package using colcon and source the workspace:

```bash
colcon build --packages-select navigation
source install/setup.bash
```

2.  Use the provided launch script to run all the components at once:

```bash
python3 launch_mock_test.py --test
```
