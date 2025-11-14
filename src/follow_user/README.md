# Follow-User Package

This package enables a robot to follow a path generated from its current pose to a target pose specified by clicking on the RViz interface. The target pose simulates a moving person that the robot will follow.

## Execution

There are two ways to launch the follow-user functionality: click the point on RViz manually or generate a path automatically.

### Building the Packages

Before launching, ensure all necessary packages are built:

```bash
# Build all required packages and source the setup file
colcon build --packages-select follow_user task_coordinator motion_common && source install/setup.bash
```

### Launching the System

The client script handles launching the Slamware driver, RViz, and the chosen follow-user architecture.

__1. Launching the Original Pure Pursuit Architecture (Default)__

To launch the original system that uses the `pure_pursuit_controller`, run the client script without any special flags:

```bash
./src/follow_user/follow_user_client.py
```
__2. Launching the New 3-Node Architecture__

To launch the refactored 3-node system, use the `-n` or `--new-arch` flag:

```bash
# Three main nodes:
# - `follow_user_vision_node.py`: Simulates user detection by taking a `/clicked_point` and publishing the user's pose relative to the robot.
# - `task_coordinator_node.py`: Manages the overall follow-user task, acting as a bridge between the vision and motion nodes.
# - `follow_user_motion_node.py`: Subscribes to the coordinator, controls the robot's head, calculates the absolute path to the user, and publishes the path for the robot to follow.

./src/follow_user/follow_user_client.py -n
```

The script will open new terminal windows for each of the following components:

1.  **Slamware ROS SDK**: Connects to the AMR.
    - `ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.xml ip_address:=192.168.11.1 port:=1448`
2.  **RViz**: Visualizes the robot and the environment.
    - `ros2 launch slamware_ros_sdk view_slamware_ros_sdk_server_node.xml`
3.  **Path Following and Search**: Launches the pure pursuit controller and the search path client.
    - `ros2 launch follow_user pure_pursuit.launch.py`

Click the "Publish Point" button in RViz to set a target point for the robot to follow. The robot will calculate a path to that point using slamware_sdk_server and then start following it. The script checks if a process with a similar name is already running to avoid launching duplicate nodes.

### Common Options

- __Debug Mode__: To keep the terminal tabs open after the nodes are closed, use the `-d` or `--debug` flag. This can be combined with either architecture.

  ```bash
  # Debug the new architecture
  ./src/follow_user/follow_user_client.py -n -d
  ```

- __Person Simulator__: To automatically simulate a person's movement, use the `-s` or `--simulate-person` flag. 
This will launch an additional terminal for the `person_simulator.py` script. The simulator reads a predefined path from `src/follow_user/test/path.json` and publishes a series of `geometry_msgs/PointStamped` messages to the `/clicked_point` topic, simulating a moving person. This is useful for testing and can be combined with either architecture.
  ```bash
  # Simulate a person with the new architecture
  ./src/follow_user/follow_user_client.py -n -s
  ```

## Vision Services

Launching all vision services 

```bash
bash ~/SW2/VisionService/launch_vision_all_tmux_terminal.sh
```

Re-launch all vision services by disabling and re-enabling the following service:

```bash
ros2 service call /enable_CV vision_srv/srv/SetTask “{task: follow realsense2 realsense, active: True}”
```

### Commit to GitHub with My Account

```bash
git -c user.name="TingYing Wu" -c user.email="tingying.wu@gmail.com" commit -m "Your commit message"
```

## Code Flow

Here is a high-level overview of the code and data flow for the follow_user functionality:

1. **Initiation (`follow_user_client.py`)**
    - This is the main entry point script.
    - It can be run with a `-s` or `--simulate-person` flag to launch the `person_simulator.py` for automated testing.
    - If the simulation flag is not used, the system expects manual goal setting via RViz.

2. **Goal Publishing (`person_simulator.py` or RViz)**
    - **`person_simulator.py`**:
        - Reads a predefined path from `src/follow_user/test/path.json`.
        - Publishes a series of `geometry_msgs/PointStamped` messages to the `/clicked_point` topic, simulating a moving person.
    - **RViz (Manual)**:
        - A user can click on the "Publish Point" button in RViz to publish a single `geometry_msgs/PointStamped` message to the `/clicked_point` topic.

3. **Path Planning (`path_search_server.py`)**
    - **Subscribes to**: `/clicked_point` (`geometry_msgs/PointStamped`).
    - Upon receiving a point, it calculates a path from the robot's current location to the received point.
    - **Publishes**: A `nav_msgs/Path` message to the `/follow_user/planned_path` topic.

4. **Path Following (`pure_pursuit_controller.cpp`)**
    - **Subscribes to**:
        - `/follow_user/planned_path` (`nav_msgs/Path`): To get the path to follow.
        - `/odom` (`nav_msgs/Odometry`): For the robot's current pose and velocity.
        - `/tf`: For coordinate frame transformations (e.g., `map` to `base_link`).
    - It calculates the required linear and angular velocities to follow the path using the pure pursuit algorithm.
    - **Publishes**: `geometry_msgs/Twist` messages to the `/cmd_vel` topic.

5. **Robot Control (`slamware_ros_sdk_server_node`)**
    - This is the low-level driver for the robot hardware.
    - **Subscribes to**: `/cmd_vel` (`geometry_msgs/Twist`) to receive velocity commands.
    - It translates the velocity commands into motor commands for the robot.
    - **Publishes**:
        - `/odom` (`nav_msgs/Odometry`): The robot's estimated position and velocity.
        - `/map` (`nav_msgs/OccupancyGrid`): The map of the environment.
        - `/tf`: Transformations between different coordinate frames (e.g., `odom` to `base_link`).
  