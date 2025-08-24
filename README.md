# asus_rb
The repository is mainly created for Robot BU's AMR project


## Install ROS 2 on Remote PC

Install ROS 2 Humble and its dependencies [here] (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) and [TurtleBot3] (https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) Packages

```bash
sudo apt install ros-${ROS_DISTRO}-tf-transformations
pip install transforms3d
```

### 使用 ROS2 SDK控制思岚科技

1. Follow the [slamtec instruction](https://slamtec.feishu.cn/docx/IV5edQISqoqGuXx8wD3cTNA2nMg) to download, install, and build slamtec SDK [slamware_ros2_sdk_linux-x86_64-gcc11.tar.gz](https://www.slamtec.com/cn/Support#apollo)

2. Launch the slamware ROS2 SDK server node in a new terminal
    ```bash
    ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.xml ip_address:=192.168.12.1 port:=1448
    ```

3. Open the RViz2 to view the map and the AMR in a new terminal
    ```bash
    ros2 launch slamware_ros_sdk view_slamware_ros_sdk_server_node.xml
    ```

4. Example commands:
    ```bash
    # Go home
    ros2 topic pub --once /slamware_ros_sdk_server_node/go_home slamware_ros_sdk/msg/GoHomeRequest "{}"
    # Set the max. speed
    ros2 topic pub /slamware_ros_sdk_server_node/move_by_direction slamware_ros_sdk/msg/MoveByDirectionRequest "{direction: {direction: 0}, options: {speed_ratio: {is_valid: true, value: 0.5}}}"

    # Move to a target pose
    ros2 topic pub /slamware_ros_sdk_server_node/move_to slamware_ros_sdk/msg/MoveToRequest "{location: {x: 2.0, y: 0.0, z: 0.0}, yaw: 0.0}"
    ```

## How to Launch

To launch the project, use the `amr_client.py` script. This script is used to launch the various nodes of the system.

### Arguments

*   `-s`, `--sim`: Launch the simulation environment.
*   `--headless`: Launch Gazebo in headless mode.
*   `-m`, `--mock`: Launch mock HTTP API server and perception manager.
*   `-l`, `--low-level`: Launch the low-level, such as navigation and follow_user, packages.
*   `-t`, `--task-coordinator`: Launch the high-level task coordinator (a.k.a. controller).
*   `-d`, `--debug`: Print commands before execution.

### Examples

*   To launch the mock HTTP API server and perception manager, the low-level packages, and the task-coordinator:
    ```bash
    python3 amr_client.py -mlt
    ```

## How to Send Tasks

To send tasks to the AMR, use the `task_client.py` script. This script can be used to send navigation or follow-user goals.

### Navigation Task

*   **Command:** `navigate`
*   **Arguments:**
    *   `--x`: X position for the navigation goal (default: 1.0)
    *   `--y`: Y position for the navigation goal (default: 1.0)
    *   `--w`: W orientation for the navigation goal (default: 1.0)
*   **Example:**
    ```bash
    ros2 run task_coordinator task_client.py navigate --x 2.0 --y 3.0
    ```

To update the goal mid-task while the robot is still navigating to the first point, you can open a new terminal and publish a new PoseStamped message to the navigate_to_pose/update_goal topic.

*   **Example:**
    ```bash
    ros2 topic pub --once /navigate_to_pose/update_goal geometry_msgs/msg/PoseStamped '{
        header: {
            frame_id: "map"
        },
        pose: {
            position: {x: -2.0, y: -8.0, z: 0.0},
            orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
        }
    }'
    ```

### Follow User Task

*   **Command:** `follow`
*   **Arguments:**
    *   `user_id`: The ID of the user to follow.
*   **Example:**
    ```bash
    ros2 run task_coordinator task_client.py follow user_123
    ```
