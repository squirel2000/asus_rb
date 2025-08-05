# asus_rb
The repository is mainly created for Robot BU's AMR project


## Install ROS 2 on Remote PC

Install ROS 2 Humble and its dependencies [here] (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) and [TurtleBot3] (https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) Packages

## How to Launch

To launch the project, use the `amr_client.py` script. This script is used to launch the various nodes of the system.

### Arguments

*   `-s`, `--sim`: Launch the simulation environment.
*   `-t`, `--task-coordinator`: Launch the task coordinator.
*   `-m`, `--mock`: Launch mock HTTP API server and perception manager.
*   `--headless`: Launch Gazebo in headless mode.
*   `-d`, `--debug`: Print commands before execution.

### Examples

*   To launch the simulation and the task coordinator:
    ```bash
    python3 amr_client.py -s -t
    ```
*   To launch the entire system with the mock server:
    ```bash
    python3 amr_client.py -t -m
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

### Follow User Task

*   **Command:** `follow`
*   **Arguments:**
    *   `user_id`: The ID of the user to follow.
*   **Example:**
    ```bash
    ros2 run task_coordinator task_client.py follow user_123
    ```


