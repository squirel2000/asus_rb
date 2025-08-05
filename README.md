# asus_rb
The repository is mainly created for Robot BU's AMR project


## Install ROS 2 on Remote PC

Follow the official ROS 2 documentation [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to install ROS 2 Humble.

Install Dependent ROS 2 Packages
```bash
# Install Gazebo
sudo apt install -y ros-humble-gazebo-*

# Install Cartographer
#sudo apt install -y ros-humble-cartographer
#sudo apt install -y ros-humble-cartographer-ros

# Install Navigation2
sudo apt install -y ros-humble-navigation2
sudo apt install -y ros-humble-nav2-bringup
```

Install TurtleBot3 Packages
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/

git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
sudo apt install python3-colcon-common-extensions

cd ~/turtlebot3_ws
colcon build --symlink-install
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## How to Launch

To launch the project, use the `amr_client.py` script. This script is used to launch the various nodes of the system.

### Arguments

*   `-s`, `--sim`: Launch the simulation environment.
*   `-t`, `--task-coordinator`: Launch the task coordinator.
*   `-m`, `--mock`: Launch mock HTTP API server and perception manager.
*   `-c`, `--client`: Launch the task client.
*   `--test`: Launch navigation test script.
*   `--headless`: Launch Gazebo in headless mode.
*   `-d`, `--debug`: Print commands before execution.

### Examples

*   To launch the simulation and the task coordinator:
    ```bash
    python3 amr_client.py -s -t
    ```
*   To launch the entire system with the mock server and the task client:
    ```bash
    python3 amr_client.py -s -t -m -c
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

