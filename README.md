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

To launch the project, use the `follow_user_script.py` script.

### Arguments

*   `-s`, `--sim`: Launch the simulation environment.
*   `-t`, `--task-coordinator`: Launch the task coordinator.
*   `--headless`: Launch Gazebo in headless mode.
*   `-d`, `--debug`: Print commands before execution.

### Examples

*   To launch just the simulation:
    ```bash
    python3 follow_user_script.py -s
    ```
*   To launch the simulation in headless mode:
    ```bash
    python3 follow_user_script.py -s --headless
    ```
*   To launch just the task coordinator:
    ```bash
    python3 follow_user_script.py -t
    ```
*   To launch both:
    ```bash
    python3 follow_user_script.py -s -t
    ```

