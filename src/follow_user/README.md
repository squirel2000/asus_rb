# Follow User Package

This package enables a robot to follow a path generated from its current pose to a target pose specified by clicking on the RViz interface.

## Execution

To launch the necessary nodes for the follow user functionality, run the following command from the root of the workspace:

```bash
./src/follow_user/follow_user_client.py
or 
./src/follow_user/follow_user_client.py -d # or --debug
```

The script will open new terminal windows for each of the following components:

1.  **Slamware ROS SDK**: Connects to the AMR.
    - `ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.xml ip_address:=192.168.12.1 port:=1448`
2.  **RViz**: Visualizes the robot and the environment.
    - `ros2 launch slamware_ros_sdk view_slamware_ros_sdk_server_node.xml`
3.  **Path Following and Search**: Launches the pure pursuit controller and the search path client.
    - `ros2 launch follow_user pure_pursuit.launch.py`

The script checks if a process with a similar name is already running to avoid launching duplicate nodes.

## Person Simulation

This package also includes a person simulator for testing the follow-user functionality without manual clicking in RViz. The simulator publishes `/clicked_point` messages to simulate a moving person in front of the robot.

To run the simulation, use the `--simulate-person` flag with the client script:

```bash
./src/follow_user/follow_user_client.py -s # or --simulate-person
```

This will launch an additional terminal for the `person_simulator.py` script.
