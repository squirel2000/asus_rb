# Task Coordinator

This package contains the `task_coordinator_node`, which is responsible for coordinating between different tasks, such as following a user and navigating to a specific pose.

## Nodes

### `task_coordinator_node`

The `task_coordinator_node` is the central node in this package. It provides an action server for navigation tasks and another for follow-user tasks. It also provides a service to switch between navigation and follow-user modes.

#### Actions

*   **`/task_server`** ([`task_coordinator/action/Task`](action/Task.action))

    An action server for navigation tasks. The goal is a `PoseStamped` message.

*   **`/follow_user_task`** ([`follow_user/action/FollowUser`](../follow_user/action/FollowUser.action))

    An action server for follow-user tasks. The goal is a `user_id`.

## Usage

To run the `task_coordinator_node`, use the following command:

```bash
ros2 launch task_coordinator task_coordinator.launch.py
```

### Example Client

This package includes an example client script, `send_task_goal_example.py`, which can be used to send goals to the `task_coordinator_node`.

To send a navigation goal:

```bash
ros2 run task_coordinator send_task_goal_example.py navigate --x 1.0 --y 2.0 --w 1.0
```

To send a follow-user goal:

```bash
ros2 run task_coordinator send_task_goal_example.py follow --user_id user_1
```
