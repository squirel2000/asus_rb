#!/usr/bin/env python3
import argparse
import subprocess
import time

def main():
    """
    Main entry point for the script.
    """
    parser = argparse.ArgumentParser(description='Launch script for the AMR project.')
    parser.add_argument('-s', '--sim', action='store_true', help='Launch the simulation environment.')
    parser.add_argument('-t', '--task_coordinator', action='store_true', help='Launch the task coordinator.')
    parser.add_argument('-m', '--mock', action='store_true', help='Launch mock HTTP API server and perception manager.')
    parser.add_argument('--test', action='store_true', help='Launch navigation test script (only with --mock).')
    parser.add_argument('--headless', action='store_true', help='Launch Gazebo in headless mode.')
    parser.add_argument('-d', '--debug', action='store_true', help="Keep terminals open and print commands.")
    args = parser.parse_args()

    # Dependency: If mock is enabled, task_coordinator must also be launched
    if args.mock:
        args.task_coordinator = True

    # Keep the terminal open after the command has been executed
    exec_bash = '; exec bash' if args.debug else ''

    commands_to_run = []

    if args.sim:
        sim_launch_cmd = 'ros2 launch amr_sim amr_sim.launch.py'
        if args.headless:
            sim_launch_cmd += ' headless:=true'
        sim_command = f'gnome-terminal --tab --title="AMR Simulation" -- /bin/bash -c "{sim_launch_cmd}{exec_bash}"'
        commands_to_run.append(sim_command)

    if args.mock:
        # Launch mock HTTP API server
        mock_http_server_command = f'gnome-terminal --tab --title="Mock HTTP API Server" -- /bin/bash -c "python3 mock_api_server.py{exec_bash}"'
        commands_to_run.append(mock_http_server_command)
        time.sleep(1.0) # Give time for the server to start

        # Launch perception manager
        perception_manager_command = f'gnome-terminal --tab --title="Perception Manager" -- /bin/bash -c "ros2 run perception_control_manager perception_control_manager_node.py{exec_bash}"'
        commands_to_run.append(perception_manager_command)
        time.sleep(1.0)

    if args.task_coordinator:
        task_coordinator_launch_cmd = 'ros2 launch task_coordinator task_coordinator.launch.py'
        task_coordinator_command = f'gnome-terminal --tab --title="Task Coordinator" -- /bin/bash -c "{task_coordinator_launch_cmd}{exec_bash}"'
        commands_to_run.append(task_coordinator_command)

    if args.test:
        test_navigation_command = f'gnome-terminal --tab --title="Test Navigation" -- /bin/bash -c "ros2 run navigation test_navigation.py{exec_bash}"'
        commands_to_run.append(test_navigation_command)
        time.sleep(1.0)

    if args.debug:
        print("\nCommands to be executed:")
        for cmd in commands_to_run:
            print(f"  {cmd}")

    for command in commands_to_run:
        subprocess.Popen(command, shell=True)
        time.sleep(1.0) # Give time for the new terminal to open

    if not commands_to_run:
        print("No launch options selected. Use -h for help.")
    else:
        print("\nAll selected components have been launched in new terminals.")

if __name__ == '__main__':
    main()