#!/usr/bin/env python3
import argparse
import subprocess
import time

def format_command(title, cmd, debug):
    """
    Formats a command to be executed in a new gnome-terminal tab.
    """
    exec_bash = '; exec bash' if debug else ''
    return f'gnome-terminal --tab --title="{title}" -- /bin/bash -c "{cmd}{exec_bash}"'

def parseCommandLine(args):
    commands_to_run = []
    if args.sim:
        sim_launch_cmd = 'ros2 launch amr_sim amr_sim.launch.py'
        if args.headless:
            sim_launch_cmd += ' headless:=true'
        commands_to_run.append(format_command("AMR Simulation", sim_launch_cmd, args.debug))
        
    if args.task_coordinator or args.mock:
        task_coordinator_launch_cmd = 'ros2 launch task_coordinator task_coordinator.launch.py'
        commands_to_run.append(format_command("Task Coordinator", task_coordinator_launch_cmd, args.debug))

    if args.mock:
        perception_manager_cmd = 'ros2 run perception_control_manager perception_control_manager_node.py'
        commands_to_run.append(format_command("Perception Manager", perception_manager_cmd, args.debug))
        
        mock_http_server_cmd = 'ros2 run perception_control_manager perception_control_manager_mock_api_server.py'
        commands_to_run.append(format_command("Mock HTTP API Server", mock_http_server_cmd, args.debug))

    if args.client:
        client_cmd = 'ros2 run task_coordinator task_client.py'
        commands_to_run.append(format_command("Task Client", client_cmd, args.debug))

    if args.test:
        test_navigation_cmd = 'ros2 run navigation test_navigation.py'
        commands_to_run.append(format_command("Test Navigation", test_navigation_cmd, args.debug))
        
    return commands_to_run

def execute_commands(commands_list, debug_mode, sleep_time=1.0):
    """
    Executes a list of commands in separate terminals.
    """
    if debug_mode:
        print("\nCommands to be executed:")
        for cmd in commands_list:
            print(f"  {cmd}")

    for command in commands_list:
        subprocess.Popen(command, shell=True)
        time.sleep(sleep_time)

def helper():
    """
    Sets up and parses command-line arguments.
    """
    parser = argparse.ArgumentParser(description='Launch script for the AMR project.')
    parser.add_argument('-s', '--sim', action='store_true', help='Launch the simulation (Gazebo) environment.')
    parser.add_argument('-t', '--task-coordinator', action='store_true', help='Launch the task coordinator.')
    parser.add_argument('-m', '--mock', action='store_true', help='Launch mock HTTP API server and perception manager.')
    parser.add_argument('-c', '--client', action='store_true', help='Launch the task client.')
    parser.add_argument('--test', action='store_true', help='Launch navigation test script.')
    parser.add_argument('--headless', action='store_true', help='Launch Gazebo in headless mode.')
    parser.add_argument('-d', '--debug', action='store_true', help="Keep terminals open and print commands.")
    return parser.parse_args()

def main():
    """
    Main entry point for the script.
    """
    args = helper()
    commands_to_run = parseCommandLine(args)
    
    if not commands_to_run:
        print("No launch options selected. Use -h for help.")
    else:
        execute_commands(commands_to_run, args.debug)
        print("All selected components have been launched in new terminals.")

if __name__ == '__main__':
    main()