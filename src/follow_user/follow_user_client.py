#!/usr/bin/env python3
import subprocess
import time
import os
import argparse

def is_process_running(process_name):
    """
    Checks if a process with a given name is running.
    """
    try:
        subprocess.check_output(["pgrep", "-f", process_name])
        return True
    except subprocess.CalledProcessError:
        return False

def format_command(title, cmd, debug):
    """
    Formats a command to be executed in a new gnome-terminal tab.
    """
    exec_bash = '; exec bash' if debug else ''
    return f'gnome-terminal --tab --title="{title}" -- /bin/bash -c "{cmd}{exec_bash}"'

def main():
    """
    Main entry point for the script.
    """
    parser = argparse.ArgumentParser(description='Launch script for the follow_user project.')
    parser.add_argument('-d', '--debug', action='store_true', help="Keep terminals open and print commands.")
    parser.add_argument('-s', '--simulate-person', action='store_true', help="Launch the person simulator.")
    args = parser.parse_args()

    commands = {
        "slamware_ros_sdk_server_node.xml": 'ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.xml ip_address:=192.168.12.1 port:=1448',
        "view_slamware_ros_sdk_server_node.xml": 'ros2 launch slamware_ros_sdk view_slamware_ros_sdk_server_node.xml',
        "pure_pursuit.launch.py": 'ros2 launch follow_user pure_pursuit.launch.py'
    }
    if args.simulate_person:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        simulator_path = os.path.join(script_dir, 'test', 'person_simulator.py')
        commands["person_simulator.py"] = f'python3 {simulator_path}'

    for process_name, cmd in commands.items():
        if not is_process_running(process_name):
            print(f"Launching {process_name}...")
            formatted_cmd = format_command(process_name, cmd, args.debug)
            subprocess.Popen(formatted_cmd, shell=True)
            time.sleep(2)  # Give some time for the terminal to open
        else:
            print(f"{process_name} is already running. Skipping.")

if __name__ == '__main__':
    main()
