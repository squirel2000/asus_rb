#!/usr/bin/env python3
import argparse
import os
import subprocess
import time
from termcolor import colored

def formatExeCommand(args):
    """
    Formats the execution commands based on the provided arguments.
    """
    # Keep the terminal open after the command has been executed
    exec_bash = '; exec bash' if args.debug else ''

    return {
        'mock_server': f'gnome-terminal --tab --title="Mock API Server" -- /bin/bash -c "python3 mock_api_server.py{exec_bash}"',
        'perception_manager': f'gnome-terminal --tab --title="Perception Manager" -- /bin/bash -c "ros2 run perception_control_manager perception_control_manager_node.py --ros-args -p robot_port:={args.port}{exec_bash}"',
        'navigation_motion': f'gnome-terminal --tab --title="Navigation Motion" -- /bin/bash -c "ros2 run navigation navigation_motion_node.py{exec_bash}"',
        'test_navigation': f'gnome-terminal --tab --title="Test Navigation" -- /bin/bash -c "ros2 run navigation test_navigation.py{exec_bash}"',
    }

def execCommandLine(commands=[], sleep_time: float = 0.5, args=None):
    """
    Executes a list of commands in the shell.
    """
    if args.debug:
        print("\nCommands:")
    for command in commands:
        if args.debug:
            print(colored(f"\t{command}", 'green'))
        subprocess.run(command, shell=True, check=True)
        time.sleep(sleep_time)

def parseCommandLine(args):
    """
    Parses the command line arguments and executes the corresponding commands.
    """
    if args.debug:
        print('args: ', args)

    commands = formatExeCommand(args)
    
    # Launch all the components
    execCommandLine([commands['mock_server']], sleep_time=2.0, args=args)
    execCommandLine([commands['perception_manager']], sleep_time=2.0, args=args)
    execCommandLine([commands['navigation_motion']], sleep_time=2.0, args=args)
    if args.test:
        execCommandLine([commands['test_navigation']], args=args)


def helper():
    """
    Sets up and parses command-line arguments.
    """
    parser = argparse.ArgumentParser(description='Launch script for the mock navigation test.')
    parser.add_argument('-p', '--port', type=int, default=1448, help='Port for the mock API server.')
    parser.add_argument('-t', '--test', action='store_true', help='Launch the test_navigation.py script.')
    parser.add_argument('-d', '--debug', action='store_true', help="add 'exec bash' for debug and print commands")
    args = parser.parse_args()
    parseCommandLine(args)

def main():
    """
    Main entry point for the script.
    """
    helper()

if __name__ == '__main__':
    main()
