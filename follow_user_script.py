#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
import time
from termcolor import colored

def formatExeCommand(args):
    """
    Formats the execution commands based on the provided arguments.
    """
    # Keep the terminal open after the command has been executed
    exec_bash = '; exec bash' if args.debug else ''

    # Path to the RViz configuration file
    rviz_config = os.path.join(os.path.dirname(__file__), 'src/follow_user/rviz/follow_user.rviz')

    if args.debug:
        print(colored(f"RViz configuration file: {rviz_config}", "yellow"))

    return {
        'follow_user': f'gnome-terminal --tab --title="FollowUser" -- /bin/bash -c "ros2 launch follow_user follow_user.launch.py{exec_bash}"',
        'gazebo': f'gnome-terminal --tab --title="Gazebo" -- /bin/bash -c "ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py{exec_bash}"',
        'rviz': f'gnome-terminal --tab --title="RViz" -- /bin/bash -c "rviz2 -d {rviz_config}{exec_bash}"',
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
    
    # Always launch the main follow_user_bot node
    execCommandLine([commands['follow_user']], sleep_time=1.0, args=args)

    if args.gazebo:
        execCommandLine([commands['gazebo']], sleep_time=1.0, args=args)
    
    if args.rviz:
        execCommandLine([commands['rviz']], args=args)

def helper():
    """
    Sets up and parses command-line arguments.
    """
    parser = argparse.ArgumentParser(description='Launch script for the follow_user project.')
    parser.add_argument('-g', '--gazebo', action='store_true', help='Launch Gazebo with the turtlebot3_house world.')
    parser.add_argument('-r', '--rviz', action='store_true', help='Launch RViz with the follow_user configuration.')
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
