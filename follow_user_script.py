#!/usr/bin/env python3
import argparse
import subprocess

def main():
    """
    Main entry point for the script.
    """
    parser = argparse.ArgumentParser(description='Launch script for the AMR project.')
    parser.add_argument('-s', '--sim', action='store_true', help='Launch the simulation environment.')
    parser.add_argument('-t', '--task_coordinator', action='store_true', help='Launch the task coordinator.')
    parser.add_argument('--headless', action='store_true', help='Launch Gazebo in headless mode.')
    parser.add_argument('-d', '--debug', action='store_true', help="Print commands before execution")
    args = parser.parse_args()

    commands = []

    if args.sim:
        sim_command = [
            'ros2',
            'launch',
            'amr_sim',
            'amr_sim.launch.py',
        ]
        if args.headless:
            sim_command.append('headless:=true')
        commands.append(sim_command)

    if args.task_coordinator:
        task_coordinator_command = [
            'ros2',
            'launch',
            'task_coordinator',
            'task_coordinator.launch.py'
        ]
        commands.append(task_coordinator_command)

    for command in commands:
        if args.debug:
            print(f"Executing command: {' '.join(command)}")
        subprocess.Popen(command)

    # Keep the script running until interrupted
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\nShutting down.")

if __name__ == '__main__':
    main()