#!/usr/bin/env python3
import argparse
import subprocess
import time

def get_slamware_ip():
    """
    Detects the active network interface and returns the corresponding IP for Slamware.
    Returns '192.168.12.1' for Wi-Fi and '192.168.11.1' for Ethernet.
    Defaults to the Wi-Fi IP if detection fails.
    """
    wifi_ip = '192.168.12.1'
    ethernet_ip = '192.168.11.1'
    
    # Prioritize Ethernet if available, as it's often more stable for robotics
    try:
        # Check for a route to the Ethernet subnet
        result = subprocess.check_output("ip route get 192.168.11.1", shell=True, text=True, stderr=subprocess.DEVNULL)
        if '192.168.11.1' in result:
            interface = result.split('dev')[1].split()[0]
            print(f"Ethernet connection for Slamware detected on {interface}. Using IP: {ethernet_ip}")
            return ethernet_ip
    except subprocess.CalledProcessError:
        pass # No route to Ethernet subnet, check for Wi-Fi

    try:
        # Check for a route to the Wi-Fi subnet
        result = subprocess.check_output("ip route get 192.168.12.1", shell=True, text=True, stderr=subprocess.DEVNULL)
        if '192.168.12.1' in result:
            interface = result.split('dev')[1].split()[0]
            print(f"Wi-Fi connection for Slamware detected on {interface}. Using IP: {wifi_ip}")
            return wifi_ip
    except subprocess.CalledProcessError:
        print(f"Could not determine network interface for Slamware. Defaulting to Wi-Fi IP: {wifi_ip}")
    return wifi_ip

def format_command(title, cmd, debug):
    """
    Formats a command to be executed in a new gnome-terminal tab.
    """
    exec_bash = '; exec bash' if debug else ''
    return f'gnome-terminal --tab --title="{title}" -- /bin/bash -c "{cmd}{exec_bash}"'

def parseCommandLine(args, robot_ip):
    """
    Parse bottom-up command-line arguments and returns a list of commands to be executed.
    """
    commands_to_run = []
    if args.sim:
        sim_launch_cmd = 'ros2 launch amr_sim amr_sim.launch.py'
        if args.headless:
            sim_launch_cmd += ' headless:=true'
        commands_to_run.append(format_command("AMR Simulation", sim_launch_cmd, args.debug))

    if args.nav2:
        nav2_launch_cmd = 'ros2 launch amr_sim navigation2.launch.py use_sim_time:=True map:=./src/amr_sim/map/map.yaml'
        commands_to_run.append(format_command("Navigation2", nav2_launch_cmd, args.debug))
    
    if args.mock:
        # Mock HTTP API Server
        mock_http_server_cmd = 'ros2 run perception_control_manager perception_control_manager_mock_api_server.py'
        commands_to_run.append(format_command("Mock HTTP API Server", mock_http_server_cmd, args.debug))
        
        perception_manager_cmd = f'ros2 run perception_control_manager perception_control_manager_node.py --ros-args -p robot_ip:={robot_ip}'
        commands_to_run.append(format_command("Perception Manager", perception_manager_cmd, args.debug))
            
    if args.low_level:
        low_level_launch_cmd = 'ros2 launch task_coordinator low_level_packages.launch.py'
        commands_to_run.append(format_command("Low Level Packages", low_level_launch_cmd, args.debug))

    if args.task_coordinator or args.mock:
        task_coordinator_launch_cmd = 'ros2 launch task_coordinator task_coordinator.launch.py'
        commands_to_run.append(format_command("Task Coordinator", task_coordinator_launch_cmd, args.debug))

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
    parser.add_argument('-s', '--sim', action='store_true', help='Launch the Gazebo simulation environment.')
    parser.add_argument('-n', '--nav2', action='store_true', help='Launch the Navigation2 stack.')
    parser.add_argument('--headless', action='store_true', help='Launch Gazebo in headless mode.')
    parser.add_argument('-m', '--mock', action='store_true', help='Launch mock HTTP API server and perception manager.')
    parser.add_argument('-l', '--low-level', action='store_true', help='Launch the low-level (e.g., navigation, follow-user) packages.')
    parser.add_argument('-t', '--task-coordinator', action='store_true', help='Launch the task coordinator.')
    parser.add_argument('-d', '--debug', action='store_true', help="Keep terminals open and print commands.")
    return parser.parse_args()

def main():
    """
    Main entry point for the script.
    """
    args = helper()
    robot_ip = get_slamware_ip()
    commands_to_run = parseCommandLine(args, robot_ip)
    
    if not commands_to_run:
        print("No launch options selected. Use -h for help.")
    else:
        execute_commands(commands_to_run, args.debug)
        print("All selected components have been launched in new terminals.")

if __name__ == '__main__':
    main()