#!/usr/bin/env python3
import subprocess
import time
import os
import argparse

from click import command

def is_process_running(process_name):
    """
    Checks if a process with a given name is running.
    """
    try:
        subprocess.check_output(["pgrep", "-f", process_name])
        return True
    except subprocess.CalledProcessError:
        return False

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
        # No route to Ethernet subnet, check for Wi-Fi
        pass

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

def main():
    """
    Main entry point for the script.
    """
    parser = argparse.ArgumentParser(description='Launch script for the follow_user project.')
    parser.add_argument('-d', '--debug', action='store_true', help="Keep terminals open and print commands.")
    parser.add_argument('-s', '--simulate-person', action='store_true', help="Launch the person simulator.")
    parser.add_argument('-n', '--new-arch', action='store_true', help="Launch the new 3-node architecture.")
    args = parser.parse_args()

    slamware_ip = get_slamware_ip()
    commands = {
        "slamware_ros_sdk_server_node.xml": f'ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.xml ip_address:={slamware_ip} port:=1448',
        "view_slamware_ros_sdk_server_node.xml": 'ros2 launch slamware_ros_sdk view_slamware_ros_sdk_server_node.xml',
    }

    if args.new_arch:
        print("Launching new 3-node architecture in separate terminals...")
        commands["follow_user_vision_node.py"] = 'ros2 run follow_user follow_user_vision_node.py'
        commands["follow_user_motion_node.py"] = f'ros2 launch follow_user follow_user.launch.py robot_ip:={slamware_ip}'
        commands["task_coordinator_node.py"] = 'ros2 run task_coordinator task_coordinator_node.py'
        commands["task_client.py"] = f'ros2 run task_coordinator task_client.py follow'
    else:
        print("Launching original pure pursuit architecture...")
        commands["pure_pursuit.launch.py"] = f'ros2 launch follow_user pure_pursuit.launch.py robot_ip:={slamware_ip}'
    
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
