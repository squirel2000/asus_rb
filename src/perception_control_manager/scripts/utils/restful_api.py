#!/usr/bin/env python3
import requests
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

class RestfulAPI:
    def __init__(self, robot_ip, logger, port=1448):
        self.base_url = f"http://{robot_ip}:{port}/api/core"
        self.logger = logger

    def create_actions(self, payload):
        """Sends a POST request to create a action."""
        url = f"{self.base_url}/motion/v1/actions"
        try:
            response = requests.post(url, json=payload, timeout=5)
            response.raise_for_status()
            return response.json().get("action_id")
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Error creating action: {e}")
            return None

    def get_action_status(self, action_id=":current"):
        """Get the AMR current or specific id action status"""
        url = f"{self.base_url}/motion/v1/actions/{action_id}"
        try:
            response = requests.get(url, timeout=2)
            response.raise_for_status()
            
            return response.json()
        except requests.exceptions.RequestException as e:
            
            if e.response is not None and e.response.status_code == 404:
                # if no action is currently executed, will get 404 error.
                self.logger.debug(f"Error getting events: {e}")
                return {}
            else:
                self.logger.error(f"Error getting events: {e}")
                return None 


    def cancel_current_action(self):
        """Sends a DELETE request to cancel the current action."""
        url = f"{self.base_url}/motion/v1/actions/:current"
        try:
            response = requests.delete(url, timeout=5)
            response.raise_for_status()
            self.logger.info("Successfully sent cancel request to robot.")
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Error canceling action: {e}")
    
    def get_remaining_targets(self):
        """Gets the remaining target points of the current action"""
        url = f"{self.base_url}/motion/v1/milestones"
        try:
            response = requests.get(url, timeout=2)
            response.raise_for_status()

            return response.json()
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Error getting remaining targets: {e}")
            return None

    def get_health(self):
        """Get the AMR health status information"""
        url = f"{self.base_url}/system/v1/robot/health"
        try:
            response = requests.get(url, timeout=2)
            response.raise_for_status()
            
            return response.json()
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Error getting health information: {e}")
            return None

    def get_events(self):
        """Get events that occur on the AMR"""
        url = f"{self.base_url[:-5]}/platform/v1/events"
        try:
            response = requests.get(url, timeout=2)
            response.raise_for_status()

            return response.json()
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Error getting events: {e}")
            return None


    def set_max_speed(self, param = "base.max_moving_speed", value = 0.5):
        try:
            url = f"{self.base_url}/system/v1/parameter"
            payload = {
                "param": param,
                "value": value
            }
            
            response = requests.put(url, json=payload)

            response.raise_for_status()
            result = response.json()
            
            return result
        except requests.exceptions.RequestException as e:
            self.logging.error(f"Error setting max speed: {str(e)}")
            return None  

    def set_emergency_stop(self, param = "base.emergency_stop", value = "on"):
        # To release brake use "base.brake_release" on/off(released/braked)
        try:
            url = f"{self.base_url}/system/v1/parameter"
            payload = {
                "param": param,
                "value": value
            }
            
            response = requests.put(url, json=payload)

            response.raise_for_status()
            result = response.json()
            #return result

            # Currently test "base.emergency_stop" didn't work, but "base.brake_release" workable
            self.logging.error(f"Currently 'base.emergency_stop' didn't work!")
            return False 
    
        except requests.exceptions.RequestException as e:
            self.logging.error(f"Error setting max speed: {str(e)}")
            return None  
        

