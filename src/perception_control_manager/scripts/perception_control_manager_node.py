#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_srvs.srv import Trigger, SetBool
from utils.restful_api import RestfulAPI
from perception_control_manager.srv import GetActionStatus, SetMaxSpeed, CreateMoveTo
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension

# slamtec https://bucket-download.slamtec.com/df3d216e95439541c6f0fafb5ad8dd61d1865a78/AM201_SLAMTEC_Apollo2.0_usermanual_A5M31_v1_en_0613.pdf
# GET http://127.0.0.1:1448/api/core/system/v1/power/status
ROBOT_API_IP = '192.168.12.1'
ROBOT_API_PORT = 1448

class PerceptionControlManagerNode(Node):
    def __init__(self):
        super().__init__('perception_control_manager_node')
        self.declare_parameter('robot_ip', ROBOT_API_IP)
        self.declare_parameter('robot_port', ROBOT_API_PORT)
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        robot_port = self.get_parameter('robot_port').get_parameter_value().integer_value
        self.api = RestfulAPI(robot_ip, self.get_logger(), port=robot_port)

        # Publishers and Timers
        self.remaining_targets_publisher = self.create_publisher(Float32MultiArray, 'remaining_targets', 10)
        self.timer = self.create_timer(0.1, self.publish_remaining_targets) # 10 Hz

        self.amr_health_publisher = self.create_publisher(String, 'amr_health', 10)
        self.amr_events_publisher = self.create_publisher(String, 'amr_events', 10)
        self.info_timer = self.create_timer(0.2, self.publish_amr_status) # 5 Hz

        self.current_action_publisher = self.create_publisher(String, 'current_action', 10)
        self.action_timer = self.create_timer(0.2, self.publish_current_action) # 5 Hz

        
        # Services
        self.get_status_service = self.create_service(
            GetActionStatus, 'get_action_status', self.get_action_status_callback)
        self.cancel_action_service = self.create_service(
            Trigger, 'cancel_action', self.cancel_action_callback)

        self.create_move_to_service = self.create_service(
            CreateMoveTo, 'create_move_to', self.create_move_to_callback)
        self.create_go_home_service = self.create_service(
            Trigger, 'create_go_home', self.create_go_home_callback)
        self.set_relocalization_service = self.create_service(
            Trigger, 'set_relocalization', self.set_relocalization_callback)
        
        self.set_max_speed_service = self.create_service(
            SetMaxSpeed, 'set_max_speed', self.set_max_speed_callback)
        self.set_emergency_brake = self.create_service(
            SetBool, 'emergency_stop', self.set_emergency_stop_callback)


        self.clock = Clock()

    def delay(self, seconds):
        start_time = self.clock.now()
        end_time = start_time + rclpy.duration.Duration(seconds=seconds)
        while self.clock.now() < end_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    # Service Callbacks
    def create_move_to_callback(self, request, response):
        payload = {
            "action_name": "slamtec.agent.actions.MoveToAction",
            "options": {
                "target": {
                    "x": request.target.location.x,
                    "y": request.target.location.y,
                },
                "move_options": {
                    "mode": 0,
                    "yaw": request.target.yaw,
                    "speed_ratio": request.target.options.speed_ratio.value,
                    "flags": ['with_yaw','precise']
                }
            }
        }
        action_id = self.api.create_actions(payload)

        if action_id:
            self.delay(0.2)
            action_status = self.api.get_action_status(action_id).get("state")
            response.success = True if action_status.get("result") == 0 else False
            response.action_id = action_id
        else:
            response.success = False
            response.action_id = ""
        return response
    
    def create_go_home_callback(self, request, response):
        payload = {
            "action_name": "slamtec.agent.actions.GoHomeAction",
            "options": {
                "gohome_options": {
                    "charging_retry_count": 1,
                }
            }
        }
        action_id = self.api.create_actions(payload)

        if action_id:
            self.delay(0.2)
            action_status = self.api.get_action_status(action_id).get("state")
            if action_status.get("result") == 0:
                response.success = True
                response.message = f"The AMR is navigating to home, action ID:{action_id}"
            else:
                response.success = False 
                response.message = f"The GoHomeAction failed to execute, action ID:{action_id}"
        else:
            response.success = False
            response.message = "Failed to send POST request for the GoHomeAction."
        return response

    def set_relocalization_callback(self, request, response):
        payload = {
            "action_name": "slamtec.agent.actions.RecoverLocalizationAction",
            "options": {
                "area":{},
                "relocalization_options": {}
            }
        }
        action_id = self.api.create_actions(payload)

        if action_id:
            self.delay(0.2)
            action_status = self.api.get_action_status(action_id).get("state")
            if action_status.get("result") == 0:
                response.success = True
                response.message = f"The AMR is recovering localization, action ID:{action_id}"
            else:
                response.success = False 
                response.message = f"The RecoverLocalizationAction failed to execute, action ID:{action_id}"
        else:
            response.success = False
            response.message = "Failed to send POST request for the RecoverLocalizationAction."
        return response
     
    def set_max_speed_callback(self, request, response):
        self.get_logger().info(f"Set max moving speed: {request.max_moving_speed}")
        result_move = self.api.set_max_speed(param= "base.max_moving_speed", value= request.max_moving_speed)

        self.get_logger().info(f"Set max angular speed: {request.max_angular_speed}")
        result_ang = self.api.set_max_speed(param= "base.max_angular_speed", value= request.max_angular_speed)
        
        response.success = result_move and result_ang
        return response
    
    def set_emergency_stop_callback(self, request, response):

        self.get_logger().info(f"Set emergency stop: {request.data}")

        stop = "on" if request.data else "off"
        result = self.api.set_emergency_stop(value= stop)
        
        response.success = result
        return response

    def get_action_status_callback(self, request, response):
        action_id = request.action_id if request.action_id else ":current"
        status = self.api.get_action_status(action_id)
        if status:
            response.status = str(status)
        else:
            response.status = ""
        return response

    def cancel_action_callback(self, request, response):
        self.get_logger().info('Cancel action service called')
        self.api.cancel_current_action()
        response.success = True
        return response

    
    # Publisher Callbacks
    def publish_remaining_targets(self):
        data = self.api.get_remaining_targets()
        if data is not None:
            points = data['path_points']
            msg = Float32MultiArray()
            # Define 2D array layout
            # First dimension: number of points
            dim1 = MultiArrayDimension()
            dim1.label = 'points'
            dim1.size = len(points)  # Number of points, e.g., 2
            dim1.stride = len(points) * 2  # Total data length (points * 2 coordinates)
            msg.layout.dim.append(dim1)
            # Second dimension: number of coordinates per point (x, y)
            dim2 = MultiArrayDimension()
            dim2.label = 'coordinates'
            dim2.size = 2  # Each point has 2 values (x, y)
            dim2.stride = 2  # Data length per point
            msg.layout.dim.append(dim2)
            # Fill data (flattened, but layout describes 2D structure)
            msg.data = [float(coord) for point in points for coord in point]
            
            self.remaining_targets_publisher.publish(msg)
            # Log the published 2D array
            self.get_logger().debug(f'Published 2D array: {points}')

    def publish_amr_status(self):
        health = self.api.get_health()
        events = self.api.get_events()
        if health is not None:
            msg = String(data = str(health))
            self.amr_health_publisher.publish(msg)
        if events is not None:
            msg = String(data = str(events))
            self.amr_events_publisher.publish(msg)

    def publish_current_action(self):
        action_status = self.api.get_action_status()

        if action_status is not None:
            msg = String(data = str(action_status))
            self.current_action_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionControlManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()