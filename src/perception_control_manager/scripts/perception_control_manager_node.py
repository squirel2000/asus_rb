#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from utils.restful_api import RestfulAPI
from perception_control_manager.srv import CreateNavigation, GetActionStatus, SetMaxSpeed
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
        self.timer = self.create_timer(0.2, self.publish_amr_status) # 5 Hz

        """self.current_pose_publisher = self.create_publisher(PoseStamped, 'current_pose', 10)
        self.pose_timer = self.create_timer(0.04, self.publish_current_pose) # 25 Hz

        self.laser_scan_publisher = self.create_publisher(LaserScan, 'laser_scan', 10)
        self.laser_scan_timer = self.create_timer(0.04, self.publish_laser_scan) # 25 Hz

        # Control Services
        self.create_nav_service = self.create_service(
            CreateNavigation, 'create_navigation', self.create_navigation_callback)
        self.get_status_service = self.create_service(
            GetActionStatus, 'get_action_status', self.get_action_status_callback)
        self.cancel_action_service = self.create_service(
            Trigger, 'cancel_action', self.cancel_action_callback)"""
        self.set_max_speed_service = self.create_service(
            SetMaxSpeed, 'set_max_speed', self.set_max_speed_callback)

        print('Perception Control Manager Node has been started.')
        print('Publishers:')
        print(f'  - {self.remaining_targets_publisher.topic} ({self.remaining_targets_publisher.msg_type.__name__}) at {1.0/self.timer.timer_period_ns * 1e9:.2f} Hz')
        
        #print(f'  - {self.current_pose_publisher.topic} ({self.current_pose_publisher.msg_type.__name__}) at {1.0/self.pose_timer.timer_period_ns * 1e9:.2f} Hz')
        #print(f'  - {self.laser_scan_publisher.topic} ({self.laser_scan_publisher.msg_type.__name__}) at {1.0/self.laser_scan_timer.timer_period_ns * 1e9:.2f} Hz')
        #print('Services:')
        #print(f'  - {self.create_nav_service.srv_name} ({self.create_nav_service.srv_type.__name__})')
        #print(f'  - {self.get_status_service.srv_name} ({self.get_status_service.srv_type.__name__})')
        #print(f'  - {self.cancel_action_service.srv_name} ({self.cancel_action_service.srv_type.__name__})')

    # Service Callbacks
    def set_max_speed_callback(self, request, response):
        self.get_logger().info(f"Set max moving speed: {request.max_moving_speed}")
        result_move = self.api.set_max_speed(param= "base.max_moving_speed", value= request.max_moving_speed)

        self.get_logger().info(f"Set max angular speed: {request.max_angular_speed}")
        result_ang = self.api.set_max_speed(param= "base.max_angular_speed", value= request.max_angular_speed)
        

        response.success = result_move and result_ang
        return response
    
    def create_navigation_callback(self, request, response):
        self.get_logger().info(f'Create navigation service called with pose: {request.pose}')
        action_id = self.api.create_navigation_action(request.pose)
        if action_id:
            response.success = True
            response.action_id = action_id
        else:
            response.success = False
            response.action_id = ""
        return response

    def get_action_status_callback(self, request, response):
        # self.get_logger().info(f"Get action status service called for ID: {request.action_id}")
        status = self.api.get_action_status(request.action_id)
        if status and 'state' in status:
            response.status = status['state']
        else:
            response.status = "error"
        return response

    def cancel_action_callback(self, request, response):
        self.get_logger().info('Cancel action service called')
        self.api.cancel_current_action()
        response.success = True
        return response

    # Publisher Callbacks
    def publish_current_pose(self):
        pose = self.api.get_current_pose(self.get_clock())
        if pose:
            self.current_pose_publisher.publish(pose)

    def publish_laser_scan(self):
        scan = self.api.get_laser_scan(self.get_clock())
        if scan:
            self.laser_scan_publisher.publish(scan)

    def publish_remaining_targets(self):
        data = self.api.get_remaining_targets()
        if data:
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

        msg = String(data = str(health))
        self.amr_health_publisher.publish(msg)

        msg = String(data = str(events))
        self.amr_events_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionControlManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()