#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from utils.restful_api import RestfulAPI
from perception_control_manager.srv import CreateNavigation, GetActionStatus

# slamtec https://bucket-download.slamtec.com/df3d216e95439541c6f0fafb5ad8dd61d1865a78/AM201_SLAMTEC_Apollo2.0_usermanual_A5M31_v1_en_0613.pdf
# GET http://127.0.0.1:1448/api/core/system/v1/power/status
ROBOT_API_IP = '127.0.0.1'
ROBOT_API_PORT = 1448

class PerceptionControlManagerNode(Node):
    def __init__(self):
        super().__init__('perception_control_manager_node')
        self.declare_parameter('robot_ip', ROBOT_API_IP)
        self.declare_parameter('robot_port', ROBOT_API_PORT)
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        robot_port = self.get_parameter('robot_port').get_parameter_value().integer_value
        self.api = RestfulAPI(robot_ip, self.get_logger(), port=robot_port)

        # Perception Publishers and Timers
        self.current_pose_publisher = self.create_publisher(PoseStamped, 'current_pose', 10)
        self.pose_timer = self.create_timer(0.04, self.publish_current_pose) # 25 Hz

        self.laser_scan_publisher = self.create_publisher(LaserScan, 'laser_scan', 10)
        self.laser_scan_timer = self.create_timer(0.04, self.publish_laser_scan) # 25 Hz

        # Control Services
        self.create_nav_service = self.create_service(
            CreateNavigation, 'create_navigation', self.create_navigation_callback)
        self.get_status_service = self.create_service(
            GetActionStatus, 'get_action_status', self.get_action_status_callback)
        self.cancel_action_service = self.create_service(
            Trigger, 'cancel_action', self.cancel_action_callback)

        print('Perception Control Manager Node has been started.')
        print('Publishers:')
        print(f'  - {self.current_pose_publisher.topic} ({self.current_pose_publisher.msg_type.__name__}) at {1.0/self.pose_timer.timer_period_ns * 1e9:.2f} Hz')
        print(f'  - {self.laser_scan_publisher.topic} ({self.laser_scan_publisher.msg_type.__name__}) at {1.0/self.laser_scan_timer.timer_period_ns * 1e9:.2f} Hz')
        print('Services:')
        print(f'  - {self.create_nav_service.srv_name} ({self.create_nav_service.srv_type.__name__})')
        print(f'  - {self.get_status_service.srv_name} ({self.get_status_service.srv_type.__name__})')
        print(f'  - {self.cancel_action_service.srv_name} ({self.cancel_action_service.srv_type.__name__})')

    # Perception Callbacks
    def publish_current_pose(self):
        pose = self.api.get_current_pose(self.get_clock())
        if pose:
            self.current_pose_publisher.publish(pose)

    def publish_laser_scan(self):
        scan = self.api.get_laser_scan(self.get_clock())
        if scan:
            self.laser_scan_publisher.publish(scan)

    # Control Callbacks
    def create_navigation_callback(self, request, response):
        self.get_logger().info('Create navigation service called')
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
        self.api.cancel_action()
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionControlManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()