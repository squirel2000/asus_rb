#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from utils.restful_api import RestfulAPI
from perception_control_manager.srv import CreateNavigation, GetActionStatus

class PerceptionControlManagerNode(Node):
    def __init__(self):
        super().__init__('perception_control_manager_node')
        self.declare_parameter('robot_ip', '127.0.0.1')
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.api = RestfulAPI(robot_ip, self.get_logger())

        self.current_pose_publisher = self.create_publisher(PoseStamped, 'current_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_current_pose)
        
        self.create_nav_service = self.create_service(
            CreateNavigation, 'create_navigation', self.create_navigation_callback)
        self.get_status_service = self.create_service(
            GetActionStatus, 'get_action_status', self.get_action_status_callback)
        self.cancel_action_service = self.create_service(
            Trigger, 'cancel_action', self.cancel_action_callback)

    def publish_current_pose(self):
        pose = self.api.get_current_pose(self.get_clock())
        if pose:
            self.current_pose_publisher.publish(pose)

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
        self.get_logger().info(f"Get action status service called for ID: {request.action_id}")
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


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionControlManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
