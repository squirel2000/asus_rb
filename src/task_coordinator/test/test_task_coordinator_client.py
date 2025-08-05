#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_srvs.srv import SetBool
from follow_user.action import FollowUser
from navigation.action import Navigate
import time

class TaskCoordinatorTestClient(Node):
    def __init__(self):
        super().__init__('task_coordinator_test_client')
        self.get_logger().info('Task Coordinator Test Client Node has been started.')

        # Action Clients (for receiving results/feedback from task_coordinator_node's actions)
        # Note: The task_coordinator_node itself is an ActionClient to follow_user and navigate.
        # This test client will interact with the task_coordinator_node's service and indirectly trigger its actions.
        
        # Service Client for mode switching
        self.set_mode_client = self.create_client(SetBool, 'set_follow_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_follow_mode service not available, waiting...')

        # Publisher for user pose (to simulate user tracking)
        self.user_pose_publisher = self.create_publisher(PoseStamped, '/user_tracker/target_pose', 10)

    def call_set_mode_service(self, follow_mode: bool):
        request = SetBool.Request()
        request.data = follow_mode
        self.get_logger().info(f'Calling set_follow_mode service with data: {follow_mode}')
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(self.set_mode_response_callback)

    def set_mode_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Service call successful: {response.message}')
            else:
                self.get_logger().warn(f'Service call failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')

    def publish_user_pose(self, x, y, z=0.0, ox=0.0, oy=0.0, oz=0.0, ow=1.0):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map" # Assuming user pose is in map frame
        pose_msg.pose.position = Point(x=x, y=y, z=z)
        pose_msg.pose.orientation = Quaternion(x=ox, y=oy, z=oz, w=ow)
        self.get_logger().info(f'Publishing user pose: x={x}, y={y}')
        self.user_pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    client_node = TaskCoordinatorTestClient()

    try:
        # Scenario 1: Test Follow User mode
        client_node.get_logger().info("--- Testing Follow User Mode ---")
        client_node.call_set_mode_service(True) # Switch to Follow User mode
        time.sleep(2) # Give time for service call to process

        # Publish a few user poses to trigger follow_user action
        client_node.publish_user_pose(x=5.0, y=0.0)
        time.sleep(2)
        client_node.publish_user_pose(x=5.0, y=2.0)
        time.sleep(2)
        client_node.publish_user_pose(x=3.0, y=2.0)
        time.sleep(2)

        # Scenario 2: Test Navigate mode
        client_node.get_logger().info("--- Testing Navigate Mode ---")
        client_node.call_set_mode_service(False) # Switch to Navigate mode (this should trigger a default navigation goal in task_coordinator_node)
        time.sleep(5) # Give time for service call and navigation goal to process

        client_node.get_logger().info("Test scenarios completed.")

    except KeyboardInterrupt:
        pass
    finally:
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
