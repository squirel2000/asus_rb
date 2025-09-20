#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
import math
import random

class HumanPoseSimulatorAdvanced(Node):
    """
    A test node that simulates a human following the robot with periodic distance changes.
    The distance changes smoothly with added noise to mimic realistic human movement.
    """
    def __init__(self):
        super().__init__('human_pose_simulator_advanced')
        
        # Parameters
        self.declare_parameter('min_distance', 0.8)  # Minimum distance behind robot (meters)
        self.declare_parameter('max_distance', 3.5)  # Maximum distance behind robot (meters)
        self.declare_parameter('publish_rate', 20.0)  # Publish rate (Hz)
        self.declare_parameter('cycle_duration', 6.0)  # Duration of each phase, total cycle = 3 * cycle_duration (seconds)
        
        self.min_distance = self.get_parameter('min_distance').get_parameter_value().double_value
        self.max_distance = self.get_parameter('max_distance').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.cycle_duration = self.get_parameter('cycle_duration').get_parameter_value().double_value

        # Subscriber to robot pose
        self.robot_pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.robot_pose_callback, 10)
        
        # Publisher for human relative pose
        self.human_pose_pub = self.create_publisher(
            PoseStamped, '/human_relative_pose', 10)
        
        self.human_pose_view_pub = self.create_publisher(
            PoseStamped, '/human_pose_view', 10)
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_human_pose)
        
        # State variables
        self.robot_pose = None
        self.start_time = self.get_clock().now()
        
        self.get_logger().info(
            f'Human Pose Simulator Advanced started with min_distance={self.min_distance:.2f}, '
            f'max_distance={self.max_distance:.2f}, publish_rate={self.publish_rate:.2f} Hz, '
            f'cycle_duration={self.cycle_duration:.2f} s'
        )

    def robot_pose_callback(self, msg):
        """Store the latest robot pose."""
        self.robot_pose = msg
        self.get_logger().debug(
            f'Received robot pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}'
        )

    def publish_human_pose(self):
        """Publish a simulated human relative pose with smooth distance changes and noise."""
        if self.robot_pose is None:
            self.get_logger().warn('No robot pose received yet, skipping publish.')
            return
        
        # Calculate elapsed time
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
        
        # Define distance ranges (aligned with guidance_action_server.py)
        cycle_period = 3 * self.cycle_duration  # Total cycle: 15.0 s
        
        # Calculate smooth distance using squared sine function
        human_distance = self.min_distance + (self.max_distance - self.min_distance) * (math.sin(math.pi * elapsed_time / cycle_period) ** 2)
        human_distance = max(self.min_distance, min(self.max_distance, human_distance))

        # Get robot's position and orientation
        robot_x = self.robot_pose.pose.position.x
        robot_y = self.robot_pose.pose.position.y
        robot_z = self.robot_pose.pose.position.z
        quaternion = (
            self.robot_pose.pose.orientation.x,
            self.robot_pose.pose.orientation.y,
            self.robot_pose.pose.orientation.z,
            self.robot_pose.pose.orientation.w
        )
        _, _, robot_yaw = euler_from_quaternion(quaternion)
        
        # Calculate human position (behind the robot) with noise
        noise_sigma = 0.01  # Standard deviation for Gaussian noise (meters)
        noise_x = random.gauss(0, noise_sigma)
        noise_y = random.gauss(0, noise_sigma)
        human_x = robot_x - human_distance * math.cos(robot_yaw) + noise_x
        human_y = robot_y - human_distance * math.sin(robot_yaw) + noise_y
        
        # Create human absolute pose (for visualization)
        human_pose = PoseStamped()
        human_pose.header = self.robot_pose.header
        human_pose.pose.position.x = human_x
        human_pose.pose.position.y = human_y
        human_pose.pose.position.z = robot_z
        human_pose.pose.orientation = self.robot_pose.pose.orientation
        self.human_pose_view_pub.publish(human_pose)

        # Create human relative pose
        human_pose.pose.position.x = human_x - robot_x
        human_pose.pose.position.y = human_y - robot_y
        self.human_pose_pub.publish(human_pose)

        self.get_logger().debug(
            f'Published human relative pose: x={human_pose.pose.position.x:.2f}, '
            f'y={human_pose.pose.position.y:.2f}, distance={human_distance:.2f} m, '
            f'noise_x={noise_x:.3f}, noise_y={noise_y:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = HumanPoseSimulatorAdvanced()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()