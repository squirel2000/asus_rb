#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import random
import math

class HumanPoseSimulatorAdvanced(Node):
    """
    A test node that simulates a human following the robot with a random walk model.
    The human moves at a random speed and may temporarily lose tracking.
    """
    def __init__(self):
        super().__init__('human_pose_simulator_advanced')
        
        # Parameters
        self.declare_parameter('min_distance', 0.5)  # Minimum distance behind robot (meters)
        self.declare_parameter('max_distance', 5.0)  # Maximum distance behind robot (meters)
        self.declare_parameter('publish_rate', 10.0)  # Publish rate (Hz)
        self.declare_parameter('human_speed_min', 0.2)  # Minimum human speed (m/s)
        self.declare_parameter('human_speed_max', 0.8)  # Maximum human speed (m/s)
        self.declare_parameter('lost_probability', 0.05)  # Probability of losing human per second
        self.declare_parameter('lost_duration_min', 2.0)  # Min duration of losing human (seconds)
        self.declare_parameter('lost_duration_max', 5.0)  # Max duration of losing human (seconds)
        
        self.min_distance = self.get_parameter('min_distance').get_parameter_value().double_value
        self.max_distance = self.get_parameter('max_distance').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.human_speed_min = self.get_parameter('human_speed_min').get_parameter_value().double_value
        self.human_speed_max = self.get_parameter('human_speed_max').get_parameter_value().double_value
        self.lost_probability = self.get_parameter('lost_probability').get_parameter_value().double_value
        self.lost_duration_min = self.get_parameter('lost_duration_min').get_parameter_value().double_value
        self.lost_duration_max = self.get_parameter('lost_duration_max').get_parameter_value().double_value

        # Subscriber to robot pose
        self.robot_pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.robot_pose_callback, 10)
        
        # Publisher for human relative pose
        self.human_pose_pub = self.create_publisher(
            PoseStamped, '/human_relative_pose', 10)
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_human_pose)
        
        # State variables
        self.robot_pose = None
        self.human_distance = random.uniform(self.min_distance, self.max_distance)  # Initial distance
        self.human_speed = random.uniform(self.human_speed_min, self.human_speed_max)  # Initial speed
        self.is_lost = False
        self.lost_start_time = None
        self.lost_duration = 0.0
        
        self.get_logger().info(
            f'Human Pose Simulator Advanced started with min_distance={self.min_distance:.2f}, '
            f'max_distance={self.max_distance:.2f}, publish_rate={self.publish_rate:.2f} Hz, '
            f'human_speed_min={self.human_speed_min:.2f}, human_speed_max={self.human_speed_max:.2f}, '
            f'lost_probability={self.lost_probability:.2f}'
        )

    def robot_pose_callback(self, msg):
        """Store the latest robot pose."""
        self.robot_pose = msg
        self.get_logger().debug(
            f'Received robot pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}'
        )

    def publish_human_pose(self):
        """Publish a simulated human relative pose with random walk and occasional loss."""
        if self.robot_pose is None:
            self.get_logger().warn('No robot pose received yet, skipping publish.')
            return
        
        # Check if human is lost
        current_time = self.get_clock().now()
        if not self.is_lost and random.random() < self.lost_probability / self.publish_rate:
            self.is_lost = True
            self.lost_start_time = current_time
            self.lost_duration = random.uniform(self.lost_duration_min, self.lost_duration_max)
            self.get_logger().info(f'Human lost for {self.lost_duration:.2f} seconds')
        
        if self.is_lost:
            if (current_time - self.lost_start_time).nanoseconds / 1e9 < self.lost_duration:
                self.get_logger().debug('Human is lost, skipping publish.')
                return
            else:
                self.is_lost = False
                self.lost_start_time = None
                self.get_logger().info('Human re-detected.')

        # Update human distance with random walk
        dt = 1.0 / self.publish_rate
        self.human_speed += random.uniform(-0.1, 0.1)  # Random speed change
        self.human_speed = max(self.human_speed_min, min(self.human_speed_max, self.human_speed))
        self.human_distance -= self.human_speed * dt  # Move closer or farther
        self.human_distance = max(self.min_distance, min(self.max_distance, self.human_distance))

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
        
        # Calculate human position (behind the robot)
        human_x = robot_x - self.human_distance * math.cos(robot_yaw)
        human_y = robot_y - self.human_distance * math.sin(robot_yaw)
        
        # Create human relative pose
        human_pose = PoseStamped()
        human_pose.header = self.robot_pose.header
        human_pose.pose.position.x = human_x - robot_x
        human_pose.pose.position.y = human_y - robot_y
        human_pose.pose.position.z = robot_z
        human_pose.pose.orientation = self.robot_pose.pose.orientation
        
        self.human_pose_pub.publish(human_pose)
        self.get_logger().debug(
            f'Published human relative pose: x={human_pose.pose.position.x:.2f}, '
            f'y={human_pose.pose.position.y:.2f}, distance={self.human_distance:.2f} m'
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