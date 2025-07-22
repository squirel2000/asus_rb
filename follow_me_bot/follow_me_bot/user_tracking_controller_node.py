import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import math

class UserTrackingControllerNode(Node):
    """
    This node controls the robot's motion to follow a user.
    It subscribes to a target pose, calculates the necessary velocity commands,
    and publishes them. It also reports its status.
    """
    def __init__(self):
        super().__init__('user_tracking_controller_node')
        self.get_logger().info('User Tracking Controller Node has been started.')

        # Parameters
        self.declare_parameter('target_pose_topic', '/motion_controller/target_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('status_topic', '/motion_controller/status')
        self.declare_parameter('target_distance', 1.0)  # meters
        self.declare_parameter('linear_speed_factor', 0.5)
        self.declare_parameter('angular_speed_factor', 1.0)

        # Subscribers
        target_pose_topic = self.get_parameter('target_pose_topic').get_parameter_value().string_value
        self.target_pose_subscription = self.create_subscription(
            PoseStamped,
            target_pose_topic,
            self.pose_callback,
            10)

        # Publishers
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        
        status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self.status_publisher = self.create_publisher(String, status_topic, 10)

    def pose_callback(self, msg):
        """
        Callback function to process the target pose and control the robot.
        """
        target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        linear_speed_factor = self.get_parameter('linear_speed_factor').get_parameter_value().double_value
        angular_speed_factor = self.get_parameter('angular_speed_factor').get_parameter_value().double_value

        # Extract the position from the pose
        position = msg.pose.position
        
        # Calculate the error in distance and angle
        distance_error = math.sqrt(position.x**2 + position.y**2) - target_distance
        angle_error = math.atan2(position.y, position.x)

        # --- Simple Proportional Controller ---
        # This is a basic controller. You might want to implement a more
        # sophisticated one (e.g., PID controller) for smoother motion.
        
        twist_msg = Twist()

        # Set linear velocity
        twist_msg.linear.x = linear_speed_factor * distance_error
        # Clamp linear velocity to a reasonable range
        twist_msg.linear.x = max(-0.5, min(0.5, twist_msg.linear.x))

        # Set angular velocity
        twist_msg.angular.z = angular_speed_factor * angle_error
        # Clamp angular velocity
        twist_msg.angular.z = max(-1.0, min(1.0, twist_msg.angular.z))

        # Stop if the target is very close
        if abs(distance_error) < 0.1:
            twist_msg.linear.x = 0.0
        
        # Stop turning if the angle is small enough
        if abs(angle_error) < 0.1:
            twist_msg.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist_msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = f"Following user. Distance error: {distance_error:.2f}m, Angle error: {angle_error:.2f}rad"
        self.status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UserTrackingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
