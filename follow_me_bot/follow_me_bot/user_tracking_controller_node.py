import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import math
from follow_me_bot.action import FollowUser

class UserTrackingControllerNode(Node):
    """
    This node controls the robot's motion to follow a user using an action server.
    It receives a goal to follow a user, calculates velocity commands,
    and provides feedback on the distance.
    """
    def __init__(self):
        super().__init__('user_tracking_controller_node')
        self.get_logger().info('User Tracking Controller Node has been started.')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('linear_speed_factor', 0.5)
        self.declare_parameter('angular_speed_factor', 1.0)

        # Publishers
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Action Server
        self._action_server = ActionServer(
            self,
            FollowUser,
            'follow_user',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        """
        Execute the follow user action.
        """
        self.get_logger().info(f'Executing goal for user: {goal_handle.request.user_id}')

        target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        linear_speed_factor = self.get_parameter('linear_speed_factor').get_parameter_value().double_value
        angular_speed_factor = self.get_parameter('angular_speed_factor').get_parameter_value().double_value

        feedback_msg = FollowUser.Feedback()
        
        # Main control loop
        while rclpy.ok():
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return FollowUser.Result(final_status='Goal aborted')

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return FollowUser.Result(final_status='Goal canceled')

            # Get the latest pose from the goal request
            # In a real scenario, you might need a more dynamic way to get pose updates
            position = goal_handle.request.user_pose.pose.position
            
            # Calculate errors
            distance_error = math.sqrt(position.x**2 + position.y**2) - target_distance
            angle_error = math.atan2(position.y, position.x)

            # --- Proportional Controller ---
            twist_msg = Twist()
            twist_msg.linear.x = linear_speed_factor * distance_error
            twist_msg.linear.x = max(-0.5, min(0.5, twist_msg.linear.x))
            twist_msg.angular.z = angular_speed_factor * angle_error
            twist_msg.angular.z = max(-1.0, min(1.0, twist_msg.angular.z))

            if abs(distance_error) < 0.1:
                twist_msg.linear.x = 0.0
            if abs(angle_error) < 0.1:
                twist_msg.angular.z = 0.0

            self.cmd_vel_publisher.publish(twist_msg)

            # Publish feedback
            feedback_msg.current_distance = math.sqrt(position.x**2 + position.y**2)
            goal_handle.publish_feedback(feedback_msg)

            # Check if goal is reached
            if abs(distance_error) < 0.1 and abs(angle_error) < 0.1:
                break

            rclpy.spin_once(self, timeout_sec=0.1)

        goal_handle.succeed()
        result = FollowUser.Result()
        result.final_status = 'Successfully reached the target distance.'
        return result


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
