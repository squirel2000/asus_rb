import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class BehaviorCoordinatorNode(Node):
    """
    This node coordinates the overall behavior of the user-following feature.
    It subscribes to the raw pose from the visual tracker, processes it,
    and forwards it to the motion controller. It also monitors the system's
    status and can take high-level actions.
    """
    def __init__(self):
        super().__init__('behavior_coordinator_node')
        self.get_logger().info('Behavior Coordinator Node has been started.')

        # Parameters
        self.declare_parameter('raw_pose_topic', '/user_tracker/target_pose')
        self.declare_parameter('processed_pose_topic', '/motion_controller/target_pose')
        self.declare_parameter('status_topic', '/motion_controller/status')

        # Subscribers
        raw_pose_topic = self.get_parameter('raw_pose_topic').get_parameter_value().string_value
        self.raw_pose_subscription = self.create_subscription(
            PoseStamped,
            raw_pose_topic,
            self.raw_pose_callback,
            10)
        
        status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self.status_subscription = self.create_subscription(
            String,
            status_topic,
            self.status_callback,
            10)

        # Publisher for the processed pose
        processed_pose_topic = self.get_parameter('processed_pose_topic').get_parameter_value().string_value
        self.processed_pose_publisher = self.create_publisher(PoseStamped, processed_pose_topic, 10)

    def raw_pose_callback(self, msg):
        """
        Callback for the raw pose from the visual tracker.
        This is where you can filter, smooth, or regulate the pose data
        before sending it to the motion controller.
        """
        # --- Placeholder for pose processing ---
        # For now, we will just forward the pose directly.
        # You could add logic here like:
        # - A low-pass filter to smooth the pose data.
        # - Sanity checks (e.g., is the pose realistic?).
        # - State-based logic (e.g., if the robot is stuck, stop sending poses).
        
        processed_pose = msg  # No processing for now
        self.processed_pose_publisher.publish(processed_pose)

    def status_callback(self, msg):
        """
        Callback for status messages from other nodes (e.g., motion controller).
        This is where you can implement high-level error handling and recovery.
        """
        self.get_logger().info(f'Received status: "{msg.data}"')
        
        # --- Placeholder for high-level actions ---
        # Example: If the motion controller reports an error, you could:
        # - Stop the robot.
        # - Send a notification (e.g., via an MCP message).
        # - Attempt a recovery behavior.
        if "error" in msg.data.lower() or "stuck" in msg.data.lower():
            self.get_logger().error("An error or warning was reported by the motion controller!")
            # Here you would add logic to handle the error, e.g.,
            # self.send_mcp_alert("Robot may be stuck!")


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
