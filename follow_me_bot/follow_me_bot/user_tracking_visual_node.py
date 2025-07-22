import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge

class UserTrackingVisualNode(Node):
    """
    This node handles the computer vision part of the user following feature.
    It subscribes to the robot's camera feed, processes the images to detect
    and locate a user, and then publishes the user's estimated pose.
    """
    def __init__(self):
        super().__init__('user_tracking_visual_node')
        self.get_logger().info('User Tracking Visual Node has been started.')

        # Parameters
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('user_pose_topic', '/user_tracker/target_pose')

        # QoS Profile for sensor data (to match Gazebo camera publishers)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Subscribers
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile)
        self.depth_subscription = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            qos_profile)

        # Publisher for the user's pose
        user_pose_topic = self.get_parameter('user_pose_topic').get_parameter_value().string_value
        self.pose_publisher = self.create_publisher(PoseStamped, user_pose_topic, 10)

        # CV Bridge
        self.bridge = CvBridge()
        self.latest_cv_image = None
        self.latest_depth_image = None

    def image_callback(self, msg):
        """Callback function for processing the RGB image."""
        try:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_images()
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def depth_callback(self, msg):
        """Callback function for processing the depth image."""
        try:
            # The depth image is usually in 16UC1 format (16-bit unsigned integer)
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.process_images()
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')

    def process_images(self):
        """
        Processes the latest RGB and depth images to find the user's pose.
        This is where the core image processing logic should be implemented.
        """
        if self.latest_cv_image is None or self.latest_depth_image is None:
            return

        # --- Placeholder for your image processing algorithm ---
        # 1. Detect the user in the self.latest_cv_image (e.g., using Haar cascades, HOG, or a deep learning model).
        # 2. Find the coordinates (u, v) of the user in the image.
        # 3. Get the depth value at (u, v) from self.latest_depth_image.
        # 4. Convert (u, v, depth) to a 3D point (x, y, z) in the camera's coordinate frame.
        # 5. Create and publish a PoseStamped message.
        #
        # For now, we'll just log that we have the images.
        # self.get_logger().info('Processing new pair of RGB and Depth images.')

        # Example: Create a dummy pose and publish it
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_color_optical_frame"  # Frame for the Waffle's RealSense camera
        
        # Dummy pose values - replace with your calculated pose
        pose_msg.pose.position.x = 2.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.pose_publisher.publish(pose_msg)

        # Reset images to avoid reprocessing the same pair
        self.latest_cv_image = None
        self.latest_depth_image = None


def main(args=None):
    rclpy.init(args=args)
    node = UserTrackingVisualNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
