#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import numpy as np
import math

class FollowUserVisionNode(Node):
    """
    This node simulates the vision part of the user following feature.
    It subscribes to a clicked point (simulating user detection) and the robot's pose,
    calculates the user's pose relative to the robot, and publishes it.
    """
    def __init__(self):
        super().__init__('follow_user_vision_node')
        self.get_logger().info('Follow User Vision Node has been started.')

        # Publisher for the user's relative pose
        self.human_relative_pose_front_raw_publisher = self.create_publisher(PoseStamped, '/human_relative_pose_front_raw', 10)

        # Subscribers
        self.robot_pose_sub = self.create_subscription(PoseStamped, '/robot_pose', self.robot_pose_callback, 10)
        self.clicked_point_sub = self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)

        self.robot_pose = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def robot_pose_callback(self, msg):
        """Callback function for storing the robot's current pose."""
        self.robot_pose = msg

    def clicked_point_callback(self, msg):
        """
        Callback function for processing the clicked point.
        Calculates the relative pose of the point with respect to the robot's base_link.
        """
        if self.robot_pose is None:
            self.get_logger().warn('Robot pose not available yet, skipping clicked point.')
            return

        try:
            # We assume the clicked point is in the 'map' frame. We want to transform it to 'base_link'.
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # Target frame
                msg.header.frame_id,  # Source frame ('map')
                rclpy.time.Time()
            )
            
            point_in_base_frame = do_transform_point(msg, transform)

            human_relative_pose_front_raw = PoseStamped()
            human_relative_pose_front_raw.header.stamp = self.get_clock().now().to_msg()
            human_relative_pose_front_raw.header.frame_id = 'base_link'
            human_relative_pose_front_raw.pose.position.x = point_in_base_frame.point.x
            human_relative_pose_front_raw.pose.position.y = point_in_base_frame.point.y
            human_relative_pose_front_raw.pose.position.z = point_in_base_frame.point.z

            # Since the clicked point only provides position, we cannot know the user's
            # orientation. We assume a neutral orientation (no rotation) relative to the base_link frame.
            # A quaternion with w=1.0 represents no rotation.
            human_relative_pose_front_raw.pose.orientation.w = 1.0
            human_relative_pose_front_raw.pose.orientation.x = 0.0
            human_relative_pose_front_raw.pose.orientation.y = 0.0
            human_relative_pose_front_raw.pose.orientation.z = 0.0

            self.human_relative_pose_front_raw_publisher.publish(human_relative_pose_front_raw)
            self.get_logger().info(f"User pose relative to robot: x={human_relative_pose_front_raw.pose.position.x:.2f}, y={human_relative_pose_front_raw.pose.position.y:.2f}, dist={math.sqrt(human_relative_pose_front_raw.pose.position.x**2 + human_relative_pose_front_raw.pose.position.y**2):.2f}")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform point: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FollowUserVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
