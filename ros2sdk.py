import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from transforms3d.euler import euler2quat

''' This script publishes a goal pose to the `/move_base_simple/goal` topic in ROS 2.

ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped "{
  header: { frame_id: 'map' },
  pose: {
    position: { x: -0.1, y: 0.0, z: 0.0 },
    orientation: { x: 0.0, y: 0.0, z: 0.7071, w: 0.7071 }
  }
}"

'''

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

        # Prepare PoseStamped
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = -0.1
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0

        # Convert yaw (-1.571 rad) to quaternion
        q = euler2quat(0, 0, 1.571)  # (w, x, y, z)
        goal.pose.orientation.x = q[1]
        goal.pose.orientation.y = q[2]
        goal.pose.orientation.z = q[3]
        goal.pose.orientation.w = q[0]

        # Publish once
        self.pub.publish(goal)
        self.get_logger().info('Published goal pose')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()

    # Spin once to make sure message is sent
    rclpy.spin_once(node, timeout_sec=0.1)

    # Shutdown cleanly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
