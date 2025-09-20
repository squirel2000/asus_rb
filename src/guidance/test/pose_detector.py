import cv2
import numpy as np
from pupil_apriltags import Detector
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class AprilTagDetector(Node):
    def __init__(self, visualize=True):
        # Initialize ROS2 node
        super().__init__('april_tag_detector')
        self.publisher_ = self.create_publisher(PoseStamped, '/human_relative_pose_rear', 10)
        
        # Initialize AprilTag detector
        self.detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        
        # Initialize webcam with 640x480 resolution
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open webcam')
            rclpy.shutdown()
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Camera parameters (adjusted for 640x480; calibrate your camera for accuracy)
        self.fx = 600.0  # Focal length x
        self.fy = 600.0  # Focal length y
        self.cx = 320.0  # Principal point x (center of 640)
        self.cy = 240.0  # Principal point y (center of 480)
        self.tag_size = 0.15  # Tag size in meters (adjust based on your tag)

        self.camera_matrix = np.array([[self.fx, 0, self.cx],
                                       [0, self.fy, self.cy],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros(5)  # Assuming no distortion

        # Visualization flag
        self.visualize = visualize

        # Create timer for detection and publishing
        self.timer = self.create_timer(0.1, self.detect_and_publish)

    def detect_and_publish(self):
        # Capture frame from webcam
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to grab frame')
            return

        # Convert frame to grayscale for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'

        # Prepare visualization frame
        if self.visualize:
            display_frame = frame.copy()  # Create a copy for visualization

        if tags:
            for tag in tags:
                if tag.tag_id == 0:  # Assuming we're looking for tag ID 0; adjust as needed
                    corners = tag.corners
                    object_points = np.array([
                        [-self.tag_size/2, -self.tag_size/2, 0],
                        [self.tag_size/2, -self.tag_size/2, 0],
                        [self.tag_size/2, self.tag_size/2, 0],
                        [-self.tag_size/2, self.tag_size/2, 0]
                    ], dtype=np.float32)
                    image_points = np.array(corners, dtype=np.float32)

                    success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs)

                    if success:
                        # Convert rotation vector to quaternion
                        rotation_matrix, _ = cv2.Rodrigues(rvec)
                        from scipy.spatial.transform import Rotation as R
                        rot = R.from_matrix(rotation_matrix)
                        quat = rot.as_quat()  # [x, y, z, w]

                        pose_msg.pose.position.x = tvec[2][0]*-1
                        pose_msg.pose.position.y = tvec[0][0]
                        pose_msg.pose.position.z = tvec[1][0]*-1
                        """pose_msg.pose.orientation.x = quat[0]
                        pose_msg.pose.orientation.y = quat[1]
                        pose_msg.pose.orientation.z = quat[2]
                        pose_msg.pose.orientation.w = quat[3]"""

                        self.get_logger().info(f'Detected tag {tag.tag_id} at position: {tvec.flatten()}')
                        self.publisher_.publish(pose_msg)
                        
                        # Visualize detected tag and distance
                        if self.visualize:
                            for i in range(4):
                                pt1 = (int(corners[i][0]), int(corners[i][1]))
                                pt2 = (int(corners[(i+1)%4][0]), int(corners[(i+1)%4][1]))
                                cv2.line(display_frame, pt1, pt2, (0, 255, 0), 2)
                            cv2.putText(display_frame, f'ID: {tag.tag_id}', 
                                        (int(corners[0][0]), int(corners[0][1] - 30)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                            cv2.putText(display_frame, f'Distance: {tvec[2][0]:.2f} m', 
                                        (int(corners[0][0]), int(corners[0][1] - 10)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    break  # Publish the first detected tag; adjust if multiple
        else:
            # If no tags detected, show message on visualization
            if self.visualize:
                cv2.putText(display_frame, 'No tag detected', 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Update visualization window
        if self.visualize:
            cv2.imshow('AprilTag Detection', display_frame)
            cv2.waitKey(1)

    def destroy_node(self):
        # Release webcam and destroy OpenCV windows
        self.cap.release()
        if self.visualize:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    # Enable visualization by setting visualize=True, disable with False
    node = AprilTagDetector(visualize=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()