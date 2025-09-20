#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Twist
from perception_control_manager.srv import SetMaxSpeed
from navigation_base.base_navigation_node import BaseNavigationNode
from motion_common.action import Guidance

class GuidanceActionServer(BaseNavigationNode):
    """
    Action server to guide the user to a target pose.
    It communicates with the perception_control_manager node to execute the navigation.
    """
    def __init__(self):
        super().__init__('guidance_motion_node')
        
        # Declare additional ROS parameters with default values
        self.declare_parameter('normal_distance_min', 1.2)
        self.declare_parameter('normal_distance_max', 2.0)
        self.declare_parameter('lost_distance_threshold', 3.0)
        self.declare_parameter('lost_timeout_sec', 10.0)
        self.declare_parameter('max_moving_speed', 1.5)
        self.declare_parameter('max_angular_speed', 1.2)

        # Get additional parameter values
        self.normal_distance_min = self.get_parameter('normal_distance_min').get_parameter_value().double_value
        self.normal_distance_max = self.get_parameter('normal_distance_max').get_parameter_value().double_value
        self.lost_distance_threshold = self.get_parameter('lost_distance_threshold').get_parameter_value().double_value
        self.lost_timeout_sec = self.get_parameter('lost_timeout_sec').get_parameter_value().double_value
        self.max_moving_speed = self.get_parameter('max_moving_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        # Additional subscriber for human relative pose
        self.human_relative_pose = None
        self.human_pose_subscriber = self.create_subscription(
            PoseStamped, '/human_relative_pose_rear', self.human_pose_callback, 10, callback_group=self.callback_group)
        
        # Additional publisher for set_max_speed
        self.publisher_set_max_speed = self.create_publisher(
            Twist, '/set_max_speed', 10, callback_group=self.callback_group)

        # Service client for set_max_speed
        self.set_max_speed_client = self.create_client(
            SetMaxSpeed, 'set_max_speed', callback_group=self.callback_group)
        """while not self.set_max_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_max_speed service...')"""

        # Action server for Guidance
        self._action_server = ActionServer(
            self,
            Guidance,
            'guide_user_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        # State variables for speed adjustment and human tracking
        self._last_speed_change_time = None
        self._is_human_lost = False
        self._human_lost_start_time = None
        self._last_max_moving_speed = None
        self.current_max_moving_speed = self.max_moving_speed
        self.current_max_angular_speed = self.max_angular_speed

        self.get_logger().info("Guide User to Pose Action Server has been started.")

        self.get_logger().info(
            f"Parameters: success_distance_threshold={self.success_distance_threshold:.2f}, "
            f"success_yaw_threshold={self.success_yaw_threshold:.2f}, "
            f"stuck_timeout_sec={self.stuck_timeout_sec:.2f}, "
            f"stuck_distance_threshold={self.stuck_distance_threshold:.2f}, "
            f"normal_distance_min={self.normal_distance_min:.2f}, "
            f"normal_distance_max={self.normal_distance_max:.2f}, "
            f"lost_distance_threshold={self.lost_distance_threshold:.2f}, "
            f"lost_timeout_sec={self.lost_timeout_sec:.2f}, "
            f"max_moving_speed={self.max_moving_speed:.2f}, "
            f"max_angular_speed={self.max_angular_speed:.2f}"
        )

    def human_pose_callback(self, msg):
        """Callback to store the human's relative pose."""
        self.human_relative_pose = msg

    def publish_set_max_speed(self, max_moving_speed: float, max_angular_speed: float):
        """Publish Twist to set max speed."""
        msg = Twist()
        msg.linear.x = max_moving_speed
        msg.angular.z = max_angular_speed
        self.publisher_set_max_speed.publish(msg)

    async def set_max_speed(self, max_moving_speed: float, max_angular_speed: float):
        """Call service to set max moving and angular speed with retry on failure."""
        request = SetMaxSpeed.Request()
        request.max_moving_speed = max_moving_speed
        request.max_angular_speed = max_angular_speed
        future = await self.set_max_speed_client.call_async(request)
        if future.success:
            self.get_logger().info(
                f'Set max speed: max_moving_speed={max_moving_speed:.2f}, max_angular_speed={max_angular_speed:.2f}')
            return True
        else:
            self.get_logger().error('Failed to set max speed.')
            return False

    def _get_human_distance(self, human_relative_pose) -> float:
        """Calculate the distance to the human from human_relative_pose."""
        if human_relative_pose is None:
            self.get_logger().debug("Human relative pose does not exist.")
            return float('inf')
        dx = human_relative_pose.pose.position.x
        dy = human_relative_pose.pose.position.y
        distance = (dx**2 + dy**2)**0.5
        self.get_logger().debug(f"Human relative distance: {distance:.2f} m")
        return distance

    async def _adjust_speed_and_handle_lost(self, human_distance: float):
        """Adjust robot speed based on human distance and handle lost scenarios."""
        current_time = self.get_clock().now()

        # Speed adjustment logic
        status = "USER_FOLLOWING"
        if human_distance < self.normal_distance_min:
            # Human too close, accelerate
            self.current_max_moving_speed = self.max_moving_speed * 1.2
            self.current_max_angular_speed = self.max_angular_speed * 1.2
            status = "USER_TOO_CLOSE"
            self._is_human_lost = False
            self._human_lost_start_time = None
            self.get_logger().info(f"USER_TOO_CLOSE ---> Distance:{human_distance:.2f}, Adjust speed to - linear:{self.current_max_moving_speed:.2f}, angular:{self.current_max_angular_speed:.2f}")
        elif human_distance > self.normal_distance_max:

            if human_distance > self.lost_distance_threshold:
                status = "USER_NOT_FOUND"
                self.get_logger().warn(f"USER_NOT_FOUND ---> Distance:{human_distance:.2f}")

                if not self._is_human_lost:
                    self._is_human_lost = True
                    self._human_lost_start_time = current_time
                elif (current_time - self._human_lost_start_time).nanoseconds / 1e9 > self.lost_timeout_sec:
                    self.get_logger().error(f"Human lost for {self.lost_timeout_sec} seconds, aborting action")
                    status = "ABORTING"
                    return status
                else:
                    # smooth deceleration
                    self.current_max_moving_speed *= 0.8
                    self.current_max_angular_speed *= 0.8

                self.get_logger().warn(f"USER_NOT_FOUND ---> Distance:{human_distance:.2f}, Adjust speed to - linear:{self.current_max_moving_speed:.2f}, angular:{self.current_max_angular_speed:.2f}")
            else:
                # Human lagging, smooth deceleration
                k = (self.max_moving_speed - 0.05) / (self.lost_distance_threshold - self.normal_distance_max)
                self.current_max_moving_speed = self.max_moving_speed - k * (human_distance - self.normal_distance_max)
                self.current_max_angular_speed = self.max_angular_speed - k * (human_distance - self.normal_distance_max)
                status = "USER_LAGGING_BEHIND"
                self._is_human_lost = False
                self._human_lost_start_time = None
                self.get_logger().info(f"USER_LAGGING_BEHIND ---> Distance:{human_distance:.2f}, Adjust speed to - linear:{self.current_max_moving_speed:.2f}, angular:{self.current_max_angular_speed:.2f}")
        else:
            # Normal following
            self.current_max_moving_speed = self.max_moving_speed
            self.current_max_angular_speed = self.max_angular_speed
            status = "USER_FOLLOWING"
            self._is_human_lost = False
            self._human_lost_start_time = None
            self.get_logger().info(f"USER_FOLLOWING ---> Distance:{human_distance:.2f}, Adjust speed to - linear:{self.current_max_moving_speed:.2f}, angular:{self.current_max_angular_speed:.2f}")
        
        self.publish_set_max_speed(self.current_max_moving_speed, self.current_max_angular_speed)
        
        # Update speed if changed
        if (self._last_max_moving_speed is None or abs(self.current_max_moving_speed - self._last_max_moving_speed) > 0.01) \
                and (current_time - self._last_speed_change_time).nanoseconds / 1e9 > 0.1: # Periodically adjust the speed
            
            success = await self.set_max_speed(self.current_max_moving_speed, self.current_max_angular_speed)
            if not success:
                self.get_logger().error("Speed adjustment failed, service failure")
                return status
            
            self._last_max_moving_speed = self.current_max_moving_speed
            self._last_speed_change_time = current_time
        return status

    async def _resume_navigation(self, target_pose: PoseStamped, speed_ratio: float):
        """Resume navigation when human is re-detected."""
        self.get_logger().info("Human re-detected, resuming navigation")
        await self.publish_cancel()
        self.publish_move_to(target_pose, speed_ratio)

    async def publish_cancel(self):
        """Override base publish_cancel to reset speed."""
        await super().publish_cancel()
        # Reset to default speed before aborting
        await self.set_max_speed(self.max_moving_speed, self.max_angular_speed)
        self.publish_set_max_speed(self.max_moving_speed, self.max_angular_speed)

    async def _abort_goal(self, goal_handle, message: str):
        """Helper method to abort the goal with a specific message."""
        await self.publish_cancel()
        goal_handle.abort()
        result = Guidance.Result()
        result.success = False
        result.message = message
        self.get_logger().info(result.message)
        return result
    
    async def execute_callback(self, goal_handle):
        """Executes the guidance action with human following."""
        target_pose = goal_handle.request.target_pose
        speed_ratio = goal_handle.request.speed_ratio
        user_id = goal_handle.request.user_id
        feedback_msg = Guidance.Feedback()
        result = Guidance.Result()

        # Create a navigation action via publisher
        self.publish_move_to(target_pose, speed_ratio)
        
        _waiting_timeout = self.stuck_timeout_sec / 2
        _last_move_time = self.get_clock().now()
        _last_pose = self.current_pose
        
        # Confirm that the AMR has successfully started executing action
        while rclpy.ok():
            current_state = self._get_status(self.amr_events)

            # Publish feedback
            if self.current_pose:
                feedback_msg.current_pose = self.current_pose
            feedback_msg.status = current_state
            goal_handle.publish_feedback(feedback_msg)
            
            if current_state == "DEVICE_ERROR_DETECTED":
                self.get_logger().error('Device error detected on the AMR!')
                timeout_message = f"Failed to dismiss the AMR device error warning within {_waiting_timeout} seconds."
                result.message = "Goal aborted due to a device error on the AMR."
                # trying to publish again
                self.publish_move_to(target_pose, speed_ratio)
            
            elif not self.remaining_targets:
                self.get_logger().warn('Waiting for the AMR remaining target points.')
                timeout_message = f"Waiting for the AMR remaining target points for {_waiting_timeout} seconds."
                result.message = "Goal aborted because no valid target points exist."
                # trying to publish again
                self.publish_move_to(target_pose, speed_ratio)
            else:
                if not self.global_path and not self._is_goal_reached(self.current_pose, target_pose, with_yaw=False):
                    self.get_logger().warn('Try to find a path to the target pose.')
                    timeout_message = f"Failed to find a valid path to the target within {_waiting_timeout} seconds."
                    result.message = "Goal aborted because the target pose is unreachable."
                else:
                    self.get_logger().info('MoveTo action published successfully.')
                    break
            
            if (self.get_clock().now() - _last_move_time).nanoseconds / 1e9 > _waiting_timeout:
                self.get_logger().error(timeout_message)
                result = await self._abort_goal(goal_handle, result.message)
                return result
            
            await self.ros_async_sleep(0.2)

        self.get_logger().info('Executing guidance goal...')
        self._last_speed_change_time = self.get_clock().now()
        self._is_human_lost = False
        self._human_lost_start_time = None
        self._last_max_moving_speed = None
        _last_move_time = self.get_clock().now()
        _last_pose = self.current_pose

        while rclpy.ok():
            current_state = self._get_status(self.amr_events)

            """Safety prevention"""
            if current_state == "DEVICE_ERROR_DETECTED":
                self.get_logger().error("Device error detected on the AMR!")
                result = await self._abort_goal(goal_handle, "Goal aborted due to a device error on the AMR.")
                return result
            elif current_state == "COLLISION_DETECTED_BY_BUMPER":
                self.get_logger().warn("Collision detected by the bumper!")
                result = await self._abort_goal(goal_handle, "Goal aborted due to a collision detected by the AMR's bumper.")
                return result
            
            """Check if robot is stuck"""
            if self._is_stuck(self.current_pose, _last_pose):
                if (self.get_clock().now() - _last_move_time).nanoseconds / 1e9 > self.stuck_timeout_sec:
                    self.get_logger().error(
                        f"Robot stuck for {self.stuck_timeout_sec} seconds at position "
                        f"(x={self.current_pose.pose.position.x:.2f}, y={self.current_pose.pose.position.y:.2f})"
                    )
                    result = await self._abort_goal(goal_handle, "Goal aborted because the AMR failed to find a valid path and got stuck.")
                    return result
            else:
                _last_move_time = self.get_clock().now()
                _last_pose = self.current_pose

            """Check for cancel request"""
            if goal_handle.is_cancel_requested:
                await self.publish_cancel()
                goal_handle.canceled()
                result.success = False
                result.message = "Goal canceled by the client."
                self.get_logger().info(result.message)
                return result

            """Calculate human distance and publish feedback"""
            human_distance = self._get_human_distance(self.human_relative_pose)
            guiding_stage = await self._adjust_speed_and_handle_lost(human_distance)

            if guiding_stage == "ABORTING":
                result = await self._abort_goal(goal_handle, "Goal aborted due to failure to detect the human follower after timeout.")
                return result

            """Check if human is lost and needs to resume navigation"""
            if self._is_human_lost and self._human_lost_start_time and \
               (self.get_clock().now() - self._human_lost_start_time).nanoseconds / 1e9 > self.lost_timeout_sec:
                if human_distance <= self.lost_distance_threshold:
                    await self._resume_navigation(target_pose, speed_ratio)
                    self._is_human_lost = False
                    self._human_lost_start_time = None

            """Publish feedback if current pose is available"""
            if self.current_pose:
                feedback_msg.current_pose = self.current_pose
                feedback_msg.status = f"{current_state}, {guiding_stage}"
                goal_handle.publish_feedback(feedback_msg)

            """Check whether the goal is completed"""
            if self.current_pose and not self.remaining_targets:
                if self._is_goal_reached(self.current_pose, target_pose):
                    await self.set_max_speed(self.max_moving_speed, self.max_angular_speed)
                    self.publish_set_max_speed(self.max_moving_speed, self.max_angular_speed)
                    goal_handle.succeed()
                    result.success = True
                    result.message = "Goal achieved successfully."
                    self.get_logger().info(result.message)
                    return result
                else:
                    result = await self._abort_goal(goal_handle, "Goal aborted because the MoveTo action completed but the target pose was not reached.")
                    return result

            # Reset pose to none until new msg is posted
            self.current_pose = None
            self.human_relative_pose = None
            
            await self.ros_async_sleep(0.1)

        self.get_logger().info("RCLPY shutdown, aborting goal.")
        return await self._abort_goal(goal_handle, "Goal aborted due to RCLPY shutdown.")

def main(args=None):
    rclpy.init(args=args)
    action_server = GuidanceActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()