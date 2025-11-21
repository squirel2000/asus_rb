#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from navigation_base.base_navigation_node import BaseNavigationNode
from motion_common.action import Navigate
import math
GO_HOME_CONSTANT = 99.99

class NavigateActionServer(BaseNavigationNode):
    """
    Action server to navigate the robot to a target pose.
    It communicates with the perception_control_manager node to execute the navigation.
    """
    def __init__(self):
        super().__init__('navigation_motion_node')

        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info("Navigate to Pose Action Server has been started.")

        self.get_logger().info(
            f"Parameters: success_distance_threshold={self.success_distance_threshold:.2f}, "
            f"success_yaw_threshold={self.success_yaw_threshold:.2f}, "
            f"stuck_timeout_sec={self.stuck_timeout_sec:.2f}, "
            f"stuck_distance_threshold={self.stuck_distance_threshold:.2f}, "
            f"max_moving_speed={self.max_moving_speed:.2f}, "
            f"max_angular_speed={self.max_angular_speed:.2f}"
        )

    async def _abort_goal(self, goal_handle, message: str):
        """Helper method to abort the goal with a specific message."""
        self.publish_cancel()
        goal_handle.abort()
        result = Navigate.Result()
        result.success = False
        result.message = message
        self.get_logger().info(result.message)
        return result
    
    async def execute_callback(self, goal_handle):
        """Executes the navigation action by publishing to slamware_ros_sdk topics."""
        target_pose = goal_handle.request.target_pose
        speed_ratio = goal_handle.request.speed_ratio
        align_yaw = goal_handle.request.align_yaw
        nav_mode = goal_handle.request.nav_mode

        feedback_msg = Navigate.Feedback()
        result = Navigate.Result()

        self.is_gohome = True if math.isclose(target_pose.pose.position.z, GO_HOME_CONSTANT) else False

        # Set the max speed to the navigation preset speed.
        await self.set_max_speed(self.max_moving_speed, self.max_angular_speed)
        
        # Create a navigation action via publisher
        self.publish_move_to(target_pose, speed_ratio, align_yaw, nav_mode)

        _waiting_timeout = self.stuck_timeout_sec / 2
        _last_move_time = self.get_clock().now()

        # Confirm that the AMR has successfully started executing action
        while rclpy.ok():
            current_state = self._get_status(self.amr_events)

            # Publish feedback
            if self.current_pose:
                feedback_msg.current_pose = self.current_pose
            feedback_msg.status = "CHECKING_EXECUTION_STATUS"
            goal_handle.publish_feedback(feedback_msg)
            
            if current_state == "DEVICE_ERROR_DETECTED":
                self.get_logger().error('Device error detected on the AMR!')
                timeout_message = f"Failed to dismiss the AMR device error warning within {_waiting_timeout} seconds."
                result.message = "Goal aborted due to a device error on the AMR."
                # trying to publish again
                self.publish_move_to(target_pose, speed_ratio, align_yaw, nav_mode)

            elif not self.remaining_targets:
                self.get_logger().warn('Waiting for the AMR remaining target points.')
                timeout_message = f"Waiting for the AMR remaining target points for {_waiting_timeout} seconds."
                result.message = "Goal aborted because no valid target points exist."
                # trying to publish again
                self.publish_move_to(target_pose, speed_ratio, align_yaw, nav_mode)
            else:
                if self.is_gohome:
                    target_pose.pose.position.x = self.remaining_targets[0][0]
                    target_pose.pose.position.y = self.remaining_targets[0][1]

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

            """Check for cancel request"""
            if goal_handle.is_cancel_requested:
                self.publish_cancel()
                goal_handle.canceled()
                result.success = False
                result.message = "Goal canceled by the client."
                self.get_logger().info(result.message)
                return result
            
            await self.ros_async_sleep(0.2)

        self.get_logger().info('Executing goal...')
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
                self.publish_cancel()
                goal_handle.canceled()
                result.success = False
                result.message = "Goal canceled by the client."
                self.get_logger().info(result.message)
                return result

            """Check whether the goal is completed"""
            if not self.remaining_targets: # No more target points

                # Check if the robot has reached the docking pose
                if self.is_gohome and self.amr_basic_state:
                    if self.amr_basic_state.is_charging:
                        goal_handle.succeed()
                        result.success = True
                        result.message = "Goal achieved successfully."
                        self.get_logger().info(result.message)
                        return result
                    else:
                        current_state = "TRYING_TO_DOCK_WITH_CHARGER"
                # Check if the robot has reached the target pose
                elif self._is_goal_reached(self.current_pose, target_pose, align_yaw):
                    goal_handle.succeed()
                    result.success = True
                    result.message = "Goal achieved successfully."
                    self.get_logger().info(result.message)
                    return result
                # AMR action is done but did not meet threshold criteria
                else:
                    result = await self._abort_goal(goal_handle, "Goal aborted because the MoveTo action completed but the target pose was not reached.")
                    return result

            """Publish feedback if current pose is available"""
            if self.current_pose:
                feedback_msg.current_pose = self.current_pose
                feedback_msg.status = current_state
                goal_handle.publish_feedback(feedback_msg)

            # Reset pose to none until new msg is posted
            self.current_pose = None
            
            await self.ros_async_sleep(0.1)

        self.get_logger().info("RCLPY shutdown, aborting goal.")
        return await self._abort_goal(goal_handle, "Goal aborted due to RCLPY shutdown.")

def main(args=None):
    rclpy.init(args=args)
    action_server = NavigateActionServer()
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