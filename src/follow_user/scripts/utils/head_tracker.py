import math

class HeadTracker:
    def __init__(self, logger):
        self.logger = logger

        # Constants migrated from Java code
        self.NECK_YAW_TOLERANCE_DEGREE_STOP = math.radians(3.0)
        self.NECK_YAW_TOLERANCE_DEGREE_SLOW = math.radians(20.0)
        self.MAX_YAW_VEL = math.radians(150)  # From RobotSpec.java
        self.MAX_PITCH_VEL = math.radians(140) # From RobotSpec.java
        
        # Pitch control parameters
        self.STATIC_NECK_PITCH_DEG = 15.0
        self.PITCH_KP = 2.0 # Proportional gain for pitch position control

    def get_long_dist_rotation_suppress_factor(self, cur_person_dist):
        """
        Suppresses rotation if the user is far away.
        Migrated from getLongDistRotationSupressFactor in TrackFollowUser.java
        """
        LONG_RANGE_SUPPRESS_ROTATE = 1.3
        supress_factor = 1.0
        # In the Java code, DIST_STOP is added to cur_person_dist, but here we get the direct distance.
        # We will assume the caller provides the correct distance.
        if cur_person_dist > LONG_RANGE_SUPPRESS_ROTATE:
            supress_factor = 1.0 - abs(cur_person_dist - LONG_RANGE_SUPPRESS_ROTATE) / LONG_RANGE_SUPPRESS_ROTATE
            supress_factor = max(supress_factor, 0.35)
        return supress_factor

    def calculate_velocities(self, target_yaw_rad, target_dist, current_neck_yaw_rad, 
                               current_neck_yaw_vel_rps, base_angular_vel_rps):
        """
        Calculates the required neck yaw and pitch velocities to track the user.
        This is a Python migration of the calculateNeckJointSPD method from TrackFollowUser.java.
        
        Args:
            target_yaw_rad (float): The target yaw angle to the user, relative to the robot's base_link.
            target_dist (float): The distance to the user.
            current_neck_yaw_rad (float): The current yaw angle of the neck from encoders.
            current_neck_yaw_vel_rps (float): The current yaw velocity of the neck.
            base_angular_vel_rps (float): The current angular velocity of the robot's base.

        Returns:
            tuple: A tuple containing (yaw_velocity_dps, pitch_velocity_dps).
        """
        # --- Yaw Velocity Calculation ---

        # A. Compute Error
        diff_yaw = target_yaw_rad - current_neck_yaw_rad

        # B. Base-Neck Coupling Compensation
        is_base_inverse = (base_angular_vel_rps * (diff_yaw - target_yaw_rad * 0.6)) < 0
        inv_neck_vel = 0.0
        if is_base_inverse:
            inv_neck_vel -= base_angular_vel_rps * 0.8
        else:
            inv_neck_vel -= base_angular_vel_rps * 0.1

        # C. Overshoot Compensation
        total_yaw_speed = base_angular_vel_rps + current_neck_yaw_vel_rps
        overshoot_compensate_vel = 0.0
        if abs(diff_yaw) < abs(total_yaw_speed * 0.65):
            overshoot_compensate_vel = total_yaw_speed * -0.4

        # D. Deadzone and Nonlinear Suppression
        if abs(diff_yaw) < self.NECK_YAW_TOLERANCE_DEGREE_STOP:
            diff_yaw = 0.0
        elif abs(diff_yaw) < self.NECK_YAW_TOLERANCE_DEGREE_SLOW:
            yaw_target_suppress = abs(diff_yaw) / self.NECK_YAW_TOLERANCE_DEGREE_SLOW
            diff_yaw *= yaw_target_suppress ** 0.7

        # E. Long-Distance Suppression
        diff_yaw *= self.get_long_dist_rotation_suppress_factor(target_dist)

        # F. Main Control Law
        yaw_vel_rad_s = (self.MAX_YAW_VEL * 0.36 * math.tanh(6 * diff_yaw) + 
                         inv_neck_vel + overshoot_compensate_vel)
        
        # Bounding
        yaw_vel_rad_s = max(-self.MAX_YAW_VEL, min(self.MAX_YAW_VEL, yaw_vel_rad_s))

        # --- Pitch Velocity Calculation (Simple P-controller to hold position) ---
        # This part is new, as the original Java code used a different logic for pitch.
        # The goal is to maintain a static pitch angle.
        target_pitch_rad = math.radians(self.STATIC_NECK_PITCH_DEG)
        # We need current_neck_pitch_rad, but it's not passed. Assuming it's available where this is called.
        # For now, let's return 0 for pitch, and implement it in the main node.
        pitch_vel_rad_s = 0.0 # Placeholder

        # Convert to degrees per second for the controller
        yaw_velocity_dps = math.degrees(yaw_vel_rad_s)
        pitch_velocity_dps = math.degrees(pitch_vel_rad_s)

        return yaw_velocity_dps, pitch_velocity_dps

    def calculate_pitch_velocity(self, current_pitch_rad):
        """
        Calculates the pitch velocity to maintain a static pitch angle.
        """
        target_pitch_rad = math.radians(self.STATIC_NECK_PITCH_DEG)
        pitch_error = target_pitch_rad - current_pitch_rad
        
        # Simple P-controller for pitch
        pitch_vel_rad_s = self.PITCH_KP * pitch_error
        
        # Bound the velocity
        pitch_vel_rad_s = max(-self.MAX_PITCH_VEL, min(self.MAX_PITCH_VEL, pitch_vel_rad_s))
        
        return math.degrees(pitch_vel_rad_s)
