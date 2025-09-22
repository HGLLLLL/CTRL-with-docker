#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import tf.transformations
import numpy as np
import math
from wall_localization.srv import SetWallNavigation, SetWallNavigationResponse

def get_segmented_speed(error, thresholds, speeds):
    abs_error = abs(error)
    sign = np.sign(error)
    if abs_error > thresholds[0]: return sign * speeds[0]
    elif abs_error > thresholds[1]: return sign * speeds[1]
    elif abs_error > thresholds[2]: return sign * speeds[2]
    else: return 0.0

def normalize_angle(angle_deg):
    # Keep in [-180, 180]
    v = (angle_deg + 180) % 360 - 180
    return v

def normalize_angle_rad(angle_rad):
    # Keep in [-pi, pi]
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi

class WallNavigator:
    def __init__(self):
        rospy.init_node('wall_navigator')

        rospy.Subscriber('/wall_distances', Float64MultiArray, self.localization_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/dlv/cmd_vel', Twist, queue_size=10)
        self.nav_service = rospy.Service('navigate_by_wall', SetWallNavigation, self.handle_navigation_request)

        # --- Controller Parameters ---
        # Wall-based navigation parameters
        self.dist_thresholds = [0.5, 0.3, 0.015]; self.dist_speeds = [0.4, 0.35, 0.1]
        
        # Odom-based navigation
        self.odom_dist_thresholds = [0.3, 0.1, 0.01]; self.odom_dist_speeds = [0.25, 0.1, 0.08]
        self.odom_angle_thresholds = [30, 20, 2.5]; self.odom_angle_speeds = [0.4, 0.2, 0.1]
        
        # Angle control parameters
        self.final_angle_thresholds = [30.0, 15.0, 1.5]; self.final_angle_speeds = [0.8, 0.6, 0.15]  
        self.moving_align_thresholds = [30.0, 20.0, 5.0]; self.moving_align_speeds = [0.2, 0.15, 0.1]
        
        # --- State Management ---
        self.state = "IDLE"; self.current_goal = None; self.is_active = False
        self.rotation_sub_state = None; self.rotation_target_angles = {}
        self.alignment_wall = None
        
        # --- Odometry-related State ---
        self.odom_received = False
        self.current_yaw = 0.0
        self.current_pos = Point()
        self.odom_initial_pos = None
        self.odom_initial_yaw = 0.0
        self.odom_target_yaw = 0.0

        rospy.loginfo("Wall Navigator Node launched. Waiting for goals on service '/navigate_by_wall'...")

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        q_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion(q_list)
        self.current_pos = msg.pose.pose.position
        if not self.odom_received:
            self.odom_received = True

    def handle_navigation_request(self, req):
        if (req.target_front_distance != -1.0 and req.target_rear_distance != -1.0) or \
           (req.target_left_distance != -1.0 and req.target_right_distance != -1.0):
            rospy.logerr("Mission Denied: Conflicting linear goals.")
            return SetWallNavigationResponse(success=False, message="Conflicting linear goals.")

        self.current_goal = req
        self.is_active = True
        
        if req.use_odometry:
            if not self.odom_received:
                rospy.logerr("Mission Denied: Odometry data not yet received.")
                self.is_active = False
                return SetWallNavigationResponse(success=False, message="Odometry not ready.")
            
            rospy.loginfo("New ODOMETRY-BASED mission received.")
            self.odom_initial_pos = self.current_pos
            self.odom_initial_yaw = self.current_yaw
            target_angle_deg = req.target_angle if req.target_angle != -1.0 else 0.0
            self.odom_target_yaw = normalize_angle_rad(self.current_yaw + np.radians(target_angle_deg))
            self.state = "ODOM_NAVIGATION"
        else:
            rospy.loginfo("New WALL-BASED mission received. State transition to -> MOVING_TO_POINT")
            self.state = "MOVING_TO_POINT"

        rate = rospy.Rate(20)
        while self.is_active and not rospy.is_shutdown():
            rate.sleep()

        if self.state == "IDLE":
            rospy.loginfo("Mission Accomplished! Sending success response.")
            return SetWallNavigationResponse(success=True, message="Navigation goal reached successfully.")
        else:
            rospy.logerr("Mission Failed or Interrupted. Sending failure response.")
            self.cmd_pub.publish(Twist())
            return SetWallNavigationResponse(success=False, message="Navigation failed or was interrupted.")

    def predict_and_prepare_rotation(self, sensed_data, angle_offset_deg):
        rospy.loginfo(f"State: PREPARE_ROTATION. Preparing to rotate by {angle_offset_deg} degrees.")
        candidates = []
        for wall in ['front','right','rear','left']:
            ang = sensed_data.get(f"{wall}_angle")
            if ang is not None and not np.isnan(ang):
                candidates.append((wall, ang))
        
        if not candidates:
            rospy.logwarn("No reference wall found. Fallback to odometry-only rotation.")
            self.rotation_sub_state = "ROTATING_BY_ODOM"
            self.odom_target_yaw = normalize_angle_rad(self.current_yaw + np.radians(angle_offset_deg))
            self.state = "ROTATING_TO_ANGLE"
            return

        ref_wall, ref_angle = candidates[0]
        initial_ang = ref_angle
        rot = angle_offset_deg
        new_angle = normalize_angle(initial_ang + rot)

        if -44.0 <= new_angle <= 44.0:
            self.rotation_target_angles = {ref_wall: new_angle}
            self.rotation_sub_state = "ROTATING_SIMPLE"
            rospy.loginfo(f"predict_and_prepare_rotation: Small angle rotation. Keep tracking '{ref_wall}', new target: {new_angle:.2f}")
        else:
            walls = ['front','right','rear','left']
            idx = walls.index(ref_wall)
            shift = int(round(rot / 90.0))
            new_idx = (idx + shift) % 4
            new_wall = walls[new_idx]
            
            self.rotation_target_angles = {new_wall: normalize_angle(initial_ang)}
            self.rotation_sub_state = "ROTATING_COMPLEX_COARSE"
            self.odom_target_yaw = normalize_angle_rad(self.current_yaw + np.radians(angle_offset_deg))
            rospy.loginfo(f"predict_and_prepare_rotation: Large angle rotation. Odom rotate first, then fine-tune '{new_wall}' to target: {self.rotation_target_angles[new_wall]:.2f}")
        
        self.state = "ROTATING_TO_ANGLE"

    def localization_callback(self, msg):
        if not self.is_active: 
            return

        sensed_data = {
            'front_dist': msg.data[0], 'rear_dist': msg.data[1], 'left_dist': msg.data[2], 'right_dist': msg.data[3],
            'front_angle': msg.data[4], 'rear_angle': msg.data[5], 'left_angle': msg.data[6], 'right_angle': msg.data[7]
        }
        cmd = Twist()
        # A single point of control for rotation direction. Adjust (1.0 or -1.0) if behavior is inverted.
        angular_sign_correction = 1.0 

        if self.state == "ODOM_NAVIGATION":
            has_linear_goal = any(d != -1.0 for d in [self.current_goal.target_front_distance, self.current_goal.target_rear_distance, self.current_goal.target_left_distance, self.current_goal.target_right_distance])
            has_angular_goal = self.current_goal.target_angle != -1.0
            
            linear_achieved = not has_linear_goal
            angular_achieved = not has_angular_goal

            if has_linear_goal:
                dx = self.current_pos.x - self.odom_initial_pos.x
                dy = self.current_pos.y - self.odom_initial_pos.y
                initial_yaw_cos = math.cos(self.odom_initial_yaw)
                initial_yaw_sin = math.sin(self.odom_initial_yaw)

                dist_traveled_robot_x = dx * initial_yaw_cos + dy * initial_yaw_sin
                dist_traveled_robot_y = -dx * initial_yaw_sin + dy * initial_yaw_cos
                
                target_dist_x = self.current_goal.target_front_distance if self.current_goal.target_front_distance != -1.0 else -self.current_goal.target_rear_distance if self.current_goal.target_rear_distance != -1.0 else 0
                error_x = target_dist_x - dist_traveled_robot_x
                cmd.linear.x = get_segmented_speed(error_x, self.odom_dist_thresholds, self.odom_dist_speeds)

                target_dist_y = self.current_goal.target_left_distance if self.current_goal.target_left_distance != -1.0 else -self.current_goal.target_right_distance if self.current_goal.target_right_distance != -1.0 else 0
                error_y = target_dist_y - dist_traveled_robot_y
                cmd.linear.y = get_segmented_speed(error_y, self.odom_dist_thresholds, self.odom_dist_speeds)

                if cmd.linear.x == 0.0 and cmd.linear.y == 0.0:
                    linear_achieved = True

            if has_angular_goal:
                error_rad = normalize_angle_rad(self.odom_target_yaw - self.current_yaw)
                speed = get_segmented_speed(np.degrees(error_rad), self.odom_angle_thresholds, self.odom_angle_speeds)
                cmd.angular.z = speed * angular_sign_correction
                if cmd.angular.z == 0.0:
                    angular_achieved = True

            if linear_achieved and angular_achieved:
                rospy.loginfo("Odometry navigation goal reached.")
                self.state = "GOAL_REACHED"
        
        elif self.state == "MOVING_TO_POINT":
            has_linear_goal = any(getattr(self.current_goal, f'target_{d}_distance') != -1.0 for d in ['front', 'rear', 'left', 'right'])
            active_controls, achieved_controls = 0, 0
            if has_linear_goal:
                for direction in ['front', 'rear', 'left', 'right']:
                    target_dist_val = getattr(self.current_goal, f'target_{direction}_distance')
                    if target_dist_val != -1.0:
                        active_controls += 1
                        dist = sensed_data.get(f'{direction}_dist')
                        if not np.isnan(dist):
                            error = target_dist_val - dist
                            speed = get_segmented_speed(error, self.dist_thresholds, self.dist_speeds)
                            if direction == 'front': cmd.linear.x = -speed
                            elif direction == 'rear': cmd.linear.x = speed
                            elif direction == 'left': cmd.linear.y = -speed
                            else: cmd.linear.y = speed
                            if speed == 0.0: achieved_controls += 1

            is_alignment_goal = self.current_goal.target_angle == 0.0 and self.current_goal.align_to_wall in ['front', 'rear', 'left', 'right']
            if is_alignment_goal:
                alignment_wall = self.current_goal.align_to_wall
                wall_angle = sensed_data.get(f'{alignment_wall}_angle')
                if wall_angle is not None and not np.isnan(wall_angle):
                    error = 0.0 - wall_angle
                    speed = get_segmented_speed(error, self.moving_align_thresholds, self.moving_align_speeds)
                    cmd.angular.z = speed * angular_sign_correction
                    rospy.loginfo_throttle(1, f"Moving & Aligning (Loose). Err: {error:.2f}, Ang.Z: {cmd.angular.z:.2f}")

            if not has_linear_goal or (achieved_controls == active_controls and active_controls > 0):
                if self.current_goal.target_angle != -1.0:
                    if is_alignment_goal:
                        self.alignment_wall = self.current_goal.align_to_wall
                        rospy.loginfo(f"Transition -> Final Alignment for '{self.alignment_wall}'.")
                        self.state = "ROTATING_TO_ANGLE"
                        self.rotation_sub_state = "ALIGNING_TO_WALL"
                    else:
                        rospy.loginfo("Transition -> Prepare for In-Place Rotation.")
                        self.predict_and_prepare_rotation(sensed_data, self.current_goal.target_angle)
                else:
                    self.state = "GOAL_REACHED"

        elif self.state == "ROTATING_TO_ANGLE":
            if self.rotation_sub_state == "ALIGNING_TO_WALL":
                wall_angle = sensed_data.get(f'{self.alignment_wall}_angle')
                if wall_angle is not None and not np.isnan(wall_angle):
                    error = 0.0 - wall_angle
                    speed = get_segmented_speed(error, self.final_angle_thresholds, self.final_angle_speeds)
                    cmd.angular.z = speed * angular_sign_correction
                    if abs(error) < self.final_angle_thresholds[-1]:
                        self.state = "GOAL_REACHED"
                else:
                    rospy.logwarn(f"Cannot align: wall '{self.alignment_wall}' not detected.")
                    self.state = "GOAL_REACHED"
            
            elif self.rotation_sub_state == "ROTATING_SIMPLE":
                fine_tune_achieved = False
                for wall_type, target_angle in self.rotation_target_angles.items():
                    current_wall_angle = sensed_data.get(f'{wall_type}_angle')
                    if current_wall_angle is not None and not np.isnan(current_wall_angle):
                        error = normalize_angle(target_angle - current_wall_angle)
                        speed = get_segmented_speed(error, self.final_angle_thresholds, self.final_angle_speeds)
                        cmd.angular.z = speed * angular_sign_correction
                        rospy.loginfo_throttle(1, f"Fine-tuning '{wall_type}' | Target:{target_angle:.2f}, Current:{current_wall_angle:.2f}, Err:{error:.2f}")
                        if abs(error) < self.final_angle_thresholds[-1]:
                            fine_tune_achieved = True
                        break # Only use the first valid wall
                if fine_tune_achieved:
                    self.state = "GOAL_REACHED"

            elif self.rotation_sub_state == "ROTATING_COMPLEX_COARSE":
                error_rad = normalize_angle_rad(self.odom_target_yaw - self.current_yaw)
                speed = get_segmented_speed(np.degrees(error_rad), self.final_angle_thresholds, self.final_angle_speeds)
                cmd.angular.z = speed * angular_sign_correction
                if abs(np.degrees(error_rad)) < self.final_angle_thresholds[1]:
                    rospy.loginfo("Coarse rotation finished, switching to fine-tune.")
                    
                    can_fine_tune = False
                    if self.rotation_target_angles:
                        target_wall_type = list(self.rotation_target_angles.keys())[0]
                        current_angle = sensed_data.get(f'{target_wall_type}_angle')
                        if current_angle is not None and not np.isnan(current_angle):
                            can_fine_tune = True
                    
                    if can_fine_tune:
                        self.rotation_sub_state = "ROTATING_SIMPLE"
                    else:
                        rospy.logwarn(f"Cannot fine-tune: target wall not detected after coarse rotation. Mission complete.")
                        self.state = "GOAL_REACHED"
            
            elif self.rotation_sub_state == "ROTATING_BY_ODOM":
                error_rad = normalize_angle_rad(self.odom_target_yaw - self.current_yaw)
                speed = get_segmented_speed(np.degrees(error_rad), self.odom_angle_thresholds, self.odom_angle_speeds)
                cmd.angular.z = speed * angular_sign_correction
                if abs(np.degrees(error_rad)) < self.odom_angle_thresholds[-1]:
                    rospy.loginfo("Odom-only rotation finished.")
                    self.state = "GOAL_REACHED"

        elif self.state == "GOAL_REACHED":
            rospy.loginfo("Goal Reached! Resetting state to IDLE.")
            self.is_active = False
            self.state = "IDLE"
            self.current_goal = None
            cmd = Twist()

        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        navigator = WallNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt received. Shutting down Wall Navigator.")