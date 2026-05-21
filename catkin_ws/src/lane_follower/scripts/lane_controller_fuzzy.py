#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys

try:
    from lane_follower.msg import LaneData, TurnDetect
except ImportError:
    rospy.logerr("Cannot import LaneData or TurnDetect! Please ensure you have run 'catkin_make' and 'source devel/setup.bash' after creating the custom message.")
    sys.exit(1)

class FuzzyLogicController:
    """
    Lightweight, dependency-free Fuzzy Controller using Sugeno inference.
    """
    def __init__(self):
        # Define the center points of the fuzzy sets (NL, NM, Z, PM, PL)
        self.offset_centers = [-100.0, -50.0, 0.0, 50.0, 100.0]
        self.angle_centers = [-50.0, -25.0, 0.0, 25.0, 50.0]
        
        # Create a 5x5 Rule Base (Sugeno singletons, range -1.0 to 1.0)
        # Column: Offset (NL, NM, Z, PM, PL)
        # Row: Angle (NL, NM, Z, PM, PL)
        # Logic: offset > 0 or angle > 0 means the car is drifting right, requires left turn (positive output)
        self.rule_matrix = [
            [-1.0, -1.0, -0.8, -0.4,  0.0],
            [-1.0, -0.6, -0.4,  0.0,  0.4],
            [-0.8, -0.4,  0.0,  0.4,  0.8],
            [-0.4,  0.0,  0.4,  0.6,  1.0],
            [ 0.0,  0.4,  0.8,  1.0,  1.0]
        ]
        
    def fuzzify(self, val, centers):
        """
        Fuzzify the input value, returning a dictionary of adjacent set memberships {index: weight}
        """
        if val <= centers[0]:
            return {0: 1.0}
        if val >= centers[-1]:
            return {len(centers)-1: 1.0}
            
        for i in range(len(centers) - 1):
            if centers[i] <= val <= centers[i+1]:
                # Simple linear interpolation (triangular/trapezoidal membership functions)
                ratio = (val - centers[i]) / float(centers[i+1] - centers[i])
                return {i: 1.0 - ratio, i+1: ratio}
        return {2: 1.0}
        
    def compute(self, offset, angle):
        offset_memberships = self.fuzzify(offset, self.offset_centers)
        angle_memberships = self.fuzzify(angle, self.angle_centers)
        
        num = 0.0
        den = 0.0
        
        # Rule Evaluation - using Product Inference
        for o_idx, o_weight in offset_memberships.items():
            for a_idx, a_weight in angle_memberships.items():
                rule_weight = o_weight * a_weight
                out_val = self.rule_matrix[a_idx][o_idx]
                
                num += rule_weight * out_val
                den += rule_weight
                
        if den == 0:
            return 0.0
        return num / den

class LaneControllerFuzzy:
    def __init__(self):
        rospy.init_node('lane_controller_fuzzy', anonymous=True)
        
        # Read parameters
        self.base_speed = rospy.get_param('~base_speed', 0.5)
        self.max_angular = rospy.get_param('~max_angular', 1.0) # Max angular velocity is +/- 1.0 rad/s
        self.turn_pixel_threshold = rospy.get_param('~turn_pixel_threshold', 1000.0)
        self.hard_turn_angular = rospy.get_param('~hard_turn_angular', 2.0)
        self.hard_turn_duration = rospy.get_param('~hard_turn_duration', 1.0)
        
        # Turn state
        self.hard_turn_end_time = 0.0
        self.active_hard_turn_dir = None
        self.last_sign_time = 0.0
        self.approaching_sign = False
        
        # Initialize Fuzzy Controller
        self.fuzzy_controller = FuzzyLogicController()
        
        # Publisher
        self.cmd_pub = rospy.Publisher('arduino_vel', Twist, queue_size=10)
        
        # Subscriber: Subscribe to the custom message containing offset and angle
        self.lane_sub = rospy.Subscriber('lane_detect', LaneData, self.lane_callback)
        self.turn_sub = rospy.Subscriber('turn_detect', TurnDetect, self.turn_callback)
        
        # 註冊關閉時的回調函數，讓車子可以安全煞停
        rospy.on_shutdown(self.shutdown_hook)
        
        rospy.loginfo("Fuzzy Lane Controller Started.")
        rospy.loginfo("Max Angular Speed: %.2f rad/s, Base Speed: %.2f m/s", self.max_angular, self.base_speed)

    def shutdown_hook(self):
        rospy.loginfo("Shutting down... Stopping the car.")
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        # 大量發送停機指令，確保信號送到 Arduino
        for _ in range(10):
            try:
                self.cmd_pub.publish(twist)
                rospy.sleep(0.05)
            except Exception:
                pass

    def turn_callback(self, msg):
        now = rospy.Time.now().to_sec()
        
        # 如果目前正在大轉彎，先忽略新的標誌避免重複觸發
        if now < self.hard_turn_end_time:
            return
            
        if msg.turn_direction in ['left', 'right']:
            self.last_sign_time = now
            self.approaching_sign = True
            
            # 當標誌大於門檻，觸發大轉彎
            if msg.pixel_size >= self.turn_pixel_threshold:
                self.active_hard_turn_dir = msg.turn_direction
                self.hard_turn_end_time = now + self.hard_turn_duration
                self.approaching_sign = False
                rospy.loginfo("Executing hard turn %s for %.2fs", 
                              msg.turn_direction, self.hard_turn_duration)

    def lane_callback(self, msg):
        now = rospy.Time.now().to_sec()
        
        twist = Twist()
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        
        # 第一優先級：目前正在大轉彎
        if now < self.hard_turn_end_time:
            twist.linear.x = self.base_speed
            twist.angular.z = self.hard_turn_angular if self.active_hard_turn_dir == 'left' else -self.hard_turn_angular
            self.cmd_pub.publish(twist)
            return
            
        # 若超過 0.5 秒沒看到標誌，解除靠近狀態
        if self.approaching_sign and (now - self.last_sign_time > 0.5):
            self.approaching_sign = False
            
        # 第二優先級：看到了轉彎標誌，但尚未到達門檻 (關閉循線控制，維持直線)
        if self.approaching_sign:
            twist.linear.x = self.base_speed
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return
        
        # 第三優先級：正常的模糊循線控制
        offset = msg.offset
        angle = msg.angle
        
        # Get output from fuzzy inference (range -1.0 to 1.0)
        fuzzy_out = self.fuzzy_controller.compute(offset, angle)
        
        # Scale inference result to the maximum control angular velocity
        angular_z = fuzzy_out * self.max_angular
        
        twist.linear.x = self.base_speed
        twist.angular.z = angular_z
        
        # Publish motor control command
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    try:
        LaneControllerFuzzy()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
