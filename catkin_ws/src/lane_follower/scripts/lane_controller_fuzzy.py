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
        
        # Params for Turn 1
        self.turn_pixel_threshold_1 = rospy.get_param('~turn_pixel_threshold_1', 1000.0)
        self.hard_turn_angular_1 = rospy.get_param('~hard_turn_angular_1', 2.0)
        self.hard_turn_duration_1 = rospy.get_param('~hard_turn_duration_1', 1.0)
        
        # Params for Turn 2
        self.turn_pixel_threshold_2 = rospy.get_param('~turn_pixel_threshold_2', 1000.0)
        self.hard_turn_angular_2 = rospy.get_param('~hard_turn_angular_2', 2.0)
        self.hard_turn_duration_2 = rospy.get_param('~hard_turn_duration_2', 1.0)
        
        # Cooldown after a hard turn to ignore signs and resume lane following
        self.hard_turn_cooldown = rospy.get_param('~hard_turn_cooldown', 2.0)

        # Params for the scheduled turn triggered X seconds after the first hard turn
        # 第一次大轉彎完成後，經過 scheduled_turn_delay 秒，再執行一次同方向的硬轉
        self.scheduled_turn_delay = rospy.get_param('~scheduled_turn_delay', 3.0)
        self.scheduled_turn_angular = rospy.get_param('~scheduled_turn_angular', 2.0)
        self.scheduled_turn_duration = rospy.get_param('~scheduled_turn_duration', 1.0)
        
        # Sign alignment parameters
        self.sign_detect_pixel_threshold = rospy.get_param('~sign_detect_pixel_threshold', 5000.0)
        self.sign_offset_threshold = rospy.get_param('~sign_offset_threshold', 50.0)
        self.sign_align_angular = rospy.get_param('~sign_align_angular', 0.5)
        self.scan_angular_z = rospy.get_param('~scan_angular_z', 0.5)
        
        # Turn state
        self.hard_turn_count = 0  # 紀錄大轉彎次數
        self.hard_turn_end_time = 0.0
        self.ignore_sign_end_time = 0.0
        self.active_hard_turn_dir = None
        self.active_hard_turn_angular = 0.0
        self.last_sign_time = 0.0
        self.approaching_sign = False
        self.aligning_sign = False
        self.align_angular_z = 0.0
        self.is_scanning = False
        self.scan_start_time = 0.0

        # Scheduled-turn state (armed after the first hard turn)
        self.scheduled_turn_pending = False
        self.scheduled_turn_trigger_time = 0.0
        self.scheduled_turn_dir = None
        
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
        
        # 如果目前正在大轉彎或是處於轉彎後的冷卻期，先忽略新的標誌避免重複觸發或影響循線
        if now < self.ignore_sign_end_time:
            return
            
        if msg.turn_direction in ['left', 'right']:
            # 如果路標太小，視為還沒真正到達需要考慮路標的距離，直接忽略讓系統維持正常循線
            if msg.pixel_size < self.sign_detect_pixel_threshold:
                return

            self.last_sign_time = now
            self.approaching_sign = True
            
            # 若找到了路標，關閉反轉找標的狀態
            if self.is_scanning:
                self.is_scanning = False
            
            # 決定當前要使用的轉彎參數
            if self.hard_turn_count == 0:
                current_pixel_threshold = self.turn_pixel_threshold_1
                current_hard_turn_angular = self.hard_turn_angular_1
                current_hard_turn_duration = self.hard_turn_duration_1
            else:
                # 第二次以後直接使用 Turn 2 的參數
                current_pixel_threshold = self.turn_pixel_threshold_2
                current_hard_turn_angular = self.hard_turn_angular_2
                current_hard_turn_duration = self.hard_turn_duration_2

            # 當標誌大於門檻，觸發大轉彎
            if msg.pixel_size >= current_pixel_threshold:
                self.active_hard_turn_dir = msg.turn_direction
                self.hard_turn_end_time = now + current_hard_turn_duration
                self.ignore_sign_end_time = self.hard_turn_end_time + self.hard_turn_cooldown
                self.active_hard_turn_angular = current_hard_turn_angular
                self.approaching_sign = False
                self.aligning_sign = False
                self.hard_turn_count += 1
                rospy.loginfo("Executing hard turn #%d (%s) for %.2fs",
                              self.hard_turn_count, msg.turn_direction, current_hard_turn_duration)

                # 第一次大轉彎後，排程 scheduled_turn_delay 秒後再執行一次同方向硬轉
                if self.hard_turn_count == 1:
                    self.scheduled_turn_pending = True
                    self.scheduled_turn_dir = msg.turn_direction
                    self.scheduled_turn_trigger_time = self.hard_turn_end_time + self.scheduled_turn_delay
                    rospy.loginfo("Scheduled follow-up %s turn in %.2fs after first hard turn ends",
                                  self.scheduled_turn_dir, self.scheduled_turn_delay)
                return
                
            # 根據 offset 決定是否需要左右轉校正
            if abs(msg.offset) >= self.sign_offset_threshold:
                self.aligning_sign = True
                # 若標誌在右側 (offset > 0)，車子往右偏 (-sign_align_angular) 進行校正
                if msg.offset > 0:
                    self.align_angular_z = -self.sign_align_angular
                else:
                    self.align_angular_z = self.sign_align_angular
            else:
                self.aligning_sign = False

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
            twist.angular.z = self.active_hard_turn_angular if self.active_hard_turn_dir == 'left' else -self.active_hard_turn_angular
            self.cmd_pub.publish(twist)
            return

        # 第一優先級 (排程)：第一次大轉彎後 X 秒，無視循線/路標，再執行一次同方向硬轉
        if self.scheduled_turn_pending and now >= self.scheduled_turn_trigger_time:
            self.active_hard_turn_dir = self.scheduled_turn_dir
            self.active_hard_turn_angular = self.scheduled_turn_angular
            self.hard_turn_end_time = now + self.scheduled_turn_duration
            self.ignore_sign_end_time = self.hard_turn_end_time + self.hard_turn_cooldown
            self.scheduled_turn_pending = False
            # 清掉其他可能干擾的狀態，硬轉完直接回到循線
            self.approaching_sign = False
            self.aligning_sign = False
            self.is_scanning = False
            rospy.loginfo("Executing scheduled follow-up hard turn (%s) for %.2fs",
                          self.active_hard_turn_dir, self.scheduled_turn_duration)
            twist.linear.x = self.base_speed
            twist.angular.z = self.active_hard_turn_angular if self.active_hard_turn_dir == 'left' else -self.active_hard_turn_angular
            self.cmd_pub.publish(twist)
            return

        # 若超過 0.5 秒沒看到標誌，解除靠近狀態並進入尋找路標狀態
        if self.approaching_sign and (now - self.last_sign_time > 0.3):
            self.approaching_sign = False
            self.aligning_sign = False
            self.is_scanning = True
            self.scan_start_time = now
                
        # 第二優先級：原本有看到路標但卻丟失，停止向前，左右小幅掃描找尋
        if self.is_scanning:
            twist.linear.x = 0.0
            
            # 使用週期性切換的方式來左右轉找尋 (左轉1秒 -> 右轉2秒 -> 左轉1秒 -> 不斷循環)
            cycle = (now - self.scan_start_time) % 4.0
            if cycle < 1.0:
                twist.angular.z = self.scan_angular_z
            elif cycle < 3.0:
                twist.angular.z = -self.scan_angular_z
            else:
                twist.angular.z = self.scan_angular_z
                
            self.cmd_pub.publish(twist)
            return

        # 第三優先級：看見路標時，根據 offset 進行對齊校正，或小於 threshold 則直走
        if self.approaching_sign:
            twist.linear.x = self.base_speed-0.2
            if self.aligning_sign:
                twist.angular.z = self.align_angular_z
            else:
                twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return
        
        # 第四優先級：正常的模糊循線控制 (沒有路標時的日常循線)
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
