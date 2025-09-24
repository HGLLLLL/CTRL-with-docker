#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
# from line_follower.srv import SetLineFollower
from wall_localization.srv import SetWallNavigation
from geometry_msgs.msg import Twist

# from object_detect.srv import DetectOrangeGoal
import numpy as np
import time

# from object_detect.srv import DetectObjects
# from object_detect.srv import  DetectCoffee

class MainController:

    def __init__(self):
        rospy.init_node('main_control_node')
        rospy.loginfo("Main Controller Node Started.")

        # 等待所有服務啟動
        # rospy.loginfo("Waiting for services...")
        # rospy.wait_for_service('set_line_follower')
        # self.line_follower_client = rospy.ServiceProxy('set_line_follower', SetLineFollower)
        self.cmd_vel_pub = rospy.Publisher('dlv/cmd_vel', Twist, queue_size=10)
        rospy.loginfo("Publisher to '/cmd_vel' created.")

        rospy.wait_for_service('navigate_by_wall')
        self.wall_nav_client = rospy.ServiceProxy('navigate_by_wall', SetWallNavigation)
        
        # rospy.wait_for_service('detect_orange_goal')
        # self.orange_detect_client = rospy.ServiceProxy('detect_orange_goal', DetectOrangeGoal)
        
        rospy.loginfo("All services are ready.")

        # self.last_intersection_type = None
        # self.intersection_sub = rospy.Subscriber('/line_detect/intersection_type', String, self.intersection_callback)
        # rospy.loginfo("Subscribed to '/line_detect/intersection_type'.")

        # #object detection
        # rospy.wait_for_service('detect_objects_srv')
        # self.detect_client = rospy.ServiceProxy('detect_objects_srv', DetectObjects)

        # #coffeesupply
        # rospy.wait_for_service('detect_coffee_srv')
        # self.detect_client = rospy.ServiceProxy('detect_coffee_srv',DetectCoffee)

        self.run_competition_flow()

    # def intersection_callback(self, msg):
    #     self.last_intersection_type = msg.data

    # def toggle_line_follower(self, enable):
    #     """Start or stop the line follower service."""
    #     try:
    #         response = self.line_follower_client(enable)
    #         rospy.loginfo(f"Line follower toggled to {enable}: {response.message}")
    #         return response.success
    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Service call to 'set_line_follower' failed: {e}")
    #         return False

    # def follow_line_until_t_junction(self, timeout_sec=60.0):
    #     """Line following task until a T-junction is detected."""
    #     rospy.loginfo("Executing task: Follow line until T-junction...")
    #     self.last_intersection_type = ""
    #     if not self.toggle_line_follower(True):
    #         return False
    #     start_time = rospy.Time.now()
    #     rate = rospy.Rate(10)
    #     while not rospy.is_shutdown():
    #         if self.last_intersection_type == "T_JUNCTION":
    #             rospy.loginfo("T-junction detected! Stopping.")
    #             return self.toggle_line_follower(False)
    #         if (rospy.Time.now() - start_time).to_sec() > timeout_sec:
    #             rospy.logerr(f"Timeout ({timeout_sec}s) reached. Stopping.")
    #             return self.toggle_line_follower(False)
    #         rate.sleep()
    #     return self.toggle_line_follower(False)

    def navigate_by_wall(self, front=-1.0, rear=-1.0, left=-1.0, right=-1.0, angle=-1.0, align_wall=""):
        """Control the robot to navigate by wall."""
        rospy.loginfo(f"Executing task: Wall navigation with params: front={front}, right={right}, angle={angle}...")
        try:
            response = self.wall_nav_client(
                target_front_distance=front, target_rear_distance=rear,
                target_left_distance=left, target_right_distance=right,
                target_angle=angle, align_to_wall=align_wall
            )
            rospy.loginfo(f"Navigation result: {response.message}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to 'navigate_by_wall' failed: {e}")
            return False

# 在 MainController 類別中，與您現有的 navigate_by_wall 函式並列

    def navigate_by_odometry(self, forward=0.0, left=0.0, angle=0.0):
        """
        基於里程計的導航函式。
        所有距離單位為米(m)，角度單位為度(deg)。
        """
        rospy.loginfo(f"Executing ODOMETRY navigation: move forward={forward}m, left={left}m, turn angle={angle}deg")
        try:
        # 關鍵：將 use_odometry 設為 True
            response = self.wall_nav_client(
                target_front_distance=forward if forward > 0 else -1.0,
                target_rear_distance=-forward if forward < 0 else -1.0,
                target_left_distance=left if left > 0 else -1.0,
                target_right_distance=-left if left < 0 else -1.0,
                target_angle=angle,
                align_to_wall="", # 里程計模式下，此參數無效
                use_odometry=True  # <--- 模式切換的總開關！
            )
        
            if response.success:
                rospy.loginfo(f"Odometry navigation successful: {response.message}")
                return True
            else:
                rospy.logerr(f"Odometry navigation failed: {response.message}")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call for odometry navigation failed: {e}")
            return False
        
    def move_for_duration(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, duration=1.0):
            """
            以指定速度移動特定時間。
            :param linear_x: x 軸線速度 (m/s)
            :param linear_y: y 軸線速度 (m/s)
            :param angular_z: z 軸角速度 (rad/s)
            :param duration: 移動持續時間 (秒)
            """
            rospy.loginfo(f"Executing move_for_duration: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}, duration={duration}s")
            
            # 建立 Twist 訊息
            vel_msg = Twist()
            vel_msg.linear.x = linear_x
            vel_msg.linear.y = linear_y
            vel_msg.angular.z = angular_z
            
            # 設定發布頻率
            rate = rospy.Rate(10) # 10 Hz
            
            # 記錄開始時間
            start_time = rospy.Time.now()
            
            # 在指定時間內持續發布速度指令
            while (rospy.Time.now() - start_time).to_sec() < duration:
                if rospy.is_shutdown():
                    break
                self.cmd_vel_pub.publish(vel_msg)
                rate.sleep()
                
            # 時間到後，發布停止指令
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            rospy.loginfo("Movement duration ended. Stopping robot.")
            return True # 表示執行成功


    # def detect_and_navigate_to_orange(self, search_timeout_sec=120.0):
    #     """
    #     Detects an orange goal, translates its absolute world coordinates
    #     into relative wall distances, and navigates to goal.
    #     """

    #     origin_dist_to_right_wall = rospy.get_param("~origin_to_right_wall_dist", 0.5)  # x1 (meters)
    #     origin_dist_to_front_wall = rospy.get_param("~origin_to_front_wall_dist", 3.0)  # y1 (meters)

    #     rospy.loginfo("[GOAL轉換參數] Origin→右牆距離: %.2fm, Origin→前牆距離: %.2fm" %
    #               (origin_dist_to_right_wall, origin_dist_to_front_wall))

    #     start_time = rospy.Time.now()
    #     detect_response = None

    #     while not rospy.is_shutdown():
    #         if (rospy.Time.now() - start_time).to_sec() > search_timeout_sec:
    #             rospy.logerr(f"Search timed out after {search_timeout_sec}s. Could not find orange goal.")
    #             return False

    #         rospy.loginfo_throttle(5, "Continuously searching for orange pair...")
    #         try:
    #             response = self.orange_detect_client()
    #             if response.success:
    #                 rospy.loginfo("Orange goal found! Proceeding to translation and navigation.")
    #                 detect_response = response
    #                 break
    #             else:
    #                 rospy.sleep(1.0)
    #         except rospy.ServiceException as e:
    #             rospy.logerr(f"Service call to 'detect_orange_goal' failed: {e}. Retrying in 2 seconds...")
    #             rospy.sleep(2.0)

    #     if detect_response is None:
    #         return False

    #     world_x = detect_response.target_x
    #     world_y = detect_response.target_y

    #     nav_target_right = abs(world_x) + origin_dist_to_right_wall
    #     nav_target_front = origin_dist_to_front_wall - world_y

    #     rospy.loginfo("Coordinate Translation:")
    #     rospy.loginfo(f"  - Detected World Goal (X, Y): ({world_x:.3f}, {world_y:.3f})")
    #     rospy.loginfo(f"  - ==> Nav Goal (target_right, target_front, target_angle): ({nav_target_right:.3f}, {nav_target_front:.3f}, {np.degrees(detect_response.target_final_yaw):.3f})")

    #     if not self.navigate_by_wall(front=nav_target_front, right=nav_target_right ,target_angle=0.0, align_wall="right"):
    #         rospy.logerr("Failed to navigate to the target point.")
    #         return False

    #     target_angle_deg = np.degrees(detect_response.target_final_yaw)
    #     if not self.navigate_by_wall(angle=target_angle_deg):
    #         rospy.logerr("Failed to rotate to the final angle.")
    #         return False
            
    #     rospy.loginfo("Successfully navigated to the orange goal!")
    #     return True

    # #咖啡偵測coffee detect用來判斷菜單內容
    # def detect_objects(self):
    #     try:
    #         resp = self.detect_client()
    #         detections = json.loads(resp.detection_result_json)
    #         return detections
    #     except rospy.ServiceException as e:
    #             rospy.logerr(f"Object detection service call failed: {e}")
    #             return []
        
    # #coffeesupply 根據物件偵測結果（位置與名稱）判斷咖啡要放在哪一張桌子上，然後發佈 ROS topic 指令給其他控制單元執行
    # def detect_coffee(self):
    #     try:
    #         resp = self.coffee_client()  # 呼叫 detect_coffee_srv (無參數)
    #         if resp.success:
    #             return {
    #                 'target_name': resp.target_name,
    #                 'target_xyz': list(resp.target_xyz)
    #             }
    #         else:
    #             rospy.loginfo("Coffee detection succeeded, but no valid target matched.")
    #             return None
    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Coffee detection service call failed: {e}")
    #         return None

    def run_competition_flow(self):
        current_state = "NAV_AFTER_BRIDGE"
        

        while not rospy.is_shutdown():
            rospy.loginfo(f"====== Current State: {current_state} ======")

##########################################################################################
            if current_state == "NAV_AFTER_BRIDGE":
                # time.sleep(3)
                if self.navigate_by_wall(rear=2.039, angle=0.0, align_wall="right"):
                    current_state = "1"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "1":
                # time.sleep(3)
                if self.navigate_by_wall(left=2.33, angle=0.0, align_wall="right"):
                    current_state = "2"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "2":
                if self.navigate_by_wall(rear=2.382, angle=0.0, align_wall="rear"):
                    # time.sleep(0.5)
                    current_state = "3"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "3":
                if self.move_for_duration(linear_x=0.38, angular_z=-0.81, duration=2.1):
                    current_state = "3.1"
                else:
                    current_state = "ERROR_RECOVERY"
                
##########################################################################################

            elif current_state =="3.1":
                if self.navigate_by_wall(angle=0.0, align_wall="rear"):
                    current_state = "4"
                else:
                    current_state = "ERROR_RECOVERY"
        
            elif current_state == "4":
                if self.navigate_by_wall(right=3.004, angle=0.0, align_wall="rear"):
                    current_state = "7"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "7":
                if self.navigate_by_wall(rear=4.02,right = 3.068, angle=0.0, align_wall="rear"):
                    current_state = "10"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "10":
                if self.navigate_by_wall(right=3.068, angle=0.0, align_wall="rear"):
                    self.navigate_by_wall(right=3.068, angle=0.0, align_wall="rear")
                    current_state = "11"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "11":
                if self.move_for_duration(linear_x=0.38, angular_z=0.86, duration=4.1):
                    current_state = "12"
                else:
                    current_state = "ERROR_RECOVERY"

##########################################################################################

            elif current_state == "12":
                if self.navigate_by_wall(angle=0.0, align_wall="left"):
                    current_state = "13"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "13":
                if self.navigate_by_wall(left=4.016, angle=0.0, align_wall="left"):
                    current_state = "20"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "20":
                if self.navigate_by_wall(front=1.620, left = 4.05, angle=0.0, align_wall="left"):
                    self.navigate_by_wall(front=1.620, left = 4.05, angle=0.0, align_wall="left")
                    current_state = "21"
                else:
                    current_state = "ERROR_RECOVERY"
            
            elif current_state == "21":
                if self.navigate_by_wall(left=4.06, angle=0.0, align_wall="left"):
                    current_state = "22"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "22":
                if self.move_for_duration(linear_x=0.38, angular_z=-0.86, duration=4.1):
                    current_state = "23"
                else:
                    current_state = "ERROR_RECOVERY"

##########################################################################################

            elif current_state == "23":
                if self.navigate_by_wall(angle=0.0, align_wall="rear"):
                    current_state = "24"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "24":
                if self.navigate_by_wall(right=5.045, angle=0.0,align_wall="rear"):
                    current_state = "25"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "25":
                if self.navigate_by_wall(rear=2.292, angle=0.0, align_wall="rear"):
                    current_state = "26"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "26":
                if self.navigate_by_wall(right=5.041, angle=0.0, align_wall="rear"):
                    current_state = "29"
                else:
                    current_state = "ERROR_RECOVERY"

            # elif current_state == "27":
            #     if self.navigate_by_wall(left=2.17, angle=0.0, align_wall="rear"):
            #         current_state = "28"
            #     else:
            #         current_state = "ERROR_RECOVERY"

            # elif current_state == "28":
            #     if self.navigate_by_wall(angle=-90):
            #         current_state = "29"
            #     else:
            #         current_state = "ERROR_RECOVERY"
            
            elif current_state == "29":
                rospy.loginfo("All tasks completed successfully!")
                break

            elif current_state == "ERROR_RECOVERY":
                rospy.logerr("A task failed. Entering error recovery mode.")
                break
            
            rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        MainController()
    except rospy.ROSInterruptException:
        pass
