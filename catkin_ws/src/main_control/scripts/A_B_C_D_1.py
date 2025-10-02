#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from wall_localization.srv import SetWallNavigation
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from line_follower.srv import SetLineFollower
from object_detect.srv import DetectCoffeeSupply
import time
from std_msgs.msg import Float32, Bool
from step_motor.srv import SetDistance, SetDistanceRequest
from std_srvs.srv import Trigger

class MainController:

    def __init__(self):
        rospy.init_node('main_control_node')
        rospy.loginfo("Main Controller Node Started.")

        # 等待所有服務啟動
        rospy.loginfo("Waiting for services...")

        # wall navigation service client
        rospy.wait_for_service('navigate_by_wall')
        self.wall_nav_client = rospy.ServiceProxy('navigate_by_wall', SetWallNavigation)

        # for move_by_duration
        self.cmd_vel_pub = rospy.Publisher('dlv/cmd_vel', Twist, queue_size=10)
        rospy.loginfo("Publisher to '/cmd_vel' created.")
        
        # line follower client
        rospy.wait_for_service('set_line_follower')
        self.line_follower_client = rospy.ServiceProxy('set_line_follower', SetLineFollower)
        self.last_intersection_type = None
        self.line_follower_goal_reached = False
        self.line_follower_lost_line = False
        self.is_line_follower_active = False

        self.intersection_sub = rospy.Subscriber('/line_detect/intersection_type', String, self.intersection_callback)
        rospy.loginfo("Subscribed to '/line_detect/intersection_type'.")

        # coffee detection service clients
        rospy.wait_for_service('start_coffee_camera', timeout=10.0)
        self.start_coffee_cam_client = rospy.ServiceProxy('start_coffee_camera', Trigger)
        rospy.wait_for_service('stop_coffee_camera', timeout=10.0)
        self.stop_coffee_cam_client = rospy.ServiceProxy('stop_coffee_camera', Trigger) 
        rospy.wait_for_service('CoffeeSupply', timeout=10.0)
        self.detect_coffee_client = rospy.ServiceProxy('CoffeeSupply', DetectCoffeeSupply)
        rospy.loginfo("All coffee detection services are ready.")

        # dc motor service clients
        self.dc_motor_ready = False
        self.ready_sub = rospy.Subscriber('/dc_zero_ready', Bool, self.ready_callback)
        rospy.loginfo("Subscribing to /dc_zero_ready topic for handshake.")
        self.height_pub = rospy.Publisher('/slider_setpoint', Float32, queue_size=10, latch=True)
        rospy.loginfo("Created Publisher to /slider_setpoint for DC motor control.")

        # 訂閱者：用於接收來自 Arduino 的 /slider_current_height 回報
        self.current_height = None # 用於儲存最新的高度回報值

        # for coffee
        self.coffee_color = None
        self.table = 0
        self.cup_side = None
        self.motor_num = None

        self.height_sub = rospy.Subscriber('/slider_current_height', Float32, self.height_callback)
        rospy.loginfo("Created Subscriber to /slider_current_height for feedback.")
    
        rospy.loginfo("All services are ready.")

        self.A_B_C_D_flow()


    def intersection_callback(self, msg):
        """
        事件驅動的回調函數，當循線服務啟用時，直接處理關鍵事件。
        """
        # 只有在循線功能啟用時，才處理這些消息，避免在其他模式下誤觸發
        if not self.is_line_follower_active:
            return

        self.last_intersection_type = msg.data

        # 成功事件：偵測到T字路口
        if self.last_intersection_type == 'T_JUNCTION':
            rospy.loginfo(">>> Event: T_JUNCTION detected! Stopping line follower immediately.")
            self.toggle_line_follower(False)
            self.line_follower_goal_reached = True

        
        elif self.last_intersection_type == 'STOP':
            rospy.logwarn(">>> Event: STOP signal detected! Line lost.")
            self.toggle_line_follower(False)
            self.line_follower_lost_line = True


    def toggle_line_follower(self, enable):
        """
        啟動或關閉循線服務，並更新內部狀態。
        """
        rospy.loginfo(f"Attempting to toggle line follower to: {enable}")
        try:
            response = self.line_follower_client(enable)
            if response.success:
                self.is_line_follower_active = enable
                rospy.loginfo(f"Line follower toggled successfully to: {enable}")
            else:
                rospy.logwarn(f"Line follower toggle to {enable} failed but service responded.")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to set_line_follower failed: {e}")
            self.is_line_follower_active = False
            return False

    def follow_line_until_t_junction(self, timeout_sec=30.0):
        """
        --- MODIFIED: Implemented the 'wait and proceed' logic for line loss ---
        Returns 'SUCCESS', 'RECOVERED_FROM_LOSS', 'TIMEOUT', or 'ERROR'.
        """
        rospy.loginfo("Executing task: Follow line (with line-loss recovery)...")

        # 1. 重置狀態旗標
        self.line_follower_goal_reached = False
        self.line_follower_lost_line = False
        self.last_intersection_type = None

        # 2. 啟動循線
        if not self.toggle_line_follower(True):
            rospy.logerr("Failed to start the line follower service.")
            return 'ERROR'

        start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.line_follower_goal_reached:
                rospy.loginfo("Goal (T-Junction) reached successfully!")
                return 'SUCCESS'
           
            if self.line_follower_lost_line:
                rospy.logwarn("Line lost detected. Assuming end of path. Initiating recovery...")
                # a. 確保機器人完全停止
                self.stop_robot()
                # b. 執行阻塞式等待5秒
                rospy.loginfo("Waiting for 7 seconds before proceeding...")
                rospy.sleep(7.0)
                rospy.loginfo("7-second wait complete. Proceeding to the next flow step.")
                # c. 返回一個新的狀態，表示是從丟失中恢復的
                return 'RECOVERED_FROM_LOSS'

            if (rospy.Time.now() - start_time).to_sec() > timeout_sec:
                rospy.logerr(f"Timeout ({timeout_sec}s) reached. Stopping line follower.")
                self.toggle_line_follower(False)
                return 'TIMEOUT'
            
            rate.sleep()
        
        rospy.logwarn("ROS shutdown requested during line following.")
        self.toggle_line_follower(False)
        return 'ERROR'
    
    def stop_robot(self):
        """確保機器人完全停止的工具函數。"""
        rospy.loginfo("Sending zero velocity to stop the robot.")
        stop_msg = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_msg)
            rospy.sleep(0.02)
        
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

# 咖啡告示排辨識與咖啡辨識 (object detection)
    def detect_coffee_supply(self):
        """
        This function is a direct copy from the main production script.
        It manages the camera lifecycle to detect coffee supply, ensuring
        the test is representative of the real use case.
        """
        rospy.loginfo("--- Starting Coffee Supply Detection Task ---")
        try:
            # Step 1: Start the camera via its service
            rospy.loginfo("Requesting to start camera...")
            start_resp = self.start_coffee_cam_client()
            if not start_resp.success:
                rospy.logerr(f"Failed to start coffee camera: {start_resp.message}")
                return False
            
            rospy.loginfo("Camera started successfully. Calling detection service...")
            # Allow a brief moment for camera auto-exposure and focus to stabilize
            rospy.sleep(0.5)

            # Step 2: Call the main detection service
            resp = self.detect_coffee_client()
            
            if not resp.success or resp.table == 0:
                rospy.logwarn(f"CoffeeSupply service reported no success or no table found. Message: {resp}")
                return False

            # Step 3: Process the successful response and store the results
            self.coffee_color = resp.target_name.lower()
            self.table = int(resp.table)
            self.coffee = resp.cup_side.lower()
            
            # Determine which motor would be used based on coffee color
            if self.coffee == 'right':
                self.motor_num = 2
            elif self.coffee == 'left':
                self.motor_num = 1
            else:
                rospy.logwarn(f"Unknown coffee color '{self.coffee_color}', can't determine motor number.")
                self.motor_num = None

            rospy.loginfo(f"DETECTION SUCCESS! Target: '{self.coffee_color}' coffee for table: {self.table}. Cup is on the '{self.coffee}' side.")
            return True

        except rospy.ServiceException as e:
            rospy.logerr(f"A service call failed during the detection process: {e}")
            return False
        finally:
            # Step 4: CRITICAL! Always ensure the camera is stopped,
            # regardless of whether the detection succeeded or failed.
            rospy.loginfo("Requesting to stop camera (in 'finally' block)...")
            try:
                stop_resp = self.stop_coffee_cam_client()
                if not stop_resp.success:
                    rospy.logwarn(f"Could not stop coffee camera cleanly: {stop_resp.message}")
                else:
                    rospy.loginfo("Camera stopped successfully.")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to call the stop_coffee_camera service: {e}")

            rospy.loginfo("--- Finished Coffee Supply Detection Task ---")

# 步進馬達控制 (step motor control)            
    def call_set_distance(self, motor_id, distance):
        service_name = 'cmd_distance_srv'
        rospy.wait_for_service(service_name)
        service_proxy = rospy.ServiceProxy(service_name, SetDistance)
        try:
            resp = service_proxy(motor_id, distance)
            return resp.result == f"motor{motor_id}_end"
        except rospy.ServiceException as e:
            rospy.logerr(f"呼叫 {service_name} 失敗: {e}")
        return False
    
    def move_stepper_to(self,position_cm):
    # 1. 等待名為 "step_merge" 的 Service 上線
        rospy.wait_for_service('step_merge')
        try:
            service_client = rospy.ServiceProxy('step_merge', SetDistance)
            req = SetDistanceRequest()
            req.distance = position_cm
            response = service_client(req)
            rospy.loginfo(f"Service call successful, response: {response.result}")
            return response.result
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return None
    
# PG36直流馬達控制 (step motor control) 
    def height_callback(self, msg):
        """
        當收到 /slider_current_height 的新訊息時，此函式會被自動呼叫。
        它的功能是更新儲存的當前高度值。
        """
        self.current_height = msg.data

    def ready_callback(self, msg):
        """當收到 /dc_zero_ready 的訊息時，更新就緒狀態旗標。"""
        if msg.data:
            self.dc_motor_ready = True
            rospy.loginfo("Received 'dc_zero_ready' signal from Arduino. DC motor is ready.")
            # 我們可以取消訂閱，因為這是一個一次性的信號
            self.ready_sub.unregister()

    def wait_for_dc_motor_ready(self, timeout_sec=3.0):
        """等待直到收到來自 Arduino 的 /dc_zero_ready 信號。"""
        rospy.loginfo("Waiting for DC motor node to publish ready signal...")
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        while not self.dc_motor_ready and not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > timeout_sec:
                rospy.logerr(f"Timeout! Did not receive /dc_zero_ready signal in {timeout_sec} seconds.")
                return False
            rate.sleep()
        
        return self.dc_motor_ready

    def move_slider_to_height(self, target_height_cm, tolerance_cm=0.7, timeout_sec=15.0):
        start_wait = rospy.Time.now()
        rate = rospy.Rate(10)  # 以 10Hz 頻率運行

        # 等待收到第一次的高度回報
        while self.current_height is None:
            if (rospy.Time.now() - start_wait).to_sec() > timeout_sec / 2:
                rospy.logerr("Timeout waiting for first height feedback")
                return False
            rate.sleep()  # 非阻塞式等待
        
        rospy.loginfo(f"Commanding slider to move to {target_height_cm:.2f} cm...")
        
        # 雙重保險，確保回報通道也正常
        if self.current_height is None:
            rospy.logwarn("Height feedback is not available yet. Waiting for first feedback message...")
            wait_start = rospy.Time.now()
            while self.current_height is None:
                if (rospy.Time.now() - wait_start).to_sec() > 1.0:
                    rospy.logerr("Still no height feedback. Aborting move.")
                    return False
                rate.sleep()  # 非阻塞式等待
                
        # 發布目標高度指令
        height_msg = Float32()
        height_msg.data = float(target_height_cm)
        self.height_pub.publish(height_msg)

        # 進入等待迴圈，直到到達目標或逾時
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            # 檢查是否已到達目標
            if self.current_height is not None and abs(self.current_height - target_height_cm) < tolerance_cm:
                rospy.loginfo(f"Slider reached target. Current height: {self.current_height:.2f} cm.")
                # 到達後等待 0.5 秒穩定，使用 rate.sleep()
                for _ in range(5):  # 10Hz * 0.5s = 5 次循環
                    rate.sleep()
                return True

            # 檢查是否逾時
            if (rospy.Time.now() - start_time).to_sec() > timeout_sec:
                rospy.logwarn(f"Timeout! Did not reach {target_height_cm:.2f} cm. Last height: {self.current_height:.2f} cm. Continuing.")
                return True

            rate.sleep()  # 主迴圈的非阻塞式等待
        
        return False # 只有在 rospy 被關閉時才會執行到這裡
    



    def coffee_flow(self):
            current_state = "after bridge"
            choose_table = 0
            rate = rospy.Rate(10)

            while not rospy.is_shutdown():
                rospy.loginfo(f"====== Current State: {current_state} ======")
                if current_state == "after bridge":
                    if self.navigate_by_wall(front=1.65, angle=0.0, align_wall="left"):
                        current_state = "Gripper extended"
                    else:
                        current_state = "ERROR_RECOVERY"

                elif current_state == "Gripper extended":
                    if self.call_set_distance(1, 13) and self.call_set_distance(2, 13):
                        current_state = "first_up"
                    else:
                        current_state = "ERROR_RECOVERY"

                elif current_state == "first_up":
                    if self.move_slider_to_height(20.2):
                        current_state = "navigate to table"
                    else:
                        current_state = "ERROR_RECOVERY"

                elif current_state == "navigate to table":
                    if self.navigate_by_wall(left=0.447, angle=0.0, align_wall="left") and self.navigate_by_wall(front=1.214, angle=0.0, align_wall="left"):
                        current_state = "alingn to table"
                    else:
                        current_state = "ERROR_RECOVERY"

                elif current_state == "alingn to table":
                    if self.navigate_by_wall(angle=0.0, align_wall="front"):
                        current_state = "sign detect"
                    else:
                        current_state = "ERROR_RECOVERY"

                elif current_state == "sign detect":
                    if self.detect_coffee_supply():
                        current_state = "detect over and up"
                    else:
                        current_state = "ERROR_RECOVERY"

                elif current_state == "detect over and up":
                    if self.move_slider_to_height(45):
                        current_state = "navigate to get coffee"
                    else:
                        current_state = "ERROR_RECOVERY"

                elif current_state == "navigate to get coffee":
                    if self.navigate_by_wall(front=0.985, angle=0.0, align_wall="left"):
                        if self.coffee == "right":
                            current_state = "align to get coffee right"
                        elif self.coffee == "left":
                            current_state = "align to get coffee left"
                        else:
                            current_state = "ERROR_RECOVERY"
                        
                    else:
                        current_state = "ERROR_RECOVERY"

                elif current_state == "align to get coffee right":
                    if self.navigate_by_wall(left=0.31, angle=0.0, align_wall="front"):
                        current_state = "Gripper reach out"
                    else:
                        current_state = "ERROR_RECOVERY"

                elif current_state == "align to get coffee left":
                    if self.navigate_by_wall(left=0.43, angle=0.0, align_wall="front"):
                        current_state = "Gripper reach out"
                    else:
                        current_state = "ERROR_RECOVERY"
                
                elif current_state == "Gripper reach out":
                    if self.call_set_distance(self.motor_num, 60):
                        current_state = "Gripper down to grip coffee"
                    else:
                        current_state = "ERROR_RECOVERY"        

                elif current_state == "Gripper down to grip coffee":
                    if self.move_slider_to_height(34.8):
                        current_state = "Gripper withdraw"
                    else:
                        current_state = "ERROR_RECOVERY"

                elif current_state == "Gripper withdraw":
                    if self.call_set_distance(self.motor_num, 14):
                        current_state = "Gripper up to withdraw coffee"
                    else:
                        current_state = "ERROR_RECOVERY"

                elif current_state == "Gripper up to withdraw coffee":
                    if self.move_slider_to_height(40):
                        current_state = "robot back from table"
                    else:
                        current_state = "ERROR_RECOVERY"
                
                elif current_state == "robot back from table":
                    if self.navigate_by_wall(front=1.291, angle=0.0, align_wall="left"):
                        current_state = "Gripper down to robot"
                    else:
                        current_state = "ERROR_RECOVERY"

                elif current_state == "Gripper down to robot":
                    if self.move_slider_to_height(7):
                        current_state = "grip to hold coffee"
                    else:
                        current_state = "ERROR_RECOVERY"

                elif current_state == "grip to hold coffee":
                    if self.call_set_distance(self.motor_num, 9):
                        current_state = "back from table more"
                    else:
                        current_state = "ERROR_RECOVERY"

                elif current_state == "back from table more":
                    if self.navigate_by_wall(left=0.7, angle=0.0, align_wall="left"):
                        current_state = "navigate to spin"
                        choose_table = 1
                        rospy.loginfo(choose_table)
                    else:
                        current_state = "ERROR_RECOVERY"

##########################################################################   

                elif self.table == 1 and choose_table == 1 and self.motor_num == 2:
                    if current_state == "navigate to spin":
                        if self.navigate_by_wall(angle=0.0, align_wall="front"):
                            current_state = "navigate to customer"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "navigate to customer":
                        if self.navigate_by_wall(left = 1.98, front = 1.18592, angle=0.0, align_wall="left"):
                            current_state = "6.5"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "6.5":
                        if self.navigate_by_wall(angle=0.0, align_wall="left"):
                            current_state = "7"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "7":
                        if self.navigate_by_wall(front=1.05):
                            current_state = "put_coffee_down1"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down1":
                        if self.move_slider_to_height(5.5):
                            current_state = "first_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "first_put_coffee":
                        if self.call_set_distance(self.motor_num, 22):
                            current_state = "put_coffee_down2"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down2":
                        if self.move_slider_to_height(20):
                            current_state = "second_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "second_put_coffee":
                        if self.call_set_distance(self.motor_num, 10):
                            current_state = "8.8"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "8.8":
                        if self.navigate_by_wall(front = 1.182, angle=0.0, align_wall="front"):
                            current_state = "turn_to_orange"
                        else:
                            current_state = "ERROR_RECOVERY"
                            
                    elif current_state == "turn_to_orange":
                        self.move_slider_to_height(0)
                        self.call_set_distance(1, 0)
                        self.call_set_distance(2, 0)
                        self.navigate_by_odometry(angle=-80)
                        self.navigate_by_wall(angle=0.0, align_wall="left")
                        choose_table = 0
                        current_state = "0"

    ##########################################################################      

                elif self.table == 1 and choose_table == 1 and self.motor_num == 1:
                    if current_state == "navigate to spin":
                        if self.navigate_by_wall(angle=0.0, align_wall="front"):
                            current_state = "navigate to customer"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "navigate to customer":
                        if self.navigate_by_wall(left = 2.27, front = 1.18592, angle=0.0, align_wall="left"):
                            current_state = "6.5"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "6.5":
                        if self.navigate_by_wall(angle=0.0, align_wall="left"):
                            current_state = "7"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "7":
                        if self.navigate_by_wall(front=1.05):
                            current_state = "put_coffee_down1"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down1":
                        if self.move_slider_to_height(5.5):
                            current_state = "first_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "first_put_coffee":
                        if self.call_set_distance(self.motor_num, 22):
                            current_state = "put_coffee_down2"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down2":
                        if self.move_slider_to_height(20):
                            current_state = "second_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "second_put_coffee":
                        if self.call_set_distance(self.motor_num, 10):
                            current_state = "8.8"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "8.8":
                        if self.navigate_by_wall(front = 1.182, angle=0.0, align_wall="front"):
                            current_state = "turn_to_orange"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "turn_to_orange":
                        self.move_slider_to_height(0)
                        self.call_set_distance(1, 0)
                        self.call_set_distance(2, 0)
                        self.navigate_by_odometry(angle=-80)
                        self.navigate_by_wall(angle=0.0, align_wall="left")
                        
                        choose_table = 0
                        current_state = "0"

    ##########################################################################                  
                elif self.table == 2 and choose_table == 1 and self.motor_num == 2:
                    if current_state == "navigate to spin":
                        if self.navigate_by_wall(angle=0.0, align_wall="front"):
                            current_state = "navigate to customer"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "navigate to customer":
                        if self.navigate_by_wall(left = 2.63, front = 1.12592, angle=0.0, align_wall="left"):
                            current_state = "6.5"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "6.5":
                        if self.navigate_by_wall(angle=0.0, align_wall="left"):
                            current_state = "7"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "7":
                        if self.navigate_by_wall(front=1.05):
                            current_state = "put_coffee_down1"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down1":
                        if self.move_slider_to_height(5.5):
                            current_state = "first_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "first_put_coffee":
                        if self.call_set_distance(self.motor_num, 22):
                            current_state = "put_coffee_down2"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down2":
                        if self.move_slider_to_height(20):
                            current_state = "second_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "second_put_coffee":
                        if self.call_set_distance(self.motor_num, 10):
                            current_state = "8.8"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "8.8":
                        if self.navigate_by_wall(front = 1.182, angle=0.0, align_wall="front"):
                            current_state = "turn_to_orange"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "turn_to_orange":
                        self.move_slider_to_height(0)
                        self.call_set_distance(1, 0)
                        self.call_set_distance(2, 0)
                        self.navigate_by_odometry(angle=-80)
                        self.navigate_by_wall(angle=0.0, align_wall="left")

                        choose_table = 0
                        current_state = "0"

    ##########################################################################                  
                elif self.table == 2 and choose_table == 1 and self.motor_num == 1:
                    if current_state == "navigate to spin":
                        if self.navigate_by_wall(angle=0.0, align_wall="front"):
                            current_state = "navigate to customer"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "navigate to customer":
                        if self.navigate_by_wall(left = 2.92, front = 1.12592, angle=0.0, align_wall="left"):
                            current_state = "6.5"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "6.5":
                        if self.navigate_by_wall(angle=0.0, align_wall="left"):
                            current_state = "7"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "7":
                        if self.navigate_by_wall(front=1.05):
                            current_state = "put_coffee_down1"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down1":
                        if self.move_slider_to_height(5.5):
                            current_state = "first_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "first_put_coffee":
                        if self.call_set_distance(self.motor_num, 22):
                            current_state = "put_coffee_down2"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down2":
                        if self.move_slider_to_height(20):
                            current_state = "second_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "second_put_coffee":
                        if self.call_set_distance(self.motor_num, 10):
                            current_state = "8.8"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "8.8":
                        if self.navigate_by_wall(front = 1.182, angle=0.0, align_wall="front"):
                            current_state = "turn_to_orange"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "turn_to_orange":
                        self.move_slider_to_height(0)
                        self.call_set_distance(1, 0)
                        self.call_set_distance(2, 0)
                        self.navigate_by_odometry(angle=-80)
                        self.navigate_by_wall(angle=0.0, align_wall="left")
                        
                        choose_table = 0
                        current_state = "0"
                    
    ##########################################################################  

                elif self.table == 3 and choose_table == 1 and self.motor_num == 2:
                    if current_state == "navigate to spin":
                        if self.navigate_by_wall(front= 1.33, angle=0.0, align_wall="front"):
                            current_state = "5.1"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "5.1":
                        if self.navigate_by_odometry(angle = -170):
                            current_state = "5.4"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "5.4":
                        if self.navigate_by_wall(angle=0.0, align_wall="rear"):
                            current_state = "navigate to customer"
                        else:
                            current_state = "ERROR_RECOVERY"                

                    elif current_state == "navigate to customer":
                        if self.navigate_by_wall(right = 2.27, rear = 1.38, angle=0.0, align_wall="right"):
                            current_state = "6.5"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "6.5":
                        if self.navigate_by_wall(angle=0.0, align_wall="right"):
                            current_state = "7"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "7":
                        if self.navigate_by_wall(rear = 1.46, angle=0.0, align_wall="right"):
                            current_state = "put_coffee_down1"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down1":
                        if self.move_slider_to_height(5.5):
                            current_state = "first_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "first_put_coffee":
                        if self.call_set_distance(self.motor_num, 22):
                            current_state = "put_coffee_down2"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down2":
                        if self.move_slider_to_height(20):
                            current_state = "second_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "second_put_coffee":
                        if self.call_set_distance(self.motor_num, 10):
                            current_state = "8.8"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "8.8":
                        if self.navigate_by_wall(rear = 1.3, angle=0.0, align_wall="rear"):
                            current_state = "turn_to_orange"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "turn_to_orange":
                        self.move_slider_to_height(0)
                        self.call_set_distance(1, 0)
                        self.call_set_distance(2, 0)
                        self.navigate_by_odometry(angle=80)
                        self.navigate_by_wall(angle=0.0, align_wall="left")
                        
                        choose_table = 0
                        current_state = "0"

    ##########################################################################  

                elif self.table == 3 and choose_table == 1 and self.motor_num == 1:
                    if current_state == "navigate to spin":
                        if self.navigate_by_wall(front= 1.33, angle=0.0, align_wall="front"):
                            current_state = "5.1"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "5.1":
                        if self.navigate_by_odometry(angle = -170):
                            current_state = "5.4"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "5.4":
                        if self.navigate_by_wall(angle=0.0, align_wall="rear"):
                            current_state = "navigate to customer"
                        else:
                            current_state = "ERROR_RECOVERY"            

                    elif current_state == "navigate to customer":
                        if self.navigate_by_wall(right = 1.98, rear = 1.38, angle=0.0, align_wall="right"):
                            current_state = "6.5"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "6.5":
                        if self.navigate_by_wall(angle=0.0, align_wall="right"):
                            current_state = "7"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "7":
                        if self.navigate_by_wall(rear = 1.46, angle=0.0, align_wall="right"):
                            current_state = "put_coffee_down1"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down1":
                        if self.move_slider_to_height(5.5):
                            current_state = "first_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "first_put_coffee":
                        if self.call_set_distance(self.motor_num, 22):
                            current_state = "put_coffee_down2"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down2":
                        if self.move_slider_to_height(20):
                            current_state = "second_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "second_put_coffee":
                        if self.call_set_distance(self.motor_num, 10):
                            current_state = "8.8"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "8.8":
                        if self.navigate_by_wall(rear = 1.3, angle=0.0, align_wall="rear"):
                            current_state = "turn_to_orange"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "turn_to_orange":
                        self.move_slider_to_height(0)
                        self.call_set_distance(1, 0)
                        self.call_set_distance(2, 0)
                        self.navigate_by_odometry(angle=80)
                        self.navigate_by_wall(angle=0.0, align_wall="left")
                        
                        choose_table = 0
                        current_state = "0"
                    

    ##########################################################################  
                elif self.table == 4 and choose_table == 1 and self.motor_num == 2:
                    if current_state == "navigate to spin":
                        if self.navigate_by_wall(front= 1.33, angle=0.0, align_wall="front"):
                            current_state = "5.2"
                        else:
                            current_state = "ERROR_RECOVERY"
                    
                    elif current_state == "5.2":
                        if self.navigate_by_odometry(angle = -170):
                            current_state = "5.4"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "5.4":
                        if self.navigate_by_wall(angle=0.0, align_wall="rear"):
                            current_state = "navigate to customer"
                        else:
                            current_state = "ERROR_RECOVERY"                 

                    elif current_state == "navigate to customer":
                        if self.navigate_by_wall(right = 2.92, rear = 1.38, angle=0.0, align_wall="right"):
                            current_state = "6.5"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "6.5":
                        if self.navigate_by_wall(angle=0.0, align_wall="rear"):
                            current_state = "7"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "7":
                        if self.navigate_by_wall(rear = 1.46):
                            current_state = "put_coffee_down1"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down1":
                        if self.move_slider_to_height(5.5):
                            current_state = "first_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "first_put_coffee":
                        if self.call_set_distance(self.motor_num, 22):
                            current_state = "put_coffee_down2"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down2":
                        if self.move_slider_to_height(20):
                            current_state = "second_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "second_put_coffee":
                        if self.call_set_distance(self.motor_num, 10):
                            current_state = "8.8"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "8.8":
                        if self.navigate_by_wall(rear = 1.25, angle=0.0, align_wall="rear"):
                            current_state = "turn_to_orange"
                        else:
                            current_state = "ERROR_RECOVERY" 
                        
                    elif current_state == "turn_to_orange":
                        self.move_slider_to_height(0)
                        self.call_set_distance(1, 0)
                        self.call_set_distance(2, 0)
                        self.navigate_by_odometry(angle=80)
                        self.navigate_by_wall(angle=0.0, align_wall="left")
                        
                        choose_table = 0
                        current_state = "0"

    ###########################################################################################
                elif self.table == 4 and choose_table == 1 and self.motor_num == 1:
                    if current_state == "navigate to spin":
                        if self.navigate_by_wall(front= 1.33, angle=0.0, align_wall="front"):
                            current_state = "5.2"
                        else:
                            current_state = "ERROR_RECOVERY"
                    

                    elif current_state == "5.2":
                        if self.navigate_by_odometry(angle = -170):
                            current_state = "5.4"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "5.4":
                        if self.navigate_by_wall(angle=0.0, align_wall="rear"):
                            current_state = "navigate to customer"
                        else:
                            current_state = "ERROR_RECOVERY"                

                    elif current_state == "navigate to customer":
                        if self.navigate_by_wall(right = 2.63, rear = 1.38, angle=0.0, align_wall="rear"):
                            current_state = "6.5"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "6.5":
                        if self.navigate_by_wall(angle=0.0, align_wall="rear"):
                            current_state = "7"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "7":
                        if self.navigate_by_wall(rear = 1.46):
                            current_state = "put_coffee_down1"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down1":
                        if self.move_slider_to_height(5.5):
                            current_state = "first_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "first_put_coffee":
                        if self.call_set_distance(self.motor_num, 22):
                            current_state = "put_coffee_down2"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "put_coffee_down2":
                        if self.move_slider_to_height(20):
                            current_state = "second_put_coffee"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "second_put_coffee":
                        if self.call_set_distance(self.motor_num, 10):
                            current_state = "8.8"
                        else:
                            current_state = "ERROR_RECOVERY"

                    elif current_state == "8.8":
                        if self.navigate_by_wall(rear = 1.25, angle=0.0, align_wall="rear"):
                            current_state = "turn_to_orange"
                        else:
                            current_state = "ERROR_RECOVERY"
                        
                    elif current_state == "turn_to_orange":
                        self.move_slider_to_height(0)
                        self.call_set_distance(1, 0)
                        self.call_set_distance(2, 0)
                        self.navigate_by_odometry(angle=80)
                        self.navigate_by_wall(angle=0.0, align_wall="left")
                        
                        choose_table = 0
                        current_state = "0"

    ##########################################################################    

                elif current_state == "0":
                    self.navigate_by_wall(angle=0.0, align_wall="left")
                    self.navigate_by_wall(rear=4.75, angle=0.0, align_wall="left")
                    rospy.loginfo("All tasks completed successfully!")
                    return True
        
                elif current_state == "ERROR_RECOVERY":
                    rospy.logerr("A task failed. Entering error recovery mode.")
                    return False
                
                rate.sleep()

            return False   

    def take_basket(self):
        current_state = "After Coffee"
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo(f"====== Current State: {current_state} ======")
            if current_state == "After Coffee":
                if self.navigate_by_odometry(angle = 80):
                    self.navigate_by_wall(angle=0.0, align_wall="right")
                    current_state = "back to get basket"
                else:
                    current_state = "ERROR_RECOVERY" 

            elif current_state == "back to get basket":
                if self.navigate_by_wall(left=3.266, angle=0, align_wall="left") and self.navigate_by_wall(rear = 1.3, angle=0, align_wall="left"):
                    self.navigate_by_odometry(forward=-0.2)
                    current_state = "grip down"
                else:
                    current_state = "ERROR_RECOVERY" 

            elif current_state == "grip down":
                if self.move_stepper_to(-13):
                    current_state = "go to put basket"
                else:
                    current_state = "ERROR_RECOVERY" 

            elif current_state == "go to put basket":
                if self.navigate_by_wall(rear = 4.090, left=3.266, angle=0, align_wall="left"):
                    current_state = "turn for put basket"
                else:
                    current_state = "ERROR_RECOVERY" 

            elif current_state == "turn for put basket":
                if self.navigate_by_odometry(angle=-80):
                    self.navigate_by_wall(angle=0.0, align_wall="rear")
                    current_state = "back to put basket"
                else:
                    current_state = "ERROR_RECOVERY" 

            elif current_state == "back to put basket":
                if self.navigate_by_wall(rear=0.841, right = 4.090, angle = 0, align_wall="right"):
                    current_state = "grip up"
                else:
                    current_state = "ERROR_RECOVERY" 

            elif current_state == "grip up":
                if self.move_stepper_to(0):
                    current_state = "turn to s shape"
                else:
                    current_state = "ERROR_RECOVERY" 

            elif current_state == "turn to s shape":
                self.navigate_by_wall(rear = 1.8, angle = 0, align_wall = "right")
                if self.navigate_by_odometry(angle = 80):
                    self.navigate_by_wall(angle=0.0, align_wall="left")
                    current_state = "go to s shape"
                else:
                    current_state = "ERROR_RECOVERY" 
            
            elif current_state == "go to s shape":
                rospy.loginfo("All tasks completed successfully!")
                return True

            elif current_state == "ERROR_RECOVERY":
                rospy.logerr("A task failed. Entering error recovery mode.")
                return False
            
            rate.sleep()
            
        return False                

    def s_shape_contest(self):

        current_state = "NAV_AFTER_BASKET"
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo(f"====== Current State: {current_state} ======")

##########################################################################################
            if current_state == "NAV_AFTER_BASKET":
                # time.sleep(3)
                if self.navigate_by_wall(rear=4.54, angle=0.0, align_wall="left"):
                    current_state = "1"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "1":
                # time.sleep(3)
                if self.navigate_by_wall(left=1.77, angle=0.0, align_wall="left"):
                    current_state = "2"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "2":
                if self.navigate_by_wall(left = 1.81, rear=4.83, angle=0.0, align_wall="left"):
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
                if self.navigate_by_wall(right = 5.37, angle=0.0, align_wall="rear"):
                    current_state = "6"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "6":
                if self.navigate_by_wall(right = 5.37, front=1.04, angle=0.0, align_wall="rear"):
                    current_state = "7"
                else:
                    current_state = "ERROR_RECOVERY"
            

            elif current_state == "7":
                if self.navigate_by_wall(right = 5.39,front = 1.04, angle=0.0, align_wall="front"):
                    self.navigate_by_wall(right = 5.39,front = 1.04, angle=0.0, align_wall="front")
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
                if self.navigate_by_wall(angle=0.0, align_wall="rear"):
                    current_state = "13"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "13":
                if self.navigate_by_wall(left = 6.43, angle=0.0, align_wall="rear"):
                    current_state = "20"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "20":
                if self.navigate_by_wall(left = 6.45,front=0.99, angle=0.0, align_wall="front"):
                    self.navigate_by_wall(left = 6.45,front=0.99, angle=0.0, align_wall="front")
                    current_state = "21"
                else:
                    current_state = "ERROR_RECOVERY"
            
            elif current_state == "21":
                if self.navigate_by_wall(left=6.45, angle=0.0, align_wall="front"):
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
                if self.navigate_by_wall(left=0.48, angle=0.0,align_wall="left"):
                    current_state = "25"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "25":
                if self.navigate_by_wall(left = 0.48, rear=1.43, angle=0.0, align_wall="rear"):
                    current_state = "29"
                else:
                    current_state = "ERROR_RECOVERY"
            
            elif current_state == "29":
                rospy.loginfo("All tasks completed successfully!")
                break

            elif current_state == "ERROR_RECOVERY":
                rospy.logerr("A task failed. Entering error recovery mode.")
                break
            
            rate.sleep()
            
        return False



    def A_B_C_D_flow(self):
        current_state = "CROSS_BRIDGE"
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            if current_state == 'CROSS_BRIDGE':
                self.move_for_duration(linear_x=0.2, duration=3)

                result = self.follow_line_until_t_junction()
                if result == 'SUCCESS' or result == 'RECOVERED_FROM_LOSS':
                    current_state = 'CROSS_BRIDGE_DONE'
                else:
                    rospy.logerr(f"Failed to cross bridge. Reason: {result}")
                    current_state = 'ERROR_RECOVERY'

            elif current_state == "CROSS_BRIDGE_DONE":
                rospy.loginfo("Bridge crossing completed successfully!")
                if self.coffee_flow():
                    current_state = "COFFEE_FLOW_DONE"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "COFFEE_FLOW_DONE":
                rospy.loginfo("coffee delivery completed successfully!")
                if self.take_basket():
                    current_state = "BASKET_FLOW_DONE"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "BASKET_FLOW_DONE":
                if self.s_shape_contest():
                    current_state = "S_SHAPE_DONE"
                else:
                    current_state = "ERROR_RECOVERY"
            
            elif current_state == "S_SHAPE_DONE":
                rospy.loginfo("All tasks completed successfully!")
                break

            elif current_state == "ERROR_RECOVERY":
                rospy.logerr("A task failed. Entering error recovery mode.")
                break
                
            rate.sleep()

        
if __name__ == '__main__':
    try:
        MainController()
    except rospy.ROSInterruptException:
        pass
