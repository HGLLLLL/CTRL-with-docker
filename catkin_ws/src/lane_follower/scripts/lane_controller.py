#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class LaneController:
    def __init__(self):
        rospy.init_node('lane_controller', anonymous=True)
        
        # 控制參數 (P/D gains) - 可透過 ROS parameter 自行調校
        self.kp = rospy.get_param('~kp', 0.003)
        self.kd = rospy.get_param('~kd', 0.002)
        self.base_speed = rospy.get_param('~base_speed', 0.5) # 車子的預設線速度 (m/s)
        
        self.prev_offset = 0.0
        self.last_time = rospy.Time.now()
        
        # Publisher: 控制差動馬達 (geometry_msgs/Twist)
        self.cmd_pub = rospy.Publisher('arduino_vel', Twist, queue_size=10)
        
        # Subscriber: 接收車道辨識的 offset (std_msgs/Float32)
        self.offset_sub = rospy.Subscriber('lane_detect', Float32, self.offset_callback)
        
        rospy.loginfo("Lane Controller Node Started.")
        rospy.loginfo("Params -> Kp: %.4f, Kd: %.4f, Base_Speed: %.2f", self.kp, self.kd, self.base_speed)

    def offset_callback(self, msg):
        offset = msg.data
        
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        if dt <= 0:
            return
            
        # P 項目
        p_term = offset
        
        # D 項目
        d_term = (offset - self.prev_offset) / dt
        
        # 根據定義：offset 正 = 車偏右 -> 需左轉 -> angular.z 為正
        # 根據實驗可以調整正負號，這裡我們預設使用正的增益乘以 offset
        angular_z = (self.kp * p_term) + (self.kd * d_term)
        
        # 建立並填充 Twist 訊息
        twist = Twist()
        twist.linear.x = self.base_speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_z
        
        # 發布馬達控制指令
        self.cmd_pub.publish(twist)
        
        # 紀錄狀態供下一次 D 項目計算
        self.prev_offset = offset
        self.last_time = current_time

if __name__ == '__main__':
    try:
        LaneController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
