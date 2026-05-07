#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

def arduino_vel_publisher():
    # 初始化 ROS 節點，名稱為 arduino_vel_publisher
    rospy.init_node('arduino_vel_publisher', anonymous=True)
    
    # 建立一個 Publisher，發送 Topic 名稱為 'arduino_vel'，訊息格式為 geometry_msgs/Twist
    pub = rospy.Publisher('arduino_vel', Twist, queue_size=10)
    
    # 設定發送頻率為 10 Hz
    rate = rospy.Rate(10)
    rospy.loginfo("Arduino Velocity Publisher Started. Publishing to '/arduino_vel'...")

    while not rospy.is_shutdown():
        # 建立訊息物件
        vel_msg = Twist()
        
        # --- 在這裡撰寫您想要發送給 Arduino 的邏輯 ---
        # 範例：給定線速度(x)與角速度(z)
        vel_msg.linear.x = 0.5   # m/s
        vel_msg.angular.z = 0.1  # rad/s
        
        # 顯示發送出的訊息 (可選)
        # rospy.loginfo(f"Sending velocity - linear.x: {vel_msg.linear.x}, angular.z: {vel_msg.angular.z}")
        
        # 發送訊息
        pub.publish(vel_msg)
        
        # 依照設定的頻率暫停
        rate.sleep()

if __name__ == '__main__':
    try:
        arduino_vel_publisher()
    except rospy.ROSInterruptException:
        pass
