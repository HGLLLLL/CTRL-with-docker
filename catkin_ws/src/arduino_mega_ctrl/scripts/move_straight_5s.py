#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

def main():
    # Initialize the node
    rospy.init_node('move_straight_5s', anonymous=True)
    
    # Create the publisher
    pub = rospy.Publisher('arduino_vel', Twist, queue_size=10)
    
    # Wait briefly to ensure the subscriber on the Arduino/ROS side is connected.
    rospy.loginfo("Waiting for publisher to establish connection...")
    rospy.sleep(1.0)
    
    # Create Twist message for moving forward
    twist_msg = Twist()
    twist_msg.linear.x = 0.3
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = 0.0
    
    duration = 10.0
    rate = rospy.Rate(10) # 10 Hz
    
    rospy.loginfo("Publishing continuously: linear.x = %.2f for %.1f seconds", twist_msg.linear.x, duration)
    
    # Wait for valid time
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() == 0:
        rate.sleep()
        
    start_time = rospy.Time.now()
    
    while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < duration:
        pub.publish(twist_msg)
        rate.sleep()
    
    # Create Twist message for stopping
    twist_msg.linear.x = 0.0
    
    # Publish the stop command a few times to ensure it is received
    rospy.loginfo("Publishing stop command: linear.x = 0.0")
    for _ in range(5):
        if rospy.is_shutdown():
            break
        pub.publish(twist_msg)
        rate.sleep()
    
    rospy.loginfo("Finished. Exiting node.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
