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
    # Otherwise, publishing immediately might result in a lost message.
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
    
    # Publish the forward command once
    rospy.loginfo("Publishing single command: linear.x = 0.5")
    pub.publish(twist_msg)
    
    # Wait for exactly 5 seconds
    rospy.loginfo("Sleeping for 2 seconds...")
    rospy.sleep(10.0)
    
    # Create Twist message for stopping
    twist_msg.linear.x = 0.0
    
    # Publish the stop command
    rospy.loginfo("Publishing single command: linear.x = 0.0 (Stop)")
    pub.publish(twist_msg)
    
    # Wait a tiny bit to ensure the message is transmitted over the ROS network
    # before the node is killed and sockets are closed.
    rospy.sleep(0.5)
    
    rospy.loginfo("Finished. Exiting node.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
