#!/usr/bin/env python

import rospy


# from dlv_plate_ctrl.s_shape_motor import s_shape

from geometry_msgs.msg import Twist
from time import sleep

def A_to_C_motor():
    # Initialize the ROS node
    # rospy.init_node('move_robot_node', anonymous=True)
    
    # Create a publisher for the 'dlv/cmd_vel' topic
    velocity_publisher = rospy.Publisher('dlv/cmd_vel', Twist, queue_size=10)
    
    # Set the loop rate (10 Hz)
    rate = rospy.Rate(10)
    
    # Create a Twist message
    vel_msg = Twist()

    def move_forward(duration, speed_x, angular_z):
        vel_msg.linear.x = speed_x  # Move forward
        vel_msg.angular.z = angular_z  # Set rotation speed
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            velocity_publisher.publish(vel_msg)
            rate.sleep()

    def left_move(duration, speed_x, speed_y, z):
        vel_msg.linear.y = speed_y  # Move forward
        vel_msg.linear.x = speed_x
        vel_msg.angular.z = z  # No rotation
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            velocity_publisher.publish(vel_msg)
            rate.sleep()   

    def turn_left(duration, speed):
        vel_msg.linear.x = 0.0  # No forward movement
        vel_msg.angular.z = -speed  # Turn left
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            velocity_publisher.publish(vel_msg)
            rate.sleep()

    def turn_right(duration, speed):
        vel_msg.linear.x = 0.0  # No forward movement
        vel_msg.angular.z = speed  # Turn right
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            velocity_publisher.publish(vel_msg)
            rate.sleep()

    def stop_robot():
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.angular.z = 0.0
        velocity_publisher.publish(vel_msg)



    def A_to_C():
        left_move(6, 0.3, 0.08, 0)  # (duration, x, y, z)
        stop_robot()
        sleep(1)                 # 1-second delay

        # left_move(5.8,-0.6,0.6)  # Move forward for 5 seconds at 0.15 m/s
        # stop_robot()
        # sleep(1)  




        # move_forward(5.7, 0.3, -0.634)  # Move forward for 7.4 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1) 

        # turn_left(6.2, 0.5)
        # stop_robot()
        # sleep(1)

        # move_forward(2, 1, 0)  # Move forward for 7.4 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1)

        # left_move(2, 0.7)  # Move forward for 7.4 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1)  

        # left_move(2, -0.7)  # Move forward for 7.4 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1)      
        
        # move_forward(2, -0.5, 0)  # Move forward for 7.4 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1) 

        # left_move(2, 0.5)  # Move forward for 7.4 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1)    

        # left_move(6, -0.25)  # Move forward for 7.4 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1)  

        # left_move(6, 0.25)  # Move forward for 7.4 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1)  

        # left_move(3, -0.25)  # Move forward for 7.4 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1)  

        # left_move(3, 0.25)  # Move forward for 7.4 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1)  

        # move_forward(2.5, 0.5)  # Move forward for 7.4 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1)        

        # move_forward(2.5, -0.5)  # Move forward for 7.4 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1)      

        # move_forward(2.5, 0.5)  # Move forward for 7.4 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1)        

        # move_forward(2.5, -0.5)  # Move forward for 7.4 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1)        


        # turn_left(3.09, 0.5)     # Turn left for 3.09 seconds at 0.5 rad/s
        # stop_robot()
        # sleep(1)                 # 1-second delay

        # move_forward(10.2, 0.25) # Move forward for 10.2 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1)                 # 1-second delay

        # turn_right(3.09, 0.5)    # Turn right for 3.09 seconds at 0.5 rad/s
        # stop_robot()
        # sleep(1)                 # 1-second delay

        # move_forward(6.1, 0.25)  # Move forward for 6.1 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1)                 # 1-second delay

        # turn_right(3.09, 0.5)    # Turn right for 3.09 seconds at 0.5 rad/s
        # stop_robot()
        # sleep(1)                 # 1-second delay

        # move_forward(10.2, 0.25) # Move forward for 10.2 seconds at 0.25 m/s
        # stop_robot()
        # sleep(1)                 # 1-second delay

        # turn_left(3.09, 0.5)     # Turn left for 3.09 seconds at 0.5 rad/s
        # stop_robot()
        # sleep(1)                 # 1-second delay

        # move_forward(10.4, 0.25) # Move forward for 10.4 seconds at 0.25 m/s
        # stop_robot()


    A_to_C()


def main():
    rospy.init_node('main_control')
    
    A_to_C_motor()


if __name__ == '__main__':
    main()