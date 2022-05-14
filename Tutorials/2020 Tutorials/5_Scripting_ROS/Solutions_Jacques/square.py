#!/usr/bin/env python

import rospy
import numpy as np
import time

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


def rel_rotate(angle):
# Function to rotate turtle by a specified angle ACW
    loop_rate = rospy.Rate(10)
    velocity_publisher = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size = 10)
    velocity_message = Twist()

    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < angle):
        velocity_message.angular.z = angle*(2*np.pi)/360
        velocity_publisher.publish(velocity_message)
        t1 = rospy.Time.now().to_sec()
        current_angle = 90*(t1-t0)
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)




def rel_move(dist):
# Function to move turtle forward by a specified distance
    loop_rate = rospy.Rate(10)
    velocity_publisher = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size = 10)
    velocity_message = Twist()

    t0 = rospy.Time.now().to_sec()
    current_dist = 0

    while(current_dist < dist):
        velocity_message.linear.x = 5
        velocity_publisher.publish(velocity_message)
        t1 = rospy.Time.now().to_sec()
        current_dist = 5*(t1-t0)
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)



def square():

    global x, y, disp_pos
    disp_pos = 0    # Used to stop stream of position co-ords while awaiting inputs
    # Prompt user for inputs
    length = input("Input Side Length: ")
    start_angle = input("Input Rotation Amount: ")
    start_x = input("Input Centre X-Coordinate: ")
    start_y = input("Input Centre Y-Coordinate: ")
    disp_pos = 1
    # Move to centre of square
    rel_move(start_x-x)
    rel_rotate(90)
    rel_move(start_y-y)
    rel_rotate(90)
    rel_rotate(90)
    rel_rotate(90)
    # Draw square
    rel_rotate(start_angle)
    rel_move(length/2)
    rel_rotate(90)
    rel_move(length/2)
    k = 0
    while k < 4:
        rel_rotate(90)
        rel_move(length)
        k += 1




x,y,theta = 0,0,0
def poseCallback(pose_message):

    global x, y, theta, disp_pos
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta

    if disp_pos == 1:
        print(x,y,theta)




if __name__ == "__main__":
    try:
        rospy.init_node("move_forward", anonymous = True)
        pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
        square()
    except rospy.ROSInterruptException:
        pass
