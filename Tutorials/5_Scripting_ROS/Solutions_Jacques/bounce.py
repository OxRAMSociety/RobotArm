#!/usr/bin/env python

import rospy
import time

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


x,y,theta = 0,0,0
def poseCallback(pose_message):
    global x, y, theta, disp_pos
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta
    if disp_pos == 1:
        print(x,y,theta)


def move():
# Function used here to cause movement
    global x, y, disp_pos
    loop_rate = rospy.Rate(10)
    velocity_publisher = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size = 10)
    velocity_message = Twist()

    disp_pos = 0    # Used to stop stream of position co-ords while awaiting inputs
    # Prompt user for inputs
    vel_x0 = input("Input X-Velocity: ")
    vel_y0 = input("Input Y-Velocity: ")
    duration = input("Input Duration (seconds): ")
    disp_pos = 1

    vel_x = vel_x0
    vel_y = vel_y0

    start_time = time.time()
    while time.time() - start_time < duration:
    # If the turtle is in contact with a wall and is trying to move through it, change the sign of the relevant velocity
        if x > 11.088889 and vel_x > 0:
            vel_x = -vel_x
        if x < 0.001 and vel_x < 0:
            vel_x = -vel_x
        if y > 11.088889 and vel_y > 0:
            vel_y = -vel_y
        if y < 0.001 and vel_y < 0:
            vel_y = -vel_y

        velocity_message.linear.x = vel_x
        velocity_message.linear.y = vel_y
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()



if __name__ == "__main__":
    try:
        rospy.init_node("move_forward", anonymous = True)
        pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
        move()
    except rospy.ROSInterruptException:
        pass
