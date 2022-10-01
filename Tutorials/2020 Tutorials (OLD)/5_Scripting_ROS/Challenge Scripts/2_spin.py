#!/usr/bin/env python

import numpy as np
import rospy
import time

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

SPEED = 4
RADIUS = 2

x, y, theta = 0, 0, 0
def poseCallback(pose_message):
	global x, y, max_x, max_y, theta
	x, y, theta = pose_message.x, pose_message.y, pose_message.theta


if __name__ == "__main__":
	try:
		# Create this node
		rospy.init_node("move_around", anonymous=True)
		loop_rate = rospy.Rate(1000)

		# Create the subscribers and publishers
		pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
		velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
		
		# Set-up velocity message
		velocity_message = Twist()
		velocity_message.linear.x = RADIUS * SPEED
		velocity_message.angular.z = SPEED

		# Spin to change theta
		first_theta = theta
		while theta == first_theta:
			velocity_publisher.publish(velocity_message)
			loop_rate.sleep()
		
		# Store current and a previous theta value (+1 to make different from start_theta)
		start_theta = theta
		prev_theta = theta + 1
		
		# Move until returned to original position
		while not (prev_theta < start_theta and theta > start_theta):
			# Move
			velocity_publisher.publish(velocity_message)
			# Update previous theta
			prev_theta = theta
			# Sleep
			loop_rate.sleep()
		# Stop
		velocity_message.linear.x = 0
		velocity_message.angular.z = 0
		velocity_publisher.publish(velocity_message)

		
	except rospy.ROSInterruptException:
		pass
