#!/usr/bin/env python

import numpy as np
import rospy
import time

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

x, y, theta = 0, 0, 0
def poseCallback(pose_message):
	global x, y, max_x, max_y, theta
	x, y, theta = pose_message.x, pose_message.y, pose_message.theta

def distance(p1, p2):
	return np.sqrt((p1[0]-p2[0])**2 + (p1[1] - p2[1])**2)

def theta2deg(theta_val):
	d = theta_val / np.pi
	if d > 0: return 180 * d
	else: return 180 * (2+d)

def move_forward(target_dist, speed=2):
	# Setup move forward message
	velocity_message.linear.x = speed
	# Move until not reading 0, 0
	while x == 0 and y == 0:
		# Move and wait
		velocity_publisher.publish(velocity_message)
		loop_rate.sleep()
	# Get start position to move from 
	start_pos = (x, y)
	while distance(start_pos, (x, y)) < target_dist:
		# Move and wait
		velocity_publisher.publish(velocity_message)
		loop_rate.sleep()
	# Stop
	velocity_message.linear.x = 0
	velocity_publisher.publish(velocity_message)	

def rotate(rotation, speed=1, tolerance=0.2):
	# Create velocity message
	velocity_message.angular.z = speed if rotation > 0 else -speed
	# Convert theta to degrees and find target
	target = theta2deg(theta) + rotation
	if target > 360: target -= 360

	# Move until reached target angle
	while abs(target - theta2deg(theta)) > tolerance:
		# Move
		velocity_publisher.publish(velocity_message)
		# Sleep
		loop_rate.sleep()
	# Stop
	velocity_message.angular.z = 0
	velocity_publisher.publish(velocity_message)
		
if __name__ == "__main__":
	try:
		# Create this node
		rospy.init_node("move_around", anonymous=True)
		loop_rate = rospy.Rate(10000)

		# Create the subscribers and publishers
		pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
		velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

		# Set up velocity message
		velocity_message = Twist()
		
		# Square procedure
		for _ in range(4):
			# Move forward
			move_forward(3, speed=2)
			# Rotate 90 degress left
			rotate(-90, speed=2)
		
	except rospy.ROSInterruptException:
		pass
