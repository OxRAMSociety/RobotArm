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
	

def rotate(target_spin, angular_speed=1, clockwise = True):
	# Create velocity message
	velocity_message.angular.z = angular_speed if clockwise else -angular_speed	
	# Get time and set angle_moved to 0	
	t0 = rospy.Time.now().to_sec()
	angle_moved = 0
	# Rotate until moved desired angle
	while angle_moved < target_spin:
		velocity_publisher.publish(velocity_message)
		t1 = rospy.Time.now().to_sec()
		angle_moved = angular_speed * (t1 - t0)
	# When moved, tell to stop moving
	velocity_message.angular.z = 0
	velocity_publisher.publish(velocity_message)
	
def bounce():
	global theta
	# Test if at an edge
	target_angle = None
	if x <= 0 or x >= 11:
		# If on one of the vertical edges, flip the direction around vertical
		target_angle = np.pi-theta if theta > 0 else -np.pi-theta
	elif y <= 0 or y >= 11:
		# If on horizontal edges, flip around vertical then 180 degrees		
		target_angle = (np.pi-theta)+np.pi if theta > 0 else -np.pi-theta-np.pi
		
	# Rotate to target angle
	if target_angle:
		# Slow down
		velocity_message.linear.x = max(0.5, velocity_message.linear.x/5)
		# Calculate difference between current and target angle, correct over
		angle_diff = target_angle - theta
		if angle_diff > np.pi: angle_diff -= 2*np.pi
		if angle_diff < -np.pi: angle_diff += 2*np.pi
		# Work out if turning clockwise/anticlockwise
		cws = True if angle_diff > 0 else False
		# Take the absolute of the angle difference 
		angle_diff = angle_diff if angle_diff > 0 else -angle_diff
		# Rotate this difference, in the chosen direction
		rotate(angle_diff, angular_speed = angle_diff, clockwise=cws)
	
if __name__ == "__main__":
	try:
		# Create this node
		rospy.init_node("move_around", anonymous=True)
		loop_rate = rospy.Rate(1000)

		# Create the subscribers and publishers
		pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
		velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

		# Set up velocity message
		speed = 5
		velocity_message = Twist()
		velocity_message.linear.x = speed
		time.sleep(1)
		# Run with while loop, for "duration" seconds
		duration = 30
		t0 = rospy.Time.now().to_sec()
		while rospy.Time.now().to_sec() - t0 < duration:
			bounce()
			# Publish message
			velocity_message.linear.x = speed
			velocity_publisher.publish(velocity_message)
		
	except rospy.ROSInterruptException:
		pass
