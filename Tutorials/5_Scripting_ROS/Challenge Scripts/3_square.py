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

def move_forward(target_dist, speed=2):
	# Setup move forward message
	velocity_message.linear.x = speed
	# Start time
	t0 = rospy.Time.now().to_sec()
	distance = 0
	while distance < target_dist:
		velocity_publisher.publish(velocity_message)
		t1 = rospy.Time.now().to_sec()
		distance = speed * (t1 - t0)

	velocity_message.linear.x = 0
	velocity_publisher.publish(velocity_message)	

def rotate(target_spin, angular_speed=1):
	# Create velocity message
	velocity_message.angular.z = angular_speed	
	# Get time and set angle_moved to 0	
	t0 = rospy.Time.now().to_sec()
	angle_moved = 0
	# Rotate until moved desired angle
	while angle_moved < target_spin:
		velocity_publisher.publish(velocity_message)
		t1 = rospy.Time.now().to_sec()
		angle_moved = angular_speed * (t1 - t0)
		#loop_rate.sleep()
	print(angle_moved/target_spin)
	# When moved, tell to stop moving
	velocity_message.angular.z = 0
	velocity_publisher.publish(velocity_message)
		
if __name__ == "__main__":
	try:
		# Create this node
		rospy.init_node("move_around", anonymous=True)
		loop_rate = rospy.Rate(1000)

		# Create the subscribers and publishers
		pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
		velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

		# Set up velocity message
		velocity_message = Twist()
		
		# Square procedure
		for _ in range(4):
			# Move forward
			move_forward(2)
			# Rotate 90 degress left
			rotate(np.pi/2)
		
	except rospy.ROSInterruptException:
		pass
