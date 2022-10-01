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

def theta2deg(theta_val):
	d = theta_val / np.pi
	if d > 0: return 180 * d
	else: return 180 * (2+d)

def rotate(rotation, speed=1, tolerance=0.5):
	# Convert theta to degrees and find target
	target = theta2deg(theta) + rotation
	if target > 360: target -= 360
	if target < 0: target += 360
	# Create velocity message
	velocity_message.linear.x = 0
	velocity_message.angular.z = speed if rotation > 0 else -speed
	# Move until reached target angle
	while abs(target - theta2deg(theta)) >= tolerance:
		# Move
		velocity_publisher.publish(velocity_message)
		# Sleep
		loop_rate.sleep()
	# Stop
	velocity_message.angular.z = 0
	velocity_publisher.publish(velocity_message)

def bounce():
	global last_bounce
	# Test if at an edge
	target_angle = None
	if ((x <= 0 and last_bounce != "left") or \
		(x >= 11 and last_bounce != "right")):
		# If on one of the vertical edges, flip the direction around vertical
		target_angle = np.pi-theta if theta > 0 else -np.pi-theta
		last_bounce = "left" if x <= 0 else "right"
	elif ((y <= 0 and last_bounce != "top") or \
		  (y >= 11 and last_bounce != "bottom")):
		# If on horizontal edges, flip around vertical then 180 degrees		
		target_angle = (np.pi-theta)+np.pi if theta > 0 else -np.pi-theta-np.pi
		last_bounce = "top" if y <= 0 else "bottom"

	# Rotate to target angle
	if target_angle:
		# Calculate difference between current and target angle, correct over
		angle_diff = target_angle - theta
		if angle_diff > np.pi: angle_diff -= 2*np.pi
		if angle_diff < -np.pi: angle_diff += 2*np.pi
		# Rotate this difference
		rotate(theta2deg(angle_diff), speed = 1)
	
if __name__ == "__main__":
	try:
		# Create this node
		rospy.init_node("move_around", anonymous=True)
		loop_rate = rospy.Rate(10000)

		# Create the subscribers and publishers
		pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
		velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

		# Set up velocity message
		speed = 5
		last_bounce = ""
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
