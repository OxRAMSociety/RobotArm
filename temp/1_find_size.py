#!/usr/bin/env python

import rospy
import time

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

x, y, max_x, max_y, theta = 0, 0, 0, 0, 0
def poseCallback(pose_message):
	global x, y, max_x, max_y, theta
	x, y, theta = pose_message.x, pose_message.y, pose_message.theta
	# Update max previous position
	max_x = max([max_x, x])
	max_y = max([max_y, y])


if __name__ == "__main__":
	try:
		# Create this node
		rospy.init_node("move_around", anonymous=True)
		
		# Create the subscribers and publishers
		pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
		loop_rate = rospy.Rate(10)
		velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
		
		# Spiral outwards
		start_time = time.time()
		# Set-up velocity message
		velocity_message = Twist()
		velocity_message.linear.x = 1
		velocity_message.angular.z = 1.1
		# Loop for the set time
		while (time.time() - start_time) < 5:
			# Log action taken
			rospy.loginfo("Turtle stepped")
			# Update velocity message
			velocity_message.linear.x += 1
			# Pubish
			velocity_publisher.publish(velocity_message)
			# Sleep to enforce loop rate
			loop_rate.sleep()
		print(max_x, max_y)

	except rospy.ROSInterruptException:
		pass
