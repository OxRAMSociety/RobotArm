#!/usr/bin/env python3

import rospy
import std_msgs.msg
import random	# Used for random number generation

pub = rospy.Publisher('Square_Number_Order', std_msgs.msg.Int16, queue_size=10)

rospy.init_node('Square_Number_Customer')

r = rospy.Rate(2)	# 2 Hz


while not rospy.is_shutdown():
	
	order = random.randint(1,10)	# Randomly generate a new order
	
	rospy.loginfo("Number to be Squared: %d",order)
	
	pub.publish(std_msgs.msg.Int16(order))	# Publish order
	
	r.sleep()
