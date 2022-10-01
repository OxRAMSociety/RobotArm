#!/usr/bin/env python3

import rospy
import std_msgs.msg

def callback(message):

	rospy.loginfo("Square Number Received: %d \n ---",message.data)	# Display received square number


if __name__ == '__main__':

	rospy.init_node('Square_Number_Receiver')
	
	rospy.Subscriber('Square_Number_Processed', std_msgs.msg.Int16, callback) # Listen for processed orders
	
	rospy.spin()
