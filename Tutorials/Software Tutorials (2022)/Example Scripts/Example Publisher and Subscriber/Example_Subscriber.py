#!/usr/bin/env python3

import rospy	# Imports rospy, which provides all the basic ROS-based Python tools we need
import std_msgs.msg	# Imports the standard ROS message types

# Define function 'callback' that is called when the subscriber receives a message:
def callback(message):

	# For this simple case, just print the message received in Terminal
	rospy.loginfo("Message Received: %s",message.data)

if __name__ == '__main__':	# Write your main code in here!

	# Set up a new ROS node to act as the subscriber:
	rospy.init_node('Example_Subscriber')
	
	# Define the node to act as a subscriber to 'Example_Topic', calling 'callback' when a message is received:
	rospy.Subscriber('Example_Topic', std_msgs.msg.String, callback)
	
	rospy.spin()	# Simply prevents Python from exiting until this node is terminated
