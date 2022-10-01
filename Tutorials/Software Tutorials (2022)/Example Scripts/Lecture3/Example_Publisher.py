#!/usr/bin/env python3

import rospy	# Imports rospy, which provides all the basic ROS-based Python tools we need
import std_msgs.msg	# Imports the standard ROS message types

# Define a publisher to a new topic 'Example_Topic', which can take Strings as messages:
pub = rospy.Publisher('Example_Topic', std_msgs.msg.String, queue_size=10)

# Note: queue_size determines the max. number of messages that can wait in the 'queue' to be published before any further additions are discarded (until space is made in the queue)

# Set up a new ROS node to act as the publisher:
rospy.init_node('Example_Publisher')

# Define the rate at which the publisher publishes messages:
r = rospy.Rate(1)	# 1 Hz


while not rospy.is_shutdown():	# While this node has not been terminated:

	# Instantiate the message variable "Example_Message"
	message = std_msgs.msg.String("Example_Message")

	# Publish the message to Example_Topic
	pub.publish(message)

	# Print the message published
	rospy.loginfo("Message Published: %s \n ---",message.data)
	
	# Sleep for an amount of time determined by the publishing frequency (1 second here)
	r.sleep()
