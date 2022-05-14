#!/usr/bin/env python3

import rospy
import std_msgs.msg

pub = rospy.Publisher('Square_Number_Processed', std_msgs.msg.Int16, queue_size=10)

def callback(message):

	rospy.loginfo("Order Received: %d",message.data)
	
	processed_order = message.data*message.data	# Process order (i.e. square it)
	
	rospy.loginfo("Processed Order: %d",processed_order)
	
	pub.publish(std_msgs.msg.Int16(processed_order))	# Publish processed order


if __name__ == '__main__':

	rospy.init_node('Square_Number_Factory')
	
	rospy.Subscriber('Square_Number_Order', std_msgs.msg.Int16, callback)	# Listen for orders
	
	rospy.spin()
