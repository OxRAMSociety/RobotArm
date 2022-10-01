#! /usr/bin/env python3

import rospy
import random
from tutorial_scripts.srv import ExampleService # Import message types for our Example ROS service

rospy.init_node('ExampleServiceClientNode')

r = rospy.Rate(2)	# 2 Hz


while not rospy.is_shutdown():

    order = random.randint(1,10)	# Randomly generate a new order

    rospy.loginfo("Number to be Squared: %d",order)
    rospy.wait_for_service('ExampleService')    # Wait for 'ExampleService' to become available

    try:
        request = rospy.ServiceProxy('ExampleService', ExampleService)  # Create a handle by which to call the service
        receiver = request(order)   # Call the service, using our randomly-generated order as an input
        rospy.loginfo("Square Number Received: %d \n ---",receiver.output_number)

    except rospy.ServiceException as e: # An exception will be raised if the call fails
        rospy.logerr("Service call failed: %s \n ---",e)

    r.sleep()