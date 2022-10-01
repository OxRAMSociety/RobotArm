#! /usr/bin/env python3

import rospy
from tutorial_scripts.srv import ExampleService, ExampleServiceResponse # Import message types for our Example ROS service

# Call requests are passed into this function
def handleRequest(req):
    x = req.input_number    # Extract number to be squared
    rospy.loginfo("Order Received: %d",x)
    x_squared = x*x         # Square the number
    rospy.loginfo("Processed Order: %d \n ---",x_squared)
    return ExampleServiceResponse(x_squared)    # Return square number to client


if __name__ == '__main__':

    rospy.init_node('ExampleServiceServerNode')

    # Declare a new service called 'ExampleService', of type ExampleService, using function handleRequest to handle requests
    srv = rospy.Service('ExampleService', ExampleService, handleRequest)

    rospy.spin()