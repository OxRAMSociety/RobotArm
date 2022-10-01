#! /usr/bin/env python3

import rospy
import actionlib    # Required for setting up action servers
from tutorial_scripts.msg import ExampleAction, ExampleFeedback, ExampleResult  # Import message types for our Example ROS action

# Our action server will be implemented as an object; a complex data structure with its own internal data and methods.
# Classes are the blueprints for these objects, and define these data and methods.
# This is the basis for Object-Oriented Programming (OOP).

# For classes in Python:
# -All data variables must be prefixed by 'self.'
# -Definitions of all methods must have 'self' as the first input, and calls of these methods must instead be prefixed by 'self.'
# -To access these data and methods outside the class structure, replace the prefix 'self.' with '{name of object}.'

class ExampleActionServer():

    # This function is run when the constructor for the class is called, and is used to initialise a new object of that class
    def __init__(self):
            
        # Declare an action server, called 'example_as', of type ExampleAction, using function execute_cb to handle requests
        self.a_server = actionlib.SimpleActionServer("example_as", ExampleAction, execute_cb=self.execute_cb, auto_start=False)
        # Start the action server
        self.a_server.start()

    # This function is used to handle goals requested by an action client
    def execute_cb(self, goal):

        feedback = ExampleFeedback()    # Instantiate feedback variable (of type ExampleFeedback)
        result = ExampleResult()    # Instantiate result variable (of type ExampleResult)
        result.success = True
        rate = rospy.Rate(1)

        rospy.loginfo("Action Request Received (%s) - Performing Action...",goal)

        for i in range(0, goal.seconds_requested):
            # Check whether the client has cancelled the request
            if self.a_server.is_preempt_requested():
                self.a_server.set_preempted()
                result.success = False
                break

            feedback.seconds_elapsed = i
            self.a_server.publish_feedback(feedback)    # Return feedback to the client
            rate.sleep()
            i += 1

        if result.success:  # Action successful
            feedback.seconds_elapsed += 1
            rospy.loginfo("SUCCESS - Action Complete after %s seconds \n ---",feedback.seconds_elapsed)
            self.a_server.set_succeeded(result) # Return the result (success) to the client
        else:   # Action failed (e.g. cancelled by client)
            rospy.logerr("FAILURE - Action Aborted!")
            self.a_server.set_aborted(result)   # Return the result (failure) to the client



if __name__ == "__main__":
    rospy.init_node("example_action_server_node")
    # Call the constructor for our class, instantiating it as object 's'
    s = ExampleActionServer()
    rospy.spin()