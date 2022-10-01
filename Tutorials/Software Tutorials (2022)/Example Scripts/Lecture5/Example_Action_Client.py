#! /usr/bin/env python3

import rospy
import actionlib    # Required for setting up action clients
from tutorial_scripts.msg import ExampleAction, ExampleGoal # Import message types for our Example ROS action

# Feedback from the action server is handled by this function
def feedback_cb(msg):
    rospy.loginfo("Feedback Received: %s", msg)

# This function is run to call the action server
def call_server():

    # Declare an action client for action 'example_as' of type ExampleAction
    client = actionlib.SimpleActionClient('example_as', ExampleAction)

    # Wait for the corresponding action server to become available
    client.wait_for_server()

    # Instantiate a goal of type ExampleGoal
    goal = ExampleGoal()
    goal.seconds_requested = int(input("For how many seconds should I perform the action? (1<=t<=9): "))
    if goal.seconds_requested < 1:
        goal.seconds_requested = 1
    elif goal.seconds_requested > 9:
        goal.seconds_requested = 9

    # Send the goal to the action server, using the function feedback_cb to handle feedback
    client.send_goal(goal, feedback_cb=feedback_cb)

    # Wait for the result
    client.wait_for_result()

    # Retrieve the result once it is available
    result = client.get_result()

    return result

if __name__ == '__main__':

    try:
        rospy.init_node('example_action_client_node')
        result = call_server()
        rospy.loginfo("Result Received: %s", result)
    except rospy.ROSInterruptException as e:    # An exception will be raised if the request fails
        rospy.logerr('Something went wrong: %s', e)