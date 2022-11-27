#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import random

import actionlib

from tutorial_scripts.srv import ExampleService, ExampleServiceRequest
from tutorial_scripts.msg import ExampleAction, ExampleGoal

        
if __name__ == '__main__':
    rospy.init_node('example_sm_ros')

    # Create a SMACH state machine, defining the possible outcomes
    sm = smach.StateMachine(['succeeded','aborted','preempted'])

    # Open the container
    with sm:
        # Add states to the container

        #  Example ROS Service Client State:
        def request_cb(userdata, request):  # This is called upon transitioning to the service client state
            request = ExampleServiceRequest()
            request.input_number = random.randint(1,3)
            return request  # This is used as the service request

        def response_cb(userdata, response):    # This is called upon receiving a response from the server
            userdata.service_response = response.output_number
            return 'succeeded'  # This is used as the state outcome

        # Initialise the state
        smach.StateMachine.add('ROS_SERVICE',
                                smach_ros.ServiceState('ExampleService',
                                ExampleService,
                                request_cb=request_cb,
                                response_cb=response_cb,
                                output_keys=['service_response']),
                                transitions={'succeeded' : 'ROS_ACTION'})


        #  Example ROS Action Client State:
        def goal_cb(userdata, goal): # This is called upon transitioning to the action client state
            goal = ExampleGoal()
            goal.seconds_requested = userdata.service_response
            return goal # This is used as the action goal

        def result_cb(userdata, status, result): # This is called upon receiving a result from the server
            if status == actionlib.GoalStatus.SUCCEEDED: # Check goal status
                return 'succeeded' # This is used as the state outcome
            elif status == actionlib.GoalStatus.PREEMPTED:
                return 'preempted'
            else:
                return 'aborted'
            
        # Initialise the state
        smach.StateMachine.add('ROS_ACTION',
                                smach_ros.SimpleActionState('example_as',
                                ExampleAction,
                                goal_cb=goal_cb,
                                result_cb=result_cb,
                                input_keys=['service_response']),
                                transitions={'succeeded' : 'succeeded',
                                'preempted' : 'preempted',
                                'aborted' : 'aborted'})

    # Initialise userdata
    sm.userdata.service_response = 0

    # Execute SMACH plan
    outcome = sm.execute()