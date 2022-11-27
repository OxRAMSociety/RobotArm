#!/usr/bin/env python3

import rospy
import smach
import smach_ros


# Define a class for state Foo
class Foo(smach.State):
    def __init__(self): # This method runs when the state is first initialised
        # Initialise the state and define its possible outcomes:
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata): # This method runs when the state machine transitions to this state
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1' # The outcome returned by the method will determine the next state
        else:
            return 'outcome2'


# Define a class for state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome1'
        

if __name__ == '__main__':
    rospy.init_node('example_sm_simple')

    # Create a SMACH state machine, defining the possible outcomes
    sm = smach.StateMachine(outcomes=['end'])

    # Open the container
    with sm:
        # Add states to the container
        # Provide the name of the state, the class from which it is derived,
        # and the next state for each possible outcome
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 'outcome2':'end'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'})

    # Execute SMACH plan
    outcome = sm.execute()

    