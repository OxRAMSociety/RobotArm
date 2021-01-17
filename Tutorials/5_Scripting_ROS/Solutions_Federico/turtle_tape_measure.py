#!/usr/bin/env python

# Author: Federico Igne <git@federicoigne.com>
# Last changed: 07 Jan 2020
# License: This file is placed in the public domain.

import math
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Challenge:
#   - Mesure the size of arena communicating with the turtle bot.

class Turtle:
    """A simple wrapper for the turtle bot."""
    def __init__(self):
        # Initialise node (to communicate with other nodes)
        rospy.init_node("turtle", anonymous = True)
        # Current pose
        self.x = .0
        self.y = .0
        self.theta = .0
        self.valid = False
        # Velocity message
        self.vel = Twist()
        # '/turtle1/pose' subscriber
        self.sub = rospy.Subscriber("/turtle1/pose", Pose, self.update_pose)
        # '/turtle1/cmd_vel' publisher
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        # Limit message rate
        self.rate = rospy.Rate(10)
        # During initialization the current turtle pose is not to be
        # considered valid until the first message from '/turtle1/pose'
        # is received.
        while not self.valid:
            pass
    def update_pose(self, msg):
        """Callback function for '/turtle1/pose' subscriber."""
        self.x, self.y, self.theta = msg.x, msg.y, msg.theta
        self.valid = True
    def set_speed(self, linear = .0, angular = .0):
        self.vel.linear.x = linear
        self.vel.angular.z = angular
        self.pub.publish(self.vel)
        self.rate.sleep()
    def stop(self):
        """Stop any movement setting velocity to 0."""
        self.set_speed()
    def turn(self, target):
        """Turn the turtle to `target` radians."""
        while abs(target - self.theta) > .01:
            self.set_speed(0., target - self.theta)
        self.stop()

turtle = Turtle()

width, height = -1, -1

# Measuring width
turtle.turn(0)
while width != turtle.x:
    width = width * .05 + turtle.x * .95
    turtle.set_speed(5)
turtle.stop()

# Measuring height
turtle.turn(math.pi / 2)
while height != turtle.y:
    height = height * .05 + turtle.y * .95
    turtle.set_speed(5)
turtle.stop()

print("Arena size:", round(width, 2), "x", round(height, 2))

# Keep the main process running.
rospy.spin()

