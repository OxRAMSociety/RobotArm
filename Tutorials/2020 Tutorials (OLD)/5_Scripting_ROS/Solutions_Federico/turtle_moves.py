#!/usr/bin/env python

# Author: Federico Igne <git@federicoigne.com>
# Last changed: 07 Jan 2020
# License: This file is placed in the public domain.

import math
from math import radians, degrees

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Challenges:
#   - Move the turtle along a rectangular path given a starting point
#     and the width and height of the rectangle.
#   - Move the turtle along a circular path given the center and radius
#     of the circle.
#   - Make the turtle move forward and bounce off walls.

# Lessons learnt:
#   - if you want to move to a precise position, forget about it!
#   - if you really want to do it, the closer to the target you are, the
#     slower you should move (basically like in real life, who would
#     have thought!);
#   - tolerance for precise movements works best when proportional to
#     travelled distance;
#   - Interrupting the publishing of messages is not enough to stop the
#     turtle from moving. You need to publish a new message that sets
#     linear and angular velocity to 0.0. At the moment this doesn't
#     make much sense... I would expect the queues used in the
#     publish-subscribe paradigm to be FIFO, but this 'stopping' message
#     seems to cut the line.
#   - you can use `rosmsg info <message>` to get details about a
#     specific message. E.g., `rosmsg info turtlesim/Pose`
#   - this turtle is a very delicate snowflake...don't obsess about
#     minor errors/details.

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
    def distance_from(self, x, y):
        """Compute the distance from a given point on a plane."""
        return math.sqrt(pow(x - self.x, 2) + pow(y - self.y, 2))
    def set_speed(self, linear = .0, angular = .0):
        """Publish linear/angular velocity."""
        self.vel.linear.x = linear
        self.vel.angular.z = angular
        self.pub.publish(self.vel)
        self.rate.sleep()
    def stop(self):
        """Stop any movement."""
        self.set_speed()
    def turn(self, target):
        """Turn the turtle to `target` radians."""
        while abs(target - self.theta) > .01:
            self.set_speed(angular = target - self.theta)
        self.stop()
    def goto(self, x, y, tolerance = .015):
        """Move turtle to target (possibly with curvature)."""
        self.turn(math.atan2(y - self.y, x - self.x))
        # Compute distance from target. This is also used to modulate
        # linear velocity.
        distance = self.distance_from(x, y)
        # Scale tolerance w.r.t. travelling distance
        tolerance = distance * tolerance
        while distance > tolerance:
            # Slow down as you approach the target
            self.set_speed(linear = distance)
            # Update distance
            distance = self.distance_from(x, y)
        self.stop()
    def rectangle(self, x, y, width = 1., height = 1., angle = 0):
        """Move along a `width x height` rectangle (rotated by `angle` degrees)."""
        # Compute rectangle vertices.
        x1, y1 = x, y
        x2 = x1 + width * math.cos(radians(angle))
        y2 = y1 + width * math.sin(radians(angle))
        x3 = x2 + height * math.cos(radians(angle + 90))
        y3 = y2 + height * math.sin(radians(angle + 90))
        x4 = x3 + width * math.cos(radians(angle + 180))
        y4 = y3 + width * math.sin(radians(angle + 180))
        # Move!
        self.goto(x1, y1)
        self.goto(x2, y2)
        self.goto(x3, y3)
        self.goto(x4, y4)
        self.goto(x1, y1)
    def circle(self, x, y, radius = 1.):
        """Move in a circle with given center and radius."""
        # Go to position
        self.goto(x + radius, y)
        self.turn(math.pi / 2)
        # Take off from starting point.
        # NOTE: this is needed because we want to draw and exact circle
        # (i.e., starting and target position coincide).
        target_x, target_y = self.x, self.y
        distance = 0.1
        while distance < 0.5:
            # Keep linear and angular velocity in sync (w = v / r)
            self.set_speed(distance, distance / radius)
            # Update distance
            distance = self.distance_from(target_x, target_y)
        tolerance = 2 * math.pi * radius * 0.005
        while distance > tolerance:
            # Keep linear and angular velocity in sync (w = v / r)
            self.set_speed(distance, distance / radius)
            # Update distance
            distance = self.distance_from(target_x, target_y)
        self.stop()
    def bounce(self, deg):
        """Bounce like the DVD screensaver meme."""
        deg = deg % 360
        width, height = 11., 11.
        while True:
            # Determine where the turtle is facing
            if 0 <= deg and deg < 90:
                north, east = 1, 1
            elif 90 < deg and deg < 180:
                north, east = 1, 0
            elif 180 <= deg and deg < 270:
                north, east = 0, 0
            elif 270 < deg and deg < 360:
                north, east = 0, 1
            else:
                # Avoid angles for which tangent is not defined
                deg += 1; continue
            # Some trigonometry magic
            rad = math.radians(deg)
            target_x = width * east
            target_y = self.y + (width * east - self.x) * math.tan(rad)
            # Bouncing angle (symmetric on the Y axis)
            deg = (180 - deg) % 360
            if target_y < 0 or target_y > height:
                target_x = self.x + (height * north - self.y) / math.tan(rad)
                target_y = height * north
                # Bouncing angle (symmetric on the X axis)
                deg = (180 + deg) % 360
            self.goto(target_x, target_y)

turtle = Turtle()

# Rectangles

# turtle.rectangle(4., 7., 2., 3.)
# turtle.rectangle(2., 2., 7., 7., 10)
# turtle.rectangle(8.5, 9., 1., 1., 45)

# Circles

# turtle.circle(3., 5., 1.)
# turtle.circle(4., 5.2, 2.2)
# turtle.circle(5., 5.4, 3.4)
# turtle.circle(6., 5.6, 4.6)
# turtle.goto(3., 5.)

# Bounce

# turtle.bounce(30)

# Keep the main process running.
rospy.spin()

