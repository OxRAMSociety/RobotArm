#!/usr/bin/env python

import sys
import rospy
import time

from math import pi

import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose


# Setup ros node and commander
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moving_arm_livecode', anonymous=True)

# Setup moveit_commander essentials
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=20) 

# Get basic info
planning_frame = move_group.get_planning_frame()
eef_link = move_group.get_end_effector_link()
group_names = robot.get_group_names()
print(group_names)


# Move arm with a joint command
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -pi/4
joint_goal[2] = 0
joint_goal[3] = -pi/2
joint_goal[4] = 0

# Move to location
move_group.go(joint_goal, wait=True)
move_group.stop()

time.sleep(5)

# Move arm with a Pose command
pose_goal = Pose()
pose_goal.orientation.w = 1
pose_goal.position.x = 0.5
pose_goal.position.y = 0.5
pose_goal.position.z = 0.5
# Move to pose
move_group.set_pose_target(pose_goal)
plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()

