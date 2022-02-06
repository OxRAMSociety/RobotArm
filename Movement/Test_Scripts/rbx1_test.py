#!/usr/bin/env python3

# This script acts as a test for basic functions by which to control the arm

import sys
import rospy
import time


import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from math import pi
import math
import geometry_msgs.msg
from geometry_msgs.msg import Pose


# Setup ros node and commander
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moving_arm_livecode', anonymous=True)


# Setup moveit_commander essentials
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm = moveit_commander.MoveGroupCommander("rbx1_arm")
arm_frame = arm.get_planning_frame()
gripper = moveit_commander.MoveGroupCommander("rbx1_gripper")
gripper_frame = gripper.get_planning_frame()
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size = 20)

# Get basic info
eef_link = arm.get_end_effector_link()
planning_frame = arm.get_planning_frame()
group_names = robot.get_group_names()

# Define Functions

def query_acc_vel():
	# Prompts the user for the max. acceleration and velocity scaling factors
	temp = input("Max. Acceleration Scaling Factor? (0<a<=1): ")
	arm.set_max_acceleration_scaling_factor(temp)
	temp = input("Max. Velocity Scaling Factor? (0<v<=1): ")
	arm.set_max_velocity_scaling_factor(temp)



def check(box_is_known=False,box_is_attached=False,timeout=4):
	# Checks that box correctly attaches or detaches
	global box_name
	start = rospy.get_time()
	seconds = rospy.get_time()
	while (seconds - start < timeout) and not rospy.is_shutdown():
		# Test if the box is in attached objects
		attached_objects = scene.get_attached_objects([box_name])
		is_attached = len(attached_objects.keys()) > 0

		# Test if the box is in the scene.
		# Note that attaching the box will remove it from known_objects
		is_known = box_name in scene.get_known_object_names()

		# Test if we are in the expected state
		if (box_is_attached == is_attached) and (box_is_known == is_known):
			return True
			

		# Sleep so that we give other threads time on the processor
		rospy.sleep(0.1)
		seconds = rospy.get_time()

	# If we exited the while loop without returning then we timed out
	return False



def jmove(j1,j2,j3,j4,j5,j6):
	# Sets Joint Goals for a given set of inputs (in degrees) and moves accordingly
	joint_goal = arm.get_current_joint_values()
	joint_goal[0] = math.radians(j1)
	joint_goal[1] = math.radians(j2)
	joint_goal[2] = math.radians(j3)
	joint_goal[3] = math.radians(j4)
	joint_goal[4] = math.radians(j5)
	joint_goal[5] = math.radians(j6)
	print(joint_goal)
	arm.set_joint_value_target(joint_goal)

	plan = arm.plan()
	arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()



def pmove(coords,rotation_vector,rotation_angle):
	# Sets a Pose Goal for a given set of inputs (in Cartesian coords.) and moves accordingly
	# The orientation of the EEF is defined in quarternions, which I have translated into a 3D vector (that gets normalised) which acts as an axis about which to rotate, and an angle by which to rotate about this axis
	# I have found a lower bound for the locus of points to which the arm can always move, described by the following inequality:
	#				~~~ x^2 + (y + 0.2425)^2 + (z - 0.1864)^2 <= 0.19717 ~~~
	# If it's not working, check that your destination is achievable!
	global eef_link
	pose_goal = geometry_msgs.msg.Pose()
	print(pose_goal)
	magnitude = (rotation_vector[0]**2 + rotation_vector[1]**2 + rotation_vector[2]**2)**(1/2)
	pose_goal.position.x = coords[0]
	pose_goal.position.y = coords[1]
	pose_goal.position.z = coords[2]
	pose_goal.orientation.x = math.sin(math.radians(rotation_angle)/2)*rotation_vector[0]/magnitude
	pose_goal.orientation.y = math.sin(math.radians(rotation_angle)/2)*rotation_vector[1]/magnitude
	pose_goal.orientation.z = math.sin(math.radians(rotation_angle)/2)*rotation_vector[2]/magnitude
	pose_goal.orientation.w = math.cos(math.radians(rotation_angle)/2)
	arm.set_pose_target(pose_goal, end_effector_link = eef_link)
	print(pose_goal)
	
	plan = arm.plan()
	arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()



def pmovesimple(coords):
	# Sets a Position Goal for a given set of inputs (in Cartesian coords.) and moves accordingly
	global eef_link
	arm.set_position_target(coords, end_effector_link = eef_link)
	
	plan = arm.plan()
	arm.go(wait=True)
	arm.stop()
	arm.clear_pose_targets()



def spawn_box(scene, name, position, rotation):
	# Spawns the box into the scene
	rospy.sleep(0.5)
	box_pose = geometry_msgs.msg.PoseStamped()
	box_pose.header.frame_id = gripper_frame
	box_pose.pose.position.x = position[0]
	box_pose.pose.position.y = position[1]
	box_pose.pose.position.z = position[2]
	box_pose.pose.orientation.x = rotation[0]
	box_pose.pose.orientation.y = rotation[1]
	box_pose.pose.orientation.z = rotation[2]
	box_pose.pose.orientation.w = rotation[3]
	scene.add_box(name, box_pose, size=(0.08, 0.08, 0.08))
	check(box_is_known=True)



def attach_box(scene, robot, group, name):
	# Attaches the box to the gripper
	global eef_link
	touch_links = robot.get_link_names(group = group.get_name())
	scene.attach_box(eef_link, name, touch_links=touch_links)
	check(box_is_attached=True,box_is_known=False)



def detach_box(scene, name):
	# Detaches the box from the gripper
	global eef_link
	scene.remove_attached_object(eef_link, name)
	check(box_is_known=True,box_is_attached=False)



def remove_box(scene,name):
	# Removes the box from the scene
	scene.remove_world_object(name)
	check(box_is_attached=False,box_is_known=False)


box_name = "box1"
#query_acc_vel()


#print "============ Reference frame: %s" % planning_frame

#print "============ End effector: %s" % eef_link

#print "============ Robot Groups:", robot.get_group_names()

#print "============ Printing robot state"
#print robot.get_current_state()
#print ""

jmove(0,0,0,0,0,0)
jmove(0,45,45,0,-45,0)
#pmovesimple((0, 0.15, 0.1))
#spawn_box(scene, box_name, (0.4, 0.4, 0.4),(0.0, 0.0, , ))
#remove_box(scene, box_name)
pmove((0,0.15,0.1),(0,0,1),-90)
pmove((0.1,0.15,0.1),(0,0,1),-90)
pmove((0.1,0.05,0.1),(0,0,1),-90)
pmove((0,0.05,0.1),(0,0,1),-90)
pmove((0,0.15,0.1),(0,0,1),-90)
