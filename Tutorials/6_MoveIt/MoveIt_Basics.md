# MoveIt!

Moveit is an open-source software package designed to make moving robots easy and reproducible. We're using it here to learn how to move a robot arm, using a series of tutorials produced by the MoveIt developers. 

## 💽 Installation 💽

Key instructions for installation on ROS Melodic can be found here:  
http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html,  
and modifications for ROS Noetic are here: 
https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html.

## 📖 Main Tutorial 📖

The main tutorial scripts were added during the install process, so to run the tutorial simply follow the instructions here:
http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
  
## 📦 Package Installation Notes 📦

When you're adding a new package to this workspace, you'll be required to use catkin to add and then make the workspace again. Adding the package can be done in the same way as before, with the catkin_create_pkg command, and ensure that the `rospy std_msgs moveit_commander moveit_msgs geometry_msgs` packages are included in the command. 

Once that has run, you will need to navigate back out to the `src` folder and run oen of the `catkin` commands to make the workspace. This has behaved inconsistently accross different machines, but you can use either of:  

`catkin build`  
`catkin_make`  

however be aware that if you're using `catkin_make` on the tutorial packages you may need to first clear the existing `build` and `devel` folders using: `rm -rf <folder_name>`.

## 🐍 Basic MoveIt Python Script 🐍

I've included a basic script called `arm_move.py` which should placed in the `src` foler of your new package to run. For it to be visible to `rosrun` you will need to make sure it's executable by running: 

`chmod a+x <filename>`  

and making sure you've source the workspace since your `catkin` build. If you added the `source ` command to your bashrc file, the quickest way to resource is to reopen your terminal. Once that's done, you can call this script using the usual `rosrun` invocation:

`rosrun <your_package_name> arm_move`  

N.B: you will need to have the demo code to activate RViz running too, invoked with: 

`roslaunch panda_moveit_config demo.launch`

## 🦾 Using the Moveo Arm 🦾

To get the moveo files, I would recommend cloning this github repo into your Documents

`git clone https://github.com/jesseweisberg/moveo_ros.git`  

then copy and paste the

`moveo_moveit_config`  
`moveo_urdf`  

files into your active moveit workspace `src` folder.  

Then, use whichever version of `catkin_make` or `catkin build` you have been using to remake your moveit workspace.  

To run the arm, instead of the previous `demo.launch` commands, instead run:

`roslaunch moveo_moveit_config demo.launch`

and then use `rosrun` with the corresponding `arm_move` script to move it! (No pun intended!)  


There's a lot for us to explore here, but this week's challenges are:

1. Joint Limits: Find out the limits of each of the Moveo arm joints
2. 3D Stretch: Make the arm full extend, including gripper position. 
3. Box Spin: Put you arm at one extreme stretch, create and pick up box, then place it down after rotating 180 degrees. 