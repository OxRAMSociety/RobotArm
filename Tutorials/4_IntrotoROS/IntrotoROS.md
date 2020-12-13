The instructions here are complementary to this presentation: https://docs.google.com/presentation/d/1Qumn36j2WhzlIH6uK6YIfPuLTGeUQtcqjGAUU_t7FPc/edit#slide=id.ga15f7c0449_0_1726
## ROS Installation 
### ROS Installation Instructions
The ROS website has really nice instructions for installing ROS. This might take a while as ROS is a pretty hefty software. 
a) Open up Ubuntu and open up terminal.
b) Follow these instructions:  http://wiki.ros.org/melodic/Installation/Ubuntu
### Test ROS Installation
To test to see if ROS has been installed, we will be running a basic program. Open 3 separate terminals and run: 
```roscore``` (terminal 1) 
```rosrun turtlesim turtlesim_node``` (terminal 2) 
```rosrun turtlesim turtle_teleop_key``` (terminal 3) 
From terminal 3 you should be able to drive a little turtle around. You have just written your first ROS program!  
### Creating a catkin workspace 
The catkin structure simplifies the build and installation processes for ROS packages so its useful to have a catkin workspace. At a time you only have one active catkin workspace.
1) ```source /opt/ros/melodic/setup.bash```
2) ```mkdir -p ~/arm_ws/src```
3) ```cd ~/arm_ws/```
4) ```catkin_make``` (catkin make is a nice tool for working with a catkin workspace) 
You should now see a ```src```,```devel```,```build``` folder.

We now want to tell ROS that this catkin workspace is the workspace we want to activate. To do this run: 
1) ```source devel/setup.bash```

 
## Getting to know ROS 
### Packages
#### Creating a ROS package 
Follow these instructions: http://wiki.ros.org/ROS/Tutorials/CreatingPackage
#### Installing a ROS package 
```sudo apt install ros-melodic-<package-name>```
#### Useful Commands   
1) ```rospack list```: lists all packages installed in your ROS (this would list like 180+ packages)
2) ```rospack find <package-name>```: find the directory of a single package (can use to see if package exists) 
   For example: ```rospack find turtlesim``` should output something like ```/opt/ros/melodic/share/turtlesim```
3) ```rosls <package-name>```: lists contents of package directory 
4) ```roscd <package-name>```: switch to the directory of the specified package 
### Master 
To run the master, open a new terminal window and run the command: 
```roscore``` 
When you ran the turtlesim, this is why you ran this code. 
## Optional Installations/Commands to run
a) **Tmux**: ROS often involves you to open many terminal windows. This can be annoying to work sometimes so Tmux helps you split your current terminal into many terminal windows. 

Follow this link to install: https://linuxize.com/post/getting-started-with-tmux/

b) **Adding lines to bashrc**: You might have noticed that you have had to repeat the same commands over and over again when you open a new terminal. This can be tedious so to avoid that what you can do is modify the bashrc file. This file is run every time you open a new terminal automatically so any commands in this file will be run as well. 

For example, say we want to **source the melodic environmental files** every time we open a new terminal:
1) Type in command line  ```nano ~/.bashrc``` (should open a text editor) 
2) Scroll all the way to the bottom of the file and paste/type the following line: ```source /opt/ros/melodic/setup.bash```


