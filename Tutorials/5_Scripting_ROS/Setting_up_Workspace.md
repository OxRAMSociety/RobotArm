# Setting up our Workspace

In this session, we'll get ROS setup and write a basic script to interact with the "turtlesim" program  
  

## 🏢  Set-up a Catkin Workspace 🏢  
This will be the home directory in which you'll put all the files needed to interface with ROS and specifically "turtlesim". This is broken down nicely in the [IntrotoROS.md](https://github.com/JELAshford/OxRAM-Robot-Arm/blob/main/Tutorials/4_IntrotoROS/IntrotoROS.md) document that Sneha provided, but I'll cover the basic steps here for convenience. This is assuming that you're working on the Virtual Machine which we set up. If you're not, I think you can skip this step (LOOK THIS UP!).

1. Use `source` to activate the current installation of ROS  
    `source /opt/ros/melodic/setup.bash`
2. Create the folder ("workspace") to store all your projects ("packages").  
    `mkdir -p ~/catkin_ws/src`
3. Navigate into this workspace to start making  
    `cd ~/catkin_ws`
4. Run the `catkin_make` tool to create the workspace, should be left with `src`, `devel`, and `build` subfolders.

You've got a workspace - now what? We want to makes sure that this workspace is where our computer knows to run ROS from, and to run ROS for us everytime. We will modify our `.bashrc` file, as this is run every time you start a new terminal.

5. Navigate back to the home directory with `cd`
6. Open up the file `.bashrc` with your editor of choice. I recommend running  
 `gedit .bashrc`  
 to open the file in "Text Editor" from the terminal. 
7. There's a lot here! All you need to do is add the `source` command for the setup.bash file in our new catkin workspace. This is as simple as adding:  
    `source ~/catkin_ws/devel/setup.bash`  
    to a newline on the end of the file and saving/closing. 
8. To test, close the terminal and reopen. Then run `roscd` to check that it takes you to the  `devel` folder

Great! That's the workspace setup. Now let's make our first "package" which is where our scripts will be stored 

## 📦 Creating a Catkin Package 📦
In ROS (and programming in general) a "package" usually contains scripts with a common focus or usage. While many packages you can access for ROS primarily focus on a single job (e.g. Processing Images, Sensor Data, or GPS) you make your own "package" to do testing and creating scripts with no pressure for it to be all coherent or for public use. Let's make one for interacting with the turtlesim.

1. Navigate to our `src` directory, with `cd ~/catkin_ws/src`
2. Run the `catkin_create_pkg` command with our package **name** and **required packages**.  
    `catkin_create_pkg turtle_scripts std_msgs rospy`
3. Once done use `cd ..` to return to the `catkin_ws` and run  
    `catkin_make`  
    You can think of this a telling catkin to take a look around the files, and build and packages that have been added since this was last run.  

We now how have a **workspace** within which we have created a **package** which will contain our code. So now comes the question of interacting with the turtlesim.
