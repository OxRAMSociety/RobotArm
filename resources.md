# Useful resources to get started
## Git
[This](https://github.com/javedali99/git-tutorial) git tutorial is good for getting an intuition about how git works. Although there is one important difference - instead of using forks (which are your "version" of the entire repository), we are using branches (a different "version" of the code, stored in the same repository). [This guide](https://github.com/git-guides) is a bit more information on them, as well as another decent explanation of how git works.

## ROS2
In this section, I will give you a brief explanation of how ROS2 works; once you realise that you need to use a particular feature, it is a good idea to get familiar with how it can be implemented through the [ROS2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html). Please use Python, not C++.

Please make sure that you are looking at "Jazzy" wiki pages, and not "Humble", "Noetic", "Rolling", etc. when looking at online resources.

And finally, I recommend having a look at existing code in the RobotArmPackages2 repository for examples for consistency in the way we implement things.

# A brief overview of ROS2
ROS2 provides a way to split code into packages and let these packages communicate with one another.

Please briefly read the glossary below so that you know what features are available in ROS2.

# Glossary

## Code structure
[node](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html): a separate chunk of code that can be run by itself. For example, a node interfacing with the camera and publishing its images onto a topic, a node reading camera information and publishing the location of the board, etc.

[package](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html): a chunk of code that is thematically related; contains multiple nodes; could be written by us, provided by ROS2 out-of-the-box or installed via rosdep

Workspace: corresponds to the RobotArmPackages2 repo; is a collection of packages (under `src/`) and instructions on how to combine them

## Communication between nodes

[topic](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html): a "channel" on which messages can be published by *publishers* and read by *subscribers*. Useful for e.g. sending streams of data between nodes, such as camera data. Especially useful if we need multiple subscribers per publisher (e.g. many nodes need the camera image). [Here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) is a code example. Some data, e.g. Images or numbers, has built-in message types; for custom data, you will need to [create your own](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html).

[service](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html): a way to request information from/run function from a different node. Use only with functions which compute quickly (e.g. update settings); use actions if the function takes a long time to complete. [Here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html) is a code example. You will most likely need to [create your own service format](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) to be able to send the data.

[action](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html): a way to call long-running function (e.g. telling motors to move to a position). Unlike services, provides continuous feedback to the caller and allows the caller to cancel the action. [Here](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html) is a code example. You will need to [create your own action](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Creating-an-Action.html)

[parameters](hhttps://docs.ros.org/en/jazzy/How-To-Guides/Node-arguments.htmlttps://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html): a way to customise behaviour of a node (e.g. selecting which camera to use). To [set the parameters](https://docs.ros.org/en/jazzy/How-To-Guides/Node-arguments.html) of the node when starting it, use `--ros-args -p param1:=value1 param2:=value2`

## Tools

[colcon](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html): a tool that is used to build and test packages.

[Launch files](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html): let you specify how to start multiple nodes at the same time, automatically. [Here](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html) is some more information.

[ros2](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html): the main tool used in ROS2; can be used to run nodes, look up information on packages/actions/topics/etc.

[rosdep](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html): a tool that manages automatically installing ROS2 packages from the internet

[rqt](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html): A graphical tool for working with ROS2; can do things like visualising messages on topics. You can e.g. [view logs](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html) or visualise topic messages.

[rviz](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html): A tool used to run 3D simulations of the robot
