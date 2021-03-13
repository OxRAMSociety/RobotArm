## Sending ROS messages from Non-ROS Computer 
In our final product, we would want ROS to be run on the robot through Raspberry Pi but do all the computations on a computer that may not have ROS. To do this, we use ROSbridge which connects the non-ROS computer with the ROS computer through the web. 

### Nomenclature 
1) non-ROS machine is just your normal computer that will typically run Windows or Mac OS
2) ROS machine in our case will be your Linux machine (either Virtualbox or Raspberry Pi). This will be the machine on the robot. 


### How to use ROSbridge with VirtualBox
1) Go to virtual box settings and change the 'attached to' parameter under the 'Adapter 1' tab to 'Bridged Adapter' 
2) Follow these instructions to install rosbridge in your ROS machine: 
[rosbridge_suite/Tutorials/RunningRosbridge - ROS Wiki](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge#Talking_to_Rosbridge)
3) Download the 'simple.html' file from [roslibjs examples lib](https://github.com/RobotWebTools/roslibjs/tree/develop/examples) 
on your non-ROS machine and apply the following changes:
a) Change line 6 to from ```<script  src="../build/roslib.js"></script>``` to ```<script src="http://static.robotwebtools.org/roslibjs/current/roslib.min.js"></script>```
b) Find the IP address of your ROS machine by typing the command ```ifconfig``` in the terminal. Look for inet (https://miro.medium.com/max/3750/1*-pqdlPP5T6q_SHSs89ieUg.png)
c) Change line 11 from 
``` var ros = new ROSLIB.Ros();``` to ``` var ros = new ROSLIB.Ros({url : 'ws://IP_ADDRESS:9090'});```. 
   For example, for my machine I used ``` var ros = new ROSLIB.Ros({
    url : 'ws://10.40.28.15:9090'});```. 
4) Scroll to the ***running example*** part of this wiki [roslibjs/Tutorials/BasicRosFunctionality - ROS Wiki](http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality) and follow the instructions on your ROS machine. 
5) On your non-ROS machine, open up ```simple.html``` file on your browser: 
a) On Chrome: [How to Open an HTML File in Google Chrome (helpdeskgeek.com)](https://helpdeskgeek.com/how-to/open-an-html-file-in-google-chrome/#:~:text=Open%20HTML%20File%20From%20Within%20Chrome&text=Choose%20File%20from%20the%20Chrome,open%20in%20a%20new%20tab.)
b) On Firefox: [How do I open a file with the Firefox browser? | Firefox Support Forum | Mozilla Support](https://support.mozilla.org/en-US/questions/948602#:~:text=Chosen%20solution&text=Now%2C%20open%20your%20file%20explorer,launch%20and%20show%20your%20webpage.)
6) You should see ***connected*** appear on your browser page. If you don't you can find the error log by opening a webconsole ([how to open a webconsole](https://webmasters.stackexchange.com/questions/8525/how-do-i-open-the-javascript-console-in-different-browsers)). Also note that a twist message has been sent to ```\cmd_vel``` topic on your ROS machine. 


