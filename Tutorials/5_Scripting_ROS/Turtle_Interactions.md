# Scripting the Turtle

Let's control this turtle! The way we do this, in a nutshell, is publishing data to the "cmd_vel" topic that the "turtlesim" creates which allows us to set the way it moves.

## 🐢 Topics: Listen to the Turtle 🐢
We previously discussed how ROS deals in "Topics", and how these are distributed and listened to by "Publishers" and "Subscribers" respectively. The "turtlesim" instance creates some topics when it's run, so let's start there.  
Activate the "turtlesim" by running in separate terminals (or tabs, personal perference):  
- `roscore`
- `rosrun turtlesim turtlesim_node` 
- `rosrun turtlesim turtle_teleop_key`  

and then spend a couple of minutes being distracted driving the turtle around. Once you're done with that, open up another terminal and run:  

`rostopic list`  

which will show you all the currently active topics. Focussing on the most important ones at the moment, you should see:  

`/turtle1/cmd_vel`  
`/turtle1/color_sensor`  
`/turtle1/pose`  

These are all the topics that are linked to the turtle. Briefly, the `pose` topic broadcasts the current position of the turtle, the `color_sensor` outputs the colour where the turtle in standing, and `cmd_vel` is the command the turtle reads to determine it's velocity. This last one is the one we want to modify.  

## 🐍 Python: Talking to the Turtle 🐍

Let's get the turtle to do something without using the `teleop_key` script. This requires a bit of work up front, and will show us the basics of the `rospy` interface between (you guessed it) ROS and Python.

Navigate to the package folder we made before (`cd ~/catkin_ws/src/turtle_scripts`) and make a new Python file (.py) with whichever editor you're comfortable with. I recommend VSCode in general for a broad range of features including syntax highlighting and code-completion for Python. However, as long as you can write and save text any editor will do.  

### 1. Twist and Shout! What do we want the turtle to do? 
ROS enablees the passing of many different message types (topics) between nodes. For this example, we will be creating a `Twist` message to match the format of `cmd_vel`. The Twist message type contains a `linear` and `angular` component in 3 dimensions (`x`, `y`, and `z`), but we will only care about `x` and `y` as the `turtlesim` is 2D. Generating a message of this type can be done like this in Python.

```python
from geometry_msgs.msg import Twist

velocity_message = Twist()
velocity_message.linear.x = 5
```
we import the `Twist` object from the module `geometry_msgs`, create a new instance of it and assign the linear component to 5 and as such it'll move forward 5 units.

### 2. Becoming a Publisher

We need to create a script that will (a) create a recognised publisher of a ROS topic, and (b) output our message on this topic to make the turtle move how we want it. Being a publisher is achieved through the `rospy` commands:  

```python
import rospy 

loop_rate = rospy.Rate(10)
velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
```

In addition to creating a "publisher" node we created a `loop_rate` at which it outputs values. Ensuring the data is published at a reliable frequency allows us to send messages consistently, without overloading the ROS Core node. The `queue_size` parameter instructs ROS to maintain a memory of the last `10` published results, as it is possible for a Subscriber node to miss previous messages and need to catch up on them (i.e. a Subscribers processing time may be logner than the time between messages, and this has to be accomondated for.)  

As a simple example, let's combine our knowledge into a function to move the turtle forward for a set amount of time, called `move()`

```python
#!/usr/bin/env python

import rospy
import time

from geometry_msgs.msg import Twist

def move(speed, duration=5):
    # Create the publisher
    loop_rate = rospy.Rate(10)
    velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    # Create the forward velocity message
    velocity_message = Twist()
    velocity_message.linear.x = speed
    # Publish the message for "duration" seconds
    start_time = time.time()
    while (time.time() - start_time) < duration:
        # Log that this action was taken
        rospy.loginfo("Turtle goes forward!")
        # Publish
        velocity_publisher.publish(velocity_message)
        # Sleep to enforce the message rate
        loop_rate.sleep()
```

Finally, we have to a make this script license this code as a node then run the move command when called. We use a `try/except` condition to make sure that if something interrups the ROS running in the background our code doesn't fall over in the process. 

```python
if __name__ == "__main__":
    try: 
        rospy.init_node("move_forward", anonymous=True)
        move(2)
    except rospy.ROSInterruptException:
        pass
```

Great! Combine these two together into a single script, I called mine `move_turtle.py` and it should look like this:  

```python
#!/usr/bin/env python

import rospy
import time

from geometry_msgs.msg import Twist

def move(speed, duration=5):
    # Create the publisher
    loop_rate = rospy.Rate(10)
    velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    # Create the forward velocity message
    velocity_message = Twist()
    velocity_message.linear.x = speed
    # Publish the message for "duration" seconds
    start_time = time.time()
    while (time.time() - start_time) < duration:
        # Log that this action was taken
        rospy.loginfo("Turtle goes forward!")
        # Publish
        velocity_publisher.publish(velocity_message)
        # Sleep to enforce the message rate
        loop_rate.sleep()

if __name__ == "__main__":
    try: 
        rospy.init_node("move_forward", anonymous=True)
        move(2)
    except rospy.ROSInterruptException:
        pass
```

You must then provide it with running permissions by:  
`chmod a+x move_turtle.py`  

### 3. Activating Script Control (aka running the ROS script)
To see the fuits of your labours, open up your terminals as you would normally when runnning the turtle: 

`roscore`  
`rosrun turtlesim turtlesim_node`  
`rosrun turtlesim turtle_teleop_key` (optional) 

then to give your code control by running:  

`rosrun turtle_scripts move_turtle.py` (replacing names with your own)  

and watch it plow headfirst into a wall! These are humble beginnings, but with these tools (and with one more bit of instrution about litening with **Subscribers**) we're well on our way to knowing all of the essentials for control a ROS robot.

## 4. Listening with a Subscriber
Listening to the outputs from a Topic can be achieved with a `ropsy.Subscriber`, which is designed to listen into a topic and run a function the user defines when the value changes. In our code, we created a function to extract the `x`, `y`, and `theta` values, save them to global variables, and then print them to the console. The changes to the code can be seen here: 

```python
#!/usr/bin/env python

import rospy
import time

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


x, y, theta = 0, 0, 0
def poseCallback(pose_message):
    global x, y, theta
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta
    print(x, y, theta)


def move(speed, duration=5):
    # Create the publisher
    loop_rate = rospy.Rate(10)
    velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    # Create the forward velocity message
    velocity_message = Twist()
    velocity_message.linear.x = speed
    # Publish the message for "duration" seconds
    start_time = time.time()
    while (time.time() - start_time) < duration:
        # Log that this action was taken
        rospy.loginfo("Turtle goes forward!")
        # Publish
        velocity_publisher.publish(velocity_message)
        # Sleep to enforce the message rate
        loop_rate.sleep()

if __name__ == "__main__":
    try: 
        rospy.init_node("move_forward", anonymous=True)
        pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
        move(2)
        # optional, spin the node to keep printing the pose info
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
```
