# Introducing tf2

The goal is to run a turtlesim demo and see some of the power of tf2 in a multi-robot example using turtlesim.

## Installing the demo

Install the demo packages and it's dependancies 

```
sudo apt-get install ros-foxy-turtle-tf2-py ros-foxy-tf2-tools ros-foxy-tf-transformations
```

## Running the demo

After installing the <turtle_tf2_py> tutorial package, open a new terminal and source your ROS 2 installation. Then run the following command:

```
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py
```

You will see the turtlesim start with two turtles.

![image](https://user-images.githubusercontent.com/90166739/196679697-622725af-2eeb-4bc1-893d-0d7111fd97e3.png)


In the second terminal window type the following command:

```
ros2 run turtlesim turtle_teleop_key
```

You can see that one turtle continuously moves to follow the turtle you are driving around

![image](https://user-images.githubusercontent.com/90166739/196679807-fcb91334-951a-4cf1-8f84-bb7c681e18f2.png)

## How is it working?

This demo is using the tf2 library to create three coordinate frames: a world frame, a turtle1 frame, and a turtle2 frame. This tutorial uses a tf2 broadcaster to publish the turtle coordinate frames and a tf2 listener to compute the difference in the turtle frames and move one turtle to follow the other.

## How tf2 tools are used in this demo?

###### 1 Using view_frames

<view_frames> creates a diagram of the frames being broadcasted by tf2 over ROS.

```
ros2 run tf2_tools view_frames.py
```

You will see:

```
Listening to tf data during 5 seconds...
Generating graph in frames.pdf file...
```

![image](https://user-images.githubusercontent.com/90166739/196683955-833e4b61-c4bc-4c78-b8fe-c5a9e77f2bb6.png)


Frames.pdf file:
![image](https://user-images.githubusercontent.com/90166739/196680687-4a09fdad-cd05-46a1-8161-853c900321d2.png)

###### 2 Using tf2_echo

tf2_echo reports the transform between any two frames broadcasted over ROS.

Usage:

```
ros2 run tf2_ros tf2_echo [reference_frame] [target_frame]
```

Let’s look at the transform of the turtle2 frame with respect to turtle1 frame which is equivalent to:

```
ros2 run tf2_ros tf2_echo turtle2 turtle1
```

You will see the transform displayed as the tf2_echo listener receives the frames broadcasted over ROS2.

```
At time 1622031731.625364060
- Translation: [2.796, 1.039, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.202, 0.979]
At time 1622031732.614745114
- Translation: [1.608, 0.250, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.032, 0.999]
```
![image](https://user-images.githubusercontent.com/90166739/196685037-281eb218-a0d3-4062-8d1a-ab4c51f2b587.png)

As you drive your turtle around you will see the transform change as the two turtles move relative to each other.


## rviz and tf2

rviz is a visualization tool that is useful for examining tf2 frames
Start rviz with the turtle_rviz.rviz configuration file using the -d option:

```
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz
```

![image](https://user-images.githubusercontent.com/90166739/196685129-f74b6dac-4628-486f-aa55-17bf56907778.png)

As you drive the turtle around you will see the frames move in rviz.




