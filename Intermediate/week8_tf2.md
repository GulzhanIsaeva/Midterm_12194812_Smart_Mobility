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

## 1 Using view_frames

<view_frames> creates a diagram of the frames being broadcasted by tf2 over ROS.

```
ros2 run tf2_tools view_frames.py
```

You will see:

```
Listening to tf data during 5 seconds...
Generating graph in frames.pdf file...
```

Frames.pdf file:
![image](https://user-images.githubusercontent.com/90166739/196680687-4a09fdad-cd05-46a1-8161-853c900321d2.png)

