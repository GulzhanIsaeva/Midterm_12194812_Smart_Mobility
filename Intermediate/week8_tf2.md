# Itroducing tf2

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

In the second terminal window type the following command:

```
ros2 run turtlesim turtle_teleop_key
```

Once the turtlesim is started you can drive the central turtle around in the turtlesim using the keyboard arrow keys, select the second terminal window so that your keystrokes will be captured to drive the turtle.


You can see that one turtle continuously moves to follow the turtle you are driving around
