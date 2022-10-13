Contents:

- Creating Launch Files
- Integrating LAunch files into ROS2 packages

# Creating Launch Files

## 1. Setup

Create a new directory to store your launch files:

```
mkdir launch
```

## 2. Write the launch file

Put together a ROS 2 launch file using the turtlesim package and its executables.
Copy and paste the complete code into the launch/turtlesim_mimic_launch.py file:

```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

## 3. ROS2 Launch

To run the launch file created above, enter into the directory you created earlier and run the following command:

```
cd launch
ros2 launch turtlesim_mimic_launch.py
```

Two turtlesim windows will open, and you will see the following [INFO] messages telling you which nodes your launch file has started:

```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [turtlesim_node-1]: process started with pid [11714]
[INFO] [turtlesim_node-2]: process started with pid [11715]
[INFO] [mimic-3]: process started with pid [11716]
```

To see the system in action, open a new terminal and run the ros2 topic pub command on the /turtlesim1/turtle1/cmd_vel topic to get the first turtle moving:

```
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```

You will see both turtles following the same path.

![image](https://user-images.githubusercontent.com/90166739/195622841-4720dbba-ea01-4bed-b6dc-a3ded02076d7.png)



## 4. Introspect the system with rqt_graph
While the system is still running, open a new terminal and run rqt_graph to get a better idea of the relationship between the nodes in your launch file.

Run the command:

```
rqt_graph
```

![image](https://user-images.githubusercontent.com/90166739/195623086-2f567e96-36ca-4c2e-afec-40f5ab50d538.png)

##
A hidden node (the ros2 topic pub command you ran) is publishing data to the /turtlesim1/turtle1/cmd_vel topic on the left, which the /turtlesim1/sim node is subscribed to. The rest of the graph shows what was described earlier: mimic is subscribed to /turtlesim1/sim’s pose topic, and publishes to /turtlesim2/sim’s velocity command topic.

# Summary
Launch files simplify running complex systems with many nodes and specific configuration details. You can create launch files using Python, XML, or YAML, and run them using the ros2 launch command.
