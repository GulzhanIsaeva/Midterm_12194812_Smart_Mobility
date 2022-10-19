Contents:

- Creating Launch Files
- Integrating Launch files into ROS2 packages
- Using Substitutions

# CREATING LAUNCH FILES

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

![image](https://user-images.githubusercontent.com/90166739/196483251-dbf68392-987b-4b95-894e-4bb4b33d1c9f.png)


```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [turtlesim_node-1]: process started with pid [11714]
[INFO] [turtlesim_node-2]: process started with pid [11715]
[INFO] [mimic-3]: process started with pid [11716]
```

![image](https://user-images.githubusercontent.com/90166739/196483105-e6be88ec-892b-43e2-b607-bb39ba605132.png)


To see the system in action, open a new terminal and run the ros2 topic pub command on the /turtlesim1/turtle1/cmd_vel topic to get the first turtle moving:

```
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```

You will see both turtles following the same path.

![image](https://user-images.githubusercontent.com/90166739/196485043-627110e9-4507-4a12-8475-d9055c42ce5c.png)



## 4. Introspect the system with rqt_graph

While the system is still running, open a new terminal and run rqt_graph to get a better idea of the relationship between the nodes in your launch file.

Run the command:

```
rqt_graph
```

![image](https://user-images.githubusercontent.com/90166739/196485163-7b2dc88c-5914-4035-94de-2685100c5a41.png)


A hidden node (the ros2 topic pub command you ran) is publishing data to the /turtlesim1/turtle1/cmd_vel topic on the left, which the /turtlesim1/sim node is subscribed to. The rest of the graph shows what was described earlier: mimic is subscribed to /turtlesim1/sim’s pose topic, and publishes to /turtlesim2/sim’s velocity command topic.

## 5. Summary

Launch files simplify running complex systems with many nodes and specific configuration details. You can create launch files using Python, XML, or YAML, and run them using the ros2 launch command.







...



##




# INTEGRATING LAUNCH FILES INTO ROS2 PACKAGES


## 1. Create a package

Create a workspace for the package to live in:

```
mkdir -p launch_ws/src
cd launch_ws/src
```

```
ros2 pkg create py_launch_example --build-type ament_python
```


## 2. Creating the structure to hold launch files

For Python packages, the directory containing your package should look like this:

```
src/
  py_launch_example/
    package.xml
    py_launch_example/
    resource/
    setup.py
    setup.cfg
    test/
```

In order for colcon to find the launch files, we need to inform Python’s setup tools of our launch files using the data_files parameter of setup.

Inside our setup.py file:

```
import os
from glob import glob
from setuptools import setup

package_name = 'py_launch_example'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```

## 3. Writing the Launch File

Inside your launch directory, create a new launch file called my_script_launch.py. _launch.py

Your launch file should define the generate_launch_description() function which returns a launch.LaunchDescription() to be used by the ros2 launch verb

```
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'),
  ])
```

## 4. Building and running the launch file

Go to the top-level of the workspace, and build it:

```
colcon build
```

After the colcon build has been successful and you’ve sourced the workspace, you should be able to run the launch file as follows:

```
ros2 launch py_launch_example my_script_launch.py
```



# USING SUBSTITUTIONS

## 1. Create and setup the package

Create a new package of build_type ament_python called launch_tutorial:

```
ros2 pkg create launch_tutorial --build-type ament_python
```

Inside of that package, create a directory called launch:

```
mkdir launch_tutorial/launch
```

Add in changes to the setup.py of the package so that the launch files will be installed:

```
import os
from glob import glob
from setuptools import setup

package_name = 'launch_tutorial'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```

## 2. Parent launch file

Create an example_main.launch.py file in the launch folder of the launch_tutorial package.

```
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'example_substitutions.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
```

## 3. Substitutions example launch file

Now create an example_substitutions.launch.py file in the same folder.

```
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle,
        change_background_r,
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])
```

## 4. Build the package

Go to the root of the workspace, and build the package:

```
colcon build
```


## Launching example

Now you can launch the example_main.launch.py file using the ros2 launch command.

```
ros2 launch launch_tutorial example_main.launch.py
```

This will do the following:

- Start a turtlesim node with a blue background

- Spawn the second turtle

- Change the color to purple

- Change the color to pink after two seconds if the provided background_r argument is 200 and use_provided_red argument is True

![photo_5366580397225394597_x](https://user-images.githubusercontent.com/90166739/196506331-069b3f54-c56a-45d4-8c21-652cb62920bd.jpg)
![photo_5366580397225394596_x](https://user-images.githubusercontent.com/90166739/196506373-7386948f-040f-4fc2-b808-fc88877a4652.jpg)


## Modifying launch arguments

If you want to change the provided launch arguments, you can either update them in launch_arguments dictionary in the example_main.launch.py

```
ros2 launch launch_tutorial example_substitutions.launch.py --show-args
```

This will show the arguments that may be given to the launch file and their default values.

```
Arguments (pass arguments as '<name>:=<value>'):

    'turtlesim_ns':
        no description given
        (default: 'turtlesim1')

    'use_provided_red':
        no description given
        (default: 'False')

    'new_background_r':
        no description given
        (default: '200')
```


Now you can pass the desired arguments to the launch file as follows:

```
ros2 launch launch_tutorial example_substitutions.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```

## Summary
In this tutorial, Ilearned about using substitutions in launch files and their possibilities and capabilities to create reusable launch files.


















