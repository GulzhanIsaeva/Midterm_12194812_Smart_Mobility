# Managing Dependencies with rosdep

## What is rosdep?

**rosdep** is ROS’s dependency management utility that can work with ROS packages and external libraries. rosdep is a command-line utility for identifying and installing dependencies to build or install a package. 

## How does rosdep work?

**rosdep** will check for package.xml files in its path or for a specific package and find the rosdep keys stored within. Once the packages are found, they are installed and ready to go!

## How do I use the rosdep tool?

Firstly, we must be initialize **rosdep**

```
sudo rosdep init
rosdep update
```

This will initialize rosdep and update will update the locally cached rosdistro index.

Now, we can run rosdep install to install dependencies

```
rosdep install --from-paths src -y --ignore-src
```

##
##

# Creating an action

- Source your ROS2 installations
- Go to ros2_ws/src workspace
- Create a package named **action_tutorials_interfaces**

```
cd ros2_ws/src
ros2 pkg create action_tutorials_interfaces
```

## Defining an action

Actions are defined in .action files of the form:
```
# Request
---
# Result
---
# Feedback
```

An action definition is made up of three message definitions separated by ---.


- A request message is sent from an action client to an action server initiating a new goal.
- A result message is sent from an action server to an action client when a goal is done.
- Feedback messages are periodically sent from an action server to an action client with updates about a goal.

-

Create an action directory in action_tutorials_interfaces:

```
cd action_tutorials_interfaces
mkdir action
```

In the action directory, create a file called Fibonacci.action:

```
int32 order # goal request
---
int32[] sequence # final result
---
int32[] partial_sequence # feedback
```

## Building an action

In order to use the new Fibonacci action type in our code, we should add lines of code in CMakeLists.txt before the ament_package() line, in the action_tutorials_interfaces:

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```

We should also add the required dependencies to our package.xml:

```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

We should now be able to build the package containing the Fibonacci action definition:

```
# Change to the root of the workspace
cd ~/ros2_ws
# Build
colcon build
```

We can check that our action built successfully with the command line tool:

```
# Source our workspace
# On Windows: call install/setup.bat
. install/setup.bash
# Check that our action definition exists
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```

You should see the Fibonacci action definition printed to the screen.

## Summary

In this tutorial, I learned the structure of an action definition and also how to correctly build a new action interface using CMakeLists.txt and package.xml, and how to verify a successful build.


##
##

# Writing an action server and client
# Composing multiple nodes in a single process
