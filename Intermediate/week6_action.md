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
# Writing an action server and client
# Composing multiple nodes in a single process
