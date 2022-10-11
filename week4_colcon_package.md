# Using colcon to build packages

## Prerequests:

Install colcon and ROS2:

```
sudo apt install python3-colcon-common-extensions
```

# Basics
## Create a workspace

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

At this point the workspace contains a single empty directory src:

```
.
└── src

1 directory, 0 files
```

## Add some sources

Let’s clone the examples repository into the src directory of the workspace:

```
sudo apt install git
git clone https://github.com/ros2/examples src/examples -b foxy
```

Now the workspace should have the source code to the ROS 2 examples:

```
.
└── src
    └── examples
        ├── CONTRIBUTING.md
        ├── LICENSE
        ├── rclcpp
        ├── rclpy
        └── README.md

4 directories, 3 files
```

## Build the workspace

```
colcon build --symlink-install
```

After the build is finished, we should see the build, install, and log directories:

```
.
├── build
├── install
├── log
└── src

4 directories, 0 files
```
## Run tests

To run tests for the packages we just built, run the following:

```
colcon test
```

## Source the environment

```
. install/setup.bash
```

# Try a demo

With the environment sourced we can run executables built by colcon. Let’s run a subscriber node from the examples:

```
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```
In another terminal, let’s run a publisher node (don’t forget to source the setup script):

```
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

You should see messages from the publisher and subscriber with numbers incrementing.

SUBSCRIBER TERMINAL

![image](https://user-images.githubusercontent.com/90166739/193399340-9e1c4a44-a259-4a04-b209-7754e90c7a63.png)

PUBLISHER TERMINAL

![image](https://user-images.githubusercontent.com/90166739/193399369-6a07a4b3-687c-4d82-9452-4befa3deb6f9.png)

# Create your own package
```
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/foxy/" >> ~/.bashrc
```

## Setup colcon tab completion

```
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```
