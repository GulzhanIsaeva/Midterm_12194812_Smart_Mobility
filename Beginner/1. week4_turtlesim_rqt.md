# Turtlesim and RQT

Before starting a Turtlesim installation we have to make sure that the Terminal is in ROS2 DISTRO. (I am using ROS1 and ROS2 in Ubuntu 20.04 and I switch from one to another depending on a project)

In order to check your Terminal's path (ROS1 or ROS2?) run the code below:
```
printenv | grep -i ROS
```
(![image](https://user-images.githubusercontent.com/90166739/192125770-e67aeec2-e9ab-4964-b42e-5e7146f94200.png)


## 1. Install Turtlesim

After making sure that you are in ROS2 Distro, source your files and then install Turtlesim package

```
sudo update
sudo apt install ros-foxy-turtlesim
```
![image](https://user-images.githubusercontent.com/90166739/192125794-e39a7c30-5bfe-4ab2-8d17-f41f51256701.png)

In order to check list of installed packages:
```
ros2 pkg executables turtlesim
```
![image](https://user-images.githubusercontent.com/90166739/192125813-d8743326-93b2-40f3-ad10-dcda4e4655ac.png)

## 2. Start Turtlesim

To start turtlesim, enter the following command in your terminal:
```
ros2 run turtlesim turtlesim_node
```
The simulator window with a turtle will appear 

![image](https://user-images.githubusercontent.com/90166739/192125830-33874226-3ec8-459a-a721-a14fea4fe717.png)

In the terminal under the command, you will see messages from the node:
```
[INFO] [turtlesim]: Starting turtlesim with node name /turtlesim

[INFO] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
```
![image](https://user-images.githubusercontent.com/90166739/192125824-7754312b-211e-4c47-b174-fffdfefe9d15.png)

## 3. Use Turtlesim
1. Open a new terminal
2. Make sure you are in ROS2 Distro
```
printenv | grep -i ROS
```
![image](https://user-images.githubusercontent.com/90166739/192125836-7bc519cd-b82c-4579-b61a-e347e910b954.png)

3. Source your files
4. Run a new node to control the turtle in the first node:
```
ros2 run turtlesim turtle_teleop_key
```
![image](https://user-images.githubusercontent.com/90166739/192125841-41a9dc18-719b-477a-8863-9b17a921ae1e.png)

## 4. Install RQT
Open a new terminal to install rqt and its plugins:
```
sudo apt update
sudo apt install ~nros-foxy-rqt*
```

To run rqt:
```
rqt
```


## 5. Use RQT

Select: Plugins > Services > Service from the menu bar at the top as shown below

![image](https://user-images.githubusercontent.com/90166739/192125880-7a40144e-0e02-43aa-8262-ba427ecdb579.png)

Now click on the Service dropdown list to see turtlesim’s services, and select the /spawn service.

## 5.1. Try the spawn service

/spawn will create another turtle in the turtlesim window.

![image](https://user-images.githubusercontent.com/90166739/192126862-0aa7d652-06de-4fae-a6b6-f588edec73d3.png)


Give the new turtle new name, "New Turtle"

Enter new coordinates for the turtle to spawn at, like x = 2.0 and y = 2.0.

![rqt](https://user-images.githubusercontent.com/90166739/193396444-8c2bd961-2d9a-4835-9231-d0b39681df21.PNG)

## 5.2. Try the set_pen service

Now let’s give turtle1 a unique pen using the /set_pen service:

![image](https://user-images.githubusercontent.com/90166739/192127005-355dceca-5a94-4df2-9bfb-39ef642d6949.png)

To have turtle1 draw with a red line, change the value of r => 255, and the value of width => 5. 

![Capture](https://user-images.githubusercontent.com/90166739/193396721-2e8c9c7d-09cc-4292-9389-ceb039de7064.PNG)

If you return to the terminal where turtle_teleop_key is running and press the arrow keys, you will see turtle1’s pen has changed.

![image](https://user-images.githubusercontent.com/90166739/192127035-7c3b8aab-4df6-4e1b-ae68-0bca32b258be.png)



## 6. Remapping

In a new terminal, source ROS 2, and run:
```
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```

Now you can move turtle2 when this terminal is active, and turtle1 when the other terminal running the turtle_teleop_key is active.

![Capture1](https://user-images.githubusercontent.com/90166739/193396599-e6e2e1e5-51e2-4b36-b373-3c6c719490c0.PNG)

## 7. Close Turtlesim
To stop the simulation, you can enter Ctrl + C in the turtlesim_node terminal, and q in the teleop terminal.

