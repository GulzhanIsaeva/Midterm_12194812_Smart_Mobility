# Creating Custom msg and srv files

## 1. Create a new package

In ros2_ws/src, source and run the following command to create a new package:

```
ros2 pkg create --build-type ament_cmake tutorial_interfaces
```

![image](https://user-images.githubusercontent.com/90166739/195014005-fbd48a48-e33e-4841-a4e2-60d788dd0c0f.png)


Create the directories in ros2_ws/src/tutorial_interfaces:

```
mkdir msg
mkdir srv
```

## 2. Create custom definitions
## 2.1 msg definition

In the tutorial_interfaces/msg directory, make a new file called Num.msg with one line of code declaring its data structure:

```
gedit Num.msg #creates and opens text file
```

```
int64 num
```

This is a custom message that transfers a single 64-bit integer called num.

Also in the tutorial_interfaces/msg directory, make a new file called Sphere.msg with the following content:

```
gedit Sphere.msg #creates and opens text file
```

```
geometry_msgs/Point center
float64 radius
```


3# 2.2 srv definition
Back in the tutorial_interfaces/srv directory, make a new file called AddThreeInts.srv with the following request and response structure:

```
gedit AddThreeInts.srv
```

```
int64 a
int64 b
int64 c
---
int64 sum
```

## 3. CMakeLists.txt

Add the following lines to CMakeLists.txt:

```
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```

## 4. package.xml

Add the following lines to package.xml

```
<depend>geometry_msgs</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## 5. Build the tutorial_interfaces package

In the root of your workspace (~/ros2_ws), run the following command:

```
colcon build --packages-select tutorial_interfaces
```

![image](https://user-images.githubusercontent.com/90166739/195014717-f15ad85d-cebd-4912-a1b9-eb314ae3ed8c.png)


## 6. Confirm msg and srv creation

In a new terminal, run the following command from within your workspace (ros2_ws) to source it:

```
. install/setup.bash
```

Now you can confirm that your interface creation worked by using the ros2 interface show command:

```
ros2 interface show tutorial_interfaces/msg/Num
```

should return:

```
int64 num
```
![image](https://user-images.githubusercontent.com/90166739/195014830-b9b28c0f-7aab-411e-910c-8ad884160949.png)

And

```
ros2 interface show tutorial_interfaces/msg/Sphere
```

should return:

```
geometry_msgs/Point center
float64 radius
```
![image](https://user-images.githubusercontent.com/90166739/195014944-f86b3c37-a2f1-4720-bb3f-73721d0a24f6.png)


And

```
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```

should return:

```
int64 a
int64 b
int64 c
---
int64 sum
```

![image](https://user-images.githubusercontent.com/90166739/195014998-29160318-2661-4f5a-a734-e6404517f064.png)

```
colcon build --packages-select cpp_pubsub
```
![image](https://user-images.githubusercontent.com/90166739/195020294-d8e6db5d-a261-471b-8361-5feba0255ad1.png)

## Publisher

![image](https://user-images.githubusercontent.com/90166739/195020766-32e60380-7c95-4b9e-82cb-9778d339019b.png)
![image](https://user-images.githubusercontent.com/90166739/195021114-e8170e5a-650c-42b3-8b59-1ad3c568b2ee.png)

## Subscriber

![image](https://user-images.githubusercontent.com/90166739/195020990-4652bdca-9379-4c14-a7cc-f9d337b31a6a.png)
