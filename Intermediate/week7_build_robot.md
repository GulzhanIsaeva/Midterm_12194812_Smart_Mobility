# Building a visual robot model from scratch

## One Shape

```
<?xml version="1.0"?>
<robot name="myfirst">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>
</robot>
```

This is a robot with the name myfirst, that contains only one link, whose visual component is just a cylinder 0.6 meters long with a 0.2 meter radius.
This is  “hello world” type example but in ROS

To examine the model, launch the display.launch.py file:

```
ros2 launch urdf_tutorial display.launch.py model:=urdf/01-myfirst.urdf
```

This does three things:

- Loads the specified model and saves it as a parameter

- Runs nodes to publish sensor_msgs/msg/JointState and transforms 

- Starts Rviz with a configuration file


A slightly modified argument allows this to work regardless of the current working directory:

```
ros2 launch urdf_tutorial display.launch.py model:=`ros2 pkg prefix --share urdf_tutorial`/urdf/01-myfirst.urdf
```

After launching display.launch.py, you should end up with RViz showing you the following:

![image](https://user-images.githubusercontent.com/90166739/195630589-558bf4a3-cd40-4f50-b344-8b0db12dce63.png)


## Multiple Shape

Now let’s look at how to add multiple shapes/links. If we just add more link elements to the urdf, the parser won’t know where to put them. So, we have to add joints. Joint elements can refer to both flexible and inflexible joints. 
```
<?xml version="1.0"?>
<robot name="multipleshapes">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
  </joint>

</robot>
```








