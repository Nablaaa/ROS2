# Building a Robot with Unified Robot Description Format (URDF)

The goal for robotics is to have a working robot in real world. But before, it is good to simulate the robot in GAZEBO. The Gazebo simulation environment needs a robot model (written in URDF). Here, I will describe, how to set this up.

## Getting started
Set up an URDF file with
```bash
touch my_robot_name.urdf
```
and write the following minimal code in the file (the file is [here](urdf/my_robot.urdf)):
```xml
<?xml version="1.0"?>
<robot name="my_robot_name">
</robot>
```

## Add Links
[ROS Documentation](https://wiki.ros.org/urdf/XML/link) <br>
Here I can use CAD software to create bars or existing shapes like boxes, cylinders, spheres. E.g. A box of 0.6 m x 0.4 m x 0.2 m:
```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.6 0.4 0.2"/>
    </geometry>
    <origin xyz="0 0 0.1" rpy="0 0 0" />
  </visual>
</link>
```
The first link has per convention the name `base_link`. The geometry part defines the shape of the link. The origin part defines the position and orientation of the link. The `rpy` (roll, pitch, yaw) is the rotation around the x, y and z axis. The origin has an offset of 0.1 m in z direction (because otherwise the box would be symmetrically half in the ground like in unity when I create a new object).

## Visualization
This just works exactly like in [ROS Visualization](docs/Ros_visualization_tools.md), by typing
```bash
ros2 launch urdf_tutorial display.launch.py model:=/home/eric/Desktop/GitHub/ros2_ws/urdf/my_robot.urdf
```
The visualization looks like this:
![URDF](/docs/media/box_URDF.png)

## Basic shapes
The basic shapes are:
- Box
- Cylinder
- Sphere
- custom meshes from Blender (STL) or other CAD software

and the code for them is:
```xml
<geometry>
  <box size="0.6 0.4 0.2"/>
</geometry>

<geometry>
  <cylinder radius="0.1" length="0.2"/>
</geometry>

<geometry>
  <sphere radius="0.1"/>
</geometry>
```

## Colors
Calers can be added as material tags. First I have to define the material tag and than I have to refer in the visual part to this material. The definition of the material happens normally after giving the robot a name and before giving the link a name.
```xml
<material name="blue">
  <color rgba="0 0 1 1"/>
</material>

<visual>
  <geometry>
    ...
  </geometry>
  <origin ... />
  <material name="blue"/>
</visual>
```

## Join Links
`This process will create troubles` if not done carefully.

Try to follow these 5 steps:

### Step 1 - Define the new link with a proper name
```xml
# define color if necessary

<link name="shoulder_link">
<visual>
    <geometry>
        <cylinder radius="0.1" length="0.3"/>
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <material name="gray"/>
</visual>
</link>
```

### Step 2 - Define the joint
```xml
<joint name="base_shoulder_joint" type="fixed">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```
The origin describes here the origin of the child link relative to the origin of the parent link. The name is custom and the type can be changes (Step 4).
![joint](/docs/media/add_joint.png)
When I would set the xyz origin differently, the result would be a shift between the origins:
![joint](/docs/media/move_joint_origin.png)


