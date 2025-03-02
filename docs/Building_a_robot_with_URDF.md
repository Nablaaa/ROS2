# Building a Robot with Unified Robot Description Format (URDF)

The goal for robotics is to have a working robot in real world. But before, it is good to simulate the robot in GAZEBO. The Gazebo simulation environment needs a robot model (written in URDF). Here, I will describe, how to set this up.

## Getting started
Set up an URDF file with
```bash
touch my_robot_name.urdf
```
and write the following minimal code in the file (the file is [here](urdf/my_robot_preliminary.urdf)):
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

This is exactly the next step.

### Step 3 - Move the origin of the child link to a proper place
`HERE, the most errors happen`. To get less confused, make sure to always first move the origin of the JOINT, before changing the origins of the VISUALs. Because the joint is the one that defines the location of the child link relative to the parent link.

To get less confused, switch off the "visual element" in the RVIZ Display and just think about, where a joint should be placed.

![joint](/docs/media/not_visualize_link.png)

Here, I want to place the link of the shoulder on top of the box, in the middle, so it should be lifted by the height of the box, which is 0.2 m
![joint](/docs/media/shoulder_link.png)

In case, you move the visual origin, than you will end up with something wrong, like here:
![wrong origin moved](/docs/media/error_moving_visual_origin.png), which is not exactly what we wanted to achieve. (note that in this image, the joint origin is correct and the visual origin is applied afterward, important for us in this step is the CORRECT location of the JOINT origin)


### Step 4 - Set joint type
There are differnt type of joints, depending on the situation. For example, when adding a camera on top of the robot, the position of the camera is fixed. But when I have a robotic arm, than the joints should be able to move. The types are:

- fixed (no movement, camera)
- revolute (rotation with min and max angle, arms)
- continuous (rotation without limits, wheels)
- prismatic (translation with min and max and no rotation, linear actuators)

Full list: [ROS Documentation](https://wiki.ros.org/urdf/XML/joint)

So for example in our [my_robot.urdf file](urdf/my_robot.urdf), we can change the joint type to revolute:
```xml
<joint name="base_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="100"/>
</joint>
```
Here the rotation happens along the axis z within the limits of +-pi. The limits for effort and velocity are normally overwrtiten by other ROS Nodes, so just set them to defaults of 100.

This will directly allow us to rotate the shoulder (in the visualization).
![rotate shoulder](/docs/media/rotate_shoulder.png)

### Step 5 - Fix the VISUAL origin of part
Normally, the parts (box, cylinder, sphere,...) are constructed symmetrically around the origin. We do not want to have it like this. We want them to be at a physical proper location (so that one part is NOT inside another part, since this would not work out in a real world). Thats where the VISUAL modification comes in play. Just set them accordingly. Here our cylinder is 0.3 m high, so the offset should be by 0.15 m in z direction.
```xml
<visual>
    <geometry>
        <cylinder radius="0.1" length="0.3"/>
    </geometry>
    <origin xyz="0 0 0.15" rpy="0 0 0" />
    <material name="gray"/>
</visual>
```
![final urdf](/docs/media/final_urdf.png)

This is not always necessary, for example when working with wheels. There it would make more sense to shift the JOINT origin higher and have the VISUAL origin at the center of the wheel (0 0 0). This makes sense since the wheel rotates around its center.

## Conclusion
Thats it! The way to add ONE part to the URDF with correct joints and origins. Just repeat this for every new part.

## Tipp
It makes sense to have the base of the robot being sitting at the (0 0 0) position. But then, if I add something below (e.g. wheels), then it is a good practice to create something called a "virtual link" that projects the center of the robot to this position. This makes it easier in `navigation` since I can project the position of all (vacuum cleaning) robots to a 2D map, which allows easier navigation than working in a 3D environment. <br>
Do this by adding a NON-Visual link.
```xml
<link name="base_footprint" />

<joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>
```
As you can see, the base_footprint (without _link suffix) is created and the new parent for the base_link. Also the origin is shifted to the origin of the base_link (which is in this example (0 0 0.1) in z direction). The base_footprint joint will be below the base_link, even though z=0.1 is set. (Actually I am confused why it is 0.1 and not -0.1 since I thought the joint should be shifted down, but if I try -0.1 it shifts the joint up, so it seems to be correct like this).
Try it out with:
```bash
ros2 launch urdf_tutorial display.launch.py model:=/home/eric/Desktop/GitHub/ros2_ws/urdf/ros_book_car.urdf
```


## Adding Wheels
An example of adding continuous wheels is in the [robot car example](urdf/robot_car.urdf). There the important things are:
- JOINT origin is also shifted by the thickness of the wheel (additionally to the position shift), so that the wheel has the origin in the center and NOT overlaps with the body
- JOINT origin rotation is (0 0 0) and does not give the right orientation to the compartment (this is done in the VISUAlS part only. `I thought differently first`) -> this makes sure that the rotation axis is around the green y axis and not around another axis