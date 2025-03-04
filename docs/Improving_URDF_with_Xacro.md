# Improving URDF files with Xacro

Xacro helps to make URDF files scalable and to solve issues faster. It also helps to introduce variables and functions and can split an URDF into several files. This is especially useful when working with complex robots.

## Installation

```bash
sudo apt install ros-<distro>-xacro
``` 

## Usage
First change the file from `*.urdf` to `*.xacro`. Often people name the main file also `*.urdf.xacro` but important is the .xacro in the end.

Now change the robot tag to:
```xml
<robot name="ros_book_car" xmlns:xacro="http://www.ros.org/wiki/xacro">
```
Now launch the file with:
```bash
ros2 launch urdf_tutorial display.launch.py model:=path/to/model.xacro
```

Now xacro offer to define variables (so called `properties`) with the following syntax:
```xml
<robot name="ros_book_car" xmlns:xacro="http://www.ros.org/wiki/xacro">


    <xacro:property name="wheel_radius" value="0.1"/>
    <xacro:property name="wheel_separation" value="0.5"/>
```
`do not forget the  " " for the values`

And then properties can be called with
```xml
<box size="${base_length} ${base_width} {base_height}"/>
<origin xyz="0 0 ${base_height/2.0}" rpy="0 0 0"/>
```
It also allows to have access to pi as
```xml
<origin xyz="0 0 0" rpy="${pi/2.0} 0 0" />
```
