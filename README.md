# ROS 2 Workspace
![Turtlesim](docs/media/turtlesim.png)

***

This is a collection from src files that I have written with the help of a) [ROS Documentation](https://docs.ros.org/en/jazzy/Tutorials.html) b) [YouTube](https://www.youtube.com/@RoboticsBackEnd) and c) ["ROS 2 from Scratch" book](https://www.amazon.de/ROS-Scratch-started-robotics-applications/dp/B0DJCFC29Q?source=ps-sl-shoppingads-lpcontext&ref_=fplfs&psc=1&smid=A3JWKAKR8XB7XF&language=de_DE).

## Requirements
1. Ubuntu 24.04
2. ROS 2 Jazzy

## Getting Started
1. Clone the repository
2. Build the workspace with `colcon build`
```bash
$ cd ~/ROS2
$ colcon build
```
3. Source the workspace
```bash
$ source ~/ROS2/install/setup.bash
```
4. or add the following line to your .bashrc
```bash
$ echo "source ~/ROS2/install/setup.bash" >> ~/.bashrc
```
5. Create a package (folder) for the nodes
```bash
cd src/

# python
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy

# or cpp
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
```
6. Create a file for the node and make it executable
```bash
cd src/my_robot_controller/my_robot_controller

# create file
touch my_first_node.py

# make it excecutive VERY IMPORTANT
chmod +x my_first_node.py
```
7. Write the code for the node, add the reference to the setup.py and load dependencies (if necessary) in package.xml
8. Bd everything in development mode
```bash
cd ~/ros2_ws
colcon build --symlink-install 

# or optional
colcoun build --symlink-install --packages-select my_robot_controller
```
9. Source the file (in bashrc)
```bash
source ~/.bashrc

# there must be a line:
source ~/path/to/ros2_ws/install/setup.bash
```
like
```bash
# start ros2
source /opt/ros/jazzy/setup.bash
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
# install my ROS workspace
source ~/Desktop/GitHub/ros2_ws/install/setup.bash
```

In this repository, I cover the topics:
1. [ROS Terminology, Concepts and Programming](docs/Ros_terminology.md)
2. [ROS Visualization Tools](docs/Ros_visualization_tools.md)
3. [Building a Robot with URDF](docs/Building_a_robot_with_URDF.md)
4. [Improving the URDF file with Xacro](docs/Improving_URDF_with_Xacro.md)


And this is enough for getting started with ROS 2. The following topics, about Gazebo and Robot simulations will happen in a new workspace, since it makes sense to seperate workspaces for every robot we build. This workspace is already quite full with turtlesim and my_robot_controller, as well as the URDF files on top of it. So lets start a new workspace for the next robot.


# IMPORTANT LAST NOTE
The workspace, we are working with, is sourced in .bashrc when opening a terminal. 

```bash
# start ros2
source /opt/ros/jazzy/setup.bash
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
# install my ROS workspace
source ~/Desktop/GitHub/ros2_ws/install/setup.bash
```

Never source 2 workspaces together, so make sure to comment out the old workspace, when starting a new one. 

```bash
# source ~/Desktop/GitHub/ros2_ws/install/setup.bash
source ~/Desktop/GitHub/my_new_workspace_name/install/setup.bash
```
