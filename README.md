# ROS 2 Workspace
This is a collection from src files that I have written with the help of a) ROS Documentation b) YouTube and c) "ROS 2 from Scratch" book.

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
8. Build everything in development mode
```bash
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



## Coding Understanding
- interfaces are imported as modules
- inherit from Node class to get ROS Node functionality, e.g.
    - create publisher
    - create subscriber
    - create service
    - create client
    - create timer
    - ...


## ROS 2 interfaces
Interfaces are something like the syntax of the messages. They can be seen in
```bash
ros2 interface show example_interfaces/<name>
```
and can be reached in python via 
```python
from example_interfaces.srv import AddTwoInts
from example_interfaces.msg import Float64
```
and so on. See [number_publisher.py](src/my_py_pkg/my_py_pkg/number_publisher.py) for an example.


## setup.py and package.xml
- setup.py: new nodes have to be added to the entry_points dictionary
    - so for every python class, there has to be a new entry
    - the name of the entry is the name of the node
- package.xml: dependencies have to be added here
    - so for every import, there has to be a new dependency
    - see rclpy for example (python)