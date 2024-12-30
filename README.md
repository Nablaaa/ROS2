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


## Dummy Publisher and Subscriber
Sometimes, when people collaborate, one writes the publisher and one the subscriber node. Then it make sense that the communication is tested with
a dummy subscriber (for the publisher node) and a dummy publisher (for the subscriber node). This can be done with the following command line commands:

```bash
# publish for the subscriber
ros2 topic pub -r frequency /topic_name topic_type                   "data"
ros2 topic pub -r 4.1       /number     example_interfaces/msg/Int64 "{data: 5}"

# subscribe for the publisher
ros2 topic echo /topic_name
ros2 topic echo /number
```

## Recording and Replaying topic information
Sometimes, it is important to record nodes for later analysis or to reuse the information for follow up experiments. One example is, to record outdoor data of a truck ride, with rain and fog. Since it is not rainy and foggy everyday, it is helpful to reuse this information. In ROS this is done with **bags**. To record a node, use the following command:
```bash
# all nodes
ros2 bag record -a

# specific nodes
ros2 bag record /topicname -o bagname
```
This creates a directory "bagname" in which the data is recorded as a .mcap file, as well as the metadata.yaml file. To play the bag, use the following command:
```bash
ros2 bag play bag_directory/bagname

# more information
ros2 bag play -h
```
and then the bag is publishing the topics in the same way as the original nodes. So other nodes can subscribe to them.

## Interfaces

### Existing Interfaces
**DO NOT REINVENT THE WHEEL!**
Interfaces, also called messages, is the syntax of the data that is sent between nodes. They can be listed with
```bash
ros2 interface list
```
or by visiting the [ROS 2 documentation](https://github.com/ros2/common_interfaces). **They should be used to not reinvent the wheel!**
Install packages with
```bash
sudo apt install ros-<distro>-<package>

# example for sensor_msgs package
sudo apt install ros-jazzy-sensor-msgs

# source again
source ~/.bashrc

# add package to package.xml
# import package in python
```

### Custom Interfaces
Only if I do not find exactly what I need, I should create a custom interface. This process is not complicated and it is always the same process.


#### Setup Process
1. Create a new package (normally called robotname`_interfaces`)
```bash
cd src/
ros2 pkg create my_robot_interfaces --build-type ament_cmake
```
2. Remove the src and include folders and create a new folder called msg
```bash
cd my_robot_interfaces
rm -r src/ include/
mkdir msg
```
3. Modify package.xml. After `buildtool_depend ament_cmake buildtool depend` add the following lines
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
4. Modify CMakeLists.txt. After `find_package (ament_cmake REQUIRED)` and before `ament_package` add the following lines
```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  # here the name of the custom interface is added later
)
ament_export_dependencies(rosidl_default_runtime)
```
5. Remove `if(BUILD_TESTING)` block from CMakelists.txt


#### Building Interfaces
The setup process is finished (and has to be done only once) and now messages (interfaces) can be added.

Rules:
- use ThisTypeOfWritingForTheMessageName
- do not write "Msg" or "Interface" to avoid redundancy
- finish the name with ".msg"

```bash
cd src/my_robot_interfaces/msg/
touch HardwareStatus.msg
```

The content of a .msg file are listed [here](https://docs.ros.org/en/rolling/Concepts/Basic/About-Interfaces.html#field-types) (e.g. bool, float64, ...) and other existing interfaces (e.g. geometry_msgs/Twist - notice: it is not "geometry_msgs/msg/Twist"). See an example [here](src/my_robot_interfaces/msg/HardwareStatus.msg).

Now the interface has to be added to the CMakeLists.txt file. After `rosidl_generate_interfaces(${PROJECT_NAME}` add the following line
```cmake
"msg/HardwareStatus.msg"
```
Then it is time to save and build the files with colcon build