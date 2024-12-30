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