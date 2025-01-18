#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64, Float64 # here i import an interface
from my_robot_interfaces.msg import HardwareStatus

class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__('publish_number')
        

        self.declare_parameter("number", 8)
        self.declare_parameter("interval", 1.0)

        self.number = self.get_parameter("number").value
        self.interval = self.get_parameter("interval").value


        # publish to topic name 'number', with message type Int64 and queue size 10
        self.number_publisher = self.create_publisher(Int64, 'number', 10) 
        self.float_publisher = self.create_publisher(Float64, 'number_float', 10)

        self.hardware_status_publisher = self.create_publisher(HardwareStatus, 'hardware_status', 10)

        self.timer = self.create_timer(self.interval, self.publish_number)
        
        self.hardware_timer = self.create_timer(2, self.publish_hardware_status)
        
        self.get_logger().info("publishing starts")



    def publish_number(self):
        msg = Int64()
        msg.data = self.number
        self.number_publisher.publish(msg)

        msg_float = Float64()
        msg_float.data = self.number + 0.1234
        self.float_publisher.publish(msg_float)


    def publish_hardware_status(self):
        msg = HardwareStatus()
        msg.version = 1
        msg.temperature = 32.1
        msg.are_motors_ready = True
        msg.debug_message = "All is fine"
        self.hardware_status_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()