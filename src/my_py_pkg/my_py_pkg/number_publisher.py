#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64 # here i import an interface

class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__('publish_number')
        
        self.number = 42
        self.number_publisher = self.create_publisher(Int64, 'number', 10)

        self.timer = self.create_timer(0.5, self.publish_number)
        self.get_logger().info("publishing starts")

    def publish_number(self):
        msg = Int64()
        msg.data = self.number
        self.number_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()