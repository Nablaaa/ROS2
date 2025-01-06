#!/usr/bin/env python3.12
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


# DO NOT FORGET TO MAKE IT EXECUTABLE
# chmod +x my_robot_controller/my_robot_controller/name.py

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

    def pose_callback(self, msg: Pose):
        self.get_logger().info(f"Received pose: x: {msg.x:.3f}, y: {msg.y:.3f}, theta: {msg.theta:.3f}")


def main(args=None):
    rclpy.init(args=args)
    
    node = PoseSubscriberNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()