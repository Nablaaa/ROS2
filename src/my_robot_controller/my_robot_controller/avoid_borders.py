#!/usr/bin/env python3.12
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


# DO NOT FORGET TO MAKE IT EXECUTABLE
# chmod +x my_robot_controller/my_robot_controller/name.py

class AvoidBorders(Node):
    def __init__(self):
        super().__init__('border_avoider')
        self.counter = 0
        self.limits = {
            "bottom": 2.5,
            "left": 2.5,
            "top": 8.5,
            "right": 8.5,
        }
                
        # set up listener and actor 
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        self.get_logger().info(f"Start Turtle Control")

        

    def pose_callback(self, pose: Pose):
        cmd = Twist()
        cmd.linear.x = 7.0
        cmd.angular.z = 0.0

        x = pose.x
        y = pose.y
        
        if (x < self.limits["left"] 
            or x > self.limits["right"]
            or y > self.limits["top"] 
            or y < self.limits["bottom"]):
            
            cmd.angular.z = 8.0 - 0.1 * self.counter

            self.counter += 1
        else:
            self.counter = 0

        self.cmd_vel_publisher_.publish(cmd)



def main(args=None):
    rclpy.init(args=args)
    
    node = AvoidBorders()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()