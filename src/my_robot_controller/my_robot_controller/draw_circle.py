#!/usr/bin/env python3.12
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# DO NOT FORGET TO MAKE IT EXECUTABLE
# chmod +x my_robot_controller/my_robot_controller/draw_circle.py

class DrawCircle(Node):
    def __init__(self):
        self.counter = 0
        super().__init__('DrawCircle')
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info(f"Drawing circle starts")

        # now start action
        self.create_timer(1.0, self.send_velocity_cmd)
        
    def send_velocity_cmd(self):
        """
        The data which I can control, I can find with
        
        ros2 interface show geometry_msgs/msg/Twist
    
        for turtle it is

        Vector3  linear
            float64 x
            float64 y
            float64 z
        Vector3  angular
            float64 x
            float64 y
            float64 z

        """

        msg = Twist()
        msg.linear.x = 1.0 + 0.1* self.counter
        # rotate around z-axis
        msg.angular.z = 1.0 + self.counter
        self.cmd_vel_publisher_.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    
    node = DrawCircle()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()