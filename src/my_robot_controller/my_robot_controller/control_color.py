#!/usr/bin/env python3.12
import rclpy
from rclpy.node import Node
from functools import partial
from turtlesim.msg import Pose
from turtlesim.srv import SetPen


# DO NOT FORGET TO MAKE IT EXECUTABLE
# chmod +x my_robot_controller/my_robot_controller/name.py


"""script that subscribes to the position and then requests different colors
based on the position of the turtle.

The x position defines the r value
The y position defines the g value
The difference of x and y defines the b value
The width is defined by  (x/(y+1))/100"""

class ColorController(Node):
    def __init__(self):
        super().__init__('color_controller')
        self.prev_x, self.prev_y = 0, 0

                
        # set up listener and service
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.get_logger().info(f"Start Turtle Color Control")


    def call_SetPen_service(self, r, g, b, width, off=0):
        client = self.create_client(SetPen, '/turtle1/set_pen')

        # client could be None
        while not client.wait_for_service(timeout_sec=1):
            self.get_logger().warn('Waiting for service to be available...')

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        self.off = off

        # call async to not block the main thread
        future = client.call_async(request)
        future.add_done_callback(partial(self.set_pen_callback))


    def set_pen_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed {str(e)}")


    def pose_callback(self, pose: Pose):
        

        
        x_color = pose.x * 255/12
        y_color = pose.y * 255/12

        r = int(x_color)
        g = int(y_color)
        b = int(abs(r-g))
        width = int(pose.x+pose.y)


        # IF POSSIBLE, CALL THE SERVICE AS LITTLE AS POSSIBLE TO AVOID LAGGING OF THE PROGRAM
        dx = abs(pose.x - self.prev_x)
        dy = abs(pose.y - self.prev_y)
        d = dx + dy

        # SO ONLY UPDATE AFTER THE TURTLE WAS MOVING A BIT
        if d > 0.2:

            self.get_logger().info(f"Setting color to r:{r} g:{g} b:{b} width:{width}")
            self.call_SetPen_service(r, g, b, width)
    
            self.prev_x = pose.x
            self.prev_y = pose.y




def main(args=None):
    rclpy.init(args=args)
    
    node = ColorController()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()