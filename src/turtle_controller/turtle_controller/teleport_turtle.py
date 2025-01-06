#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from random import random

class TeleportTurtle(Node): # MODIFY NAME 
                          # AND add to setup.py
    def __init__(self):
        super().__init__('teleport_turtle') 
        
        # add pose subscriber
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.max_x_position = 8.0

        # check if the teleporting is triggered already
        self.is_teleporting = False
        
        self.get_logger().info(f"Start Turtle Teleport")

    def pose_callback(self, pose: Pose):

        # check if the turtle is already teleporting
        # and skip in this case the rest
        if self.is_teleporting:
            return

        client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')
        
        x_pos = pose.x

        if x_pos >= self.max_x_position:
            self.is_teleporting = True

            request = TeleportAbsolute.Request()

            request.x = random() * 2 + 3
            request.y = random() * 2 + 3

            request.theta = 0.0
            future = client.call_async(request)
            future.add_done_callback(self.callback_teleport)

    def callback_teleport(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed {str(e)}")

        finally:
            # only when the response is received, set the flag to False again
            self.is_teleporting = False

def main(args=None):
    rclpy.init(args=args)
    node = TeleportTurtle() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()