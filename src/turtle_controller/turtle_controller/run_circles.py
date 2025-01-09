#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose, Color
from turtlesim.srv import SetPen



class RunCircles(Node): 
    def __init__(self):
        super().__init__('run_circles') 

        self.screen_center_x = 5.5

        # publisher cmd_Vel
        self.pub_velocity = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # color publisher
        self.pub_color = self.create_publisher(Color, 'turtle1/color_sensor', 10)
        
        # subscriber pose
        self.sub_position = self.create_subscription(Pose, 'turtle1/pose', self.ControlTurtle, 10)

        self.client = self.create_client(SetPen, 'turtle1/set_pen')




    def ControlTurtle(self, msg: Pose):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')
        

        # print position
        self.get_logger().info('Position: x={0}, y={1}, theta={2}'.format(msg.x, msg.y, msg.theta))

        # get position
        x = msg.x

        vel_msg = Twist()
        vel_msg.linear.x = 1.0
        vel_msg.angular.z = 1.0

        request = SetPen.Request()
        request.r = 0
        request.g = 255
        request.b = 0
        request.width = 2

        if x > self.screen_center_x:
            vel_msg.linear.x = 2.0
            vel_msg.angular.z = 2.0


            request.r = 255
            request.g = 255
            request.b = 255
            request.width = 5

        future = self.client.call_async(request)
        future.add_done_callback(self.callback_set_pen)

        self.pub_velocity.publish(vel_msg)

    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed {str(e)}")



def main(args=None):
    rclpy.init(args=args)
    node = RunCircles() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()