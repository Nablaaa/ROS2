#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose, Color
from turtlesim.srv import SetPen
from my_turtle_interfaces.srv import ToggleActiveState




class RunCircles(Node): 
    def __init__(self):
        super().__init__('run_circles') 

        self.screen_center_x = 5.5

        # publisher cmd_Vel
        self.pub_velocity = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # color publisher
        self.pub_color = self.create_publisher(Color, 'turtle1/color_sensor', 10)
        
        self.sub_position = self.create_subscription(Pose, 'turtle1/pose', self.ControlTurtle, 10)

        self.toggle_state_service = self.create_service(ToggleActiveState, 'toggle_active_state', self.callback_toggle_active_state)

        self.client_pen = self.create_client(SetPen, 'turtle1/set_pen')


        self.send_velocity = True
        self.prev_x = 0.0

        self.get_logger().info(f"Please toggle active state to start the turtle.")


    def ControlTurtle(self, pose: Pose):

        while not self.client_pen.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')
        
        # get position
        x = pose.x

        vel_msg = Twist()
        vel_msg.linear.x = 1.0
        vel_msg.angular.z = 1.0

        if x < self.screen_center_x and self.prev_x >= self.screen_center_x:
            self.prev_x = x

            self.call_vel(1.0, 1.0)
            self.call_pen(0, 255, 0, 2)
        
        if x > self.screen_center_x and self.prev_x <= self.screen_center_x:
            self.prev_x = x
            vel_msg.linear.x = 2.0
            vel_msg.angular.z = 2.0

            self.call_vel(2.0, 2.0)
            self.call_pen(255, 255, 255, 5)

        if self.send_velocity:
            self.pub_velocity.publish(vel_msg)


    def call_vel(self, linear, angular):
        vel_msg = Twist()
        vel_msg.linear.x = linear
        vel_msg.angular.z = angular

        if self.send_velocity:
            self.pub_velocity.publish(vel_msg)


    def call_pen(self, r, g, b, width):
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width

        future = self.client_pen.call_async(request)
        future.add_done_callback(self.callback_set_pen)


    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed {str(e)}")


    def callback_toggle_active_state(self, request: ToggleActiveState.Request, response: ToggleActiveState.Response):
        
        if request.active_state == "on":
            self.get_logger().info("Activating turtle")

            self.send_velocity = True
            response.success = True
            response.message = "Activate turtle"



        elif request.active_state == "off":
            self.get_logger().info("Deactivating turtle")
            
            self.send_velocity = False
            response.success = True
            response.message = "Deactvate turtle"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RunCircles() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()