#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose, Color
from turtlesim.srv import SetPen
from my_turtle_interfaces.srv import ToggleActiveState
from rclpy.parameter import Parameter



class RunCircles(Node): 
    def __init__(self):
        super().__init__('run_circles') 


        # color parameters
        self.declare_parameter("color_right", [255, 255, 255])
        self.declare_parameter("color_left", [0, 255, 0])

        self.color_right = self.get_parameter("color_right").value
        self.color_left = self.get_parameter("color_left").value


        self.declare_parameter("turtle_velocity", 1.0)  
        self.turtle_velocity = self.get_parameter("turtle_velocity").value

        # make parameter callback in case i want to change the number parameter
        self.add_post_set_parameters_callback(self.parameter_callback)


        self.screen_center_x = 5.5


        # publisher cmd_Vel
        self.pub_velocity = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # color publisher
        self.pub_color = self.create_publisher(Color, 'turtle1/color_sensor', 10)
        
        self.sub_position = self.create_subscription(Pose, 'turtle1/pose', self.ControlTurtle, 10)


        self.send_velocity = True

        self.toggle_state_service = self.create_service(ToggleActiveState, 'toggle_active_state', self.callback_toggle_active_state)

        self.client_pen = self.create_client(SetPen, 'turtle1/set_pen')


        self.prev_x = 0.0

        self.get_logger().info(f"Hi, you can toggle off/on the turtle with the my_turtle_interfaces server.")



    def parameter_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == "color_right":
                self.color_right = param.value
                self.get_logger().info("color right has been changed to " + str(self.color_right))

            if param.name == "color_left":
                self.color_left = param.value
                self.get_logger().info("color left has been changed to " + str(self.color_left))

            if param.name == "turtle_velocity":
                self.turtle_velocity = param.value
                self.get_logger().info("turtle velocity has been changed to " + str(self.turtle_velocity))



    def ControlTurtle(self, pose: Pose):

        while not self.client_pen.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')
        
        if not self.send_velocity:
            return
        

        # get position
        x = pose.x

        vel_msg = Twist()

        if x < self.screen_center_x and self.prev_x >= self.screen_center_x:
            self.prev_x = x

            self.linear = self.turtle_velocity
            self.angular = self.turtle_velocity
            
            # unpack color parameters from self.color_left
            self.call_pen(*self.color_left, 2)

        
        if x > self.screen_center_x and self.prev_x <= self.screen_center_x:
            self.prev_x = x
            self.linear = 4.0 * self.turtle_velocity
            self.angular = 4.0 * self.turtle_velocity

            self.call_pen(*self.color_right, 5)


        self.call_vel(vel_msg, self.linear, self.angular)


    def call_vel(self, vel_msg, linear, angular):
        
        vel_msg.linear.x = linear
        vel_msg.angular.z = angular

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
            response.message = "Deactivate turtle"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RunCircles() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()