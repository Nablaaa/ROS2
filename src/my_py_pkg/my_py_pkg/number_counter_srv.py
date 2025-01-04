#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64, Float64 
from my_robot_interfaces.srv import ResetCounter

class NumberCounterNode(Node): 
    def __init__(self):
        super().__init__('number_counter_srv')
        self.counter_int = 0
        self.counter_float = 0.0

        # HINT: callbacks are normally called "callback_<topic>"
        # and are not called directly in the code (to avoid errors)
        self.create_subscription(Int64, 'number', self.callback_count_int, 10)
        self.create_subscription(Float64, 'number_float', self.callback_count_float, 10)

        self.reset_counter_service = self.create_service(ResetCounter, 'reset_counter', self.callback_reset_counter)

        # now publish the sum of the both counters
        self.sum_publisher = self.create_publisher(Float64, 'number_sum', 10)
        self.timer = self.create_timer(1.0, self.callback_publish_sum)


        self.get_logger().info("Subscribing to number and number_float topics")

    def callback_count_int(self, msg):
        # print counter
        self.counter_int += msg.data
        self.get_logger().info('Counter int: {0}'.format(self.counter_int))
        

    def callback_count_float(self, msg):
        # print counter
        self.counter_float += msg.data
        self.get_logger().info('Counter float: {0}'.format(self.counter_float))

    def callback_publish_sum(self):
        # publish sum of both counters
        msg = Float64()
        msg.data = self.counter_int + self.counter_float
        self.sum_publisher.publish(msg)

        # listen to the publication with "echo"
        # ros2 topic echo /number_sum 
        # and look at rqt_graph


    def callback_reset_counter(self, request: ResetCounter.request,response: ResetCounter.Response):
        if request.reset_value <0:
            response.success = False
            response.message = "Reset value cannot be negative"

        elif request.reset_value > self.counter_int:
            response.success = False
            response.message = "Reset value cannot be greater than the current value"

        else:
            self.counter_int = request.reset_value
            self.counter_float = 0.0
            response.success = True
            response.message = "Both counters reset."

            self.get_logger().info('Counter int: {0}'.format(self.counter_int))
            self.get_logger().info('Counter float: {0}'.format(self.counter_float))

        return response



def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()