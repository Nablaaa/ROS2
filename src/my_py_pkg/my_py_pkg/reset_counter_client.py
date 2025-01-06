#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ResetCounter

class ResetCounterClient(Node): 
                        
    def __init__(self):
        super().__init__('reset_counter_client') 
        self.client = self.create_client(ResetCounter, 'reset_counter') # give the correct service name
        self.get_logger().info('Reset Counter Client has been started.')

    def call_reset_counter(self, value):

        # this will wait one second to try to find the service
        # if the service is not available, it will return a message
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')


        # prepare the message
        request = ResetCounter.Request()
        request.reset_value = value

        # send request to the service and get the response in the future
        future = self.client.call_async(request)
        future.add_done_callback(self.callback_reset_counter_response)


    def callback_reset_counter_response(self, future):

        # if there is a result in future
        response = future.result()

        # log the response
        self.get_logger().info('Success: ' + str(response.success))
        self.get_logger().info('Message: ' + response.message)


def main(args=None):
    rclpy.init(args=args)
    node = ResetCounterClient()
    node.call_reset_counter(100)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()