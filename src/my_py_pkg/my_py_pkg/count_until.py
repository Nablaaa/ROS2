import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import GoalResponse # to send feedback
from rclpy.action.server import ServerGoalHandle # used when action is accepted
from my_robot_interfaces.action import CountUntil


class CountUntilServer(Node):
    def __init__(self):
        super().__init__('count_until_server')
        self.count_until_server = ActionServer(
            self,
            CountUntil,
            'count_until',
            goal_callback=self.goal_callback, # received goals are processed
            excecute_callback=self.execute_callback) # and if accepted, than excecution happens here
        

    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info('Received goal request')
        if goal_request.target_number < 0:
            self.get_logger().info('Goal request rejected, target number must be positive')
            return GoalResponse.REJECT
        
        self.get_logger().info('Goal request accepted')
        return GoalResponse.ACCEPT
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        target_number = goal_handle.request.target_number
        delay = goal_handle.request.delay
        result = CountUntil.Result()
        counter = 0