import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import GoalResponse # to send feedback
from rclpy.action.server import ServerGoalHandle # used when action is accepted
from my_robot_interfaces.action import CountUntil # has CountUntil.Goal, .Feedback, .Result

# imports for cancelling abilities
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

class CountUntilServer(Node):
    def __init__(self):
        super().__init__('count_until_server')
        self.get_logger().info('Count until server is up')
        self.count_until_server = ActionServer(
            self,
            CountUntil,
            'count_until',
            goal_callback=self.goal_callback, # received goals are processed
            execute_callback=self.execute_callback,# and if accepted, than excecution happens here
            cancel_callback=self.cancel_callback, # callback to cancel the goal
            callback_group=ReentrantCallbackGroup(), # to allow all callbacks run in parallel
            ) 
        

    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info('Received goal request')
        if goal_request.target_number < 0:
            self.get_logger().info('Goal request rejected, target number must be positive')
            return GoalResponse.REJECT
        
        self.get_logger().info('Goal request accepted')
        return GoalResponse.ACCEPT
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        # get GOAL request from client
        target_number = goal_handle.request.target_number
        delay = goal_handle.request.delay

        # prepare feedback
        self.feedback_msg = CountUntil.Feedback()

        # prepare result
        result = CountUntil.Result()


        counter = 0

        self.get_logger().info(f'Start execution. Counting until {target_number}')
        for _ in range(target_number):


            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled, now wheels could stop or robot could go back to the dock')
                goal_handle.canceled()
                
                result.reached_number = counter
                return result

            counter += 1
            self.get_logger().info(str(counter))

            # add and publish feedback
            self.feedback_msg.current_number = counter
            goal_handle.publish_feedback(self.feedback_msg)


            
            time.sleep(delay) # this is just here to simulate that an action needs time (different to a server)

        # once goal is reached
        goal_handle.succeed() # aborted and canceled are other options

        result.reached_number = counter
        return result
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Received cancel request')

        if self.feedback_msg.current_number < goal_handle.request.target_number/2:
            self.get_logger().info('Cancel request rejected, because a very important process is running')
            return CancelResponse.REJECT
        else:
            self.get_logger().info('Cancel request accepted')
            return CancelResponse.ACCEPT # this will set "goal_handle.is_cancel_requested" to True

    

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServer()
    rclpy.spin(node, executor=MultiThreadedExecutor()) 
    rclpy.shutdown()

if __name__ == '__main__':
    main()