import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus # used when action is accepted
from my_robot_interfaces.action import CountUntil # has CountUntil.Goal, .Feedback, .Result


class CountUntilClient(Node):
    def __init__(self):
        super().__init__('count_until_client')
        self.count_until_client = ActionClient(
            self,
            CountUntil,
            'count_until')

    def send_goal(self, target_number, delay):
        # define goal via Interface
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.delay = delay

        # send goal and add callback so that the NODE can keep spinning and does
        # not stop the excecution
        self.count_until_client.wait_for_server()
        self.count_until_client.send_goal_async(
            goal, # first send the goal
            feedback_callback=self.goal_feedback_callback # than add a callback for feedback
            ).add_done_callback( # than add a callback to it
                self.goal_response_callback
                )
        
    def cancel_goal(self):
        """
        Define a function to cancel the goal
        This function can be called from anywhere
        """
        self.get_logger().info("Send a cancel request")
        self.goal_handle.cancel_goal_async()

    def goal_response_callback(self, future):
        """
        This function is only to get the response of the server, 
        if the goal was accepted or rejected.
        If accepted, than the goal_result_callback is called, which
        waits for the result of the excecution.
        """

        # use type hint for this variable
        self.goal_handle: ClientGoalHandle = future.result()

        if self.goal_handle.accepted: # wait here for server "goal_callback" to return ACCEPT
            self.get_logger().info('Goal accepted')
            self.goal_handle.get_result_async().add_done_callback(
                self.goal_result_callback # once accepted, wait for the result of excecution
                )
            
        else:
            self.get_logger().info('Goal rejected')


    def goal_feedback_callback(self, feedback_msg):
        number = feedback_msg.feedback.current_number
        self.get_logger().info(f'Feedback: {number}')

        if number > 2:
            self.cancel_goal()

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Success')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'Goal aborted')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'Goal canceled')

        self.get_logger().info(f'Result: {result.reached_number}')



def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClient()
    node.send_goal(10, 0.5)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()