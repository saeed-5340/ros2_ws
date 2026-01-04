#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_custom_interfaces.action import UntilCount
from rclpy.action.client import ClientGoalHandle, GoalStatus

class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.cli_ = ActionClient(self,UntilCount,"dekho_tomra")


    def send_goal(self,target_number,period):
        # wait for server
        self.cli_.wait_for_server()

        # create a goal
        num = UntilCount.Goal()
        num.target_number = target_number
        num.period = period

        # Send the Goal
        self.get_logger().info(f"Sending Goal")
        self.cli_.send_goal_async(num,feedback_callback=self.continueous_feedback). \
            add_done_callback(self.is_it_accepted_or_rejected)
        
        # Send a Cancl request after 2 second 
        # self.time_ = self.create_timer(2.0,self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("send a cancel request")
        self.coming_.cancel_goal_async()
        self.time_.cancel()
        
    def is_it_accepted_or_rejected(self,response):
        self.coming_ :ClientGoalHandle = response.result()
        if self.coming_.accepted:
            self.get_logger().info("Accepted")
            self.coming_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().info("Rejected")
        
    def goal_result_callback(self,response):
        status = response.result().status
        result = response.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Abrord")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Cancelled")
        self.get_logger().info(f"Result is = {result.reached_number}")

    def continueous_feedback(self,fbk):
        number = fbk.feedback.current_number
        self.get_logger().info(f"got feedback = {number}")

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    node.send_goal(6,1.0)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main() 