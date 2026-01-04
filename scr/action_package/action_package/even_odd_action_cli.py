#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_custom_interfaces.action import UntilCount
import time
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle


class client(Node):
    def __init__(self):
        super().__init__('client')
        self.cli_ = ActionClient(self,UntilCount,"ans_bolo")

    def clinet_callback(self,targat_number,period):

        # wait for server
        self.get_logger().info("wait for server")
        self.cli_.wait_for_server()

        # create a goal
        num = UntilCount.Goal()
        num.target_number = targat_number
        num.period = period

        #send the goal
        self.get_logger().info(f"Sending Goal")
        self.cli_.send_goal_async(num).add_done_callback(self.is_it_accepted_or_rejected)

    def is_it_accepted_or_rejected(self,response):
        self.coming_:ClientGoalHandle = response.result()
        if self.coming_.accepted:
            self.dhekhi = self.coming_.get_result_async().add_done_callback(self.requet_for_result)
        else:
            self.get_logger().warn("Goal Rejected")

    def requet_for_result(self,saeed):
        saeed = saeed.result().result
        self.get_logger().info(f"Result is {saeed.reached_number}")

def main(args=None):
    rclpy.init(args=args)
    node = client()
    node.clinet_callback(5,1)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()