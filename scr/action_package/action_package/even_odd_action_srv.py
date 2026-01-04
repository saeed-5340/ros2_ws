#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_custom_interfaces.action import UntilCount
import time
from rclpy.action import ActionServer,GoalResponse
from rclpy.action.server import ServerGoalHandle

class server(Node):
    def __init__(self):
        super().__init__('server')
        self.ser_ = ActionServer(self,UntilCount,'ans_bolo',goal_callback=self.cheak_is_it_acceptable,
                                execute_callback=self.ok_accepted)
        self.get_logger().info("server is ready to work")

    def cheak_is_it_acceptable(self,cheak:UntilCount.Goal):

        self.get_logger().info("Recieved a Goal")

        # validate a goal request
        if (cheak.target_number <=0) or (cheak.target_number > 100):
            self.get_logger().warn("Rejected Goal")
            return GoalResponse.REJECT
        self.get_logger().info("Accepted Goal")
        return GoalResponse.ACCEPT
    
    def ok_accepted(self,response:ServerGoalHandle):
        
        # get request from goal
        number = response.request.target_number
        period = response.request.period


        # exectution
        count=0
        for i in range (number):
            count+=1
            time.sleep(period)
            if count%2 == 0:
                self.get_logger().info(f"number {count} is = Even")
            else:
                self.get_logger().info(f"number {count} is = Odd")


        # goal handle success
        response.succeed()

        # send the result
        num = UntilCount.Result()
        num.reached_number = count
        return num
    
def main(args=None):
    rclpy.init(args=args)
    node = server()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()