#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient,GoalResponse,CancelResponse
from rclpy.action.client import ClientGoalHandle,GoalStatus
from my_custom_interfaces.action import CarPosition
from std_msgs.msg import String


class CarHandleClient(Node):
    def __init__(self):
        super().__init__('cli_node')
        self.cli_ = ActionClient(self,CarPosition,'dekha_jak_ki_hoy')
        self.subs = self.create_subscription(String,'/cancel_move_topic',self.subscription_callback,10)
        
        # Wait for server

        self.get_logger().info("Waiting For server")
        self.cli_.wait_for_server()

    def subscription_callback(self,msg:String):
        self.get_logger().info(f"{msg.data}")
        self.cheak.cancel_goal_async()



    def send_goal(self,position,velocity):

        # Create a Goal
        goal = CarPosition.Goal()
        goal.position = position
        goal.velocity = velocity
        self.get_logger().info("Sanding a Goal...")
        future = self.cli_.send_goal_async(goal,feedback_callback=self.feedback_callback)
        future.add_done_callback(self.is_it_accepted_or_rejected)


    def is_it_accepted_or_rejected(self,goal_response):
        self.cheak : ClientGoalHandle = goal_response.result()
        if  self.cheak.accepted:
            self.get_logger().info("Accepted the Goal")
            self.cheak.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal is Rejected by Server")

    def goal_result_callback(self,sending_result):
        status = sending_result.result().status
        result = sending_result.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Result Comming Successfully")
        if status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Result is Aborted")
        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Cancelled the Goal")
        self.get_logger().info(f"Result is = {result.destination_position}")
        self.get_logger().info(f"{result.massage}")



    def feedback_callback(self,fbk):
        feedback = fbk.feedback.current_position
        self.get_logger().info(f"Now car position is {feedback}")


def main(args=None):
    rclpy.init(args=args)
    node = CarHandleClient()
    node.send_goal(80,4)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()