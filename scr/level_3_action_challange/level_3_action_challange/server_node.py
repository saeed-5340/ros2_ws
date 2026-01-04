#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_custom_interfaces.action import CarPosition
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading


class CarHandleServer(Node):
    def __init__(self):
        super().__init__('server_node')
        self.goal_handle_ :ServerGoalHandle = None
        self.current_position_ = 50
        self.goal_lock = threading.Lock()
        self.server_  = ActionServer(self,CarPosition,"dekha_jak_ki_hoy",
                                     goal_callback=self.goal_callback,
                                     cancel_callback=self.cancel_callback,
                                     execute_callback=self.execute_callback,
                                     callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Server is ready for work....")

    def goal_callback(self,goal_handle:CarPosition.Goal):
        # Cheak Goal is it validate or not

        self.get_logger().info("Received a Goal")
        if(goal_handle.position < 0 or goal_handle.position > 100 or goal_handle.velocity < 0):
            self.get_logger().warn("Give an invalid Goal")
            return GoalResponse.REJECT
        elif(goal_handle.velocity == 0 and goal_handle.position != 50):
            self.get_logger().info("It is Impossible to Reach the destination")
            return GoalResponse.REJECT
        
        # Goal policy ----> If one new task come then previous one is Aborted
        with self.goal_lock:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Coming a New Goal and Praoritize it and Aborted the running goal.")
                self.goal_handle_.abort()
        self.get_logger().info("Accept the Goal.")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self,goal_handle : ServerGoalHandle):
        self.get_logger().info("Coming a cancel request")
        return CancelResponse.ACCEPT


    def execute_callback(self,goal_handle:ServerGoalHandle):

        # Receive a goal
        with self.goal_lock:
            self.goal_handle_ = goal_handle

        self.position_ = goal_handle.request.position
        self.velocity_ = goal_handle.request.velocity
        result = CarPosition.Result()
        feedback = CarPosition.Feedback()

        # Execute the goal

        while(self.current_position_ < self.position_):
            if( ( self.position_ - self.current_position_ ) <= self.velocity_):
                self.current_position_ += (self.position_-self.current_position_)
            else:
                self.current_position_ += self.velocity_

            if not goal_handle.is_active:
                self.get_logger().info("New Goal Request is Come")
                goal_handle.abort()
                result.destination_position = self.current_position_
                result.massage = "New Goal is come for that running Goal is Aborted"
                return result
            
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel the Goal.")
                goal_handle.canceled()
                result.destination_position = self.current_position_
                result.massage = "Cancelled Successfully"
                return result
            
            time.sleep(1.0)
            feedback.current_position = self.current_position_
            goal_handle.publish_feedback(feedback)

        while(self.current_position_ > self.position_):
            if(self.current_position_- self.position_ <= self.velocity_):
                self.current_position_ = self.position_
            else:
                self.current_position_ -= self.velocity_

            if not goal_handle.is_active:
                self.get_logger().info("New Goal Request is Come")
                goal_handle.abort()
                result.destination_position = self.current_position_
                result.massage = "New Goal is come for that running Goal is Aborted"
                return result
            
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel the Goal.")
                goal_handle.canceled()
                result.destination_position = self.current_position_
                result.massage = "Cancelled Successfully"
                return result
            
            time.sleep(1.0)
            feedback.current_position = self.current_position_
            goal_handle.publish_feedback(feedback)

        # Goal Handle Successfully
        goal_handle.succeed()

        # Send the Goal
        result.destination_position = self.current_position_
        result.massage = "Perfectly Finish the Goal"
        return result




def main(args=None):
    rclpy.init(args=args)
    node = CarHandleServer()
    rclpy.spin(node,MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()