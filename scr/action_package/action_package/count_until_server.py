#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import threading
from my_custom_interfaces.action import UntilCount
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
 

class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__('action_server_node')
        self.goal_handle_korbe : ServerGoalHandle = None
        self.goal_lock = threading.Lock()
        self.queue_ = []
        self.server_ = ActionServer(self,UntilCount,"dekho_tomra",
                                    goal_callback=self.goal_callback,
                                    handle_accepted_callback=self.handle_accepted_callback,
                                    cancel_callback=self.cancel_callback,
                                    execute_callback=self.ok_dektesi,
                                    callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f"Server in ready for run..")


    def goal_callback(self,saeed:UntilCount.Goal):
        self.get_logger().info("Recieved a goal...")


        # Policy 01 : Refuse If one Goal is Active (rejected the second Goal)

        # with self.goal_lock:
        #     if (self.goal_handle_korbe is not None) and (self.goal_handle_korbe.is_active):
        #         self.get_logger().info("Currently Runing a Goal")
        #         return GoalResponse.REJECT

        # validate a goal request
        if saeed.target_number <= 0:
            self.get_logger().info("Rejected the goal")
            return GoalResponse.REJECT
        
        # Policy 02 : Refuse if New Goal is Coming (Abrod the First Goal)

        with self.goal_lock:
            if (self.goal_handle_korbe is not None) and (self.goal_handle_korbe.is_active):
                self.get_logger().info("Comming a new goal and proritize this goal")
                self.goal_handle_korbe.abort()
            
        self.get_logger().info("Accepted the goal")
        return GoalResponse.ACCEPT
    
    # def handle_accepted_callback(self,goal_handle:ServerGoalHandle):
    #     with self.goal_lock:
    #         if self.goal_handle_korbe is not None:
    #             self.queue_.append(goal_handle)
    #         else:
    #             goal_handle.execute()
    
    def cancel_callback(self,goal_handle:ServerGoalHandle):
        self.get_logger().info("Receiveing the Cancel Goal")
        return CancelResponse.ACCEPT

    def ok_dektesi(self,goal_handle:ServerGoalHandle):

        with self.goal_lock:
            self.goal_handle_korbe = goal_handle

 
        # get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period
        result = UntilCount.Result()

        # execute the action

        counter = 0
        feedback = UntilCount.Feedback()
        for i in range (target_number):
        
            # For policy : 02
            if not goal_handle.is_active:
                result.reached_number = counter
                # self.process_next_goal_of_queue()
                return result

            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel the Goal")
                goal_handle.canceled()
                result.reached_number = counter
                # self.process_next_goal_of_queue()
                return result
            counter += 1
            self.get_logger().info(f"currcent number is = {counter}")
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)
            
        
        # goal handle success
        # goal_handle.abort
        goal_handle.succeed()

        #and send the result
        result = UntilCount.Result()
        result.reached_number = counter
        # self.process_next_goal_of_queue()
        return result
    
    def process_next_goal_of_queue(self):
        if len(self.queue_) > 0:
            self.queue_.pop(0).execute()
        else:
            self.goal_handle_korbe = None
        
    
def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__=='__main__':
    main()