#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,GoalResponse
from rclpy.action.server import ServerGoalHandle
from my_custom_interfaces.action import CarPosition
import time

class CarHandleServer(Node):
    def __init__(self):
        super().__init__('server_node')
        self.server_  = ActionServer(self,CarPosition,"dekha_jak_ki_hoy",
                                     goal_callback=self.goal_callback,
                                     execute_callback=self.execute_callback)
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
        else:
            self.get_logger().info("Accept the Goal")
            return GoalResponse.ACCEPT


    def execute_callback(self,goal_handle:ServerGoalHandle):

        # Receive a goal
        self.goal_handle_ = goal_handle
        position = goal_handle.request.position
        velocity = goal_handle.request.velocity
        result = CarPosition.Result()
        feedback = CarPosition.Feedback()
        current_position = 50

        # Execute the goal

        while(current_position < position):
            if( ( position - current_position ) <= velocity):
                current_position += (position-current_position)
            else:
                current_position += velocity
            time.sleep(1.0)
            feedback.current_position = current_position
            goal_handle.publish_feedback(feedback)
        while(current_position > position):
            if(current_position-position <= velocity):
                current_position -= position
            else:
                current_position -= velocity
            time.sleep(1.0)
            feedback.current_position(feedback)
            goal_handle.publish_feedback(feedback)

        # Goal Handle Successfully

        goal_handle.succeed

        # Send the Goal

        result.destination_position = current_position
        result.massage = "Perfectly Finish the Goal"
        return result




def main(args=None):
    rclpy.init(args=args)
    node = CarHandleServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()