#!/usr/bin/env/python3

import rclpy
import rclpy.node
from rclpy.node import Node
import time
import math
import random
from std_msgs import msg

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.speed_pub = self.create_publisher(msg.Float64,'/car_speed',10)
        self.steering_angle_pub = self.create_publisher(msg.Float64,'/car_angle',10)
        self.break_pub = self.create_publisher(msg.Bool,'/car_break',10)
        self.forward_backward_pub = self.create_publisher(msg.Bool,'/car_mode',10)

        self.speed_sub = self.create_subscription(msg.Float64,'/car_speed',self.speed_callback,10)
        self.steering_angle_sub = self.create_subscription(msg.Float64,'/car_angle',self.steering_callback,10)
        self.break_sub = self.create_subscription(msg.Bool,'/car_break',self.break_callback,10)
        self.forward_backward_sub = self.create_subscription(msg.Bool,'/car_mode',self.forward_backward_callback,10)

        self.current_speed = 0.0
        self.current_angle = 0.0
        self.current_break = False
        self.current_mode = True  # True for forward, False for backward

        self.create_timer(3.0, self.publish_commands)


    def publish_commands(self):
        speed_msg = msg.Float64()
        speed_msg.data = self.current_speed
        self.speed_pub.publish(speed_msg)

        angle_msg = msg.Float64()
        angle_msg.data = self.current_angle
        self.steering_angle_pub.publish(angle_msg)
        
        break_msg = msg.Bool()
        break_msg.data = self.current_break
        self.break_pub.publish(break_msg)

        mode_msg = msg.Bool()
        mode_msg.data = self.current_mode
        self.forward_backward_pub.publish(mode_msg)
        self.get_logger().info(f"current Speed: {self.current_speed} km/h")
        self.get_logger().info(f"current Angle: {self.current_angle} degrees")
        self.get_logger().info(f"current break condition: {self.current_break}")
        self.get_logger().info(f"current mode condition: {self.current_mode}")

    def speed_callback(self, msg):
        self.current_speed = msg.data


    def steering_callback(self, msg):
        self.current_angle = msg.data

    def break_callback(self,msg):
        self.current_break = msg.data

    def forward_backward_callback(self,msg):
        self.current_mode = msg.data



def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()