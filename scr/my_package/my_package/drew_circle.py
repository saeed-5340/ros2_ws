#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrewCircle(Node):
    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_val_pub = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.timer_ = self.create_timer(0.5,self.send_velocity_command)
        self.get_logger().info("DrewCircle Node has been started")

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0  
        self.cmd_val_pub.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = DrewCircle()
    rclpy.spin(node)
    rclpy.shutdown()