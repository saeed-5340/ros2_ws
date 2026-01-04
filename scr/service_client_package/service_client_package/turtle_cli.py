#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from turtlesim.msg import Pose 
import math
import random


class tutle(Node):
    def __init__(self):
        super().__init__('color_node')
        self.pub = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.subs = self.create_subscription(Pose,"/turtle1/pose",self.position_callback,10)
        self.get_logger().info(f"Node has been started")

    def position_callback(self,posi:Pose):
        cmd = Twist()
        if(posi.x < 1.0 or posi.x > 9.0) or (posi.y < 1.0 or posi.y >9.0):
            cmd.linear.x = 0.5
            cmd.angular.z = 2.0
        else: 
            cmd.linear.x = 2.0
            cmd.angular.z = 0.3

        if posi.x < 5:
            self.set_color_pen(255,0,0,4,0)
        else:
            self.set_color_pen(0,0,255,4,0)
        
        self.pub.publish(cmd)


    def set_color_pen(self,r,g,b,width,off):
        self.cli = self.create_client(SetPen,'/turtle1/set_pen')
        while not self.cli.wait_for_service(1.0):
            self.get_logger().info(f"waiting for service...")
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off

        future = self.cli.call_async(req)
        future.add_done_callback(self.recieved)

    def recieved(self,future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = tutle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()