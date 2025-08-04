#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial



class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.pose_previous = 0
        self.publisher = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.subscriber = self.create_subscription(
            Pose,"/turtle1/pose",self.fn_callback,10)
        self.get_logger().info("RobotControl Node has been started.")

    def fn_callback(self,pose:Pose):
        msg = Twist()
        if pose.x < 1.0 or pose.x > 10.0 or pose.y < 1.0 or pose.y > 10.0:
            msg.linear.x = 0.5
            msg.angular.z = 1.0
        else:
            msg.linear.x = 2.0
            msg.angular.z = 0.3

        if pose.x >5.0 and self.pose_previous <5.0:
            self.pose_previous = pose.x
            self.get_logger().info("Set the color red")
            self.call_set_pen_service(255,0,0,4,0)

        elif pose.x < 5.0 and self.pose_previous > 5.0:
            self.pose_previous = pose.x
            self.get_logger().info("Set the color green")
            self.call_set_pen_service(0,255,0,4,0)

        self.publisher.publish(msg)


    def call_set_pen_service(self,r,g,b,width,off):
        client = self.create_client(SetPen,"/turtle1/set_pen")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.set_pen_callback))


    def set_pen_callback(self, future):
        try:
            response = future.result()

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()