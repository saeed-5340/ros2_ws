#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_custom_interfaces.msg import TargetCoordinate

class CustomServicePublisher(Node):
    def __init__(self):
        super().__init__('custom_service_subscriber')
        self.subs = self.create_subscription(TargetCoordinate,'/saeed',self.my_function,10)


    def my_function(self, ok: TargetCoordinate):
        self.get_logger().info(f"Name = {ok.name},Angle = {ok.angle}, X = {ok.position.x}, Y = {ok.position.y}")    


def main(args=None):
    rclpy.init(args=args)
    node = CustomServicePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()