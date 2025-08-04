#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_custom_interfaces.msg import TargetCoordinate


class CustomServicePublisher(Node):
    def __init__(self):
        super().__init__('custom_service_publisher')
        self.publisher = self.create_publisher(TargetCoordinate,'/saeed',10)
        self.timer = self.create_timer(1.0, self.publish_target_coordinate)

    def publish_target_coordinate(self):
        msg = TargetCoordinate()
        msg.name = "Saeed"
        msg.angle = 120.0
        msg.position.x = 5.0
        msg.position.y = 10.0
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CustomServicePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
