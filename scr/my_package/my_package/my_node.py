#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class MyNode(Node):
    count = 0
    def __init__(self):
        super().__init__('first_node')
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        MyNode.count +=1
        self.get_logger().info(f"Hello {MyNode.count}")



def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
