#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import EvenOdd

class Myserver(Node):
    def __init__(self):
        super().__init__('my_server')
        self.service = self.create_service(EvenOdd,"Nuru",self.service_callback)
        self.get_logger().info("Service is ready...")

    def service_callback(self,num,ans):
        if num.number %2 == 0:
            ans.result = "Even"
        else:
            ans.result = "Odd"
        self.get_logger().info(f'incoming request: num = {num.number}')
        return ans

def main(args=None):
    rclpy.init(args=args)
    node = Myserver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()