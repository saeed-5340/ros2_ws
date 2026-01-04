# #! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import AddTwoNum

class Myserver(Node):
    def __init__(self):
        super().__init__('my_server')
        self.service = self.create_service(AddTwoNum,'saeed',self.service_callback)
        self.get_logger().info(f"Service is ready...")

    def service_callback(self,req,res):
        res.sum = req.a + req.b
        self.get_logger().info(f'Incoming request: a={req.a} b = {req.b}')
        return res

def main(args=None):
    rclpy.init(args=args)
    node = Myserver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()