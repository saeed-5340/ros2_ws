#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import AddTwoNum

class Myserver(Node):
    def __init__(self):
        super().__init__('square_node_server')
        self.server = self.create_service(AddTwoNum,'Vondami',self.speed_callback)
        self.get_logger().info(f"server is ready...")

    def speed_callback(self,req,res):
        res.sum = req.a * req.a
        req.b = 0
        self.get_logger().info(f"incomeing request square of {req.a}")
        return res  

def main(args=None):
    rclpy.init(args=args)
    node = Myserver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
