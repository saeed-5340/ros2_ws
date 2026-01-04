#! usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import AddTwoNum as Ad
import time
import random

class myserver(Node):
    def __init__(self):
        super().__init__('time_srv_add')
        self.service = self.create_service(Ad,'timely_kaj_kor',self.service_callback)

    def service_callback(self,request,response):
        response.sum = request.a + request.b
        self.get_logger().info(f"incomeing request: a = {request.a} b = {request.b}")
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = myserver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__maain__':
    main()