#! /usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import AddTwoNum
import random

class Myclient(Node):
    def __init__(self):
        super().__init__('square_client_node')
        self.cli = self.create_client(AddTwoNum,"Vondami")
        while ((self.cli.wait_for_service(1.0) == False)):
            self.get_logger().info("waiting for service...")
        self.create_timer(1.0,self.launch_value_callback)
    
    def launch_value_callback(self):
        x = random.randint(0,20)
        y = random.randint(0,20)
        porer_value = self.send_request(x,y)
        porer_value.add_done_callback(self.response)

        
    def send_request(self,x,y):
        req = AddTwoNum.Request()
        req.a = x
        req.b = y
        return self.cli.call_async(req)
    
    def response(self,porer_value):
        try:
            ok = porer_value.result()
            if ok is not None:
                print(f"square is = {ok.sum}")
            else:
               self.get_logger().error("Service returned no result")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
def main(args=None):
    rclpy.init(args=args)
    node = Myclient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()