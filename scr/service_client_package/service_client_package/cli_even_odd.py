#! /usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import EvenOdd

class Myclient(Node):
    def __init__(self):
        super().__init__('my_client')
        self.cli = self.create_client(EvenOdd,"Nuru")
        
        while (self.cli.wait_for_service(1.0)==False):
            self.get_logger().info(f"Waiting For service...")
        self.num =EvenOdd.Request()
        
    def send_function(self,x):
        self.num.number = x
        return self.cli.call_async(self.num)





def main(args=None):
    rclpy.init(args=args)
    node = Myclient()
    future = node.send_function(int(sys.argv[1]))
    rclpy.spin_until_future_complete(node,future)
    print(f"Ans is = {future.result().result}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
