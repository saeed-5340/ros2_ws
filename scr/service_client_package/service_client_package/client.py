#! usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import AddTwoNum

class Myclient(Node):
    def __init__(self):
        super().__init__('my_client')
        self.cli = self.create_client(AddTwoNum,'saeed')
        while ((self.cli.wait_for_service(1.0))==False):
            self.get_logger().info("Waiting for service...")
        self.req = AddTwoNum.Request()

    def send_request(self,x,y):
        self.req.a = x
        self.req.b = y
        return self.cli.call_async(self.req)



def main(args=None):
    rclpy.init(args=args)
    node = Myclient()
    saeed = node.send_request(int(sys.argv[1]),int(sys.argv[2]))
    rclpy.spin_until_future_complete(node,saeed)
    print(f"sum is = {saeed.result().sum}")
    # try:
    #     print(f"Sum is = {saeed.result().sum}")
    # except Exception as e:
    #     node.get_logger().error(f"Service call failed: {e}")
    rclpy.shutdown()

if __name__=='__main__':
    main()
