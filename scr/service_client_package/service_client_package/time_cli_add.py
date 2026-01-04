#! usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import AddTwoNum as Ad
import time
import random


class myclient(Node):
    def __init__(self):
        super().__init__('time_cli_add')
        self.cli = self.create_client(Ad,'timely_kaj_kor')
        while not self.cli.wait_for_service(1.0):
            self.get_logger().info(f"waiting for service...")
        self.create_timer(2.0,self.launch_value_callback)


    def launch_value_callback(self):
        x = random.randint(-50,50)
        y = random.randint(-50,50)
        z = time.time()
        actual_value = self.send_request(x,y)
        actual_value.add_done_callback(self.response)

    
    def send_request(self,x,y):
        req = Ad.Request()
        req.a = x
        req.b = y
        return self.cli.call_async(req)