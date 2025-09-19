#!usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial
import random
import math
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from std_msgs.msg import String



class Spawner(Node):
    def __init__(self):
        super().__init__('spawner')
        self.num = 1
        self.publisher = self.create_publisher(Pose, '/turtle1/pose', 10)
        self.pub = self.create_publisher(String, '/new_turtle', 10)
        self.callback = self.create_timer(3.0,self.spawn_turtle)
        self.get_logger().info("Spawner Node has been started.")


    def spawn_turtle(self):
        msg = Pose()
        clint = self.create_client(Spawn, '/spawn')
        while not clint.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for service...")

        self.num += 1
        request = Spawn.Request()
        request.x = random.uniform(0.0, 11.00)
        request.y = random.uniform(0.0, 11.00)
        request.theta = random.uniform(0.0, 2 * math.pi)
        request.name = f"turtle{self.num}"
        msg.x = request.x
        msg.y = request.y
        msg.theta = request.theta
        self.publisher.publish(msg)
        future = clint.call_async(request)
        future.add_done_callback(partial(self.spawn_turtle_callback))


    def spawn_turtle_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Turtle spawned: {response.name}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")




def main(args=None):
    rclpy.init(args=args)
    node = Spawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
