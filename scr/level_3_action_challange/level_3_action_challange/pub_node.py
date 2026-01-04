#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_custom_interfaces.action import CarPosition
from std_msgs.msg import String


class PublishTopic(Node):
    def __init__(self):
        super().__init__('pub_node')
        self.pub = self.create_publisher(String,'/cancel_move_topic',10)
        msg = String()
        msg.data = "Cancel Running Work"
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PublishTopic()
    # rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()

if __name__=='__main__':
    main()
