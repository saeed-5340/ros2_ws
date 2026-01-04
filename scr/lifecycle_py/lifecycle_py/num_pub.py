#! /usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState,TransitionCallbackReturn
from std_msgs.msg import Int64

class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__('number_publisher')
        self.counter_=0
        self.publish_freq = 1.0
        self.num_pub_ = None
        self.timer = None
        self.get_logger().info("I'm in Constructor")


    # Create Ros2 Communication, Connect to hardware
    def on_configure(self, previous_state:LifecycleNode):
        self.get_logger().info("I'm in on_configure")
        self.num_pub_ = self.create_lifecycle_publisher(Int64,'/ki_hoy',10)
        self.timer = self.create_timer(1.0/self.publish_freq , self.pub_callback)
        self.get_logger().info("Publisher start to publish")
        return TransitionCallbackReturn.SUCCESS
    
    # Activate/Enable HW
    def on_activate(self, previous_state:LifecycleNode):
        self.get_logger().info("In on Active")
        return super().on_activate(previous_state)

    # Deactivate/Disable HW
    def on_deactivate(self, previous_state:LifecycleNode):
        self.get_logger().info("In on Deactivate")
        return super().on_deactivate(previous_state)
    
    # Destory Ros2 Communication, disconnect from hardware
    def on_cleanup(self, previous_state):
        self.get_logger().info("I'm in on_configure")
        self.destroy_lifecycle_publisher(self.num_pub_)
        self.destroy_timer(self.timer)
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, previous_state:LifecycleNode):
        self.get_logger().info("I'm in on_shutdown")
        self.destroy_lifecycle_publisher(self.num_pub_)
        self.destroy_timer(self.timer)
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, previous_state:LifecycleNode):
        self.get_logger().info("I'm in error")
        self.destroy_lifecycle_publisher(self.num_pub_)
        self.destroy_timer(self.timer)
        # do some cheak,
        return TransitionCallbackReturn.SUCCESS


    def pub_callback(self):
        msg = Int64()
        msg.data = self.counter_
        self.num_pub_.publish(msg)
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()