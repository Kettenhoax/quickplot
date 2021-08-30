#!/usr/bin/python3
import sys
import rclpy
import math
from rclpy.node import Node
from rclpy.time import CONVERSION_CONSTANT
from geometry_msgs.msg import TwistStamped


class PublishRealTwist(Node):

    def __init__(self):
        super().__init__('publish_real_twist')
        self._pub_twist = self.create_publisher(TwistStamped, 'cmd_vel', 1)
        self._twist_timer = self.create_timer(0.1, self._on_twist_timer)

    def _on_twist_timer(self):
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        t = self.get_clock().now()
        msg.header.stamp = t.to_msg()
        msg.twist.linear.x = math.sin(t.nanoseconds / CONVERSION_CONSTANT)
        self._pub_twist.publish(msg)


def main(args=sys.argv):
    rclpy.init(args=args)
    rclpy.spin(PublishRealTwist())


if __name__ == '__main__':
    main()
