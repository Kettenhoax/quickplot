#!/usr/bin/python3
import sys
import rclpy
import math
import random
from rclpy.node import Node
from rclpy.time import CONVERSION_CONSTANT, Duration
from geometry_msgs.msg import Vector3Stamped


class PublishAsyncStddev(Node):

    def __init__(self):
        super().__init__('publish_async_stddev')
        self._pub_value = self.create_publisher(Vector3Stamped, 'value', 1)
        self._pub_stddev = self.create_publisher(Vector3Stamped, 'stddev', 1)
        self._timer = self.create_timer(0.1, self._on_timer)

    def _on_timer(self):
        msg = Vector3Stamped()
        t = self.get_clock().now()
        t += Duration(nanoseconds=random.randint(0, CONVERSION_CONSTANT / 1e3))

        msg.header.stamp = t.to_msg()
        msg.vector.x = math.sin(t.nanoseconds / CONVERSION_CONSTANT)
        self._pub_value.publish(msg)

        msg.vector.x = 1.0
        if bool(random.getrandbits(3)):
            print('publishing')
            self._pub_stddev.publish(msg)


def main(args=sys.argv):
    rclpy.init(args=args)
    rclpy.spin(PublishAsyncStddev())


if __name__ == '__main__':
    main()
