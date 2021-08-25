#!/usr/bin/python3
import sys
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class PublishNaN(Node):

    def __init__(self):
        super().__init__('publish_nan')
        self._publisher = self.create_publisher(Vector3, 'vector', 1)
        self._publish_timer = self.create_timer(0.1, self._on_timer)

    def _on_timer(self):
        msg = Vector3()
        msg.x = math.nan
        msg.y = math.inf
        msg.z = 1.0
        self._publisher.publish(msg)


def main(args=sys.argv):
    rclpy.init(args=args)
    rclpy.spin(PublishNaN())


if __name__ == '__main__':
    main()
