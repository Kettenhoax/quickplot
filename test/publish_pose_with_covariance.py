#!/usr/bin/python3
import rclpy
import math
from rclpy.node import Node
from rclpy.time import CONVERSION_CONSTANT
from geometry_msgs.msg import PoseWithCovarianceStamped


class PublishPoseWithCovariance(Node):

    def __init__(self):
        super().__init__('publish_pose_with_covariance')
        self._pub = self.create_publisher(PoseWithCovarianceStamped, 'pose', 1)
        self._timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        msg = PoseWithCovarianceStamped()
        t = self.get_clock().now()
        msg.header.stamp = t.to_msg()
        msg.pose.pose.position.x = math.sin(t.nanoseconds / CONVERSION_CONSTANT)
        msg.pose.covariance[0] = 4.0
        self._pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(PublishPoseWithCovariance())


if __name__ == '__main__':
    main()
