#!/usr/bin/python3
import rclpy
import math
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import CONVERSION_CONSTANT
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped


class PublishSimTwist(Node):

    def __init__(self):
        super().__init__('publish_sim_twist')
        self._pub_clock = self.create_publisher(
            Clock, 'clock', qos_profile_sensor_data)
        self._pub_twist = self.create_publisher(
            TwistStamped, 'cmd_vel', qos_profile_sensor_data)

        self._i = 0
        self._t = Time()
        self._clock_timer = self.create_timer(0.01, self._on_clock_timer)
        self._twist_timer = self.create_timer(0.1, self._on_twist_timer)

    def _on_clock_timer(self):
        self._t = Time(seconds=self._i//100,
                       nanoseconds=int((self._i % 100)/100 * CONVERSION_CONSTANT))
        msg = Clock()
        msg.clock = self._t.to_msg()
        self._pub_clock.publish(msg)

        self._i += 1

    def _on_twist_timer(self):
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self._t.to_msg()
        msg.twist.linear.x = math.sin(self._t.nanoseconds / CONVERSION_CONSTANT)
        self._pub_twist.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(PublishSimTwist())


if __name__ == '__main__':
    main()
