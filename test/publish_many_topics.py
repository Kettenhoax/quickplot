#!/usr/bin/python3
import sys
import rclpy
import math
import random
from rclpy.node import Node
from rclpy.time import CONVERSION_CONSTANT
from geometry_msgs.msg import Vector3


TOPIC_COUNT=1000
nsp = ['front_camera', 'rear_camera', 'head', 'robot1', 'robot2', 'robot3']
names = ['vel', 'imu', 'speed', 'odom', 'gps', 'gnss', 'lidar', 'image', 'camera_info']


def random_topic():
    topic = '_'.join(random.sample(names, random.randint(1, 2)))
    while True:
        if random.uniform(0, 1) > 0.1:
            break
        topic = random.choice(nsp) + '/' + topic
    return topic


class PublishManyTopics(Node):

    def __init__(self):
        super().__init__('publish_real_twist')
        self._publishers = [self.create_publisher(Vector3, random_topic(), 1) for _ in range(TOPIC_COUNT)]
        self._publish_timer = self.create_timer(0.1, self._on_publish)

    def _on_publish(self):
        msg = Vector3()
        t = self.get_clock().now()
        msg.x = math.sin(t.nanoseconds / CONVERSION_CONSTANT)

        for publisher in self._publishers:
            publisher.publish(msg)


def main(args=sys.argv):
    rclpy.init(args=args)
    rclpy.spin(PublishManyTopics())


if __name__ == '__main__':
    main()
