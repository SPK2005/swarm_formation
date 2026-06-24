#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from math import sin, cos

class VirtualLeader(Node):

    def __init__(self):

        super().__init__('virtual_leader')

        self.publisher = self.create_publisher(
            PoseStamped,
            '/virtual_leader_pose',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )

        self.t = 0.0

    def timer_callback(self):

        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = 5.0*sin(0.1* self.t)
        msg.pose.position.y = 5.0*sin(0.1* self.t)*cos(0.1* self.t)
        msg.pose.position.z = -5.0

        self.publisher.publish(msg)

        self.t += 0.05


def main(args=None):

    rclpy.init(args=args)

    node = VirtualLeader()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
