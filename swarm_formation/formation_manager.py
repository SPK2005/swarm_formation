#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class FormationManager(Node):

    def __init__(self):

        super().__init__('formation_manager')

        self.sub = self.create_subscription(
            PoseStamped,
            '/virtual_leader_pose',
            self.leader_callback,
            10
        )

        self.pub1 = self.create_publisher(
            PoseStamped,
            '/drone1_target',
            10
        )

        self.pub2 = self.create_publisher(
            PoseStamped,
            '/drone2_target',
            10
        )

        self.pub3 = self.create_publisher(
            PoseStamped,
            '/drone3_target',
            10
        )

    def leader_callback(self,msg):

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        d1 = PoseStamped()
        d2 = PoseStamped()
        d3 = PoseStamped()

        d1.pose.position.x = x - 3.0
        d1.pose.position.y = y
        d1.pose.position.z = z

        d2.pose.position.x = x
        d2.pose.position.y = y + 3.0
        d2.pose.position.z = z

        d3.pose.position.x = x + 3.0
        d3.pose.position.y = y
        d3.pose.position.z = z

        self.pub1.publish(d1)
        self.pub2.publish(d2)
        self.pub3.publish(d3)


def main(args=None):

    rclpy.init(args=args)

    node = FormationManager()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
