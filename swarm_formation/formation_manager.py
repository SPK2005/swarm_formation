#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from math import sin, cos, atan2

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

        self.pub4 = self.create_publisher(
            PoseStamped,
            '/drone4_target',
            10
        )

    def leader_callback(self,msg):

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        
        leader_yaw = atan2(
            2.0*(qw*qz + qx*qy),
            1.0 - 2.0*(qy*qy + qz*qz)
        )

        g1x = 3.0
        g1y = 3.0

        g2x = -3.0
        g2y = 3.0

        g3x = 3.0
        g3y = -3.0

        g4x = -3.0
        g4y = -3.0

        c = cos(leader_yaw)
        s = sin(leader_yaw)

        d1 = PoseStamped()
        d2 = PoseStamped()
        d3 = PoseStamped()
        d4 = PoseStamped()

        d1.pose.position.x = x + g1x*c - g1y*s
        d1.pose.position.y = y + g1x*s + g1y*c
        d1.pose.position.z = z

        d2.pose.position.x = x + g2x*c - g2y*s
        d2.pose.position.y = y + g2x*s + g2y*c
        d2.pose.position.z = z

        d3.pose.position.x = x + g3x*c - g3y*s
        d3.pose.position.y = y + g3x*s + g3y*c
        d3.pose.position.z = z

        d4.pose.position.x = x + g4x*c - g4y*s
        d4.pose.position.y = y + g4x*s + g4y*c
        d4.pose.position.z = z

        d1.pose.orientation = msg.pose.orientation
        d2.pose.orientation = msg.pose.orientation
        d3.pose.orientation = msg.pose.orientation
        d4.pose.orientation = msg.pose.orientation

        self.pub1.publish(d1)
        self.pub2.publish(d2)
        self.pub3.publish(d3)
        self.pub4.publish(d4)


def main(args=None):

    rclpy.init(args=args)

    node = FormationManager()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()