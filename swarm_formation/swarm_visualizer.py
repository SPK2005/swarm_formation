#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

from px4_msgs.msg import VehicleOdometry

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy


class SwarmVisualizer(Node):

    def __init__(self):

        super().__init__('swarm_visualizer')

        px4_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/swarm_markers',
            10
        )

        self.leader = None
        self.drone1 = None
        self.drone2 = None
        self.drone3 = None

        self.leader_history = []
        self.d1_history = []
        self.d2_history = []
        self.d3_history = []

        self.max_points = 500

        self.create_subscription(
            PoseStamped,
            '/virtual_leader_pose',
            self.leader_callback,
            10
        )

        self.create_subscription(
            VehicleOdometry,
            '/px4_1/fmu/out/vehicle_odometry',
            self.drone1_callback,
            px4_qos
        )

        self.create_subscription(
            VehicleOdometry,
            '/px4_2/fmu/out/vehicle_odometry',
            self.drone2_callback,
            px4_qos
        )

        self.create_subscription(
            VehicleOdometry,
            '/px4_3/fmu/out/vehicle_odometry',
            self.drone3_callback,
            px4_qos
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_markers
        )

        self.get_logger().info(
            'Swarm Visualizer Started'
        )

    def leader_callback(self, msg):

        pos = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )

        self.leader = pos

        self.leader_history.append(pos)

        if len(self.leader_history) > self.max_points:
            self.leader_history.pop(0)

    def drone1_callback(self, msg):

        pos = (
            msg.position[0],
            msg.position[1],
            msg.position[2]
        )

        self.drone1 = pos

        self.d1_history.append(pos)

        if len(self.d1_history) > self.max_points:
            self.d1_history.pop(0)

    def drone2_callback(self, msg):

        pos = (
            msg.position[0],
            msg.position[1],
            msg.position[2]
        )

        self.drone2 = pos

        self.d2_history.append(pos)

        if len(self.d2_history) > self.max_points:
            self.d2_history.pop(0)

    def drone3_callback(self, msg):

        pos = (
            msg.position[0],
            msg.position[1],
            msg.position[2]
        )

        self.drone3 = pos

        self.d3_history.append(pos)

        if len(self.d3_history) > self.max_points:
            self.d3_history.pop(0)

    def create_marker(self, marker_id, pos, r, g, b):

        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "swarm"
        marker.id = marker_id

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(pos[0])
        marker.pose.position.y = float(pos[1])
        marker.pose.position.z = float(-pos[2])

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b

        return marker

    def create_trail(self, marker_id, history, r, g, b):

        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "trail"
        marker.id = marker_id

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.03

        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b

        for pos in history:

            p = Point()

            p.x = float(pos[0])
            p.y = float(pos[1])
            p.z = float(-pos[2])

            marker.points.append(p)

        return marker

    def publish_markers(self):

        markers = MarkerArray()

        if self.leader is not None:
            markers.markers.append(
                self.create_marker(
                    0,
                    self.leader,
                    1.0,
                    0.0,
                    0.0
                )
            )

        if self.drone1 is not None:
            markers.markers.append(
                self.create_marker(
                    1,
                    self.drone1,
                    0.0,
                    0.0,
                    1.0
                )
            )

        if self.drone2 is not None:
            markers.markers.append(
                self.create_marker(
                    2,
                    self.drone2,
                    0.0,
                    1.0,
                    0.0
                )
            )

        if self.drone3 is not None:
            markers.markers.append(
                self.create_marker(
                    3,
                    self.drone3,
                    1.0,
                    1.0,
                    0.0
                )
            )

        if len(self.leader_history) > 2:
            markers.markers.append(
                self.create_trail(
                    100,
                    self.leader_history,
                    1.0,
                    0.0,
                    0.0
                )
            )

        if len(self.d1_history) > 2:
            markers.markers.append(
                self.create_trail(
                    101,
                    self.d1_history,
                    0.0,
                    0.0,
                    1.0
                )
            )

        if len(self.d2_history) > 2:
            markers.markers.append(
                self.create_trail(
                    102,
                    self.d2_history,
                    0.0,
                    1.0,
                    0.0
                )
            )

        if len(self.d3_history) > 2:
            markers.markers.append(
                self.create_trail(
                    103,
                    self.d3_history,
                    1.0,
                    1.0,
                    0.0
                )
            )

        self.marker_pub.publish(markers)


def main(args=None):

    rclpy.init(args=args)

    node = SwarmVisualizer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
