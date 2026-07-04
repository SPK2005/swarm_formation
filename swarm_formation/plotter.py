#!/usr/bin/env python3

import math
import csv
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import Float64

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)


class Plotter(Node):

    def __init__(self):

        super().__init__('plotter')

        # ==================================================
        # PX4 QoS
        # ==================================================

        self.px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ==================================================
        # Storage
        # ==================================================

        self.targets = {}
        self.positions = {}

        # PX4 odometry is reported relative to each drone's own spawn point.
        # Add the spawn offset (NED, from start_swarm.sh ENU poses) to put
        # all drones in one shared world frame.
        self.spawn_ned = {
            1: (-3.0, 0.0, 0.0),
            2: (0.0,  3.0, 0.0),
            3: (3.0,  0.0, 0.0),
        }

        # ==================================================
        # Desired Formation Distances
        #
        # Drone1 = (-3, 0)
        # Drone2 = ( 0,+3)
        # Drone3 = (+3, 0)
        # ==================================================

        self.desired_d12 = math.sqrt(18.0)  # 4.243
        self.desired_d13 = 6.0
        self.desired_d23 = math.sqrt(18.0)  # 4.243

        # ==================================================
        # Formation Error Magnitude
        # ==================================================

        self.err_pub = {
            1: self.create_publisher(
                Float64,
                '/formation_error/drone1',
                10),

            2: self.create_publisher(
                Float64,
                '/formation_error/drone2',
                10),

            3: self.create_publisher(
                Float64,
                '/formation_error/drone3',
                10)
        }

        # ==================================================
        # X Error
        # ==================================================

        self.err_x_pub = {
            1: self.create_publisher(
                Float64,
                '/formation_error_x/drone1',
                10),

            2: self.create_publisher(
                Float64,
                '/formation_error_x/drone2',
                10),

            3: self.create_publisher(
                Float64,
                '/formation_error_x/drone3',
                10)
        }

        # ==================================================
        # Y Error
        # ==================================================

        self.err_y_pub = {
            1: self.create_publisher(
                Float64,
                '/formation_error_y/drone1',
                10),

            2: self.create_publisher(
                Float64,
                '/formation_error_y/drone2',
                10),

            3: self.create_publisher(
                Float64,
                '/formation_error_y/drone3',
                10)
        }

        # ==================================================
        # Z Error
        # ==================================================

        self.err_z_pub = {
            1: self.create_publisher(
                Float64,
                '/formation_error_z/drone1',
                10),

            2: self.create_publisher(
                Float64,
                '/formation_error_z/drone2',
                10),

            3: self.create_publisher(
                Float64,
                '/formation_error_z/drone3',
                10)
        }

        # ==================================================
        # Inter-Drone Distance
        # ==================================================

        self.d12_pub = self.create_publisher(
            Float64,
            '/interdrone/d12',
            10)

        self.d13_pub = self.create_publisher(
            Float64,
            '/interdrone/d13',
            10)

        self.d23_pub = self.create_publisher(
            Float64,
            '/interdrone/d23',
            10)

        # ==================================================
        # Formation Shape Error
        # ==================================================

        self.shape12_pub = self.create_publisher(
            Float64,
            '/formation_shape_error/d12',
            10)

        self.shape13_pub = self.create_publisher(
            Float64,
            '/formation_shape_error/d13',
            10)

        self.shape23_pub = self.create_publisher(
            Float64,
            '/formation_shape_error/d23',
            10)

        # ==================================================
        # TARGET SUBSCRIBERS
        # ==================================================

        self.create_subscription(
            PoseStamped,
            '/drone1_target',
            lambda msg: self.target_cb(msg, 1),
            10
        )

        self.create_subscription(
            PoseStamped,
            '/drone2_target',
            lambda msg: self.target_cb(msg, 2),
            10
        )

        self.create_subscription(
            PoseStamped,
            '/drone3_target',
            lambda msg: self.target_cb(msg, 3),
            10
        )

        # ==================================================
        # PX4 ODOM SUBSCRIBERS
        # ==================================================

        self.create_subscription(
            VehicleOdometry,
            '/px4_1/fmu/out/vehicle_odometry',
            lambda msg: self.odom_cb(msg, 1),
            self.px4_qos
        )

        self.create_subscription(
            VehicleOdometry,
            '/px4_2/fmu/out/vehicle_odometry',
            lambda msg: self.odom_cb(msg, 2),
            self.px4_qos
        )

        self.create_subscription(
            VehicleOdometry,
            '/px4_3/fmu/out/vehicle_odometry',
            lambda msg: self.odom_cb(msg, 3),
            self.px4_qos
        )

        # ==================================================
        # TIMER
        # ==================================================

        self.timer = self.create_timer(
            0.05,
            self.compute_metrics
        )

        self.get_logger().info("Plotter node started")

        # ==================================================
        # CSV Logging
        # ==================================================

        self.start_time = self.get_clock().now()

        log_dir = os.path.expanduser("~/ros2_ws/logs")
        os.makedirs(log_dir, exist_ok=True)

        self.csv_file = open(
            os.path.join(log_dir, "formation_metrics.csv"),
            "w",
            newline=""
        )

        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "time",
            "e1",
            "e2",
            "e3",
            "shape12",
            "shape13",
            "shape23"
        ])

    # ======================================================
    # CALLBACKS
    # ======================================================

    def target_cb(self, msg, idx):

        self.targets[idx] = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )

    def odom_cb(self, msg, idx):

        # Lift local-spawn odometry into the shared NED world frame so the
        # logged errors reflect the TRUE formation, not the per-spawn fiction.
        sx, sy, sz = self.spawn_ned[idx]
        self.positions[idx] = (
            msg.position[0] + sx,
            msg.position[1] + sy,
            msg.position[2] + sz
        )

    # ======================================================
    # UTILITIES
    # ======================================================

    def distance(self, p1, p2):

        return math.sqrt(
            (p1[0] - p2[0]) ** 2 +
            (p1[1] - p2[1]) ** 2 +
            (p1[2] - p2[2]) ** 2
        )

    # ======================================================
    # METRICS
    # ======================================================

    def compute_metrics(self):

        if len(self.targets) < 3:
            return

        if len(self.positions) < 3:
            return

        # -----------------------------------
        # Tracking Errors
        # -----------------------------------
        tracking_errors = []

        for i in [1, 2, 3]:

            tx, ty, tz = self.targets[i]
            px, py, pz = self.positions[i]

            # desired - actual
            ex = tx - px
            ey = ty - py
            ez = tz - pz

            err = math.sqrt(
                ex ** 2 +
                ey ** 2 +
                ez ** 2
            )

            tracking_errors.append(err)

            msg = Float64()
            msg.data = err
            self.err_pub[i].publish(msg)

            msg = Float64()
            msg.data = ex
            self.err_x_pub[i].publish(msg)

            msg = Float64()
            msg.data = ey
            self.err_y_pub[i].publish(msg)

            msg = Float64()
            msg.data = ez
            self.err_z_pub[i].publish(msg)

        # -----------------------------------
        # Actual Inter-Drone Distances
        # -----------------------------------

        d12 = self.distance(
            self.positions[1],
            self.positions[2]
        )

        d13 = self.distance(
            self.positions[1],
            self.positions[3]
        )

        d23 = self.distance(
            self.positions[2],
            self.positions[3]
        )

        msg = Float64()
        msg.data = d12
        self.d12_pub.publish(msg)

        msg = Float64()
        msg.data = d13
        self.d13_pub.publish(msg)

        msg = Float64()
        msg.data = d23
        self.d23_pub.publish(msg)

        # -----------------------------------
        # Formation Shape Errors
        # -----------------------------------

        e12 = d12 - self.desired_d12
        e13 = d13 - self.desired_d13
        e23 = d23 - self.desired_d23

        msg = Float64()
        msg.data = e12
        self.shape12_pub.publish(msg)

        msg = Float64()
        msg.data = e13
        self.shape13_pub.publish(msg)

        msg = Float64()
        msg.data = e23
        self.shape23_pub.publish(msg)

        current_time = (
            self.get_clock().now()
            - self.start_time
        ).nanoseconds / 1e9

        self.csv_writer.writerow([
            current_time,
            tracking_errors[0],
            tracking_errors[1],
            tracking_errors[2],
            e12,
            e13,
            e23
        ])

        self.csv_file.flush()

    def destroy_node(self):

        self.csv_file.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = Plotter()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()