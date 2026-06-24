#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from px4_msgs.msg import (
    VehicleCommand,
    TrajectorySetpoint,
    OffboardControlMode
)


class FormationController(Node):

    def __init__(self):

        super().__init__('formation_controller')

        qos = rclpy.qos.QoSProfile(
    		depth=10,
    		durability=rclpy.qos.DurabilityPolicy.VOLATILE,
    		reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT
	)

        # ==================================================
        # TARGET STORAGE
        # ==================================================

        self.drone_targets = {
            1: [0.0, 0.0, -5.0],
            2: [0.0, 0.0, -5.0],
            3: [0.0, 0.0, -5.0]
        }

        # ==================================================
        # SUBSCRIBERS
        # ==================================================

        self.create_subscription(
            PoseStamped,
            '/drone1_target',
            self.drone1_target_callback,
            qos
        )

        self.create_subscription(
            PoseStamped,
            '/drone2_target',
            self.drone2_target_callback,
            qos
        )

        self.create_subscription(
            PoseStamped,
            '/drone3_target',
            self.drone3_target_callback,
            qos
        )

        # ==================================================
        # PX4_1
        # ==================================================

        self.offboard_pub_1 = self.create_publisher(
            OffboardControlMode,
            '/px4_1/fmu/in/offboard_control_mode',
            qos
        )

        self.traj_pub_1 = self.create_publisher(
            TrajectorySetpoint,
            '/px4_1/fmu/in/trajectory_setpoint',
            qos
        )

        self.cmd_pub_1 = self.create_publisher(
            VehicleCommand,
            '/px4_1/fmu/in/vehicle_command',
            qos
        )

        # ==================================================
        # PX4_2
        # ==================================================

        self.offboard_pub_2 = self.create_publisher(
            OffboardControlMode,
            '/px4_2/fmu/in/offboard_control_mode',
            qos
        )

        self.traj_pub_2 = self.create_publisher(
            TrajectorySetpoint,
            '/px4_2/fmu/in/trajectory_setpoint',
            qos
        )

        self.cmd_pub_2 = self.create_publisher(
            VehicleCommand,
            '/px4_2/fmu/in/vehicle_command',
            qos
        )

        # ==================================================
        # PX4_3
        # ==================================================

        self.offboard_pub_3 = self.create_publisher(
            OffboardControlMode,
            '/px4_3/fmu/in/offboard_control_mode',
            qos
        )

        self.traj_pub_3 = self.create_publisher(
            TrajectorySetpoint,
            '/px4_3/fmu/in/trajectory_setpoint',
            qos
        )

        self.cmd_pub_3 = self.create_publisher(
            VehicleCommand,
            '/px4_3/fmu/in/vehicle_command',
            qos
        )

        self.counter = 0

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info(
            'Formation Controller Started'
        )

    # ==================================================
    # TARGET CALLBACKS
    # ==================================================

    def drone1_target_callback(self, msg):

        self.drone_targets[1] = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]

    def drone2_target_callback(self, msg):

        self.drone_targets[2] = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]

    def drone3_target_callback(self, msg):

        self.drone_targets[3] = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]

    # ==================================================
    # PX4 FUNCTIONS
    # ==================================================

    def publish_offboard(self, publisher):

        msg = OffboardControlMode()

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        publisher.publish(msg)

    def publish_setpoint(self, publisher, target):

        msg = TrajectorySetpoint()

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        msg.position = [
            float(target[0]),
            float(target[1]),
            float(target[2])
        ]

        msg.velocity = [0.0, 0.0, 0.0]

        msg.acceleration = [0.0, 0.0, 0.0]

        msg.yaw = 0.0

        publisher.publish(msg)

    def publish_vehicle_command(
        self,
        publisher,
        target_system,
        command,
        param1=0.0,
        param2=0.0
    ):

        msg = VehicleCommand()

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        msg.command = command

        msg.param1 = param1
        msg.param2 = param2

        msg.target_system = target_system
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        publisher.publish(msg)

    # ==================================================
    # TIMER LOOP
    # ==================================================

    def timer_callback(self):

        # OFFBOARD STREAM

        self.publish_offboard(self.offboard_pub_1)
        self.publish_offboard(self.offboard_pub_2)
        self.publish_offboard(self.offboard_pub_3)

        # TARGETS

        self.publish_setpoint(
            self.traj_pub_1,
            self.drone_targets[1]
        )

        self.publish_setpoint(
            self.traj_pub_2,
            self.drone_targets[2]
        )

        self.publish_setpoint(
            self.traj_pub_3,
            self.drone_targets[3]
        )

        # ENTER OFFBOARD

        if self.counter == 50:

            self.publish_vehicle_command(
                self.cmd_pub_1,
                2,
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )

            self.publish_vehicle_command(
                self.cmd_pub_2,
                3,
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )

            self.publish_vehicle_command(
                self.cmd_pub_3,
                4,
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )

            self.get_logger().info("OFFBOARD ENABLED")

        # ARM

        if self.counter == 60:

            self.publish_vehicle_command(
                self.cmd_pub_1,
                2,
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0
            )

            self.publish_vehicle_command(
                self.cmd_pub_2,
                3,
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0
            )

            self.publish_vehicle_command(
                self.cmd_pub_3,
                4,
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0
            )

            self.get_logger().info("ALL DRONES ARMED")

        self.counter += 1


def main(args=None):

    rclpy.init(args=args)

    node = FormationController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
