#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from px4_msgs.msg import (
    VehicleCommand,
    TrajectorySetpoint,
    OffboardControlMode,
    VehicleOdometry
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

        self.drone_states = {
            1: {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "vz": 0.0,
                "yaw": 0.0
            },
            2: {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "vz": 0.0,
                "yaw": 0.0
            },
            3: {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "vz": 0.0,
                "yaw": 0.0
            }
        }
        
        # ==================================================
        # ODOMETRY SUBSCRIBERS
        # ==================================================

        self.create_subscription(
            VehicleOdometry,
            '/px4_1/fmu/out/vehicle_odometry',
            self.drone1_odom_callback,
            qos
        )

        self.create_subscription(
            VehicleOdometry,
            '/px4_2/fmu/out/vehicle_odometry',
            self.drone2_odom_callback,
            qos
        )

        self.create_subscription(
            VehicleOdometry,
            '/px4_3/fmu/out/vehicle_odometry',
            self.drone3_odom_callback,
            qos
        )

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
    # ODOMETRY CALLBACKS
    # ==================================================

    def drone1_odom_callback(self, msg):

        self.drone_states[1]["x"] = msg.position[0]
        self.drone_states[1]["y"] = msg.position[1]
        self.drone_states[1]["z"] = msg.position[2]

        self.drone_states[1]["vx"] = msg.velocity[0]
        self.drone_states[1]["vy"] = msg.velocity[1]
        self.drone_states[1]["vz"] = msg.velocity[2]


    def drone2_odom_callback(self, msg):

        self.drone_states[2]["x"] = msg.position[0]
        self.drone_states[2]["y"] = msg.position[1]
        self.drone_states[2]["z"] = msg.position[2]

        self.drone_states[2]["vx"] = msg.velocity[0]
        self.drone_states[2]["vy"] = msg.velocity[1]
        self.drone_states[2]["vz"] = msg.velocity[2]


    def drone3_odom_callback(self, msg):

        self.drone_states[3]["x"] = msg.position[0]
        self.drone_states[3]["y"] = msg.position[1]
        self.drone_states[3]["z"] = msg.position[2]

        self.drone_states[3]["vx"] = msg.velocity[0]
        self.drone_states[3]["vy"] = msg.velocity[1]
        self.drone_states[3]["vz"] = msg.velocity[2]


    # ==================================================
    # PX4 FUNCTIONS
    # ==================================================

    def publish_offboard(self, publisher):

        msg = OffboardControlMode()

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        publisher.publish(msg)

    def compute_error(self, drone_id):

        ex = self.drone_targets[drone_id][0] - self.drone_states[drone_id]["x"]

        ey = self.drone_targets[drone_id][1] - self.drone_states[drone_id]["y"]

        ez = self.drone_targets[drone_id][2] - self.drone_states[drone_id]["z"]

        return ex, ey, ez
    
    def position_controller(self, drone_id):

        ex, ey, ez = self.compute_error(drone_id)

        kp = 0.8
        vmax = 2.0

        vx = max(min(kp * ex, vmax), -vmax)
        vy = max(min(kp * ey, vmax), -vmax)
        vz = max(min(kp * ez, 1.0), -1.0)

        return vx, vy, vz

    def publish_velocity_setpoint(self, publisher, drone_id):

        vx, vy, vz = self.position_controller(drone_id)

        msg = TrajectorySetpoint()

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        # Ignore position
        msg.position = [
            float("nan"),
            float("nan"),
            float("nan")
        ]

        # Velocity generated by our controller
        msg.velocity = [
            float(vx),
            float(vy),
            float(vz)
        ]

        msg.acceleration = [
            0.0,
            0.0,
            0.0
        ]

        msg.yaw = float("nan")

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

        self.publish_velocity_setpoint(
            self.traj_pub_1,
            1
        )

        self.publish_velocity_setpoint(
            self.traj_pub_2,
            2
        )

        self.publish_velocity_setpoint(
            self.traj_pub_3,
            3
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
