#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped,TwistStamped
from math import atan2, sin, cos, pi, sqrt

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

        self.chi_inf = pi/2
        # Scaled ~25x over the paper's 0.02, which was tuned for tens-of-metres
        # lateral errors; at this scale's ~1m errors it gave too little turn authority.
        self.k_y = 1.2
        # Longitudinal convergence rate (Ddot = -k_D*D); raised from the paper's
        # 0.02 (50s time constant) for a ~2.5s constant at this scale.
        self.k_D = 0.4
        # Exact leader yaw rate from /virtual_leader_velocity, not finite-differenced.
        self.leader_yaw_rate = 0.0
        # Real elapsed time between callbacks (leader pose @100Hz vs 10Hz timer).
        self.last_ey_time = {1: None, 2: None, 3: None, 4: None}

        # Slew-rate limit (not from the paper): stops a momentary Eq.21
        # denominator singularity from snapping the command to saturation.
        self.max_accel = 1.5
        self.prev_speed = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}

        # ==================================================
        # TAKEOFF PHASE
        # ==================================================
        # PX4 handles the climb; we watch odometry to know when to switch
        # each drone into OFFBOARD and hand control to the guidance law.
        self.takeoff_alt = -5.0        # NED, negative = up
        self.takeoff_tol = 0.3         # m, altitude band to call it "airborne"
        self.airborne = {1: False, 2: False, 3: False, 4: False}
        self.offboard_engaged = {1: False, 2: False, 3: False, 4: False}

        # ==================================================
        # PAPER CONTROLLER PARAMETERS
        # ==================================================

        self.alpha = 1.65       # Heading dynamics gain
        self.eta = pi/4         # Sliding gain

        self.n = 3.0
        self.m = 5.0

        self.prev_ey = {
            1: 0.0,
            2: 0.0,
            3: 0.0,
            4: 0.0
        }

        # ==================================================
        # LEADER STATE
        # ==================================================

        self.leader = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0
        }

        self.formation_offsets = {

            1: (3.0, 3.0),

            2: (-3.0, 3.0),

            3: (3.0, -3.0),

            4: (-3.0, -3.0)

        }

        # ==================================================
        # SPAWN OFFSETS (NED world frame)
        # --------------------------------------------------
        # PX4 SITL reports odometry relative to each drone's own spawn point,
        # not a shared world frame; this offset lifts it into the leader's NED
        # frame. Must match PX4_GZ_MODEL_POSE in start_swarm.sh and
        # formation_offsets g_i below (drones spawn already on-target).
        # ==================================================
        self.spawn_ned = {
            1: (3.0,  3.0, 0.0),
            2: (-3.0, 3.0, 0.0),
            3: (3.0, -3.0, 0.0),
            4: (-3.0,-3.0, 0.0),
        }

        # ==================================================
        # TARGET STORAGE
        # ==================================================

        self.drone_targets = {
            1: [0.0, 0.0, -5.0],
            2: [0.0, 0.0, -5.0],
            3: [0.0, 0.0, -5.0],
            4: [0.0, 0.0, -5.0]
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
            },
            4: {
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
            PoseStamped,
            "/virtual_leader_pose",
            self.leader_callback,
            10
        )

        self.create_subscription(
            TwistStamped,
            "/virtual_leader_velocity",
            self.leader_velocity_callback,
            10
        )

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

        self.create_subscription(
            VehicleOdometry,
            '/px4_4/fmu/out/vehicle_odometry',
            self.drone4_odom_callback,
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

        self.create_subscription(
            PoseStamped,
            '/drone4_target',
            self.drone4_target_callback,
            qos
        )

        # PX4_1
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

        # PX4_2
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

        # PX4_3
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

        # PX4_4
        self.offboard_pub_4 = self.create_publisher(
            OffboardControlMode,
            '/px4_4/fmu/in/offboard_control_mode',
            qos
        )

        self.traj_pub_4 = self.create_publisher(
            TrajectorySetpoint,
            '/px4_4/fmu/in/trajectory_setpoint',
            qos
        )

        self.cmd_pub_4 = self.create_publisher(
            VehicleCommand,
            '/px4_4/fmu/in/vehicle_command',
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

    def leader_callback(self, msg):

        self.leader["x"] = msg.pose.position.x
        self.leader["y"] = msg.pose.position.y
        self.leader["z"] = msg.pose.position.z
   
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        yaw = atan2(
            2.0*(qw*qz + qx*qy),
            1.0 - 2.0*(qy*qy + qz*qz)
        )

        self.leader["yaw"] = yaw

    
    def leader_velocity_callback(self, msg):

        self.leader["vx"] = msg.twist.linear.x
        self.leader["vy"] = msg.twist.linear.y
        self.leader["vz"] = msg.twist.linear.z

        # Exact, noise-free leader yaw rate published by the leader node.
        self.leader_yaw_rate = msg.twist.angular.z

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

    def drone4_target_callback(self, msg):

        self.drone_targets[4] = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]

    # ==================================================
    # ODOMETRY CALLBACKS
    # ==================================================

    def drone1_odom_callback(self, msg):

        self.drone_states[1]["x"] = msg.position[0] + self.spawn_ned[1][0]
        self.drone_states[1]["y"] = msg.position[1] + self.spawn_ned[1][1]
        self.drone_states[1]["z"] = msg.position[2] + self.spawn_ned[1][2]

        self.drone_states[1]["vx"] = msg.velocity[0]
        self.drone_states[1]["vy"] = msg.velocity[1]
        self.drone_states[1]["vz"] = msg.velocity[2]

        qw = msg.q[0]
        qx = msg.q[1]
        qy = msg.q[2]
        qz = msg.q[3]

        yaw = atan2(
            2.0 * (qw*qz + qx*qy),
            1.0 - 2.0 * (qy*qy + qz*qz)
        )

        self.drone_states[1]["yaw"] = yaw

    def drone2_odom_callback(self, msg):

        self.drone_states[2]["x"] = msg.position[0] + self.spawn_ned[2][0]
        self.drone_states[2]["y"] = msg.position[1] + self.spawn_ned[2][1]
        self.drone_states[2]["z"] = msg.position[2] + self.spawn_ned[2][2]

        self.drone_states[2]["vx"] = msg.velocity[0]
        self.drone_states[2]["vy"] = msg.velocity[1]
        self.drone_states[2]["vz"] = msg.velocity[2]

        qw = msg.q[0]
        qx = msg.q[1]
        qy = msg.q[2]
        qz = msg.q[3]

        yaw = atan2(
            2.0 * (qw*qz + qx*qy),
            1.0 - 2.0 * (qy*qy + qz*qz)
        )

        self.drone_states[2]["yaw"] = yaw

    def drone3_odom_callback(self, msg):

        self.drone_states[3]["x"] = msg.position[0] + self.spawn_ned[3][0]
        self.drone_states[3]["y"] = msg.position[1] + self.spawn_ned[3][1]
        self.drone_states[3]["z"] = msg.position[2] + self.spawn_ned[3][2]

        self.drone_states[3]["vx"] = msg.velocity[0]
        self.drone_states[3]["vy"] = msg.velocity[1]
        self.drone_states[3]["vz"] = msg.velocity[2]

        qw = msg.q[0]
        qx = msg.q[1]
        qy = msg.q[2]
        qz = msg.q[3]

        yaw = atan2(
            2.0 * (qw*qz + qx*qy),
            1.0 - 2.0 * (qy*qy + qz*qz)
        )

        self.drone_states[3]["yaw"] = yaw

    def drone4_odom_callback(self, msg):

        self.drone_states[4]["x"] = msg.position[0] + self.spawn_ned[4][0]
        self.drone_states[4]["y"] = msg.position[1] + self.spawn_ned[4][1]
        self.drone_states[4]["z"] = msg.position[2] + self.spawn_ned[4][2]

        self.drone_states[4]["vx"] = msg.velocity[0]
        self.drone_states[4]["vy"] = msg.velocity[1]
        self.drone_states[4]["vz"] = msg.velocity[2]

        qw = msg.q[0]
        qx = msg.q[1]
        qy = msg.q[2]
        qz = msg.q[3]

        yaw = atan2(
            2.0 * (qw*qz + qx*qy),
            1.0 - 2.0 * (qy*qy + qz*qz)
        )

        self.drone_states[4]["yaw"] = yaw

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
    
    def compute_formation_error(self, drone_id):

        xd = self.drone_states[drone_id]["x"]
        yd = self.drone_states[drone_id]["y"]

        xl = self.leader["x"]
        yl = self.leader["y"]

        yaw = self.leader["yaw"]

        dx = xd - xl
        dy = yd - yl

        c = cos(yaw)
        s = sin(yaw)

        x_body = dx*c + dy*s
        y_body = -dx*s + dy*c

        gx, gy = self.formation_offsets[drone_id]

        ex = x_body - gx
        ey = y_body - gy

        return ex, ey
    
    def compute_desired_heading(self, drone_id):

        _, ey = self.compute_formation_error(drone_id)

        leader_heading = self.leader["yaw"]

        chi_d = (
            leader_heading
            -
            self.chi_inf
            * (2.0/3.141592653589793)
            * atan2(self.k_y*ey,1.0)
        )

        return chi_d

    def wrap_angle(self, angle):

        while angle > pi:
            angle -= 2.0 * pi

        while angle < -pi:
            angle += 2.0 * pi

        return angle

    def compute_heading_error(self, drone_id):

        current_heading = self.drone_states[drone_id]["yaw"]

        desired_heading = self.compute_desired_heading(drone_id)

        heading_error = self.wrap_angle(
            current_heading - desired_heading
        )

        return heading_error
    
    def compute_ey_dot(self, drone_id):

        _, ey = self.compute_formation_error(drone_id)

        now = self.get_clock().now()
        last_t = self.last_ey_time[drone_id]

        if last_t is None:
            ey_dot = 0.0
        else:
            dt = (now - last_t).nanoseconds * 1e-9
            dt = max(dt, 1e-3)
            ey_dot = (ey - self.prev_ey[drone_id]) / dt

        self.prev_ey[drone_id] = ey
        self.last_ey_time[drone_id] = now

        return ey_dot

    def signed_power(self, x, p):

        return abs(x)**p * (1 if x >= 0 else -1)
    
    def compute_command_heading(self, drone_id):

        chi = self.drone_states[drone_id]["yaw"]

        chi_tilde = self.compute_heading_error(drone_id)

        _, ey = self.compute_formation_error(drone_id)

        ey_dot = self.compute_ey_dot(drone_id)

        chi_l_dot = self.leader_yaw_rate

        sliding = self.signed_power(
            chi_tilde,
            self.n/self.m
        )

        chi_c = (
            chi
            +
            chi_l_dot/self.alpha
            -
            (
                2.0 * self.chi_inf
                /
                (self.alpha*3.14159265)
            )
            *
            (
                self.k_y
                /
                (1 + (self.k_y*ey)**2)
            )
            *
            ey_dot
            -
            (self.eta/self.alpha)
            * sliding
        )

        return self.wrap_angle(chi_c)
    
    def compute_command_speed(self, drone_id):

        ex, ey = self.compute_formation_error(drone_id)

        gx, gy = self.formation_offsets[drone_id]

        chi_l = self.leader["yaw"]
        chi_i = self.drone_states[drone_id]["yaw"]

        leader_speed = sqrt(
            self.leader["vx"]**2 +
            self.leader["vy"]**2
        )

        chi_l_dot = self.leader_yaw_rate

        D2 = ex**2 + ey**2

        # Numerator (Eq. 21)
        numerator = (
            leader_speed * ex
            - self.k_D * D2
            - chi_l_dot * (ex * gy - ey * gx)
        )

        # Denominator (Eq. 21)
        denominator = (
            ex * cos(chi_i - chi_l)
            + ey * sin(chi_i - chi_l)
        )

        # Regularized speed law (smooth form of Eq. 21): blends num/den with
        # leader-speed-hold as den->0, avoiding the chatter of a hard switch.
        # eps is an error floor (den <= D_i always) -- must sit below the
        # target formation accuracy. 0.05 -> ~sub-0.1m accuracy at this scale.
        EPS_DEN = 0.05
        reg = denominator**2 + EPS_DEN**2
        speed = (numerator * denominator + leader_speed * EPS_DEN**2) / reg

        # Heading-alignment gate: Eq. 21 assumes the heading has already
        # converged (it's a cascade with the heading law). Before that, blend
        # toward leader-speed matching so a large heading error can't command
        # a reverse-speed spike; align=1 when aligned, ->0 past 90deg error.
        chi_d = (
            chi_l
            - self.chi_inf * (2.0 / 3.14159265) * atan2(self.k_y * ey, 1.0)
        )
        align = max(0.0, cos(chi_i - chi_d))
        speed = leader_speed + (speed - leader_speed) * align

        speed = max(-0.3, min(speed, 1.2))     # sized to the 0.5 m/s regime

        prev = self.prev_speed[drone_id]
        max_step = self.max_accel * 0.1  # 0.1s = controller timer period
        speed = max(prev - max_step, min(speed, prev + max_step))
        self.prev_speed[drone_id] = speed

        # Throttled diagnostics (~1 Hz, drone 1).
        if drone_id == 1 and (self.counter % 10 == 0):
            self.get_logger().info(
                f"d1 speed={speed:+.2f} denom={denominator:+.3f} "
                f"vgl={leader_speed:.2f} chi_l_dot={chi_l_dot:+.3f} "
                f"ex={ex:+.2f} ey={ey:+.2f} D={D2**0.5:.2f}"
            )

        return speed
    
    def compute_command_vz(self, drone_id, target_z=-5.0, kz=0.5, vz_max=1.5):
        z = self.drone_states[drone_id]["z"]
        vz = kz * (target_z - z)          # target_z in NED, negative = up
        return max(-vz_max, min(vz_max, vz))

    def publish_velocity_setpoint(self, publisher, drone_id):

        if not self.airborne[drone_id]:
            # Still under PX4's own NAV_TAKEOFF, not consumed yet -- safe default.
            vx = 0.0
            vy = 0.0
            vz = 0.0
            chi_c = self.drone_states[drone_id]["yaw"]  # hold current heading

        else:
            # chi_c (Eq.15) is the STEERING command, not the direction of
            # travel: the follower moves along its current heading chi_i at
            # speed vgi (Eq.21), while chi_c steers chi_i toward chi_d.
            chi_c = self.compute_command_heading(drone_id)   # steering only
            chi_i = self.drone_states[drone_id]["yaw"]       # direction of travel
            speed = self.compute_command_speed(drone_id)
            vx = speed * cos(chi_i)
            vy = speed * sin(chi_i)
            vz = self.compute_command_vz(drone_id)

        msg = TrajectorySetpoint()

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        msg.position = [
            float("nan"),
            float("nan"),
            float("nan")
        ]

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

        msg.yaw = float(chi_c)

        publisher.publish(msg)

    def publish_vehicle_command(
        self,
        publisher,
        target_system,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0
    ):

        msg = VehicleCommand()

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        msg.command = command

        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7

        msg.target_system = target_system
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        publisher.publish(msg)

    def check_takeoff_complete(self):
        """Per-drone: once altitude is within tolerance of takeoff_alt,
        switch THAT drone into OFFBOARD and let the paper's guidance law
        take over. Runs every timer tick; only fires once per drone."""

        pubs = {1: self.cmd_pub_1, 2: self.cmd_pub_2,
                3: self.cmd_pub_3, 4: self.cmd_pub_4}
        target_systems = {1: 2, 2: 3, 3: 4, 4: 5}

        for drone_id in (1, 2, 3, 4):

            if self.offboard_engaged[drone_id]:
                continue

            z = self.drone_states[drone_id]["z"]

            if abs(self.takeoff_alt - z) > self.takeoff_tol:
                continue

            self.publish_vehicle_command(
                pubs[drone_id],
                target_systems[drone_id],
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )

            self.offboard_engaged[drone_id] = True
            self.airborne[drone_id] = True

            self.get_logger().info(
                f"Drone {drone_id} reached takeoff altitude - OFFBOARD ENABLED"
            )

    # ==================================================
    # TIMER LOOP
    # ==================================================

    def timer_callback(self):

        # OFFBOARD STREAM

        self.publish_offboard(self.offboard_pub_1)
        self.publish_offboard(self.offboard_pub_2)
        self.publish_offboard(self.offboard_pub_3)
        self.publish_offboard(self.offboard_pub_4)

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

        self.publish_velocity_setpoint(
            self.traj_pub_4,
            4
        )

      

        # Watch altitude every tick; switches a drone into OFFBOARD the
        # moment IT individually finishes its PX4-handled climb.
        self.check_takeoff_complete()

        # ARM

        if self.counter == 50:

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

            self.publish_vehicle_command(
                self.cmd_pub_4,
                5,
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0
            )

            self.get_logger().info("ALL DRONES ARMED")

        # NAV_TAKEOFF - let PX4 handle the climb to 5m altitude.
        # param5/param6 = NaN tells PX4 to use the current lat/lon
        # (local/relative takeoff) instead of a global target.
        # param7 = target altitude.

        if self.counter == 70:

            self.publish_vehicle_command(
                self.cmd_pub_1,
                2,
                VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                0.0, 0.0, 0.0, 0.0,
                float('nan'), float('nan'),
                5.0
            )

            self.publish_vehicle_command(
                self.cmd_pub_2,
                3,
                VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                0.0, 0.0, 0.0, 0.0,
                float('nan'), float('nan'),
                5.0
            )

            self.publish_vehicle_command(
                self.cmd_pub_3,
                4,
                VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                0.0, 0.0, 0.0, 0.0,
                float('nan'), float('nan'),
                5.0
            )

            self.publish_vehicle_command(
                self.cmd_pub_4,
                5,
                VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                0.0, 0.0, 0.0, 0.0,
                float('nan'), float('nan'),
                5.0
            )

            self.get_logger().info("NAV_TAKEOFF sent - climbing to 5m")

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