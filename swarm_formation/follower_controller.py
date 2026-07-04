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
        # k_y sets how hard the heading law (Eq.14) turns to kill LATERAL
        # error: with chi_inf = pi/2 the deflection is exactly atan(k_y*ey).
        # The paper's 0.02 was tuned for ~tens-of-metres lateral errors
        # (15 m/s leader, 300 m path). This scenario has ~1 m errors, so the
        # paper value gives only ~1 deg of turn for a 1 m offset -> ey never
        # nulls and the speed-law denominator chatters near zero. Scaling
        # k_y up ~25x restores heading authority: 1 m error -> ~27 deg turn.
        self.k_y = 0.5
        # k_D sets the LONGITUDINAL convergence rate: the speed law gives
        # Ddot = -k_D * D, so the formation error decays with time constant
        # 1/k_D. The paper's 0.02 -> 50 s constant, which crawls at this
        # scale. 0.1 -> ~10 s. Kept small enough that the commanded speed
        # (~ vgl - k_D*ex) stays well inside the speed limits for vgl=0.5.
        self.k_D = 0.1
        # Leader yaw rate (chi_l_dot) is now received directly from the leader
        # on /virtual_leader_velocity (twist.angular.z) as an EXACT analytic
        # value, instead of being finite-differenced from the noisy leader yaw.
        self.leader_yaw_rate = 0.0
        # Timestamps used to compute the REAL elapsed time between callbacks
        # instead of assuming a fixed dt. The old self.dt=0.01 was wrong for
        # ey_dot: the leader pose arrives at 100 Hz, but this controller's
        # own timer runs at 10 Hz, so ey_dot was being inflated 10x.
        self.last_ey_time = {1: None, 2: None, 3: None}

        # Practical safeguard (NOT from the paper): a slew-rate limit on the
        # commanded speed so a momentary denominator singularity in Eq.21
        # can't snap the command to saturation in a single 0.1 s tick.
        # Relaxed to 6 m/s^2 so it only catches genuine spikes and does not
        # throttle the law's normal convergence.
        self.max_accel = 6.0
        self.prev_speed = {1: 0.0, 2: 0.0, 3: 0.0}

        # ==================================================
        # TAKEOFF PHASE
        # ==================================================
        # PX4 handles the actual climb (VEHICLE_CMD_NAV_TAKEOFF). We just
        # watch our own odometry feed to know when each drone has reached
        # altitude, then switch THAT drone into OFFBOARD mode and hand
        # control over to the paper's guidance law.
        self.takeoff_alt = -5.0        # NED, negative = up
        self.takeoff_tol = 0.3         # m, altitude band to call it "airborne"
        self.airborne = {1: False, 2: False, 3: False}
        self.offboard_engaged = {1: False, 2: False, 3: False}

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
            3: 0.0
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

            1: (-3.0, 0.0),

            2: (0.0, 3.0),

            3: (3.0, 0.0)

        }

        # ==================================================
        # SPAWN OFFSETS (NED world frame)
        # --------------------------------------------------
        # PX4 multi-vehicle SITL reports vehicle_odometry relative to EACH
        # drone's OWN spawn point (its EKF origin), not a common world frame.
        # To put every drone in one shared world frame (the leader's NED
        # frame, origin = Gazebo origin), we add each drone's spawn offset.
        #
        # Values converted from start_swarm.sh PX4_GZ_MODEL_POSE, which are
        # Gazebo ENU (x=East, y=North, z=Up):  NED = (ENU_y, ENU_x, -ENU_z)
        # Spawns are now ALIGNED with the formation offsets g_i, so each drone
        # starts already on its target (zero xy error) and only has to hold
        # station until the leader begins moving -- no initial cross-track dash.
        #   D1 ENU(0,-3,0) -> NED(-3, 0,0) = g1
        #   D2 ENU(3, 0,0) -> NED( 0, 3,0) = g2
        #   D3 ENU(0, 3,0) -> NED( 3, 0,0) = g3
        # INVARIANT: this table must always equal the ENU->NED of the
        # PX4_GZ_MODEL_POSE values in start_swarm.sh.
        # ==================================================
        self.spawn_ned = {
            1: (-3.0, 0.0, 0.0),
            2: (0.0,  3.0, 0.0),
            3: (3.0,  0.0, 0.0),
        }

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

        # NOTE: leader yaw RATE (chi_l_dot) is no longer derived here.
        # It is supplied directly by the leader on /virtual_leader_velocity
        # (twist.angular.z) and handled in leader_velocity_callback().
    
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

    # ==================================================
    # ODOMETRY CALLBACKS
    # ==================================================

    def drone1_odom_callback(self, msg):

        # Lift local-spawn odometry into the shared NED world frame
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

        # Lift local-spawn odometry into the shared NED world frame
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

        # Lift local-spawn odometry into the shared NED world frame
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

        # Current drone position
        xd = self.drone_states[drone_id]["x"]
        yd = self.drone_states[drone_id]["y"]

        # Leader position
        xl = self.leader["x"]
        yl = self.leader["y"]

        # Leader heading
        yaw = self.leader["yaw"]

        # Relative position in world frame
        dx = xd - xl
        dy = yd - yl

        c = cos(yaw)
        s = sin(yaw)

        # Rotate into leader body frame
        x_body = dx*c + dy*s
        y_body = -dx*s + dy*c

        # Desired formation offset
        gx, gy = self.formation_offsets[drone_id]

        # Formation errors
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

        # Current heading
        chi = self.drone_states[drone_id]["yaw"]

        # Heading error
        chi_tilde = self.compute_heading_error(drone_id)

        # Lateral error
        _, ey = self.compute_formation_error(drone_id)

        # Lateral error derivative
        ey_dot = self.compute_ey_dot(drone_id)

        # Leader yaw rate
        chi_l_dot = self.leader_yaw_rate

        # Sliding term
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

        # Formation errors
        ex, ey = self.compute_formation_error(drone_id)

        # Desired formation offsets
        gx, gy = self.formation_offsets[drone_id]

        # Leader and follower headings
        chi_l = self.leader["yaw"]
        chi_i = self.drone_states[drone_id]["yaw"]

        # Leader speed
        leader_speed = sqrt(
            self.leader["vx"]**2 +
            self.leader["vy"]**2
        )

        # Leader heading rate
        chi_l_dot = self.leader_yaw_rate

        # Formation error magnitude squared
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

        # Prevent division by zero / near-singularity blow-up.
        # NOTE: forcing a hard floor like max(0.1, denom) makes |speed| jump
        # towards numerator/0.1 right as the sign flips - this is the
        # "shooting" spike. Clamping the speed slew-rate below fixes the
        # visible symptom regardless of how the denominator is regularized.
        if abs(denominator) < 0.1:
            denominator = 0.1 if denominator >= 0 else -0.1

        speed = numerator / denominator

        # Saturation. A negative LOWER bound is essential: Eq.21 legitimately
        # asks for a negative ground speed when a follower is on the far side
        # of its goal and must travel BACK along its heading to converge.
        # The old max(0.0, ...) floor made that impossible, so any drone that
        # got pushed out simply froze in place (the flat plateaus in the log).
        # Allowing reverse motion lets the Lyapunov speed law actually close
        # the formation error as the paper's Proposition 2 guarantees.
        speed = max(-3.0, min(speed, 8.0))

        # Slew-rate limit: cap how fast the commanded speed can change per
        # control cycle so a momentary singularity can't snap the command
        # straight to the saturation limit.
        prev = self.prev_speed[drone_id]
        max_step = self.max_accel * 0.1  # 0.1s = controller timer period
        speed = max(prev - max_step, min(speed, prev + max_step))
        self.prev_speed[drone_id] = speed

        # Throttled diagnostics (~1 Hz, drone 1). Watch that `denom` no longer
        # crosses zero at the same instant `speed` spikes, and that `vgl`
        # reads ~0.5 (if it reads ~15-20 the leader velocity fix isn't active).
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

            # ---------- Still under PX4's own NAV_TAKEOFF ----------
            # We are NOT in offboard mode yet, so this setpoint isn't even
            # consumed by PX4 - publishing zero/hold here is just a safe
            # default in case offboard somehow engages early.
            vx = 0.0
            vy = 0.0
            vz = 0.0
            chi_c = self.drone_states[drone_id]["yaw"]  # hold current heading

        else:

            # ---------- Paper Controller ----------
            # chi_c (Eq.15) is the STEERING command: it drives the heading,
            # it is NOT the direction of travel. In the paper's unicycle the
            # follower always moves ALONG ITS CURRENT HEADING chi_i at speed
            # vgi (Eq.21), while chi_c steers chi_i toward chi_d via the
            # first-order autopilot (Eq.4 -> PX4's yaw controller).
            #
            # Previously the velocity was applied along chi_c while the speed
            # law's denominator was computed for motion along chi_i. That
            # mismatch is what saturated the command and flung the drones out.
            # Applying the velocity along chi_i makes the realised motion
            # match the law that produced the magnitude.
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

        pubs = {1: self.cmd_pub_1, 2: self.cmd_pub_2, 3: self.cmd_pub_3}
        target_systems = {1: 2, 2: 3, 3: 4}

        for drone_id in (1, 2, 3):

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