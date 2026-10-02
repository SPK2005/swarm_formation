#!/usr/bin/env python3
"""
follower_controller.py

Virtual-leader formation path-following controller (Basak & Ghosh, ICC 2025)
with two additions over the paper:

  1. Eq. 21 speed law with epsilon-regularization of the denominator
     (already present in the previous version -- UNCHANGED here).

  2. NEW: carousel correction Delta on the heading channel.

     The paper's Eq. 14 sets the follower's desired heading target to the
     leader's heading chi_l when e_yi = 0. Substituting chi_i = chi_l into the
     paper's own Eq. 13 gives  e_yi_dot = -chi_l_dot * g_xi, which is nonzero
     whenever the leader turns and the follower has a forward offset. So
     (chi_i = chi_l, e_y = 0) is not an equilibrium of Eq. 13 on a curved path.

     The system does not diverge; it parks at a stable point where the vector
     field's correction cancels the residual drift:

         e_y_ss = -tan(Delta)/k_y ,   Delta = atan2(omega*g_x, v_gl - omega*g_y)

     Adding Delta to the heading target removes that bias.

Delta is purely rigid-body kinematics: v_point = v_centre + omega x offset.
For a constant-curvature turn of radius R it reduces to

     Delta = atan( g_x / (R - g_y) )

which is INDEPENDENT of the leader's speed. If the correction is too small to
measure, shrink the turn radius -- do not raise v_gl.

Toggle the correction at runtime for A/B comparison:
    ros2 run <pkg> follower_controller --ros-args -p enable_delta:=false
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from math import atan2, sin, cos, pi, sqrt, hypot, degrees

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
        # RUNTIME PARAMETERS (for A/B testing without editing code)
        # ==================================================
        self.declare_parameter('enable_delta', True)
        self.declare_parameter('enable_delta_dot', True)
        self.declare_parameter('max_yaw_rate', 0.8)      # rad/s, slew on chi_c
        self.declare_parameter('fwd_min_frac', 0.25)     # guard on v_gl - omega*g_y

        self.enable_delta = bool(self.get_parameter('enable_delta').value)
        self.enable_delta_dot = bool(self.get_parameter('enable_delta_dot').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.fwd_min_frac = float(self.get_parameter('fwd_min_frac').value)

        self.chi_inf = pi / 2
        # Scaled ~25x over the paper's 0.02, which was tuned for tens-of-metres
        # lateral errors; at this scale's ~1m errors it gave too little turn authority.
        self.k_y = 1.2
        # Longitudinal convergence rate (Ddot = -k_D*D); raised from the paper's
        # 0.02 (50s time constant) for a ~2.5s constant at this scale.
        self.k_D = 0.4
        # Exact leader yaw rate from /virtual_leader_velocity, not finite-differenced.
        self.leader_yaw_rate = 0.0
        # NEW: exact leader yaw ACCELERATION from /virtual_leader_accel.
        # Never finite-difference this -- see the module docstring of
        # sinusoid_leader.py for why the resulting kick is dt-invariant.
        self.leader_yaw_accel = 0.0
        self.got_leader_accel = False
        # Real elapsed time between callbacks (leader pose @100Hz vs 10Hz timer).
        self.last_ey_time = {1: None, 2: None, 3: None, 4: None}

        # Slew-rate limit (not from the paper): stops a momentary Eq.21
        # denominator singularity from snapping the command to saturation.
        self.max_accel = 1.5
        self.prev_speed = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}

        # NEW: previous commanded heading, for the chi_c slew limit. Magnitude
        # clipping alone still passes a step discontinuity, and steps excite
        # the airframe's lightly-damped yaw mode.
        self.prev_chi_c = {1: None, 2: None, 3: None, 4: None}

        # NEW: throttle for the carousel ill-conditioning warning.
        self._delta_warn_counter = 0

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
        self.eta = pi / 4       # Sliding gain

        self.n = 3.0
        self.m = 5.0

        self.prev_ey = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}

        # ==================================================
        # LEADER STATE
        # ==================================================

        self.leader = {
            "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0,
            "vx": 0.0, "vy": 0.0, "vz": 0.0
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
            1: (3.0, 3.0, 0.0),
            2: (-3.0, 3.0, 0.0),
            3: (3.0, -3.0, 0.0),
            4: (-3.0, -3.0, 0.0),
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
            i: {"x": 0.0, "y": 0.0, "z": 0.0,
                "vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0}
            for i in (1, 2, 3, 4)
        }

        # ==================================================
        # LEADER SUBSCRIBERS
        # ==================================================

        self.create_subscription(
            PoseStamped, "/virtual_leader_pose", self.leader_callback, 10)

        self.create_subscription(
            TwistStamped, "/virtual_leader_velocity",
            self.leader_velocity_callback, 10)

        # NEW: analytic chi_l_ddot from the leader node.
        self.create_subscription(
            AccelStamped, "/virtual_leader_accel",
            self.leader_accel_callback, 10)

        # ==================================================
        # ODOMETRY SUBSCRIBERS
        # ==================================================

        self.create_subscription(
            VehicleOdometry, '/px4_1/fmu/out/vehicle_odometry',
            self.drone1_odom_callback, qos)

        self.create_subscription(
            VehicleOdometry, '/px4_2/fmu/out/vehicle_odometry',
            self.drone2_odom_callback, qos)

        self.create_subscription(
            VehicleOdometry, '/px4_3/fmu/out/vehicle_odometry',
            self.drone3_odom_callback, qos)

        self.create_subscription(
            VehicleOdometry, '/px4_4/fmu/out/vehicle_odometry',
            self.drone4_odom_callback, qos)

        # ==================================================
        # TARGET SUBSCRIBERS
        # ==================================================

        self.create_subscription(
            PoseStamped, '/drone1_target', self.drone1_target_callback, qos)
        self.create_subscription(
            PoseStamped, '/drone2_target', self.drone2_target_callback, qos)
        self.create_subscription(
            PoseStamped, '/drone3_target', self.drone3_target_callback, qos)
        self.create_subscription(
            PoseStamped, '/drone4_target', self.drone4_target_callback, qos)

        # ==================================================
        # PUBLISHERS
        # ==================================================

        # PX4_1
        self.offboard_pub_1 = self.create_publisher(
            OffboardControlMode, '/px4_1/fmu/in/offboard_control_mode', qos)
        self.traj_pub_1 = self.create_publisher(
            TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub_1 = self.create_publisher(
            VehicleCommand, '/px4_1/fmu/in/vehicle_command', qos)

        # PX4_2
        self.offboard_pub_2 = self.create_publisher(
            OffboardControlMode, '/px4_2/fmu/in/offboard_control_mode', qos)
        self.traj_pub_2 = self.create_publisher(
            TrajectorySetpoint, '/px4_2/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub_2 = self.create_publisher(
            VehicleCommand, '/px4_2/fmu/in/vehicle_command', qos)

        # PX4_3
        self.offboard_pub_3 = self.create_publisher(
            OffboardControlMode, '/px4_3/fmu/in/offboard_control_mode', qos)
        self.traj_pub_3 = self.create_publisher(
            TrajectorySetpoint, '/px4_3/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub_3 = self.create_publisher(
            VehicleCommand, '/px4_3/fmu/in/vehicle_command', qos)

        # PX4_4
        self.offboard_pub_4 = self.create_publisher(
            OffboardControlMode, '/px4_4/fmu/in/offboard_control_mode', qos)
        self.traj_pub_4 = self.create_publisher(
            TrajectorySetpoint, '/px4_4/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub_4 = self.create_publisher(
            VehicleCommand, '/px4_4/fmu/in/vehicle_command', qos)

        self.counter = 0
        self.dt_ctrl = 0.1
        self.timer = self.create_timer(self.dt_ctrl, self.timer_callback)

        self.get_logger().info('Formation Controller Started')
        self.get_logger().info(
            f"[carousel] enable_delta={self.enable_delta} "
            f"enable_delta_dot={self.enable_delta_dot} "
            f"max_yaw_rate={self.max_yaw_rate:.2f} rad/s")

    # ==================================================
    # LEADER CALLBACKS
    # ==================================================

    def leader_callback(self, msg):

        self.leader["x"] = msg.pose.position.x
        self.leader["y"] = msg.pose.position.y
        self.leader["z"] = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        self.leader["yaw"] = atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

    def leader_velocity_callback(self, msg):

        self.leader["vx"] = msg.twist.linear.x
        self.leader["vy"] = msg.twist.linear.y
        self.leader["vz"] = msg.twist.linear.z

        # Exact, noise-free leader yaw rate published by the leader node.
        self.leader_yaw_rate = msg.twist.angular.z

    def leader_accel_callback(self, msg):
        """NEW: exact chi_l_ddot, computed analytically by the leader node."""
        self.leader_yaw_accel = msg.accel.angular.z
        self.got_leader_accel = True

    # ==================================================
    # TARGET CALLBACKS
    # ==================================================

    def drone1_target_callback(self, msg):
        self.drone_targets[1] = [msg.pose.position.x,
                                 msg.pose.position.y,
                                 msg.pose.position.z]

    def drone2_target_callback(self, msg):
        self.drone_targets[2] = [msg.pose.position.x,
                                 msg.pose.position.y,
                                 msg.pose.position.z]

    def drone3_target_callback(self, msg):
        self.drone_targets[3] = [msg.pose.position.x,
                                 msg.pose.position.y,
                                 msg.pose.position.z]

    def drone4_target_callback(self, msg):
        self.drone_targets[4] = [msg.pose.position.x,
                                 msg.pose.position.y,
                                 msg.pose.position.z]

    # ==================================================
    # ODOMETRY CALLBACKS
    # ==================================================

    def _store_odom(self, drone_id, msg):
        sx, sy, sz = self.spawn_ned[drone_id]

        self.drone_states[drone_id]["x"] = msg.position[0] + sx
        self.drone_states[drone_id]["y"] = msg.position[1] + sy
        self.drone_states[drone_id]["z"] = msg.position[2] + sz

        self.drone_states[drone_id]["vx"] = msg.velocity[0]
        self.drone_states[drone_id]["vy"] = msg.velocity[1]
        self.drone_states[drone_id]["vz"] = msg.velocity[2]

        qw, qx, qy, qz = msg.q[0], msg.q[1], msg.q[2], msg.q[3]

        self.drone_states[drone_id]["yaw"] = atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

    def drone1_odom_callback(self, msg):
        self._store_odom(1, msg)

    def drone2_odom_callback(self, msg):
        self._store_odom(2, msg)

    def drone3_odom_callback(self, msg):
        self._store_odom(3, msg)

    def drone4_odom_callback(self, msg):
        self._store_odom(4, msg)

    # ==================================================
    # PX4 FUNCTIONS
    # ==================================================

    def publish_offboard(self, publisher):

        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

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

        x_body = dx * c + dy * s
        y_body = -dx * s + dy * c

        gx, gy = self.formation_offsets[drone_id]

        ex = x_body - gx
        ey = y_body - gy

        return ex, ey

    # ==================================================
    # NEW: CAROUSEL CORRECTION
    # ==================================================

    def leader_speed(self):
        return sqrt(self.leader["vx"] ** 2 + self.leader["vy"] ** 2)

    def compute_carousel(self, drone_id):
        """
        Heading offset Delta and required ground speed for a follower holding
        a rigid offset G_i in the leader's rotating body frame.

            v_fwd  = v_gl - omega*g_y
            v_side = omega*g_x
            Delta  = atan2(v_side, v_fwd)
            v_req  = hypot(v_fwd, v_side)

        Frame check (NED): body x is forward, body y is starboard, and yaw rate
        omega is positive nose-right about +z (down). Then
        omega_vec x r = omega*(-g_y, g_x), which is exactly the pair above --
        the same expression as the ENU/CCW derivation, so no sign flip is
        needed here.

        Returns (Delta, v_req, well_conditioned).
        """
        gx, gy = self.formation_offsets[drone_id]
        omega = self.leader_yaw_rate
        v_gl = self.leader_speed()

        # Leader stationary (start delay): no rotation to correct for.
        if v_gl < 1e-3:
            return 0.0, 0.0, True

        v_fwd = v_gl - omega * gy
        v_side = omega * gx

        # GUARD. When omega*g_y -> v_gl the formation slot sits at the leader's
        # instantaneous turn centre: its forward velocity vanishes, Delta swings
        # toward +/-90 deg, and past the boundary the slot is swept BACKWARDS.
        # Boundary: omega* = v_gl / g_y.
        floor = self.fwd_min_frac * v_gl
        well_conditioned = v_fwd > floor

        if not well_conditioned:
            v_fwd = floor if gy >= 0 else -floor
            self._delta_warn_counter += 1
            if self._delta_warn_counter % 20 == 1:
                self.get_logger().warn(
                    f"[carousel] d{drone_id}: v_gl - omega*g_y = "
                    f"{v_gl - omega * gy:+.3f} below floor {floor:.3f}; "
                    f"slot approaching the turn centre (omega={omega:+.4f}, "
                    f"boundary={v_gl / max(abs(gy), 1e-6):.4f} rad/s). "
                    "Delta clamped.")

        return atan2(v_side, v_fwd), hypot(v_fwd, v_side), well_conditioned

    def compute_delta_dot(self, drone_id, v_req):
        """
        dDelta/dt = chi_l_ddot * v_gl * g_x / v_req^2

        Derived by the atan2 quotient rule; the omega*g_x*g_y cross terms
        cancel exactly. Nothing in the derivation assumes constant curvature.

        chi_l_ddot comes from /virtual_leader_accel (analytic). If that topic
        is absent this returns 0 -- i.e. 'pointwise' mode, which the numpy
        study showed already removes >99.7% of the uncorrected error. Do NOT
        substitute a finite difference of chi_l_dot here.
        """
        if not (self.enable_delta and self.enable_delta_dot):
            return 0.0
        if not self.got_leader_accel:
            return 0.0
        if v_req < 1e-3:
            return 0.0

        gx, _ = self.formation_offsets[drone_id]
        v_gl = self.leader_speed()

        return self.leader_yaw_accel * v_gl * gx / (v_req * v_req)

    # ==================================================
    # HEADING GUIDANCE (Eq. 14 / Eq. 15, + Delta)
    # ==================================================

    def compute_desired_heading(self, drone_id):
        """Eq. 14 with the carousel correction added."""

        _, ey = self.compute_formation_error(drone_id)

        leader_heading = self.leader["yaw"]

        delta = 0.0
        if self.enable_delta:
            delta, _, _ = self.compute_carousel(drone_id)

        chi_d = (
            leader_heading
            + delta
            - self.chi_inf * (2.0 / pi) * atan2(self.k_y * ey, 1.0)
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

        return self.wrap_angle(current_heading - desired_heading)

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
        return abs(x) ** p * (1 if x >= 0 else -1)

    def compute_command_heading(self, drone_id):
        """
        Eq. 15 plus the dDelta/dt feedforward term.

        chi_c = chi_i + (1/alpha)*chi_d_dot - (eta/alpha)*sgn(chi_t)|chi_t|^(n/m)

        where chi_d_dot = chi_l_dot + Delta_dot - (vector-field drift term).
        Omitting Delta_dot while including Delta leaves a residual steady
        heading offset of size (|Delta_dot|/eta)^(m/n) -- the same mechanism as
        dropping any true feedforward from a reaching law.
        """
        chi = self.drone_states[drone_id]["yaw"]

        chi_tilde = self.compute_heading_error(drone_id)

        _, ey = self.compute_formation_error(drone_id)

        ey_dot = self.compute_ey_dot(drone_id)

        chi_l_dot = self.leader_yaw_rate

        _, v_req, _ = self.compute_carousel(drone_id)
        delta_dot = self.compute_delta_dot(drone_id, v_req)

        sliding = self.signed_power(chi_tilde, self.n / self.m)

        chi_c = (
            chi
            + chi_l_dot / self.alpha
            + delta_dot / self.alpha
            - (2.0 * self.chi_inf / (self.alpha * pi))
            * (self.k_y / (1 + (self.k_y * ey) ** 2))
            * ey_dot
            - (self.eta / self.alpha) * sliding
        )

        return self.wrap_angle(chi_c)

    # ==================================================
    # SPEED GUIDANCE (Eq. 21 + epsilon regularization) -- UNCHANGED
    # ==================================================

    def compute_command_speed(self, drone_id):

        ex, ey = self.compute_formation_error(drone_id)

        gx, gy = self.formation_offsets[drone_id]

        chi_l = self.leader["yaw"]
        chi_i = self.drone_states[drone_id]["yaw"]

        leader_speed = self.leader_speed()

        chi_l_dot = self.leader_yaw_rate

        D2 = ex ** 2 + ey ** 2

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
        reg = denominator ** 2 + EPS_DEN ** 2
        speed = (numerator * denominator + leader_speed * EPS_DEN ** 2) / reg

        # Heading-alignment gate: Eq. 21 assumes the heading has already
        # converged (it's a cascade with the heading law). Before that, blend
        # toward leader-speed matching so a large heading error can't command
        # a reverse-speed spike; align=1 when aligned, ->0 past 90deg error.
        # NOTE: chi_d here must match compute_desired_heading(), Delta included,
        # or the gate closes spuriously whenever the carousel offset is large.
        chi_d = self.compute_desired_heading(drone_id)
        align = max(0.0, cos(self.wrap_angle(chi_i - chi_d)))
        speed = leader_speed + (speed - leader_speed) * align

        speed = max(-0.3, min(speed, 1.2))     # sized to the 0.5 m/s regime

        prev = self.prev_speed[drone_id]
        max_step = self.max_accel * self.dt_ctrl
        speed = max(prev - max_step, min(speed, prev + max_step))
        self.prev_speed[drone_id] = speed

        return speed

    def compute_command_vz(self, drone_id, target_z=-5.0, kz=0.5, vz_max=1.5):
        z = self.drone_states[drone_id]["z"]
        vz = kz * (target_z - z)          # target_z in NED, negative = up
        return max(-vz_max, min(vz_max, vz))

    # ==================================================
    # NEW: HEADING COMMAND SLEW LIMIT
    # ==================================================

    def slew_limit_heading(self, drone_id, chi_c):
        """
        Bound the RATE of change of the commanded heading, not just its value.

        Magnitude clipping still lets chi_c step discontinuously between ticks,
        and a step is broadband -- it deposits energy at the airframe's yaw
        resonance. Bounding the step size makes the command continuous.
        """
        prev = self.prev_chi_c[drone_id]

        if prev is None:
            self.prev_chi_c[drone_id] = chi_c
            return chi_c

        max_step = self.max_yaw_rate * self.dt_ctrl
        d = self.wrap_angle(chi_c - prev)

        if d > max_step:
            d = max_step
        elif d < -max_step:
            d = -max_step

        out = self.wrap_angle(prev + d)
        self.prev_chi_c[drone_id] = out
        return out

    # ==================================================
    # SETPOINT PUBLISHING
    # ==================================================

    def publish_velocity_setpoint(self, publisher, drone_id):

        if not self.airborne[drone_id]:
            # Still under PX4's own NAV_TAKEOFF, not consumed yet -- safe default.
            vx = 0.0
            vy = 0.0
            vz = 0.0
            chi_c = self.drone_states[drone_id]["yaw"]  # hold current heading
            # Keep the slew filter seeded so engagement doesn't see a step.
            self.prev_chi_c[drone_id] = chi_c

        else:
            # chi_c (Eq.15) is the STEERING command, not the direction of
            # travel: the follower moves along its current heading chi_i at
            # speed vgi (Eq.21), while chi_c steers chi_i toward chi_d.
            chi_c = self.compute_command_heading(drone_id)   # steering only
            chi_c = self.slew_limit_heading(drone_id, chi_c)
            chi_i = self.drone_states[drone_id]["yaw"]       # direction of travel
            speed = self.compute_command_speed(drone_id)
            vx = speed * cos(chi_i)
            vy = speed * sin(chi_i)
            vz = self.compute_command_vz(drone_id)

        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        msg.position = [float("nan"), float("nan"), float("nan")]
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.acceleration = [0.0, 0.0, 0.0]
        msg.yaw = float(chi_c)

        publisher.publish(msg)

    def publish_vehicle_command(self, publisher, target_system, command,
                                param1=0.0, param2=0.0, param3=0.0,
                                param4=0.0, param5=0.0, param6=0.0,
                                param7=0.0):

        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

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
        take over. Runs every timer tick; only fires once per drone.

        NOTE on engagement transients: the leader node holds station for
        start_delay seconds and then ramps its speed with a smoothstep, so
        omega = 0 and therefore Delta = 0 at the moment guidance engages. The
        carousel offset grows continuously from zero. That is why no explicit
        'yaw to chi_l + Delta before engaging' phase is needed here -- the
        leader's start sequence already provides a consistent initial
        condition. If start_delay is ever removed, or the leader is started
        mid-turn, that alignment step becomes necessary."""

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
                1.0, 6.0
            )

            self.offboard_engaged[drone_id] = True
            self.airborne[drone_id] = True

            self.get_logger().info(
                f"Drone {drone_id} reached takeoff altitude - OFFBOARD ENABLED")

    # ==================================================
    # DIAGNOSTICS
    # ==================================================

    def log_diagnostics(self, drone_id=1):
        """~1 Hz heartbeat for one drone."""

        ex, ey = self.compute_formation_error(drone_id)
        delta, v_req, ok = self.compute_carousel(drone_id)
        chi_i = self.drone_states[drone_id]["yaw"]
        chi_l = self.leader["yaw"]
        chi_err = self.wrap_angle(chi_i - chi_l)

        gx, gy = self.formation_offsets[drone_id]

        chi_l_dot = self.leader_yaw_rate
        v_gl = self.leader_speed()

        # Eq. 21 denominator, and its conditioning measure. The RAW denominator
        # shrinks simply because D -> 0; what actually signals the singularity
        # is the NORMALISED value cos(angle between e and the heading), which
        # is what goes to zero when the error is perpendicular to heading.
        den = ex * cos(chi_i - chi_l) + ey * sin(chi_i - chi_l)
        D = hypot(ex, ey)
        den_norm = den / D if D > 1e-6 else float('nan')

        self.get_logger().info(
            f"d{drone_id} "
            f"ex={ex:+.3f} ey={ey:+.3f} D={D:.3f} | "
            f"Delta={degrees(delta):+.2f}deg chi_err={degrees(chi_err):+.2f}deg "
            f"(target {degrees(delta):+.2f}) | "
            f"den={den:+.3f} den/D={den_norm:+.3f} | "
            f"vgl={v_gl:.2f} chi_l_dot={chi_l_dot:+.4f} "
            f"chi_l_ddot={self.leader_yaw_accel:+.5f}"
            f"{'' if ok else '  [DELTA CLAMPED]'}")

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

        self.publish_velocity_setpoint(self.traj_pub_1, 1)
        self.publish_velocity_setpoint(self.traj_pub_2, 2)
        self.publish_velocity_setpoint(self.traj_pub_3, 3)
        self.publish_velocity_setpoint(self.traj_pub_4, 4)

        # Watch altitude every tick; switches a drone into OFFBOARD the
        # moment IT individually finishes its PX4-handled climb.
        self.check_takeoff_complete()

        if self.counter % 10 == 0 and self.airborne[1]:
            self.log_diagnostics(1)

        # Warn once if the leader never publishes acceleration.
        if self.counter == 100 and self.enable_delta_dot and not self.got_leader_accel:
            self.get_logger().warn(
                "[carousel] no /virtual_leader_accel received -- running in "
                "'pointwise' mode (Delta without dDelta/dt). This is a valid "
                "configuration; the numpy study showed pointwise removes "
                ">99.7% of the uncorrected error on its own.")

        # ARM

        if self.counter == 50:

            self.publish_vehicle_command(
                self.cmd_pub_1, 2,
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.publish_vehicle_command(
                self.cmd_pub_2, 3,
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.publish_vehicle_command(
                self.cmd_pub_3, 4,
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.publish_vehicle_command(
                self.cmd_pub_4, 5,
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

            self.get_logger().info("ALL DRONES ARMED")

        # NAV_TAKEOFF - let PX4 handle the climb to 5m altitude.
        # param5/param6 = NaN tells PX4 to use the current lat/lon
        # (local/relative takeoff) instead of a global target.
        # param7 = target altitude.

        if self.counter == 70:

            for pub, tgt in ((self.cmd_pub_1, 2), (self.cmd_pub_2, 3),
                             (self.cmd_pub_3, 4), (self.cmd_pub_4, 5)):
                self.publish_vehicle_command(
                    pub, tgt,
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

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()