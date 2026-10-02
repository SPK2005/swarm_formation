#!/usr/bin/env python3
"""
sinusoid_leader.py

Virtual leader following a sinusoidal reference path (Basak & Ghosh, ICC 2025,
Sec. IV: y = r sin(x)), scaled to the SITL arena.

Path used here:      y(x) = A * (cos(k*x) - 1)
so that y(0) = 0 and y'(0) = 0 -- the leader starts at the origin heading due
North with zero turn rate, which means the followers' carousel offset Delta
starts at zero and ramps in smoothly. (A sin-form path has y'(0) = A*k != 0,
which would demand a nonzero Delta at the instant guidance engages.)

Publishes
---------
/virtual_leader_pose      geometry_msgs/PoseStamped   x, y, z, quaternion(yaw)
/virtual_leader_velocity  geometry_msgs/TwistStamped  linear xy, angular.z = chi_l_dot
/virtual_leader_accel     geometry_msgs/AccelStamped  angular.z = chi_l_ddot   <-- NEW

chi_l_ddot is computed ANALYTICALLY from the path, not by finite-differencing
chi_l_dot. This matters: a backward difference of chi_l_dot injects a heading
kick of magnitude  chi_l_dot * v_gl * g_x / v_req^2  on the first sample, and
that kick is INDEPENDENT of the loop period (the 1/dt amplification and the dt
of integration cancel). Since the virtual leader is software, the second
derivative is available in closed form and there is no reason to difference it.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from math import sin, cos, atan, sqrt, pi, hypot, degrees


class SinusoidLeader(Node):

    def __init__(self):
        super().__init__('sinusoid_leader')

        # ==================================================
        # PARAMETERS
        # ==================================================
        self.declare_parameter('speed', 0.5)            # v_gl [m/s]
        self.declare_parameter('amplitude', 10.0)       # A [m]
        self.declare_parameter('wavelength', 130.0)     # [m] of North travel per period
        self.declare_parameter('altitude', -5.0)        # NED, negative = up
        self.declare_parameter('start_delay', 20.0)     # [s] hold before moving
        self.declare_parameter('ramp_time', 6.0)        # [s] smoothstep to full speed
        self.declare_parameter('publish_rate', 100.0)   # [Hz]
        self.declare_parameter('formation_radius', 4.2426)  # |G_i|, diagnostics only
        self.declare_parameter('g_x_max', 3.0)              # diagnostics only

        self.v = float(self.get_parameter('speed').value)
        self.amplitude = float(self.get_parameter('amplitude').value)
        self.wavelength = float(self.get_parameter('wavelength').value)
        self.z = float(self.get_parameter('altitude').value)
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.ramp_time = float(self.get_parameter('ramp_time').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.k = 2.0 * pi / self.wavelength
        self.dt = 1.0 / rate

        # ==================================================
        # STATE
        # ==================================================
        self.x = 0.0
        self.y = 0.0
        self.t = 0.0                # time since motion start
        self.elapsed_time = 0.0     # time since node start

        # ==================================================
        # PUBLISHERS
        # ==================================================
        self.pose_pub = self.create_publisher(
            PoseStamped, '/virtual_leader_pose', 10)

        self.vel_pub = self.create_publisher(
            TwistStamped, '/virtual_leader_velocity', 10)

        # NEW: analytic angular acceleration for the follower's dDelta/dt term.
        self.accel_pub = self.create_publisher(
            AccelStamped, '/virtual_leader_accel', 10)

        self.timer = self.create_timer(self.dt, self.timer_callback)

        self._log_geometry()

    # ------------------------------------------------------------------
    # PATH DERIVATIVES
    # ------------------------------------------------------------------
    def _path_derivs(self, x):
        """y', y'', y''' of  y(x) = A*(cos(k*x) - 1)  w.r.t. x."""
        A, k = self.amplitude, self.k
        y1 = -A * k * sin(k * x)
        y2 = -A * k * k * cos(k * x)
        y3 = A * k * k * k * sin(k * x)
        return y1, y2, y3

    def _heading(self, x, v, a):
        """
        Heading and its first two time derivatives along the path.

        With  chi = atan(y'),  w = 1 + y'^2,  and  xdot = v/sqrt(w):

            chi_dot  = y'' * v * w^(-3/2)

            chi_ddot = y'' * a * w^(-3/2)                 <- speed-ramp term
                     + v^2 * [ y''' * w^(-2)
                               - 3 * y' * y''^2 * w^(-3) ]

        The second line is the constant-speed result; the first vanishes once
        the smoothstep ramp finishes (a = 0).
        """
        y1, y2, y3 = self._path_derivs(x)
        w = 1.0 + y1 * y1
        w15 = w ** 1.5

        chi = atan(y1)
        chi_dot = y2 * v / w15
        chi_ddot = (y2 * a / w15
                    + v * v * (y3 / (w * w) - 3.0 * y1 * y2 * y2 / (w * w * w)))
        return chi, chi_dot, chi_ddot

    def _speed_profile(self, t):
        """C1-continuous smoothstep 3s^2 - 2s^3, and its derivative."""
        T = self.ramp_time
        if T <= 0.0 or t >= T:
            return self.v, 0.0
        s = t / T
        v = self.v * (3.0 * s * s - 2.0 * s * s * s)
        a = self.v * (6.0 * s - 6.0 * s * s) / T
        return v, a

    # ------------------------------------------------------------------
    # MAIN LOOP
    # ------------------------------------------------------------------
    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()

        if self.elapsed_time < self.start_delay:
            # Hold at the origin, heading North, no rotation. Followers see
            # Delta = 0 here, so guidance can engage with no initial mismatch.
            v_now, a_now = 0.0, 0.0
            chi, chi_dot, chi_ddot = 0.0, 0.0, 0.0
            vx = vy = 0.0
            self.elapsed_time += self.dt
        else:
            v_now, a_now = self._speed_profile(self.t)
            chi, chi_dot, chi_ddot = self._heading(self.x, v_now, a_now)

            vx = v_now * cos(chi)
            vy = v_now * sin(chi)

            self.x += vx * self.dt
            self.y += vy * self.dt
            self.t += self.dt

        # ---- pose ----
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sin(chi / 2.0)
        pose.pose.orientation.w = cos(chi / 2.0)
        self.pose_pub.publish(pose)

        # ---- velocity (angular.z = chi_l_dot) ----
        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = chi_dot
        self.vel_pub.publish(twist)

        # ---- acceleration (angular.z = chi_l_ddot) ----
        accel = AccelStamped()
        accel.header.stamp = stamp
        accel.accel.linear.x = a_now * cos(chi) - v_now * sin(chi) * chi_dot
        accel.accel.linear.y = a_now * sin(chi) + v_now * cos(chi) * chi_dot
        accel.accel.linear.z = 0.0
        accel.accel.angular.x = 0.0
        accel.accel.angular.y = 0.0
        accel.accel.angular.z = chi_ddot
        self.accel_pub.publish(accel)

    # ------------------------------------------------------------------
    # STARTUP DIAGNOSTICS
    # ------------------------------------------------------------------
    def _log_geometry(self):
        """
        Print the geometry that actually determines whether the carousel
        correction is measurable, and whether Eq. 21 stays well-conditioned.

        Key fact: with omega = v/R, the carousel offset is

            Delta = atan( g_x / (R - g_y) )

        i.e. PURELY GEOMETRIC. Raising the leader's speed does NOT increase
        Delta -- only shrinking the turn radius does. Tune amplitude and
        wavelength, not speed, if the correction is too small to measure.
        """
        A, k = self.amplitude, self.k
        gnorm = float(self.get_parameter('formation_radius').value)
        gx = float(self.get_parameter('g_x_max').value)
        gy = gx  # this formation has |g_x| = |g_y|

        R_min = 1.0 / (A * k * k) if A * k * k > 0 else float('inf')
        max_heading = atan(A * k)
        omega_max = self.v / R_min if R_min > 0 else 0.0

        denom = R_min - gy
        if denom > 0:
            delta_max = atan(gx / denom)
        else:
            delta_max = float('nan')

        self.get_logger().info(
            f"[sinusoid] v={self.v:.2f} m/s | A={self.amplitude:.1f} m | "
            f"wavelength={self.wavelength:.0f} m | "
            f"period={self.wavelength / max(self.v, 1e-6):.0f} s")
        self.get_logger().info(
            f"[geometry] max heading={degrees(max_heading):.1f} deg | "
            f"R_min={R_min:.1f} m | R_min/|G|={R_min / gnorm:.1f} (want >> 1) | "
            f"omega_max={omega_max:.4f} rad/s")
        self.get_logger().info(
            f"[carousel] Delta_max={degrees(delta_max):.2f} deg | "
            f"predicted uncorrected e_y bias = -tan(Delta)/k_y "
            f"= {-1.0 * (gx / denom) / 1.2:.3f} m  (assuming k_y=1.2)")

        # Warn if the correction will be lost in the noise.
        if abs(degrees(delta_max)) < 5.0:
            self.get_logger().warn(
                "[carousel] Delta_max < 5 deg -- the with/without-Delta "
                "difference may sit near the SITL noise floor. To increase it, "
                "SHRINK THE TURN RADIUS (raise amplitude or shorten "
                "wavelength). Raising 'speed' will NOT help: Delta = "
                "atan(g_x/(R-g_y)) is independent of v_gl.")

        # Warn if Eq. 21 is heading for trouble.
        if R_min < 4.0 * gnorm:
            self.get_logger().warn(
                f"[eq21] R_min/|G| = {R_min / gnorm:.1f} is small; the Eq. 21 "
                "denominator may approach zero near the sharpest turns. That "
                "is interesting for the singularity study but noisy for a "
                "clean carousel A/B comparison.")

        # Rotation-singularity boundary for the carousel formula itself.
        if omega_max * gy >= 0.2 * self.v:
            self.get_logger().warn(
                f"[carousel] omega_max*g_y = {omega_max * gy:.3f} vs "
                f"v_gl = {self.v:.3f}; the formation slot is approaching the "
                "leader's instantaneous turn centre. Delta becomes "
                "ill-conditioned as omega*g_y -> v_gl.")


def main(args=None):
    rclpy.init(args=args)
    node = SinusoidLeader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
