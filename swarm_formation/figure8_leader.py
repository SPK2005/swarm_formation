#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from math import sin, cos, atan, sqrt, pi, radians


class VirtualLeader(Node):

    def __init__(self):

        super().__init__('virtual_leader')

        self.publisher = self.create_publisher(
            PoseStamped,
            '/virtual_leader_pose',
            10
        )

        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            "/virtual_leader_velocity",
            10
        )

        self.dt = 0.01

        self.timer = self.create_timer(
            self.dt,
            self.timer_callback
        )

        # Trajectory time
        self.t = 0.0

        # -------------------------------
        # Delay before leader starts moving
        # -------------------------------
        self.start_delay = 20.0     # seconds
        self.elapsed_time = 0.0

        # ============================================================
        # Sinusoidal reference path  (Basak & Ghosh, ICC 2025)
        # ------------------------------------------------------------
        # The paper's reference path is  y = r sin(x)  with r = 300 m,
        # flown by a 15 m/s leader. Here the leader is 30x slower
        # (0.5 m/s), so the amplitude is scaled by the same factor
        # (300 -> ~10 m) to keep the same path SHAPE at this arena size.
        #
        # We progress along +X (North) and let Y (East) trace the sine:
        #       y(x) = A cos(k x)      (phase chosen so chi(0) = 0, i.e.
        #                               the leader leaves the hold heading
        #                               due North with no heading jump)
        # Moving at CONSTANT speed v, the heading is the path tangent
        #       chi(x)     = atan( dy/dx ) = atan( -A k sin(k x) )
        # and its exact rate (using  dx/dt = v cos chi) is
        #       chi_dot(x) = -A k^2 cos(k x) * v / (1 + (A k sin k x)^2)^1.5
        # Both analytic, so twist.angular.z is an exact, noise-free yaw rate.
        #
        # FEASIBILITY RULE (the key one): the speed law's denominator is the
        # projection of the formation error onto the follower's heading, and
        # it is driven toward zero when the path curves on a scale comparable
        # to the formation. Minimum turn radius is
        #       R_min = 1 / (A k^2),      k = 2 pi / wavelength
        # Keep R_min / |G_i| >> 1  (|G_i| = 4.24 m for the (+/-3,+/-3)
        # quadrant formation). wavelength = 130 m gives R_min ~ 42.8 m,
        # ratio ~ 10. Sweeps show this cuts turn-time formation error ~72%
        # and commanded-speed variance ~75% versus wavelength 60 / the old
        # zigzag (ratio 1.4-2.1, where den sits near zero half the run).
        # ============================================================
        self.v = 0.5               # constant ground speed [m/s]
        self.amplitude = 10.0         # r in y = r sin(x), scaled from 300
        self.wavelength = 130.0       # [m] of forward (North) travel per period
        self.k = 2.0 * pi / self.wavelength

        # Speed ramp: stepping 0 -> v instantly is not trackable by the
        # followers (they lag ~0.4-1.0 m along-track and take ~3.4 s to
        # recover -- the start-up spike in the D_i plot). Ease the leader in
        # with a C1-continuous smoothstep over `ramp_time` so the followers
        # see a feasible acceleration. Note the yaw RATE must then be computed
        # with the INSTANTANEOUS speed, since chi_dot = (dchi/dx) * v(t)cos(chi).
        self.ramp_time = 6.0          # [s] to reach full speed

        # Integrated leader position (starts at the origin)
        self.x = 0.0
        self.y = 0.0
        self.z = -5.0

        formation_offset = 4.2426     # |G_i| for the (+/-3, +/-3) quadrant
        max_heading = atan(self.amplitude * self.k)
        R_min = 1.0 / (self.amplitude * self.k * self.k)
        self.get_logger().info(
            f"[sinusoid] v={self.v:.2f} m/s | A={self.amplitude:.1f} m | "
            f"wavelength={self.wavelength:.0f} m | max heading="
            f"{max_heading*180/pi:.0f} deg | min turn radius={R_min:.1f} m | "
            f"R_min/|G|={R_min/formation_offset:.1f} (want >> 1) | "
            f"period={self.wavelength/self.v:.0f} s"
        )

    def _heading(self, x, v_now):
        """Sinusoid path tangent heading and its exact time derivative.

        x     : current North coordinate (progress along the path).
        v_now : instantaneous ground speed (may be ramping).
        Path: y = A cos(k x), so dy/dx = -A k sin(k x). The heading depends
        only on x; its RATE scales with the current speed.
        """
        A = self.amplitude
        k = self.k

        u = -A * k * sin(k * x)          # dy/dx = tan(chi); u(0) = 0
        yaw = atan(u)                    # heading from +X (North) axis

        # chi_dot = d/dt atan(u) = [u' / (1+u^2)] * dx/dt,  dx/dt = v_now cos(chi)
        yaw_rate = (
            -A * k * k * cos(k * x) * v_now
            / (1.0 + u * u) ** 1.5
        )
        return yaw, yaw_rate

    def timer_callback(self):

        msg = PoseStamped()
        vel_msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.stamp = msg.header.stamp

        # ============================================
        # Wait before starting trajectory
        # ============================================

        if self.elapsed_time < self.start_delay:

            vx = 0.0
            vy = 0.0
            yaw = 0.0
            yaw_rate = 0.0

            self.elapsed_time += self.dt

        else:

            # ============================================
            # Sinusoid with a smooth speed ramp at start
            # ============================================

            # C1-continuous smoothstep 3s^2 - 2s^3 on [0, ramp_time]:
            # zero velocity AND zero acceleration at t=0, full speed after.
            if self.t < self.ramp_time:
                s = self.t / self.ramp_time
                v_now = self.v * (3.0 * s * s - 2.0 * s * s * s)
            else:
                v_now = self.v

            yaw, yaw_rate = self._heading(self.x, v_now)

            # Velocity along the heading at the current (possibly ramped) speed
            vx = v_now * cos(yaw)
            vy = v_now * sin(yaw)

            # Integrate position
            self.x += vx * self.dt
            self.y += vy * self.dt

            self.t += self.dt

        # ============================================
        # Position
        # ============================================

        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z

        # ============================================
        # Velocity
        #   linear.{x,y} = leader ground velocity
        #   angular.z    = leader yaw rate (chi_l_dot)
        # ============================================

        vel_msg.twist.linear.x = vx
        vel_msg.twist.linear.y = vy
        vel_msg.twist.linear.z = 0.0

        vel_msg.twist.angular.x = 0.0
        vel_msg.twist.angular.y = 0.0
        vel_msg.twist.angular.z = yaw_rate

        # ============================================
        # Orientation
        # ============================================

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = sin(yaw / 2.0)
        msg.pose.orientation.w = cos(yaw / 2.0)

        self.publisher.publish(msg)
        self.velocity_publisher.publish(vel_msg)


def main(args=None):

    rclpy.init(args=args)

    node = VirtualLeader()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()