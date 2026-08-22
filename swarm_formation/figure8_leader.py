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

        self.t = 0.0

        self.start_delay = 20.0     # seconds before leader starts moving
        self.elapsed_time = 0.0

        # Sinusoidal reference path (Basak & Ghosh, ICC 2025), y = A cos(kx)
        # progressing along +X (North). Amplitude/speed scaled down 30x from
        # the paper (300m/15m/s) to fit this arena. R_min = 1/(A k^2) must
        # stay >> |G_i| (4.24m for this formation) or the speed law's
        # denominator collapses near the turns; wavelength=130 gives ratio ~10.
        self.v = 0.5               # constant ground speed [m/s]
        self.amplitude = 10.0         # r in y = r sin(x), scaled from 300
        self.wavelength = 130.0       # [m] of forward (North) travel per period
        self.k = 2.0 * pi / self.wavelength

        # Smoothstep ramp so followers see a feasible acceleration instead of
        # a step 0->v; yaw rate is computed with the instantaneous ramped speed.
        self.ramp_time = 6.0          # [s] to reach full speed

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
        """Sinusoid path tangent heading and its exact time derivative."""
        A = self.amplitude
        k = self.k

        u = -A * k * sin(k * x)          # dy/dx = tan(chi); u(0) = 0
        yaw = atan(u)                    # heading from +X (North) axis

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

        if self.elapsed_time < self.start_delay:

            vx = 0.0
            vy = 0.0
            yaw = 0.0
            yaw_rate = 0.0

            self.elapsed_time += self.dt

        else:
            # C1-continuous smoothstep 3s^2 - 2s^3 on [0, ramp_time].
            if self.t < self.ramp_time:
                s = self.t / self.ramp_time
                v_now = self.v * (3.0 * s * s - 2.0 * s * s * s)
            else:
                v_now = self.v

            yaw, yaw_rate = self._heading(self.x, v_now)

            vx = v_now * cos(yaw)
            vy = v_now * sin(yaw)

            self.x += vx * self.dt
            self.y += vy * self.dt

            self.t += self.dt

        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z

        vel_msg.twist.linear.x = vx
        vel_msg.twist.linear.y = vy
        vel_msg.twist.linear.z = 0.0

        vel_msg.twist.angular.x = 0.0
        vel_msg.twist.angular.y = 0.0
        vel_msg.twist.angular.z = yaw_rate       # leader yaw rate (chi_l_dot)

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