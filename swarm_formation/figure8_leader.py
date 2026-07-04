#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from math import sin, cos, tanh, radians, pi


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
        # Smooth zigzag trajectory (constant speed, rounded corners)
        # ------------------------------------------------------------
        # The heading is a smooth square-ish wave:
        #     chi(t) = leg_angle * tanh(sharp * sin(w t)) / tanh(sharp)
        # so it DWELLS at +/-leg_angle (the straight legs) and sweeps
        # smoothly between them (the rounded corners). Position is then
        # generated from  xdot = v cos(chi),  ydot = v sin(chi), which
        # makes the speed EXACTLY v at all times. Both the heading and
        # its rate are analytic, so the leader publishes an exact,
        # noise-free yaw rate on twist.angular.z.
        #
        # Design rule (learned the hard way from the figure-8): keep the
        # minimum turn radius  R_min = v / max|chi_dot|  well ABOVE the
        # formation offset (~3 m), or the speed law's denominator gets
        # driven through zero in the corners.
        # ============================================================
        self.v = 0.5                  # constant ground speed [m/s]
        self.leg_angle = radians(30)  # zigzag leg angle off the +X axis [rad]
        self.zig_omega = 0.09         # [rad/s]; corner spacing = pi/omega
        self.corner_sharp = 1.6       # >1 = straighter legs/sharper corners;
                                      #  ~1 = gentle serpentine

        # Integrated leader position (starts at the origin like before)
        self.x = 0.0
        self.y = 0.0
        self.z = -5.0

        # Report the worst-case turn radius vs the formation offset
        k = tanh(self.corner_sharp)
        chidot_max = self.leg_angle * self.corner_sharp * self.zig_omega / k
        self.get_logger().info(
            f"[zigzag] v={self.v:.2f} m/s | max yaw rate={chidot_max:.3f} rad/s | "
            f"min turn radius={self.v / chidot_max:.1f} m "
            f"(keep >> ~3 m offset) | corner every {pi / self.zig_omega:.0f} s"
        )

    def _heading(self, t):
        """Smooth zigzag heading and its exact time derivative."""
        w = self.zig_omega
        a = self.corner_sharp
        k = tanh(a)

        s = sin(w * t)
        c = cos(w * t)
        th = tanh(a * s)

        yaw = self.leg_angle * th / k
        # d/dt tanh(a sin wt) = (1 - tanh^2) * a w cos wt
        yaw_rate = self.leg_angle * a * w * (1.0 - th * th) * c / k
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
            # Smooth zigzag
            # ============================================

            yaw, yaw_rate = self._heading(self.t)

            # Constant-speed velocity along the heading
            vx = self.v * cos(yaw)
            vy = self.v * sin(yaw)

            # Integrate position (exact constant speed = self.v)
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