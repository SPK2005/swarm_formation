#!/usr/bin/env python3
"""
formation_logger.py

ROS 2 logging node for the virtual-leader multi-UAV formation path-following
stack. Logs leader + follower pose/speed/heading and derived formation errors
(D_i, exi, eyi) to CSV, computed from raw states + the known offsets G_i so
the logger stays independent of the controller internals.
"""

import csv
import math
import os
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from geometry_msgs.msg import PoseStamped, TwistStamped
from px4_msgs.msg import VehicleOdometry


# mirrors follower_controller.py; keep in sync
LEADER_POSE_TOPIC = "/virtual_leader_pose"        # PoseStamped
LEADER_VEL_TOPIC = "/virtual_leader_velocity"     # TwistStamped

@dataclass
class FollowerCfg:
    name: str
    odom_topic: str                          # px4_msgs/VehicleOdometry
    spawn_ned: Tuple[float, float]           # (north, east) offset to world frame
    goal_offset: Tuple[float, float]         # G_i = (g_xi, g_yi) in leader body frame


# spawn_ned and goal_offset copied verbatim from follower_controller.py
# (spawns are aligned to the formation offsets, so spawn_ned == goal_offset here).
FOLLOWERS: List[FollowerCfg] = [
    FollowerCfg("f1", "/px4_1/fmu/out/vehicle_odometry", ( 3.0, 3.0), ( 3.0, 3.0)),
    FollowerCfg("f2", "/px4_2/fmu/out/vehicle_odometry", (-3.0, 3.0), (-3.0, 3.0)),
    FollowerCfg("f3", "/px4_3/fmu/out/vehicle_odometry", ( 3.0,-3.0), ( 3.0,-3.0)),
    FollowerCfg("f4", "/px4_4/fmu/out/vehicle_odometry", (-3.0,-3.0), (-3.0,-3.0)),
]

# Seconds between diagnostic heartbeat log lines (0 disables).
DIAG_PERIOD_S = 2.0


def wrap_pi(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(qw: float, qx: float, qy: float, qz: float) -> float:
    """Yaw (rad) about the down/z axis, same formula the controller uses."""
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


@dataclass
class LeaderState:
    x: float = math.nan
    y: float = math.nan
    chi: float = math.nan          # heading = yaw(quaternion)
    vx: float = 0.0
    vy: float = 0.0
    speed: float = 0.0
    chidot: float = math.nan       # twist.angular.z
    got_pose: bool = False
    got_vel: bool = False
    n_pose: int = 0
    n_vel: int = 0


@dataclass
class FollowerState:
    x: float = math.nan
    y: float = math.nan
    chi: float = math.nan          # heading = yaw(quaternion)
    vx: float = 0.0
    vy: float = 0.0
    speed: float = 0.0
    chidot: float = math.nan       # angular_velocity[2] (body yaw rate)
    got: bool = False
    n_msgs: int = 0


class FormationLogger(Node):
    def __init__(self):
        super().__init__("formation_logger")

        self.declare_parameter("output_dir", os.path.expanduser("~/formation_logs"))
        self.declare_parameter("log_rate_hz", 50.0)
        self.declare_parameter("num_followers", 4)

        out_dir = self.get_parameter("output_dir").value
        rate = float(self.get_parameter("log_rate_hz").value)
        n_foll = int(self.get_parameter("num_followers").value)
        if not 1 <= n_foll <= len(FOLLOWERS):
            raise ValueError(
                f"num_followers={n_foll} outside 1..{len(FOLLOWERS)}")
        self.followers: List[FollowerCfg] = FOLLOWERS[:n_foll]

        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(out_dir, f"formation_{stamp}.csv")

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.leader = LeaderState()
        self.foll: Dict[str, FollowerState] = {f.name: FollowerState() for f in self.followers}
        self.fcfg: Dict[str, FollowerCfg] = {f.name: f for f in self.followers}

        # --- subscriptions --------------------------------------------------
        self.create_subscription(PoseStamped, LEADER_POSE_TOPIC,
                                 self._leader_pose_cb, 10)
        self.create_subscription(TwistStamped, LEADER_VEL_TOPIC,
                                 self._leader_vel_cb, 10)
        for f in self.followers:
            self.create_subscription(
                VehicleOdometry, f.odom_topic, self._make_foll_cb(f.name), px4_qos)
            self.get_logger().info(f"subscribed {f.name} <- {f.odom_topic}")
        self.get_logger().info(f"subscribed leader <- {LEADER_POSE_TOPIC} , "
                               f"{LEADER_VEL_TOPIC}")

        # --- CSV ------------------------------------------------------------
        self._csv_file = open(self.csv_path, "w", newline="")
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(self._header())
        self._t0: Optional[float] = None
        self._rows = 0
        self.get_logger().info(
            f"logging to {self.csv_path} "
            f"({len(self.followers)} follower(s): "
            f"{', '.join(f.name for f in self.followers)})")

        # --- timers ---------------------------------------------------------
        self.create_timer(1.0 / rate, self._on_timer)
        if DIAG_PERIOD_S > 0:
            self.create_timer(DIAG_PERIOD_S, self._on_diag)

    # ------------------------------------------------------------- callbacks
    def _leader_pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        self.leader.x = p.x
        self.leader.y = p.y
        self.leader.chi = yaw_from_quat(q.w, q.x, q.y, q.z)
        self.leader.got_pose = True
        self.leader.n_pose += 1

    def _leader_vel_cb(self, msg: TwistStamped):
        self.leader.vx = msg.twist.linear.x
        self.leader.vy = msg.twist.linear.y
        self.leader.speed = math.hypot(self.leader.vx, self.leader.vy)
        self.leader.chidot = msg.twist.angular.z
        self.leader.got_vel = True
        self.leader.n_vel += 1

    def _make_foll_cb(self, name: str):
        off = self.fcfg[name].spawn_ned
        def cb(msg: VehicleOdometry):
            s = self.foll[name]
            s.x = msg.position[0] + off[0]
            s.y = msg.position[1] + off[1]
            s.vx = msg.velocity[0]
            s.vy = msg.velocity[1]
            s.speed = math.hypot(s.vx, s.vy)
            s.chi = yaw_from_quat(msg.q[0], msg.q[1], msg.q[2], msg.q[3])  # [w,x,y,z]
            s.chidot = msg.angular_velocity[2]
            s.got = True
            s.n_msgs += 1
        return cb

    # ---------------------------------------------------------------- header
    def _header(self) -> List[str]:
        cols = ["t", "leader_x", "leader_y", "leader_chi",
                "leader_speed", "leader_chidot"]
        for f in self.followers:
            n = f.name
            cols += [f"{n}_x", f"{n}_y", f"{n}_chi",
                     f"{n}_speed", f"{n}_chidot",
                     f"{n}_xL", f"{n}_yL",
                     f"{n}_exi", f"{n}_eyi", f"{n}_Di",
                     f"{n}_chi_err"]
        return cols

    # ----------------------------------------------------------------- timer
    def _on_timer(self):
        lead = self.leader
        if not lead.got_pose:
            return
        if not all(self.foll[f.name].got for f in self.followers):
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0

        row = [f"{t:.4f}",
               f"{lead.x:.4f}", f"{lead.y:.4f}", f"{lead.chi:.6f}",
               f"{lead.speed:.4f}", f"{lead.chidot:.6f}"]

        c, s = math.cos(lead.chi), math.sin(lead.chi)
        for f in self.followers:
            st = self.foll[f.name]
            gxi, gyi = f.goal_offset

            dx, dy = st.x - lead.x, st.y - lead.y
            xL = c * dx + s * dy
            yL = -s * dx + c * dy
            exi = xL - gxi                  # eq. 7
            eyi = yL - gyi                  # eq. 8
            Di = math.hypot(exi, eyi)
            chi_err = wrap_pi(st.chi - lead.chi)

            row += [f"{st.x:.4f}", f"{st.y:.4f}", f"{st.chi:.6f}",
                    f"{st.speed:.4f}", f"{st.chidot:.6f}",
                    f"{xL:.4f}", f"{yL:.4f}",
                    f"{exi:.4f}", f"{eyi:.4f}", f"{Di:.4f}",
                    f"{chi_err:.6f}"]

        self._writer.writerow(row)
        self._rows += 1

    # ------------------------------------------------------------ diagnostics
    def _on_diag(self):
        lead = self.leader
        parts = [f"leader:pose{lead.n_pose}/vel{lead.n_vel}"
                 f"{'' if lead.got_pose else '(NO POSE)'}"]
        for f in self.followers:
            st = self.foll[f.name]
            parts.append(f"{f.name}:{st.n_msgs}"
                         f"{'' if st.got else '(NONE)'}")
        blocker = ""
        if not lead.got_pose:
            blocker = " BLOCKED: no leader pose on " + LEADER_POSE_TOPIC
        else:
            missing = [f.name for f in self.followers if not self.foll[f.name].got]
            if missing:
                blocker = f" BLOCKED: no vehicle_odometry from {missing}"
            elif not lead.got_vel:
                blocker = " (leader velocity not yet seen; speed logged as 0)"
        self.get_logger().info(
            f"[diag] rows={self._rows} | " + " ".join(parts) + blocker)

    # --------------------------------------------------------------- cleanup
    def destroy_node(self):
        try:
            self._csv_file.flush()
            self._csv_file.close()
            self.get_logger().info(f"closed {self.csv_path} ({self._rows} rows)")
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FormationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()