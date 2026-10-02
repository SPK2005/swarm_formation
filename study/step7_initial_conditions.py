#!/usr/bin/env python3
"""
step7_initial_conditions.py

Same guidance law and simulate() as step6_leader_convergence.py (copied
here, not imported, so this file stands on its own) -- answers "what if
the leader starts somewhere else?" by rerunning it for a few different
leader starting poses (xl0, yl0), overlaying the results so the
convergence behaviour can be compared directly.

This is exactly the kind of thing an advisor might ask for live: change
the initial condition, rerun, look at the graph. Nothing about the
guidance law changes -- only (xl0, yl0) passed into simulate(). Keep
this file's simulate()/constants in sync with step6_leader_convergence.py
by hand if that file's guidance law is ever revised.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---- path ----------------------------------------------------------------
A, L = 30.0, 300.0
OMEGA_ARG = 2 * np.pi / L
V_GL = 5.0

# ---- leader's own vector-field guidance (paper's Eq. 5-6) ---------------
CHI_INF = np.pi / 2
K_L = 0.05          # path-attraction gain ("k" in Eq. 5)
BETA = np.pi / 4     # heading-reaching gain ("beta" in Eq. 6)
EPS_SAT = 1.0        # saturation width ("epsilon" in Eq. 6)

# ---- follower (step 5) ----------------------------------------------------
G_X, G_Y = 3.0, 3.0
K_Y = 0.3
ETA = 1.0
P = 0.6
# Max commanded turn rate for the follower's autopilot -- no real servo/
# autopilot can execute an arbitrarily large rate, and the numerically
# differentiated dDelta/dt feedforward (chi_ddot_l) can otherwise demand
# one during a fast leader transient. Reuse BETA: it's already the max
# rate this simulation assumes the leader's own actuator can achieve
# (Lemma 1's saturation), so holding the follower to the same physical
# limit is the consistent choice, not an arbitrary new number.
CHI_DOT_MAX = BETA


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def sat(z):
    """Standard saturation function."""
    return np.clip(z, -1.0, 1.0)


def signed_pow(x, p):
    return np.sign(x) * np.abs(x) ** p


def yprime(x):
    return A * OMEGA_ARG * np.cos(OMEGA_ARG * x)


def ydoubleprime(x):
    return -A * OMEGA_ARG ** 2 * np.sin(OMEGA_ARG * x)


def path_y(x):
    return A * np.sin(OMEGA_ARG * x)


def closest_point(xl, yl, x_guess, iters=8):
    """Newton solve for the closest point on the path to (xl, yl), warm-
    started from x_guess. Returns (x*, signed cross-track d, tangent chi_p).
    d>0 means the leader is to the LEFT of the direction of travel."""
    x = x_guess
    for _ in range(iters):
        yp, ypp, y = yprime(x), ydoubleprime(x), path_y(x)
        f = (x - xl) + (y - yl) * yp
        fprime = 1 + yp * yp + (y - yl) * ypp
        x -= f / fprime
    y = path_y(x)
    chi_p = np.arctan2(yprime(x), 1.0)
    nx, ny = -np.sin(chi_p), np.cos(chi_p)
    d = (xl - x) * nx + (yl - y) * ny
    return x, d, chi_p


def required_speed_and_delta(omega):
    fwd = V_GL - omega * G_Y
    side = omega * G_X
    return np.hypot(fwd, side), np.arctan2(side, fwd)


def simulate(dt=0.001, tmax=60.0, xl0=0.0, yl0=-30.0, chi_l0=0.0):
    """
    Full corrected follower law (step 5's 'full' mode) -- omega and
    chi_ddot_l come from the leader's OWN simulated convergence, not an
    assumed on-path curvature formula.
    (xl0, yl0, chi_l0): leader's starting pose -- off the path by default,
    so this run shows the leader's own convergence transient too.

    Returns T, D, EY, XL, YL, XI, YI, CHIL_ERR, CHIF_ERR, CHIL_DOT, CHIF_DOT,
            VI, CHIL_ABS, CHII_ABS, EX:
      D        leader's signed cross-track distance, d
      EY       follower's lateral formation error, e_y
      EX       follower's longitudinal formation error, e_x
      CHIL_ERR leader's own heading error, chi_tilde_l = wrap(chi_l - chi_d_l)
      CHIF_ERR follower's heading error, chi_t = wrap(chi_i - chi_d)
      CHIL_DOT leader's commanded turn rate, chi_l_dot
      CHIF_DOT follower's commanded turn rate, u
      VI       follower's commanded ground speed, v_i
      CHIL_ABS leader's absolute heading, chi_l
      CHII_ABS follower's absolute heading, chi_i
    """
    n = int(tmax / dt)
    xl, yl, chi_l = xl0, yl0, chi_l0
    x_star = xl
    chi_p_prev = d_prev = chi_dot_l_prev = None

    c, s = np.cos(chi_l), np.sin(chi_l)
    xi, yi = xl + G_X * c - G_Y * s, yl + G_X * s + G_Y * c
    chi_i = chi_l

    T = np.zeros(n); D = np.zeros(n); EY = np.zeros(n); EX = np.zeros(n)
    XL = np.zeros(n); YL = np.zeros(n); XI = np.zeros(n); YI = np.zeros(n)
    CHIL_ERR = np.zeros(n)   # leader's own heading error, chi_tilde_l
    CHIF_ERR = np.zeros(n)   # follower's heading error, chi_t
    CHIL_DOT = np.zeros(n)   # leader's commanded turn rate
    CHIF_DOT = np.zeros(n)   # follower's commanded turn rate
    VI = np.zeros(n)         # follower's commanded speed
    CHIL_ABS = np.zeros(n)   # leader's absolute heading
    CHII_ABS = np.zeros(n)   # follower's absolute heading

    for k in range(n):
        x_star, d, chi_p = closest_point(xl, yl, x_star)

        # chi_p_dot, d_dot: backward finite difference in time -- same
        # "differentiate a noisy signal the way real telemetry would"
        # approach used for chi_ddot_l throughout step 5.
        if chi_p_prev is None:
            chi_p_dot, d_dot = 0.0, 0.0
        else:
            chi_p_dot = wrap(chi_p - chi_p_prev) / dt
            d_dot = (d - d_prev) / dt

        chi_d_l = chi_p - CHI_INF * (2 / np.pi) * np.arctan(K_L * d)
        chi_tilde_l = wrap(chi_l - chi_d_l)
        chi_l_dot = (chi_p_dot
                     - CHI_INF * (2 / np.pi) * (K_L / (1 + (K_L * d) ** 2)) * d_dot
                     - BETA * sat(chi_tilde_l / EPS_SAT))
        # Same "unknown on the very first step" guard as chi_p_prev/d_prev
        # above -- without it, chi_dot_l_prev's cold-start value (whatever
        # it's seeded to) gets differenced against the true chi_l_dot(0),
        # which is nonzero whenever the leader's initial heading doesn't
        # exactly match the path tangent there. That one-step spike in
        # chi_ddot_l then feeds straight into the follower's dDelta/dt
        # feedforward below, showing up as a huge, nonphysical turn-rate
        # transient on the very first timestep.
        if chi_dot_l_prev is None:
            chi_ddot_l = 0.0
        else:
            chi_ddot_l = (chi_l_dot - chi_dot_l_prev) / dt

        omega = chi_l_dot
        rx, ry = xi - xl, yi - yl
        c, s = np.cos(chi_l), np.sin(chi_l)
        e_bx, e_by = rx * c + ry * s, -rx * s + ry * c
        e_x, e_y = e_bx - G_X, e_by - G_Y

        req_speed, Delta = required_speed_and_delta(omega)
        v_i = np.clip(req_speed - 1.0 * e_x, 0.05, 15.0)

        dchi = wrap(chi_i - chi_l)
        e_y_dot = v_i * np.sin(dchi) - omega * G_X

        chi_d = chi_l + Delta - CHI_INF * (2 / np.pi) * np.arctan(K_Y * e_y)
        chi_t = wrap(chi_i - chi_d)
        drift = CHI_INF * (2 / np.pi) * K_Y / (1 + (K_Y * e_y) ** 2) * e_y_dot
        dDelta_dt = chi_ddot_l * V_GL * G_X / (req_speed ** 2 + 1e-9)
        chi_d_dot = omega + dDelta_dt - drift
        u = np.clip(chi_d_dot - ETA * signed_pow(chi_t, P), -CHI_DOT_MAX, CHI_DOT_MAX)

        T[k], D[k], EY[k], EX[k] = k * dt, d, e_y, e_x
        XL[k], YL[k], XI[k], YI[k] = xl, yl, xi, yi
        CHIL_ERR[k], CHIF_ERR[k] = chi_tilde_l, chi_t
        CHIL_DOT[k], CHIF_DOT[k], VI[k] = chi_l_dot, u, v_i
        CHIL_ABS[k], CHII_ABS[k] = chi_l, chi_i

        chi_p_prev, d_prev, chi_dot_l_prev = chi_p, d, chi_l_dot

        xl += dt * V_GL * np.cos(chi_l)
        yl += dt * V_GL * np.sin(chi_l)
        chi_l += dt * chi_l_dot
        xi += dt * v_i * np.cos(chi_i)
        yi += dt * v_i * np.sin(chi_i)
        chi_i += dt * u

    return (T, D, EY, XL, YL, XI, YI, CHIL_ERR, CHIF_ERR,
            CHIL_DOT, CHIF_DOT, VI, CHIL_ABS, CHII_ABS, EX)


# (label, xl0, yl0, chi_l0) -- heading left at 0 for all three, matching
# the default above
CASES = [
    ("(0, 0)",    0.0,   0.0, 0.0),
    ("(-30, 0)", -30.0,  0.0, 0.0),
    ("(30, 30)",  30.0, 30.0, 0.0),
]
COLORS = ["tab:green", "tab:orange", "tab:purple"]


def main():
    results = {}
    for label, xl0, yl0, chi_l0 in CASES:
        out = simulate(xl0=xl0, yl0=yl0, chi_l0=chi_l0)
        results[label] = out

    print(f"{'start':<12}{'leader |d| max':>16}{'leader |d| RMS(>50s)':>22}"
          f"{'follower e_y max':>18}{'follower chi_t max':>20}")
    for label, xl0, yl0, chi_l0 in CASES:
        (T, D, EY, XL, YL, XI, YI, CHIL_ERR, CHIF_ERR,
         CHIL_DOT, CHIF_DOT, VI, CHIL_ABS, CHII_ABS, EX) = results[label]
        ss = T > 50
        print(f"{label:<12}{np.max(np.abs(D)):>16.4f}"
              f"{np.sqrt(np.mean(D[ss]**2)):>22.6f}"
              f"{np.max(np.abs(EY)):>18.4f}"
              f"{np.max(np.degrees(np.abs(CHIF_ERR))):>20.3f}")

    fig = plt.figure(figsize=(18, 12))
    gs = fig.add_gridspec(3, 3)
    ax_d = fig.add_subplot(gs[0, 0])
    ax_chil = fig.add_subplot(gs[0, 1])
    ax_turn = fig.add_subplot(gs[0, 2])
    ax_ey = fig.add_subplot(gs[1, 0])
    ax_chif = fig.add_subplot(gs[1, 1])
    ax_speed = fig.add_subplot(gs[1, 2])
    ax_heading = fig.add_subplot(gs[2, 0])
    ax_dist = fig.add_subplot(gs[2, 1])
    ax_xy = fig.add_subplot(gs[2, 2])

    x_dense = np.linspace(-60, 300, 1000)
    ax_xy.plot(x_dense, path_y(x_dense), color="k", lw=1.4, label="reference path")

    for (label, xl0, yl0, chi_l0), color in zip(CASES, COLORS):
        (T, D, EY, XL, YL, XI, YI, CHIL_ERR, CHIF_ERR,
         CHIL_DOT, CHIF_DOT, VI, CHIL_ABS, CHII_ABS, EX) = results[label]
        EFORM = np.hypot(EX, EY)

        ax_d.semilogy(T, np.abs(D) + 1e-6, color=color, lw=1.4, label=label)
        ax_chil.plot(T, np.degrees(CHIL_ERR), color=color, lw=1.4, label=label)
        ax_turn.plot(T, np.degrees(CHIF_DOT), color=color, lw=1.0, alpha=0.85, label=label)
        ax_ey.plot(T, EY, color=color, lw=1.4, label=label)
        ax_chif.plot(T, np.degrees(CHIF_ERR), color=color, lw=1.4, label=label)
        ax_speed.plot(T, VI, color=color, lw=1.2, label=label)
        ax_heading.plot(T, np.degrees(CHII_ABS), color=color, lw=1.2, label=label)
        ax_dist.plot(T, EFORM, color=color, lw=1.4, label=label)
        ax_xy.plot(XL, YL, lw=1.8, color=color, label=f"leader {label}")
        ax_xy.plot(XI, YI, lw=1.0, color=color, ls="--", alpha=0.7)
        ax_xy.plot(XL[0], YL[0], "*", ms=12, color=color)

    ax_speed.axhline(V_GL, color="0.25", lw=1.2, ls="--", label="leader (const)")

    ax_d.set_xlabel("time (s)"); ax_d.set_ylabel("|d| (m), log scale")
    ax_d.set_title("Leader lateral error"); ax_d.legend(fontsize=7); ax_d.grid(alpha=0.3)

    ax_chil.axhline(0, color="k", lw=0.6)
    ax_chil.set_xlabel("time (s)"); ax_chil.set_ylabel("chi_tilde_l (deg)")
    ax_chil.set_title("Leader heading error"); ax_chil.legend(fontsize=7); ax_chil.grid(alpha=0.3)

    # Turn rate is now genuinely bounded by the follower's actuator rate
    # limit (CHI_DOT_MAX, same physical cap as the leader's own law) --
    # dotted lines mark it, same convention as step6_leader_convergence.py.
    ax_turn.axhline(0, color="k", lw=0.6)
    ax_turn.axhline(np.degrees(CHI_DOT_MAX), color="0.6", lw=0.8, ls=":")
    ax_turn.axhline(-np.degrees(CHI_DOT_MAX), color="0.6", lw=0.8, ls=":")
    ax_turn.set_xlabel("time (s)"); ax_turn.set_ylabel("follower turn rate (deg/s)")
    ax_turn.set_title("Turn-rate command (rate-limited)"); ax_turn.legend(fontsize=7); ax_turn.grid(alpha=0.3)

    ax_ey.axhline(0, color="k", lw=0.6)
    ax_ey.set_xlabel("time (s)"); ax_ey.set_ylabel("follower e_y (m)")
    ax_ey.set_title("Follower lateral error"); ax_ey.legend(fontsize=7); ax_ey.grid(alpha=0.3)

    ax_chif.axhline(0, color="k", lw=0.6)
    ax_chif.set_xlabel("time (s)"); ax_chif.set_ylabel("follower chi_t (deg)")
    ax_chif.set_title("Follower heading error"); ax_chif.legend(fontsize=7); ax_chif.grid(alpha=0.3)

    ax_speed.set_xlabel("time (s)"); ax_speed.set_ylabel("ground speed (m/s)")
    ax_speed.set_title("Follower speed profile"); ax_speed.legend(fontsize=7); ax_speed.grid(alpha=0.3)

    ax_heading.set_xlabel("time (s)"); ax_heading.set_ylabel("follower heading (deg)")
    ax_heading.set_title("Follower absolute heading"); ax_heading.legend(fontsize=7); ax_heading.grid(alpha=0.3)

    ax_dist.set_xlabel("time (s)"); ax_dist.set_ylabel("formation error D (m)")
    ax_dist.set_title("Formation distance error"); ax_dist.legend(fontsize=7); ax_dist.grid(alpha=0.3)

    ax_xy.set_xlabel("x (m)"); ax_xy.set_ylabel("y (m)")
    ax_xy.set_title("Trajectories from each start")
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.legend(fontsize=6, loc="upper left"); ax_xy.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("step7_initial_conditions.png", dpi=130)
    print("\nPlot saved: step7_initial_conditions.png")


if __name__ == "__main__":
    main()
