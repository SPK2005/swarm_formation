#!/usr/bin/env python3
"""
step6_leader_convergence.py

Steps 3-5 hardcoded the leader onto the path (d=0 always) -- a deliberate
simplification to isolate the follower's carousel/curvature correction.
The paper's actual "virtual leader" is not hardcoded: it's simulated with
its own vector-field guidance law (Eq. 5-6) and only converges onto the
path ASYMPTOTICALLY (Lemma 1) -- unlike the follower's FINITE-TIME law.
That asymmetry (leader: saturation-based, asymptotic; follower: sign-power,
finite-time) is why the paper proves two different kinds of convergence.

This step gives the leader its own controller and couples it to the
follower's step-5 correction, so both are simulated end to end: the leader
starts off the path and has to fly onto it, while the follower simultaneously
tries to hold formation through that whole transient.

Same abstraction level as steps 2-5: commanded heading is treated as
achieved instantly (no explicit first-order autopilot lag, Eq. 1-4 in the
paper) -- for the leader now too, not just the follower.
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
    assumed on-path curvature formula. (Earlier versions of this function
    also supported 'uncorrected'/'pointwise' modes for side-by-side
    comparison; that comparison lives in step5_varying_curve.py and the
    study notes -- this function now only runs the real, final law.)
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


def main():
    (T, D, EY, XL, YL, XI, YI, CHIL_ERR, CHIF_ERR,
     CHIL_DOT, CHIF_DOT, VI, CHIL_ABS, CHII_ABS, EX) = simulate()
    EFORM = np.hypot(EX, EY)   # scalar formation-error magnitude, like the MATLAB D_i

    ss = T > 50
    chif_deg = np.degrees(CHIF_ERR)
    chil_deg = np.degrees(CHIL_ERR)
    print("Leader + follower, full correction:")
    print(f"  leader   |d|         max {np.max(np.abs(D)):.4f} m,   "
          f"still {np.sqrt(np.mean(D[ss] ** 2)):.6f} m RMS after t=50s (never exactly 0)")
    print(f"  leader   chi_tilde_l max {np.max(np.abs(chil_deg)):.3f} deg, "
          f"still {np.sqrt(np.mean(chil_deg[ss] ** 2)):.4f} deg RMS after t=50s (never exactly 0)")
    print(f"  follower e_y         max {np.max(np.abs(EY)):.4f} m,   "
          f"RMS {np.sqrt(np.mean(EY[ss] ** 2)):.5f} m after t=50s")
    print(f"  follower chi_t       max {np.max(np.abs(chif_deg)):.3f} deg, "
          f"RMS {np.sqrt(np.mean(chif_deg[ss] ** 2)):.4f} deg after t=50s")

    # 3x3 grid, single line each -- this is the final system, not a
    # comparison (see step5_varying_curve.py / the study notes for the
    # uncorrected-vs-pointwise-vs-full comparison that justified this law).
    # Rows: (lateral error, heading error, turn-rate command) / (follower
    # lateral, follower heading, speed profile) / (absolute heading,
    # formation-error magnitude, XY trajectory) -- same plot types as the
    # real SITL analysis (analyze_formation_csv.m), just single-follower.
    color = "tab:green"
    leader_color = "0.25"
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

    # -- leader's own lateral (cross-track) error --------------------------
    ax_d.semilogy(T, np.abs(D) + 1e-6, color=color, lw=1.6)
    ax_d.set_xlabel("time (s)"); ax_d.set_ylabel("|d| (m), log scale")
    ax_d.set_title("Leader lateral error -- asymptotic, not finite-time")
    ax_d.grid(alpha=0.3)

    # -- leader's own heading error -----------------------------------------
    ax_chil.plot(T, chil_deg, color=color, lw=1.6)
    ax_chil.axhline(0, color="k", lw=0.7)
    ax_chil.set_xlabel("time (s)"); ax_chil.set_ylabel("chi_tilde_l (deg)")
    ax_chil.set_title("Leader heading error -- saturation law, decays but never hits 0")
    ax_chil.grid(alpha=0.3)

    # -- follower lateral error -----------------------------------------------
    ax_ey.plot(T, EY, color=color, lw=1.6)
    ax_ey.axhline(0, color="k", lw=0.7)
    ax_ey.set_xlabel("time (s)"); ax_ey.set_ylabel("follower lateral error e_y (m)")
    ax_ey.set_title("Follower lateral error through the leader's own transient")
    ax_ey.grid(alpha=0.3)

    # -- follower heading error ------------------------------------------------
    ax_chif.plot(T, chif_deg, color=color, lw=1.6)
    ax_chif.axhline(0, color="k", lw=0.7)
    ax_chif.set_xlabel("time (s)"); ax_chif.set_ylabel("follower chi_t (deg)")
    ax_chif.set_title("Follower heading error -- finite-time law, snaps to 0")
    ax_chif.grid(alpha=0.3)

    # -- turn-rate command (leader chi_l_dot, follower u) ----------------------
    # Both are now genuinely bounded by a physical actuator rate limit
    # (CHI_DOT_MAX = BETA) rather than needing a cosmetic axis clip -- see
    # the CHI_DOT_MAX comment near the constants for why.
    ax_turn.plot(T, np.degrees(CHIL_DOT), color=leader_color, lw=1.2, label="leader")
    ax_turn.plot(T, np.degrees(CHIF_DOT), color=color, lw=1.0, alpha=0.85, label="follower")
    ax_turn.axhline(0, color="k", lw=0.6)
    ax_turn.axhline(np.degrees(CHI_DOT_MAX), color="0.6", lw=0.8, ls=":")
    ax_turn.axhline(-np.degrees(CHI_DOT_MAX), color="0.6", lw=0.8, ls=":")
    ax_turn.set_xlabel("time (s)"); ax_turn.set_ylabel("turn rate (deg/s)")
    ax_turn.set_title("Turn-rate command (rate-limited, dotted = actuator max)")
    ax_turn.legend(fontsize=8); ax_turn.grid(alpha=0.3)

    # -- speed profile (follower v_i vs. leader's constant V_GL) ---------------
    ax_speed.plot(T, np.full_like(T, V_GL), color=leader_color, lw=1.4, ls="--", label="leader")
    ax_speed.plot(T, VI, color=color, lw=1.4, label="follower")
    ax_speed.set_xlabel("time (s)"); ax_speed.set_ylabel("ground speed (m/s)")
    ax_speed.set_title("Speed profile")
    ax_speed.legend(fontsize=8); ax_speed.grid(alpha=0.3)

    # -- absolute heading (leader chi_l, follower chi_i) -----------------------
    ax_heading.plot(T, np.degrees(CHIL_ABS), color=leader_color, lw=1.6, label="leader")
    ax_heading.plot(T, np.degrees(CHII_ABS), color=color, lw=1.2, alpha=0.85, label="follower")
    ax_heading.set_xlabel("time (s)"); ax_heading.set_ylabel("heading chi (deg)")
    ax_heading.set_title("UAV heading (absolute)")
    ax_heading.legend(fontsize=8); ax_heading.grid(alpha=0.3)

    # -- formation-error magnitude, sqrt(e_x^2 + e_y^2) -------------------------
    ax_dist.plot(T, EFORM, color=color, lw=1.4)
    ax_dist.set_xlabel("time (s)"); ax_dist.set_ylabel("formation error D (m)")
    ax_dist.set_title("Distance error (|formation offset error|)")
    ax_dist.grid(alpha=0.3)

    # -- XY trajectory --------------------------------------------------------
    x_dense = np.linspace(-20, 280, 1000)
    ax_xy.plot(x_dense, path_y(x_dense), color="k", lw=1.4, label="reference path")
    ax_xy.plot(XL, YL, lw=1.8, color=color, label="leader")
    ax_xy.plot(XI, YI, lw=1.2, color=color, ls="--", alpha=0.85, label="follower")
    ax_xy.plot(XL[0], YL[0], "k*", ms=12, label="leader start")
    ax_xy.set_xlabel("x (m)"); ax_xy.set_ylabel("y (m)")
    ax_xy.set_title("Leader flies onto the path; follower holds formation")
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.legend(fontsize=8); ax_xy.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("step6_leader_convergence.png", dpi=130)
    print("\nPlot saved: step6_leader_convergence.png")


if __name__ == "__main__":
    main()
