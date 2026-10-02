#!/usr/bin/env python3
"""
step4_formation.py

Step 3 had one drone on a circle. Now there are two:

    a LEADER that flies the circle perfectly, and
    a FOLLOWER that must hold a fixed offset beside it.

The offset G = (g_x, g_y) is fixed in the LEADER'S BODY FRAME, so as the
leader goes round the circle, the follower's target point goes round with
it -- the whole formation rotates.

Here is the question the paper gets wrong.

    When the formation is rotating, should the follower point
    the same way as the leader?

It feels obviously yes. It is no.

Think of two horses side by side on a carousel. The outer horse travels a
bigger circle than the inner one. At any instant they are NOT pointing in
the same direction -- the outer one is angled slightly differently. Same
for a formation going round a bend.

The paper's Eq. 14 says: when the formation error is zero, aim the
follower at chi_l -- the leader's heading. That is the carousel mistake.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------
V_GL    = 1.0            # leader speed, m/s
R       = 10.0           # circle radius, m
G_X, G_Y = 3.0, 3.0      # follower's offset in the leader's body frame
CHI_INF = np.pi / 2
K_Y     = 1.0            # vector-field gain on lateral error
K_X     = 1.0            # speed gain on longitudinal error
ETA     = 1.0
P       = 0.6

OMEGA = V_GL / R         # formation rotation rate, rad/s


def signed_pow(x, p):
    return np.sign(x) * np.abs(x) ** p


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ---------------------------------------------------------------------
# What the follower ACTUALLY has to do, from pure geometry.
#
# The whole formation is a rigid body spinning at OMEGA. The velocity of
# any point of a rigid body is
#
#     v_point  =  v_centre  +  omega x r
#
# In the leader's body frame, v_centre = (V_GL, 0) and the cross product
# term for r = (g_x, g_y) is omega * (-g_y, g_x). So:
#
#     forward component  =  V_GL - omega*g_y
#     sideways component =       + omega*g_x     <-- NOT ZERO
#
# A sideways component means the follower is NOT pointing along the
# leader's heading. The angle between them is:
# ---------------------------------------------------------------------

def required_heading_offset():
    return np.arctan2(OMEGA * G_X, V_GL - OMEGA * G_Y)


def required_speed():
    return np.hypot(V_GL - OMEGA * G_Y, OMEGA * G_X)


# ---------------------------------------------------------------------
# The two heading laws
# ---------------------------------------------------------------------

def chi_desired(chi_l, e_y, corrected):
    r"""
    PAPER (Eq. 14):
        chi_d = chi_l                     - chi_inf(2/pi) atan(K_Y e_y)

    CORRECTED:
        chi_d = chi_l + heading_offset    - chi_inf(2/pi) atan(K_Y e_y)
                        \____________/
                        the carousel term
    """
    base = chi_l + (required_heading_offset() if corrected else 0.0)
    return base - CHI_INF * (2 / np.pi) * np.arctan(K_Y * e_y)


# ---------------------------------------------------------------------
# Predicting the error the paper's law leaves behind.
#
# The heading loop is finite-time, so it WILL reach chi_i = chi_d. But
# geometry insists chi_i = chi_l + Delta. So the vector field is forced
# to manufacture that Delta out of lateral error instead:
#
#     chi_l + Delta  =  chi_l - chi_inf(2/pi) atan(K_Y e_y)
#
#     ->   e_y  =  -tan(Delta) / K_Y
#
# The follower sits permanently off to one side, and the SIGN of the
# offset flips with the sign of g_x -- which is exactly the pattern in
# your four-drone logs.
# ---------------------------------------------------------------------

def predicted_ey():
    return -np.tan(required_heading_offset()) / K_Y


# ---------------------------------------------------------------------
def simulate(corrected, dt=0.001, tmax=150.0):
    n = int(tmax / dt)
    t = np.arange(n) * dt

    theta = 0.0                                    # leader angle on circle
    xi, yi = R + G_Y, G_X                          # follower roughly in place
    chi_i = np.pi / 2

    EX = np.zeros(n); EY = np.zeros(n)
    CT = np.zeros(n); VI = np.zeros(n)
    XI = np.zeros(n); YI = np.zeros(n)
    XL = np.zeros(n); YL = np.zeros(n)

    for k in range(n):
        # ---- leader: exact circle -----------------------------------
        xl, yl = R * np.cos(theta), R * np.sin(theta)
        chi_l = theta + np.pi / 2

        # ---- follower error in the leader's body frame --------------
        rx, ry = xi - xl, yi - yl
        c, s = np.cos(chi_l), np.sin(chi_l)
        e_bx =  rx * c + ry * s          # forward component
        e_by = -rx * s + ry * c          # left component
        e_x, e_y = e_bx - G_X, e_by - G_Y

        # ---- speed: same law for BOTH runs, so only heading differs --
        # feedforward is the forward part of the rigid-body velocity,
        # which the paper's Remark 2 does get right.
        # feedforward speed. The paper's Remark 2 recovers only the
        # FORWARD part (V_GL - omega*g_y) and misses the sideways part,
        # so its follower is also slightly too slow.
        v_ff = required_speed() if corrected else (V_GL - OMEGA * G_Y)
        v_i = v_ff - K_X * e_x
        v_i = np.clip(v_i, 0.05, 3.0)

        # ---- exact error rates (paper Eq. 13) -----------------------
        dchi = wrap(chi_i - chi_l)
        e_y_dot = v_i * np.sin(dchi) - OMEGA * G_X

        # ---- heading law --------------------------------------------
        chi_d = chi_desired(chi_l, e_y, corrected)
        chi_t = wrap(chi_i - chi_d)
        chi_d_dot = OMEGA - CHI_INF * (2 / np.pi) * \
            K_Y / (1 + (K_Y * e_y) ** 2) * e_y_dot
        u = chi_d_dot - ETA * signed_pow(chi_t, P)

        EX[k], EY[k], CT[k], VI[k] = e_x, e_y, chi_t, v_i
        XI[k], YI[k], XL[k], YL[k] = xi, yi, xl, yl

        # ---- integrate ----------------------------------------------
        theta += dt * OMEGA
        chi_i += dt * u
        xi    += dt * v_i * np.cos(chi_i)
        yi    += dt * v_i * np.sin(chi_i)

    return t, EX, EY, CT, VI, XI, YI, XL, YL


# ---------------------------------------------------------------------
def main():
    D = required_heading_offset()
    print(f"Leader speed {V_GL} m/s on a {R:.0f} m circle "
          f"-> formation spins at omega = {OMEGA:.3f} rad/s")
    print(f"Follower offset G = ({G_X:.0f}, {G_Y:.0f}) m\n")

    print("GEOMETRY SAYS the follower must:")
    print(f"  point   {np.degrees(D):+.2f} deg away from the leader's heading")
    print(f"  fly at  {required_speed():.4f} m/s (not {V_GL:.1f})\n")

    tp, EXp, EYp, CTp, VIp, Xp, Yp, XL, YL = simulate(corrected=False)
    tc, EXc, EYc, CTc, VIc, Xc, Yc, _, _   = simulate(corrected=True)

    ss = tp > 100

    print("=" * 62)
    print("PAPER'S LAW (Eq. 14: aim at the leader's heading)")
    print("=" * 62)
    print(f"{'':22} {'predicted':>12} {'measured':>12}")
    print(f"{'lateral error e_y':22} {predicted_ey():12.5f} "
          f"{EYp[ss].mean():12.5f}  m")
    print(f"{'heading error':22} {'0 (claimed)':>12} "
          f"{CTp[ss].mean():12.5f}  rad")
    print(f"\n  Proposition 1 says the heading error goes to zero, and it")
    print(f"  DOES -- {abs(CTp[ss].mean()):.2e} rad. The loop works perfectly.")
    print(f"  But it converged to the wrong TARGET, so the follower parks")
    print(f"  {abs(EYp[ss].mean()):.3f} m off to the side and stays there.\n")

    print("=" * 62)
    print("CORRECTED LAW (add the carousel term)")
    print("=" * 62)
    print(f"  lateral error e_y      {EYc[ss].mean():+.6f} m")
    print(f"  longitudinal error e_x {EXc[ss].mean():+.6f} m")
    print(f"  heading error          {CTc[ss].mean():+.6f} rad")
    print(f"  follower speed         {VIc[ss].mean():.4f} m/s "
          f"(geometry wanted {required_speed():.4f})\n")

    print("=" * 62)
    print("HOW BIG IS IT?  Delta = atan2(omega*g_x, v_gl - omega*g_y)")
    print("and on a circle omega = v_gl/R, so the v_gl CANCELS:")
    print("      Delta = atan2(g_x/R, 1 - g_y/R)")
    print("It depends on FORMATION SIZE vs PATH RADIUS. Flying faster")
    print("around the same circle does NOT help -- omega grows with you.")
    print("=" * 62)
    print(f"{'path radius R':>14} {'g_x/R':>8} {'Delta (deg)':>13} {'e_y (m)':>10}")
    for r in (10.0, 20.0, 50.0, 59.0, 100.0):
        d = np.arctan2(G_X / r, 1 - G_Y / r)
        tag = "   <- your SITL run" if r == 59.0 else ""
        print(f"{r:14.0f} {G_X/r:8.3f} {np.degrees(d):13.2f} "
              f"{np.tan(d)/K_Y:10.3f}{tag}")
    print()
    print("Your logs: chi_dot_l = 0.0085 rad/s at v_gl = 0.5 m/s, so the")
    print("effective radius was 0.5/0.0085 = 59 m. With g_x = 3 m that")
    print("predicts 0.051 rad -- which is what you measured.")

    print("\nTHE POINT:")
    print("  Proposition 1 promises the heading error reaches zero in")
    print("  finite time. It keeps that promise. The problem is that")
    print("  zero heading error is the WRONG THING TO WANT on a curve.")

    # -----------------------------------------------------------------
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))

    th = np.linspace(0, 2 * np.pi, 400)
    ax[0].plot(R * np.cos(th), R * np.sin(th), "k--", lw=1, label="leader path")
    ax[0].plot(Xp, Yp, lw=1.5, color="tab:red", label="follower (paper)")
    ax[0].plot(Xc, Yc, lw=1.5, color="tab:green", label="follower (corrected)")
    ax[0].set_aspect("equal"); ax[0].set_xlabel("x (m)"); ax[0].set_ylabel("y (m)")
    ax[0].set_title("Both hold formation... roughly")
    ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)

    ax[1].plot(tp, EYp, lw=2, color="tab:red", label="paper")
    ax[1].plot(tc, EYc, lw=2, color="tab:green", label="corrected")
    ax[1].axhline(predicted_ey(), color="tab:red", ls=":", lw=1)
    ax[1].axhline(0, color="k", lw=0.8)
    ax[1].text(105, predicted_ey() + 0.03, "predicted", fontsize=8, color="tab:red")
    ax[1].set_ylim(-0.8, 0.4)
    ax[1].set_xlabel("time (s)"); ax[1].set_ylabel("lateral error $e_y$ (m)")
    ax[1].set_title("One of them never gets there")
    ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)

    ax[2].semilogy(tp, np.abs(CTp) + 1e-16, lw=2, color="tab:red", label="paper")
    ax[2].semilogy(tc, np.abs(CTc) + 1e-16, lw=2, color="tab:green",
                   label="corrected")
    ax[2].set_xlabel("time (s)")
    ax[2].set_ylabel(r"$|\tilde\chi|$ (rad), log scale")
    ax[2].set_title("BOTH heading loops work perfectly")
    ax[2].legend(fontsize=8); ax[2].grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("step4_formation.png", dpi=130)
    print("\nPlot saved: step4_formation.png")


if __name__ == "__main__":
    main()
