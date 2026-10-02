#!/usr/bin/env python3
"""
step3_curve.py

Step 2 followed a straight line. Now the path is a circle.

One thing changes, and it changes everything:

    ON A STRAIGHT LINE, the path's direction never moves.
    ON A CURVE, the path's direction is rotating the whole time.

The path tangent angle chi_p is no longer a constant. It rotates at

    omega = V / R

so the desired heading is now chasing a target that never stops moving.
In step 2 the feedforward term chi_d_dot only had to cancel motion caused
by YOU drifting. Now it also has to cancel motion of the PATH ITSELF.

This script runs the same controller twice:

    WITH    the path-rotation term  (correct)
    WITHOUT it                      (the mistake almost everyone makes)

Leaving it out does not blow up. It does not oscillate. It quietly parks
the drone a fixed distance from the path, forever -- and you can compute
that distance in closed form before you run anything.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------
V       = 1.0            # forward speed, m/s
R       = 10.0           # circle radius, m
CHI_INF = np.pi / 2      # steepest approach angle
K       = 1.0            # vector-field gain
ETA     = 1.0            # reaching gain
P       = 0.6            # fractional power

OMEGA = V / R            # how fast the path tangent rotates, rad/s


def signed_pow(x, p):
    return np.sign(x) * np.abs(x) ** p


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ---------------------------------------------------------------------
# Geometry. Flying counter-clockwise around a circle of radius R.
#
#   r     = distance from the centre
#   theta = angular position around the circle
#   d     = R - r     signed cross-track error, POSITIVE = inside
#   chi_p = theta + pi/2      direction the path points here
#   psi   = chi - chi_p       how far off the path direction you are
#
# Two facts we need (both just projecting the velocity):
#
#   d_dot     =  V sin(psi)          drift across the path
#   chi_p_dot =  V cos(psi) / r      how fast the path direction rotates
#
# Check the second one: sitting exactly on the path, psi = 0 and r = R,
# so chi_p_dot = V/R = omega. Correct.
# ---------------------------------------------------------------------

def geometry(x, y, chi):
    r = np.hypot(x, y)
    theta = np.arctan2(y, x)
    d = R - r
    chi_p = theta + np.pi/2
    psi = wrap(chi - chi_p)
    d_dot = V * np.sin(psi)
    chi_p_dot = V * np.cos(psi) / r
    return d, psi, chi_p, d_dot, chi_p_dot


def chi_desired(chi_p, d):
    """Same vector field as step 2, but built around the path tangent
    instead of around zero."""
    return chi_p - CHI_INF * (2 / np.pi) * np.arctan(K * d)


def chi_desired_dot(d, d_dot, chi_p_dot, use_curvature):
    r"""
        chi_d_dot  =  chi_p_dot  -  chi_inf (2/pi) K/(1+(Kd)^2) * d_dot
                      \_________/     \_____________________________/
                       the PATH is      YOU are drifting  (step 2 had
                       rotating         only this term)
    """
    drift = CHI_INF * (2 / np.pi) * K / (1 + (K * d) ** 2) * d_dot
    if use_curvature:
        return chi_p_dot - drift
    return -drift


def control(x, y, chi, use_curvature):
    d, psi, chi_p, d_dot, chi_p_dot = geometry(x, y, chi)
    chi_t = wrap(chi - chi_desired(chi_p, d))
    ff = chi_desired_dot(d, d_dot, chi_p_dot, use_curvature)
    return ff - ETA * signed_pow(chi_t, P), chi_t, d


# ---------------------------------------------------------------------
# Predicting the error you get by leaving the term out.
#
# Without chi_p_dot, the heading error dynamics become
#
#     chi_t_dot = -chi_p_dot - eta sign(chi_t)|chi_t|^P
#
# Settling means chi_t_dot = 0, so the reaching term has to spend itself
# just cancelling the path rotation:
#
#     eta |chi_t|^P = omega        ->   |chi_t| = (omega/eta)^(1/P)
#
# The drone then sits where the vector field's own correction is exactly
# equal and opposite to that leftover heading error:
#
#     |d| = tan( (omega/eta)^(1/P) ) / K
# ---------------------------------------------------------------------

def predicted_chi_error():
    return (OMEGA / ETA) ** (1.0 / P)


def predicted_offset():
    return np.tan(predicted_chi_error()) / K


# ---------------------------------------------------------------------
def simulate(use_curvature, x0=14.0, y0=0.0, chi0= 0.0 ,
             dt=0.001, tmax=120.0):
    n = int(tmax / dt)
    t = np.arange(n) * dt
    x = np.zeros(n); y = np.zeros(n); chi = np.zeros(n)
    D = np.zeros(n); CT = np.zeros(n)
    x[0], y[0], chi[0] = x0, y0, chi0

    for i in range(1, n):
        u, ct, d = control(x[i - 1], y[i - 1], chi[i - 1], use_curvature)
        CT[i - 1], D[i - 1] = ct, d
        chi[i] = chi[i - 1] + dt * u
        x[i]   = x[i - 1] + dt * V * np.cos(chi[i - 1])
        y[i]   = y[i - 1] + dt * V * np.sin(chi[i - 1])
    _, CT[-1], D[-1] = control(x[-1], y[-1], chi[-1], use_curvature)
    return t, x, y, D, CT


# ---------------------------------------------------------------------
def main():
    print(f"Circle radius {R:.0f} m, speed {V:.1f} m/s")
    print(f"So the path tangent rotates at omega = V/R = {OMEGA:.3f} rad/s\n")

    t, xw, yw, Dw, Cw = simulate(use_curvature=True)
    _, xo, yo, Do, Co = simulate(use_curvature=False)

    ss = t > 80          # steady state

    print("=" * 60)
    print("WITH the path-rotation term")
    print("=" * 60)
    print(f"  heading error   {Cw[ss].mean():+.6f} rad")
    print(f"  cross-track     {Dw[ss].mean():+.6f} m")
    print("  -> both go to zero. The controller works.\n")

    print("=" * 60)
    print("WITHOUT it")
    print("=" * 60)
    print(f"{'':18} {'predicted':>12} {'measured':>12}")
    print(f"{'heading error':18} {predicted_chi_error():12.5f} "
          f"{abs(Co[ss].mean()):12.5f}  rad")
    print(f"{'cross-track':18} {predicted_offset():12.5f} "
          f"{abs(Do[ss].mean()):12.5f}  m")
    print("\n  -> It is stable. It is smooth. It is simply in the")
    print("     wrong place, permanently, by an amount you could have")
    print("     calculated on paper before running anything.\n")

    print("WHY:")
    print("  The reaching term has to spend itself cancelling the path")
    print("  rotation instead of killing the error, so it settles where")
    print("      eta |chi_tilde|^P  =  omega")
    print("  Finite-time convergence to ZERO is gone. It converges in")
    print("  finite time to a nonzero OFFSET.\n")

    print("THE LESSON, and remember it:")
    print("  A missing feedforward term does not announce itself. It")
    print("  looks like a converged system sitting at the wrong value.")
    print("  This is exactly the bug in your own four-drone logs.")

    # -----------------------------------------------------------------
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))

    th = np.linspace(0, 2 * np.pi, 400)
    ax[0].plot(R * np.cos(th), R * np.sin(th), "k", lw=2, label="path")
    ax[0].plot(xw, yw, lw=1.5, color="tab:green", label="with term")
    ax[0].plot(xo, yo, lw=1.5, color="tab:red", ls="--", label="without")
    ax[0].plot(14, 0, "o", color="tab:blue", ms=7, label="start")
    ax[0].set_aspect("equal"); ax[0].set_xlabel("x (m)"); ax[0].set_ylabel("y (m)")
    ax[0].set_title("Both look fine from here")
    ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)

    ax[1].plot(t, Dw, lw=2, color="tab:green", label="with term")
    ax[1].plot(t, Do, lw=2, color="tab:red", ls="--", label="without")
    ax[1].axhline(0, color="k", lw=0.8)
    ax[1].axhline(-predicted_offset(), color="tab:red", ls=":", lw=1)
    ax[1].text(85, -predicted_offset() - 0.09, "predicted offset",
               fontsize=8, color="tab:red")
    ax[1].set_ylim(-0.6, 0.4)
    ax[1].set_xlabel("time (s)"); ax[1].set_ylabel("cross-track error $d$ (m)")
    ax[1].set_title("Zoomed in, one of them never arrives")
    ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)

    ax[2].semilogy(t, np.abs(Cw) + 1e-16, lw=2, color="tab:green",
                   label="with term")
    ax[2].semilogy(t, np.abs(Co) + 1e-16, lw=2, color="tab:red", ls="--",
                   label="without")
    ax[2].axhline(predicted_chi_error(), color="tab:red", ls=":", lw=1)
    ax[2].set_xlabel("time (s)")
    ax[2].set_ylabel(r"$|\tilde\chi|$ (rad), log scale")
    ax[2].set_title("Heading error: cliff vs floor")
    ax[2].legend(fontsize=8); ax[2].grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("step3_curve.png", dpi=130)
    print("\nPlot saved: step3_curve.png")


if __name__ == "__main__":
    main()
