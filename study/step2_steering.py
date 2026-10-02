#!/usr/bin/env python3
"""
step2_steering.py

Step 1 had a point that could move sideways on command:  y_dot = u
A drone cannot do that. It flies forward and it can only TURN.

    x_dot   = v cos(chi)
    y_dot   = v sin(chi)        <- you do NOT control this directly
    chi_dot = u                 <- this is all you control

So there is now a gap between what you want (get y to zero) and what you
can touch (the turn rate). Two layers instead of one.

    LAYER 1 (outer)  "which way SHOULD I be pointing right now?"
    LAYER 2 (inner)  "turn me to that heading, in finite time"

Layer 2 is exactly step 1, reused unchanged. Layer 1 is new.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------
V       = 1.0            # forward speed, m/s (constant, not controlled)
CHI_INF = np.pi / 2      # steepest approach angle: 90 deg
K       = 1.0            # how sharply the desired heading reacts to y
ETA     = 1.0            # step-1 reaching gain
P       = 0.6            # step-1 fractional power


def signed_pow(x, p):
    return np.sign(x) * np.abs(x) ** p


def wrap(a):
    """Keep an angle in (-pi, pi]. Do this BEFORE signed_pow, always:
    +3.1 rad and -3.2 rad are almost the same heading, but without
    wrapping they produce opposite-sign corrections."""
    return (a + np.pi) % (2 * np.pi) - np.pi


# ---------------------------------------------------------------------
# LAYER 1: the desired heading (this is the "vector field")
#
#   chi_d = -chi_inf * (2/pi) * atan(K*y)
#
# Read it as a rule:
#     far above the line (y large positive) -> atan -> +1  -> chi_d = -90 deg
#     ON the line        (y = 0)            -> atan ->  0  -> chi_d =   0 deg
#     far below          (y large negative) -> atan -> -1  -> chi_d = +90 deg
#
# So: far away, aim straight at the line. Close in, flatten out and fly
# along it. The atan just blends smoothly between those two.
# ---------------------------------------------------------------------

def chi_desired(y):
    return -CHI_INF * (2 / np.pi) * np.arctan(K * y)


def chi_desired_dot(y, chi):
    """d(chi_d)/dt, worked out by the chain rule.

        d/dt atan(K*y) = K/(1 + (K*y)^2) * y_dot,     y_dot = V sin(chi)

    We need this because chi_d is MOVING. Chasing a moving target with a
    controller built for a stationary one leaves you permanently behind.
    """
    y_dot = V * np.sin(chi)
    return -CHI_INF * (2 / np.pi) * K / (1 + (K * y) ** 2) * y_dot


# ---------------------------------------------------------------------
# LAYER 2: turn to that heading, in finite time. This is step 1.
#
#   chi_tilde = chi - chi_d                      (heading error)
#   u = chi_d_dot - eta * sign(chi_tilde)|chi_tilde|^p
#       \________/   \_______________________________/
#        keep up      step 1, unchanged
#        with the
#        moving target
#
# Substituting into chi_tilde_dot = chi_dot - chi_d_dot = u - chi_d_dot:
#
#   chi_tilde_dot = -eta * sign(chi_tilde) |chi_tilde|^p
#
# The chi_d_dot term CANCELS ITSELF OUT. That is its only job, and it is
# the whole reason it appears in the paper's Eq. 15.
# ---------------------------------------------------------------------

def control(y, chi):
    chi_t = wrap(chi - chi_desired(y))
    return chi_desired_dot(y, chi) - ETA * signed_pow(chi_t, P)


def arrival_time(chi_t0, eta=ETA, p=P):
    """Same formula as step 1 -- it applies to the HEADING error now."""
    return np.abs(chi_t0) ** (1 - p) / (eta * (1 - p))


# ---------------------------------------------------------------------
def simulate(y0, chi0, dt=0.001, tmax=25.0):
    n = int(tmax / dt)
    t = np.arange(n) * dt
    x = np.zeros(n); y = np.zeros(n); chi = np.zeros(n); ct = np.zeros(n)
    y[0], chi[0] = y0, chi0
    ct[0] = wrap(chi0 - chi_desired(y0))

    for i in range(1, n):
        u = control(y[i - 1], chi[i - 1])
        chi[i] = chi[i - 1] + dt * u
        x[i]   = x[i - 1] + dt * V * np.cos(chi[i - 1])
        y[i]   = y[i - 1] + dt * V * np.sin(chi[i - 1])
        ct[i]  = wrap(chi[i] - chi_desired(y[i]))
    return t, x, y, chi, ct


def first_below(t, sig, tol):
    idx = np.where(np.abs(sig) < tol)[0]
    return t[idx[0]] if len(idx) else np.nan


# ---------------------------------------------------------------------
def main():
    y0, chi0 = 3.0, 3.14 *1.5       # 3 m off the line, pointing along it

    t, x, y, chi, ct = simulate(y0, chi0)

    ct0 = wrap(chi0 - chi_desired(y0))
    print(f"Start: {y0:.1f} m off the line, heading {np.degrees(chi0):.0f} deg")
    print(f"Desired heading there: {np.degrees(chi_desired(y0)):+.1f} deg")
    print(f"Initial heading error: {np.degrees(ct0):+.1f} deg "
          f"({ct0:.4f} rad)\n")

    print("HEADING error (layer 2)")
    print(f"  predicted arrival  {arrival_time(ct0):.3f} s")
    print(f"  measured  arrival  {first_below(t, ct, 1e-6):.3f} s")
    print("  -> finite time, exactly as in step 1.\n")

    print("POSITION error (layer 1)")
    print(f"{'tolerance':>12} {'time':>10}")
    for tol in (1e-1, 1e-2, 1e-3, 1e-4):
        print(f"{tol:12.0e} {first_below(t, y, tol):10.2f}")
    print("  -> each extra decimal costs the SAME time again, forever.")
    print("     Asymptotic. It never actually arrives.\n")

    print("THIS IS THE POINT OF STEP 2:")
    print("  the heading converges in finite time,")
    print("  the position converges only asymptotically.")
    print("  In the paper that is Proposition 1 and Proposition 2.")

    # -----------------------------------------------------------------
    fig, ax = plt.subplots(1, 3, figsize=(15, 4))

    ax[0].axhline(0, color="k", lw=2, label="the line ($y=0$)")
    ax[0].plot(x, y, lw=2, color="tab:orange", label="drone path")
    ax[0].plot(x[0], y[0], "o", color="tab:red", ms=8, label="start")
    ax[0].set_xlabel("x (m)"); ax[0].set_ylabel("y (m)")
    ax[0].set_title("It curves on, then flattens out")
    ax[0].legend(fontsize=9); ax[0].grid(alpha=0.3)

    ax[1].plot(t, np.degrees(ct), lw=2, color="tab:orange")
    ax[1].axvline(arrival_time(ct0), color="k", ls=":", lw=1)
    ax[1].text(arrival_time(ct0) + 0.4, -20, "predicted\narrival", fontsize=9)
    ax[1].axhline(0, color="grey", lw=0.8)
    ax[1].set_xlabel("time (s)")
    ax[1].set_ylabel(r"heading error $\tilde\chi$ (deg)")
    ax[1].set_title("LAYER 2: finite time")
    ax[1].set_xlim(0, 8); ax[1].grid(alpha=0.3)

    ax[2].semilogy(t, np.abs(y) + 1e-16, lw=2, color="tab:blue")
    ax[2].set_xlabel("time (s)"); ax[2].set_ylabel("$|y|$ (m), log scale")
    ax[2].set_title("LAYER 1: asymptotic (straight line = never arrives)")
    ax[2].grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("step2_steering.png", dpi=130)
    print("\nPlot saved: step2_steering.png")


if __name__ == "__main__":
    main()
