#!/usr/bin/env python3
"""
step1_finite_time.py

The simplest possible demonstration of finite-time convergence.

Setup: a point moves forward along x at constant speed. It starts off to
one side of the line y = 0. We control its lateral velocity to bring it
back onto the line.

    y_dot = u        (y = how far off the line we are)

We compare two controllers:

    LINEAR              u = -k*y
    FINITE-TIME         u = -eta * sign(y) * |y|^p     with 0 < p < 1

They look almost identical. They are not. The linear one never actually
arrives. The second one arrives at a specific time you can calculate.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def u_linear(y, k=1.0):
    return -k * y


def u_finite_time(y, eta=1.0, p=0.6):
    """Same idea, but the error is raised to a power less than 1.

    Why sign(y)*|y|**p and not just y**p?
    In Python, (-0.5)**0.6 is nan -- you cannot raise a negative number
    to a fractional power. So we take the size |y|, raise that, and put
    the sign back on by hand. (This is the `signed_pow` trick.)
    """
    return -eta * np.sign(y) * np.abs(y) ** p


# ---------------------------------------------------------------------
# Predicted arrival time for the finite-time law
#
#   y_dot = -eta * y^p        (taking y > 0)
#
# Separate and integrate from y0 down to 0:
#
#   integral of y^(-p) dy = -eta * integral dt
#   y^(1-p) / (1-p)        = -eta * t   + C
#
# Setting y(0) = y0 and solving for the t where y hits 0:
# ---------------------------------------------------------------------

def arrival_time(y0, eta=1.0, p=0.6):
    return np.abs(y0) ** (1.0 - p) / (eta * (1.0 - p))


def simulate(controller, y0, dt=0.001, tmax=12.0, **kwargs):
    t = np.arange(0.0, tmax, dt)
    y = np.zeros_like(t)
    y[0] = y0
    for i in range(1, len(t)):
        y[i] = y[i - 1] + dt * controller(y[i - 1], **kwargs)
        # once we cross zero, stop -- we have arrived
        if np.sign(y[i]) != np.sign(y[i - 1]):
            y[i:] = 0.0
            break
    return t, y


def first_time_below(t, y, tol):
    idx = np.where(np.abs(y) < tol)[0]
    return t[idx[0]] if len(idx) else np.nan


def main():
    y0 = 2.0          # start 2 metres off the line

    t_lin, y_lin = simulate(u_linear, y0, k=1.0)
    t_ft,  y_ft  = simulate(u_finite_time, y0, eta=1.0, p=0.6)

    print("Starting 2.0 m off the line.\n")

    print("How long until the error is smaller than...")
    print(f"{'tolerance':>12} {'LINEAR':>12} {'FINITE-TIME':>14}")
    for tol in (1e-1, 1e-2, 1e-3, 1e-6, 1e-9):
        a = first_time_below(t_lin, y_lin, tol)
        b = first_time_below(t_ft,  y_ft,  tol)
        print(f"{tol:12.0e} {a:12.2f} {b:14.2f}")

    print()
    print("Look at the LINEAR column: every time you ask for 10x more")
    print("accuracy, it costs you another ~2.3 seconds, forever. It never")
    print("finishes. That is what 'asymptotic' means.")
    print()
    print("The FINITE-TIME column stops changing. It has arrived, exactly,")
    print("and it stays there.")
    print()
    print(f"Predicted arrival time  = {arrival_time(y0):.3f} s")
    print(f"Measured arrival time   = {first_time_below(t_ft, y_ft, 1e-9):.3f} s")
    print()
    print("That formula is the whole point. You can promise your prof a")
    print("number, and the simulation delivers it.")

    # -----------------------------------------------------------------
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))

    ax[0].plot(t_lin, y_lin, label="linear  $u=-ky$", lw=2)
    ax[0].plot(t_ft,  y_ft,  label="finite-time  $u=-\\eta\\,|y|^{0.6}$sign$(y)$", lw=2)
    ax[0].axvline(arrival_time(y0), color="k", ls=":", lw=1)
    ax[0].text(arrival_time(y0) + 0.15, 1.5, "predicted\narrival", fontsize=9)
    ax[0].set_xlabel("time (s)")
    ax[0].set_ylabel("distance off the line, $y$ (m)")
    ax[0].set_title("Both look fine here")
    ax[0].legend(fontsize=9)
    ax[0].grid(alpha=0.3)

    ax[1].semilogy(t_lin, np.abs(y_lin) + 1e-16, lw=2)
    ax[1].semilogy(t_ft,  np.abs(y_ft)  + 1e-16, lw=2)
    ax[1].axvline(arrival_time(y0), color="k", ls=":", lw=1)
    ax[1].set_xlabel("time (s)")
    ax[1].set_ylabel("$|y|$ (m), log scale")
    ax[1].set_title("On a log scale the difference is obvious")
    ax[1].grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("step1_finite_time.png", dpi=130)
    print("\nPlot saved: step1_finite_time.png")


if __name__ == "__main__":
    main()
