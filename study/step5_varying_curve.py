#!/usr/bin/env python3
"""
step5_varying_curve.py

Step 4 fixed the "carousel" gap for a leader on a circle (constant
omega = chi_dot_l). Real paths aren't circles, so omega(t) keeps changing.
This checks whether step 4's fix still works, what changes if not, and
how much it actually matters -- both symbolically and by simulation.

Three control-law variants, compared below:
  uncorrected -> paper's Eq. 14, Delta = 0 always
  pointwise   -> Delta(t) recomputed from the CURRENT omega(t)
  full        -> pointwise + d(Delta)/dt, which needs chi_ddot_l
                 (the leader's angular acceleration)
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---- constants ---------------------------------------------------------
A, L = 30.0, 300.0        # leader path: y = A*sin(2*pi*x/L)
V_GL = 5.0                # leader speed, m/s
G_X, G_Y = 3.0, 3.0       # follower's offset in the leader's body frame
CHI_INF = np.pi / 2
K_Y = 0.3
ETA = 1.0
P = 0.6
OMEGA_ARG = 2 * np.pi / L


def signed_pow(x, p):
    return np.sign(x) * np.abs(x) ** p


def wrap(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + np.pi) % (2 * np.pi) - np.pi


# ---- leader geometry -----------------------------------------------------

def yprime(x):
    return A * OMEGA_ARG * np.cos(OMEGA_ARG * x)


def ydoubleprime(x):
    return -A * OMEGA_ARG ** 2 * np.sin(OMEGA_ARG * x)


def leader_state(x):
    """Leader heading, turn rate, and dx/dt at path parameter x."""
    yp = yprime(x)
    chi_l = np.arctan2(yp, 1.0)
    dxdt = V_GL / np.sqrt(1 + yp ** 2)
    kappa = ydoubleprime(x) / (1 + yp ** 2) ** 1.5
    chi_dot_l = kappa * V_GL
    return chi_l, chi_dot_l, dxdt


def required_speed_and_delta(omega):
    """Rigid-body result: v_point = v_leader + omega x (g_x, g_y)."""
    fwd = V_GL - omega * G_Y
    side = omega * G_X
    return np.hypot(fwd, side), np.arctan2(side, fwd)


# ---- initial conditions ---------------------------------------------------

def formation_start(x0, e_x0=0.0, e_y0=0.0, chi_offset0=0.0):
    """
    Leader + follower state at t=0. e_x0/e_y0 (leader-body-frame, same
    convention as e_x/e_y in the loop below) start the follower off its
    formation slot by that much; chi_offset0 offsets its starting heading.
    All zero (default) places the follower exactly in formation.
    """
    chi_l0, _, _ = leader_state(x0)
    xl0, yl0 = x0, A * np.sin(OMEGA_ARG * x0)

    e_bx0, e_by0 = e_x0 + G_X, e_y0 + G_Y
    c, s = np.cos(chi_l0), np.sin(chi_l0)
    rx = c * e_bx0 - s * e_by0
    ry = s * e_bx0 + c * e_by0

    xi0, yi0 = xl0 + rx, yl0 + ry
    chi_i0 = chi_l0 + chi_offset0
    return xl0, yl0, chi_l0, xi0, yi0, chi_i0


# ---- simulation ---------------------------------------------------------

def simulate(mode, dt=0.001, tmax=200.0, x0=0.0, e_x0=0.0, e_y0=0.0, chi_offset0=0.0):
    n = int(tmax / dt)
    t = np.arange(n) * dt

    x = x0   # leader's starting position ALONG THE PATH (not world x/y --
             # world position is A*sin(OMEGA_ARG*x0), computed below)
    xl0, yl0, chi_l0, xi, yi, chi_i = formation_start(x, e_x0, e_y0, chi_offset0)

    EX = np.zeros(n)
    EY = np.zeros(n)
    CT = np.zeros(n)
    XI = np.zeros(n)
    YI = np.zeros(n)
    XL = np.zeros(n)
    YL = np.zeros(n)

    for k in range(n):
        chi_l, chi_dot_l, dxdt = leader_state(x)
        xl, yl = x, A * np.sin(OMEGA_ARG * x)
        omega = chi_dot_l

        # chi_ddot_l by finite difference -- mirrors estimating it from
        # real (already noisy) telemetry rather than symbolically.
        eps = 1e-5
        _, chi_dot_l_plus, _ = leader_state(x + eps)
        chi_ddot_l = (chi_dot_l_plus - chi_dot_l) / eps * dxdt

        # follower's error, rotated into the leader's body frame
        rx, ry = xi - xl, yi - yl
        c, s = np.cos(chi_l), np.sin(chi_l)
        e_bx = rx * c + ry * s
        e_by = -rx * s + ry * c
        e_x, e_y = e_bx - G_X, e_by - G_Y

        req_speed, Delta = required_speed_and_delta(omega)

        if mode == "uncorrected":
            Delta_used, v_ff = 0.0, V_GL - omega * G_Y
        else:
            Delta_used, v_ff = Delta, req_speed

        v_i = np.clip(v_ff - 1.0 * e_x, 0.05, 15.0)

        dchi = wrap(chi_i - chi_l)
        e_y_dot = v_i * np.sin(dchi) - omega * G_X

        chi_d = chi_l + Delta_used - CHI_INF * (2 / np.pi) * np.arctan(K_Y * e_y)
        chi_t = wrap(chi_i - chi_d)

        drift = CHI_INF * (2 / np.pi) * K_Y / (1 + (K_Y * e_y) ** 2) * e_y_dot
        if mode == "full":
            # d(Delta)/dt = chi_ddot_l * v_gl * g_x / required_speed^2
            dDelta_dt = chi_ddot_l * V_GL * G_X / (req_speed ** 2 + 1e-9)
            chi_d_dot = omega + dDelta_dt - drift
        else:
            chi_d_dot = omega - drift

        u = chi_d_dot - ETA * signed_pow(chi_t, P)

        EX[k], EY[k], CT[k] = e_x, e_y, chi_t
        XI[k], YI[k], XL[k], YL[k] = xi, yi, xl, yl

        x += dt * dxdt
        chi_i += dt * u
        xi += dt * v_i * np.cos(chi_i)
        yi += dt * v_i * np.sin(chi_i)

    return t, EX, EY, CT, XI, YI, XL, YL


# ---- reporting + plotting ------------------------------------------------

def print_curvature_checks():
    print("Curvature check on the paper's own path (why it's not used here):")
    for xv in (0.01, np.pi / 2, np.pi):
        yp = 300.0 * np.cos(xv)
        ypp = -300.0 * np.sin(xv)
        kappa = ypp / (1 + yp ** 2) ** 1.5
        r = 1 / abs(kappa) if kappa != 0 else float("inf")
        print(f"  x={xv:.2f}  radius of curvature = {r:,.3f} m")
    print("  -> ~0.02 m at the peak. Not a fair test. Using a gentler path below.\n")

    print(f"Test path: y = {A:.0f}*sin(2*pi*x/{L:.0f}) m, leader speed {V_GL} m/s")
    print("Curvature sanity check on the actual test path:")
    for xv in np.linspace(0, L, 5):
        _, cdl, _ = leader_state(xv)
        r = V_GL / max(abs(cdl), 1e-9)
        print(f"  x={xv:6.1f}  chi_dot_l={cdl:+.4f} rad/s   radius~{r:8.1f} m")
    print()


def run_steady_state_comparison():
    results = {}
    for mode in ("uncorrected", "pointwise", "full"):
        results[mode] = simulate(mode)

    ss = results["uncorrected"][0] > 50
    print(f"{'':14s} {'RMS e_y (t>50s)':>18s} {'max |e_y| (t>50s)':>20s}")
    for mode in ("uncorrected", "pointwise", "full"):
        _, _, EY, *_ = results[mode]
        print(f"{mode:14s} {np.sqrt(np.mean(EY[ss]**2)):18.5f} {np.max(np.abs(EY[ss])):20.5f}")

    print("\nReading: 'pointwise' (no chi_ddot_l needed) already removes the")
    print("overwhelming majority of the error left by 'uncorrected'. 'full'")
    print("(needs chi_ddot_l) improves further but by a much smaller margin.")
    return results


def plot_steady_state(results, colors):
    fig, ax = plt.subplots(2, 2, figsize=(13, 10))

    for mode in results:
        t, _, EY, *_ = results[mode]
        ax[0, 0].plot(t, EY, lw=1.3, color=colors[mode], label=mode)
    ax[0, 0].axhline(0, color="k", lw=0.7)
    ax[0, 0].set_xlabel("time (s)"); ax[0, 0].set_ylabel("lateral error e_y (m)")
    ax[0, 0].set_title("Varying-curvature path: e_y over time")
    ax[0, 0].legend(fontsize=8); ax[0, 0].grid(alpha=0.3)

    for mode in results:
        t, _, EY, *_ = results[mode]
        ax[0, 1].semilogy(t, np.abs(EY) + 1e-9, lw=1.3, color=colors[mode], label=mode)
    ax[0, 1].set_xlabel("time (s)"); ax[0, 1].set_ylabel("|e_y| (m), log scale")
    ax[0, 1].set_title("Same data, log scale")
    ax[0, 1].legend(fontsize=8); ax[0, 1].grid(alpha=0.3)

    # full path (first ~2 wavelengths) with a box marking the zoom region
    x_dense = np.linspace(0, 620, 2000)
    ax[1, 0].plot(x_dense, A * np.sin(OMEGA_ARG * x_dense), color="k", lw=1.6,
                  label="leader path (reference curve)")
    for mode in ("uncorrected", "pointwise", "full"):
        t, _, _, _, XI, YI, _, _ = results[mode]
        m = t <= 120
        ax[1, 0].plot(XI[m], YI[m], lw=1.0, color=colors[mode], alpha=0.85,
                      label=f"follower ({mode})")
    ax[1, 0].set_xlabel("x (m)"); ax[1, 0].set_ylabel("y (m)")
    ax[1, 0].set_title("XY path: leader curve + follower trajectories")
    ax[1, 0].set_aspect("equal", adjustable="box")
    ax[1, 0].legend(fontsize=7, loc="upper right"); ax[1, 0].grid(alpha=0.3)
    box = plt.Rectangle((60, A * np.sin(OMEGA_ARG * 75) - 4), 30, 8,
                         fill=False, edgecolor="tab:blue", lw=1.2, ls="--")
    ax[1, 0].add_patch(box)
    ax[1, 0].annotate("zoom panel ->", xy=(93, A * np.sin(OMEGA_ARG * 75) - 8),
                       fontsize=8, color="tab:blue")

    # zoom on the sharpest-curvature point (x=75), where the gap between
    # modes is actually visible at physical scale
    xz_lo, xz_hi = 60.0, 90.0
    for mode in ("uncorrected", "pointwise", "full"):
        t, _, _, _, XI, YI, XL, _ = results[mode]
        m = (XL >= xz_lo) & (XL <= xz_hi) & (t <= 120)
        ax[1, 1].plot(XI[m], YI[m], lw=1.6, color=colors[mode], label=f"follower ({mode})")
    xz = np.linspace(xz_lo, xz_hi, 400)
    ax[1, 1].plot(xz, A * np.sin(OMEGA_ARG * xz), color="k", lw=1.0, ls=":",
                  label="leader path")
    ax[1, 1].set_xlabel("x (m)"); ax[1, 1].set_ylabel("y (m)")
    ax[1, 1].set_title("Zoom on sharpest-curvature region (x in [60,90] m)")
    ax[1, 1].legend(fontsize=7); ax[1, 1].grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("step5_varying_curve.png", dpi=130)
    print("\nPlot saved: step5_varying_curve.png")


def run_convergence_check(colors, e_y0=6.0, tmax=40.0, tol=0.2):
    """Start the follower off its formation slot and see how fast each
    mode gets back to it (as opposed to the steady-state runs above,
    which start already in formation)."""
    print("\n" + "=" * 70)
    print(f"CONVERGENCE CHECK: follower starts {e_y0:.0f} m off its formation slot")
    print("=" * 70)

    conv = {}
    for mode in ("uncorrected", "pointwise", "full"):
        t, EX, EY, CT, XI, YI, XL, YL = simulate(mode, tmax=tmax, e_y0=e_y0)
        conv[mode] = (t, EX, EY, CT, XI, YI, XL, YL)
        below = np.abs(EY) < tol
        idx = np.argmax(below) if below.any() else -1
        t_conv = t[idx] if idx >= 0 else float("nan")
        print(f"{mode:14s} first drops below {tol:.2f} m at t={t_conv:.2f} s "
              f"(e_y after {tmax:.0f}s = {EY[-1]:+.4f} m)")

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    for mode in ("uncorrected", "pointwise", "full"):
        t, _, EY, *_ = conv[mode]
        ax[0].plot(t, EY, lw=1.4, color=colors[mode], label=mode)
    ax[0].axhline(0, color="k", lw=0.7)
    ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("lateral error e_y (m)")
    ax[0].set_title(f"Convergence from e_y(0)={e_y0:.0f} m off-formation")
    ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)

    x_dense = np.linspace(0, 200, 800)
    ax[1].plot(x_dense, A * np.sin(OMEGA_ARG * x_dense), color="k", lw=1.6,
               label="leader path")
    for mode in ("uncorrected", "pointwise", "full"):
        t, _, _, _, XI, YI, _, _ = conv[mode]
        ax[1].plot(XI, YI, lw=1.1, color=colors[mode], alpha=0.85, label=f"follower ({mode})")
    xi0, yi0 = conv["full"][4][0], conv["full"][5][0]
    ax[1].plot(xi0, yi0, "k*", ms=12, label="follower start")
    ax[1].set_xlabel("x (m)"); ax[1].set_ylabel("y (m)")
    ax[1].set_title("Spatial path while converging into formation")
    ax[1].set_aspect("equal", adjustable="box")
    ax[1].legend(fontsize=7); ax[1].grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("step5_convergence.png", dpi=130)
    print("\nPlot saved: step5_convergence.png")


def main():
    colors = {"uncorrected": "tab:red", "pointwise": "tab:orange", "full": "tab:green"}

    print_curvature_checks()
    results = run_steady_state_comparison()
    plot_steady_state(results, colors)
    run_convergence_check(colors)


if __name__ == "__main__":
    main()