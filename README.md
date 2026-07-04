# Virtual Leader-based Multi-UAV Formation Path Following

A ROS 2 / PX4 SITL / Gazebo implementation of the vector-field + Lyapunov hybrid guidance strategy from:

> **S. Basak and S. Ghosh**, *"Virtual Leader-based Multi-UAV Formation Path Following Guidance Using Vector Field,"* 2025 Eleventh Indian Control Conference (ICC), IISc Bengaluru, India. DOI: 10.1109/ICC69100.2025.11372370

The system drives a swarm of PX4 SITL quadrotors to converge to and hold a rigid geometric formation around a **virtual leader**, while the virtual leader follows a reference path using a vector-field guidance law. Followers regulate their heading and speed through the paper's guidance laws (Eqs. 14, 15, 21), giving finite-time heading convergence and asymptotic formation-error convergence.

---

## Table of Contents

- [Overview](#overview)
- [What's Implemented](#whats-implemented)
- [System Architecture](#system-architecture)
- [Guidance Laws](#guidance-laws)
- [Prerequisites](#prerequisites)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Running the Simulation](#running-the-simulation)
- [Configuration & Parameters](#configuration--parameters)
- [Diagnostics & Visualization](#diagnostics--visualization)
- [Post-processing (MATLAB)](#post-processing-matlab)
- [Results](#results)
- [Design Notes & Lessons Learned](#design-notes--lessons-learned)
- [Known Limitations](#known-limitations)
- [Citation](#citation)

---

## Overview

The guidance strategy combines two ideas:

1. **Virtual structure / virtual leader.** A constant-speed virtual leader follows the reference path via a vector-field path-following law. Because the leader's forward speed is a known constant, followers never depend on noisy or delayed leader-velocity estimates.
2. **Hybrid follower guidance.** Each follower uses a vector field to generate its *desired heading* and a Lyapunov-based controller to generate its *commanded ground speed*, so that it converges to a fixed offset `G_i = (g_xi, g_yi)` in the leader's body frame.

The result is precise formation path-following with quadrant-separated initial partitions that provide inter-agent collision avoidance.

---

## What's Implemented

- Constant-speed virtual leader with vector-field path following (Eq. 5, 6).
- Follower desired-heading vector field (Eq. 14).
- Follower finite-time heading controller (Eq. 15).
- Follower Lyapunov-based ground-speed law (Eq. 21).
- Multi-vehicle PX4 SITL orchestration in Gazebo.
- Live diagnostics via per-drone internal controller topics for `rqt_plot`.
- Dual CSV logging for offline MATLAB analysis.
- RViz swarm visualization.
- Coordinate-frame handling for PX4 spawn-relative odometry (ENU ↔ NED).

Formation tracking error has been driven from ~6.5 m down to a steady-state of roughly **0.1–0.5 m** with stable shape maintenance through turns.

---

## System Architecture

The stack is a set of ROS 2 nodes orchestrated by `start_swarm.sh`:

| Node | Responsibility |
|------|----------------|
| **Virtual leader** | Integrates constant-speed leader kinematics; runs the vector-field path-following law; publishes leader pose, heading `χ_l`, and analytic yaw rate `χ̇_l` on `twist.angular.z`. |
| **Formation manager** | Holds the desired formation geometry `G_i`; computes each follower's goal point in the leader body frame `F_L`; publishes per-follower targets. |
| **Follower controller** | One instance per drone. Computes formation errors `(e_xi, e_yi)`, the desired heading (Eq. 14), the commanded course (Eq. 15), and the commanded ground speed (Eq. 21); sends velocity/attitude setpoints to PX4. |
| **Plotter** | Streams tracking error, shape error, and heading to CSV loggers and diagnostic topics. |
| **Swarm visualizer** | Publishes markers/paths for RViz. |

Data flow (simplified):

```
reference path ─▶ [virtual leader] ─▶ leader state (pose, χ_l, χ̇_l)
                                          │
                                          ▼
                                 [formation manager] ─▶ goal points G_i
                                          │
                                          ▼
              odometry ─────────▶ [follower controller ×N] ─▶ PX4 setpoints
                                          │
                                          ▼
                              [plotter] ─▶ CSV + diagnostic topics
                                          │
                                          ▼
                              [swarm visualizer] ─▶ RViz
```

---

## Guidance Laws

The core equations reproduced from the paper (see Section III):

**Leader vector field (Eq. 5):**
```
χ_l^d = χ^p − χ^∞ · (2/π) · atan(k·d)
```

**Follower desired heading (Eq. 14):**
```
χ_i^d = χ_l − χ^∞ · (2/π) · atan(k_y · e_yi)
```

**Follower commanded course, finite-time convergence (Eq. 15):**
```
χ_i^c = χ_i + (χ̇_l / α_i)
        − χ^∞ · (2 / (α_i·π)) · (k_y / (1 + (k_y·e_yi)^2)) · ė_yi
        − (η / α_i) · (χ̃_i)^(n/m)
```

**Follower ground-speed law, asymptotic formation-error convergence (Eq. 21):**
```
v_gi = ( v_gl·e_xi − k_D·D_i^2 − χ̇_l·[e_xi·g_yi − e_yi·g_xi] )
       / ( e_xi·cos(χ_i − χ_l) + e_yi·sin(χ_i − χ_l) )
```

where `D_i = sqrt(e_xi² + e_yi²)` is the formation error magnitude.

---

## Prerequisites

- **Ubuntu 22.04** (recommended for the ROS 2 / PX4 combination)
- **ROS 2** (Humble or your target distro)
- **PX4-Autopilot** (SITL build)
- **Gazebo** (the version bundled with your PX4 SITL setup)
- **Python 3** with `numpy`
- **MATLAB R2023a+** (for offline analysis — optional)
- `rqt_plot`, `rviz2` for live diagnostics

> Adjust the exact ROS distro / PX4 branch to whatever you built against and record it here so the setup is reproducible.

---

## Repository Structure

```
.
├── README.md
├── start_swarm.sh                 # Multi-instance orchestration entry point
├── src/
│   ├── virtual_leader/            # Leader kinematics + VF path following
│   ├── formation_manager/         # Formation geometry & goal points
│   ├── follower_controller/       # Eqs. 14, 15, 21
│   ├── plotter/                   # CSV loggers + diagnostic topics
│   └── swarm_visualizer/          # RViz markers/paths
├── config/                        # Gains, formation offsets, spawn offsets
├── launch/                        # ROS 2 launch files
├── matlab/
│   └── analyze_formation.m        # Post-processing & paper-quality figures
└── logs/                          # Generated CSV logs
```

> Update the tree to match your actual package/workspace layout.

---

## Installation

1. **Clone into your ROS 2 workspace:**
   ```bash
   cd ~/ros2_ws/src
   git clone <your-repo-url> multi_uav_formation
   ```

2. **Build the workspace:**
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Point the launcher at your PX4 install.** Edit the PX4 path and SITL model settings at the top of `start_swarm.sh` (or in `config/`).

---

## Running the Simulation

The full swarm is launched with a single script:

```bash
./start_swarm.sh
```

This script:

1. Starts Gazebo with the PX4 SITL world.
2. Spawns the drones **exactly at their formation offsets** (so there's no initial convergence scramble).
3. Launches the virtual leader, formation manager, per-drone follower controllers, plotter, and swarm visualizer.

To watch the formation:

```bash
rviz2 -d config/swarm.rviz
```

To stop everything, `Ctrl-C` in the launcher terminal (add cleanup of stray PX4/Gazebo processes to the script if you don't already).

---

## Configuration & Parameters

Key tunables (matching the paper's notation) live in `config/`:

| Parameter | Symbol | Working value | Notes |
|-----------|--------|---------------|-------|
| Leader speed | `v_gl` | 0.5 m/s | Scaled for the Gazebo world (paper uses 15 m/s). |
| Lateral gain | `k_y` | 0.5 | Raised from 0.02 for reasonable convergence time. |
| Longitudinal gain | `k_D` | 0.1 | Raised from 0.02 to shorten the time constant. |
| Path VF gain | `k` | — | Cross-track vector-field gain. |
| Course-hold rate | `α_i` | — | Autopilot loop constant. |
| Finite-time gain | `η` | — | Heading-error convergence gain. |
| Exponents | `n, m` | odd co-prime, 0 < n < m | Finite-time term `(χ̃_i)^(n/m)`. |
| Formation offsets | `G_i` | — | Per-drone `(g_xi, g_yi)` in `F_L`. |
| Spawn offsets | `spawn_ned` | — | Per-drone NED offset added to every consumer node. |

> **Trajectory geometry constraint:** the minimum turn radius of the reference path **must exceed the formation offset**, or the Eq. 21 denominator can go singular. The current path is a `tanh(a·sin(ωt))` zigzag giving a constant 0.5 m/s speed and a ~6.1 m minimum turn radius against a 3 m formation offset.

---

## Diagnostics & Visualization

- **Live plots:** each follower publishes its internal controller signals (errors, headings, commands) on dedicated topics for `rqt_plot`:
  ```bash
  rqt_plot
  ```
- **RViz:** formation shape, leader/follower paths, and goal markers.
- **CSV logs:** two loggers write time-stamped state to `logs/` for offline analysis.

---

## Post-processing (MATLAB)

Run the analysis script against the generated logs:

```matlab
cd matlab
analyze_formation
```

It produces:

- Tracking-error vs. time
- Formation shape-error vs. time
- Trajectory (leader + followers)
- Heading alignment vs. time
- Leader body-frame scatter (follower positions in `F_L`)
- A convergence table: steady-state error, RMS, peak, and settling time per drone

---

## Results

- Formation tracking error converged from **~6.5 m → ~0.1–0.5 m**.
- Stable formation shape maintained through turns.
- Follower headings asymptotically align with the leader's heading (as predicted by Proposition 1 + Remark 1).
- Followers remain in their assigned quadrants, giving inter-agent collision avoidance.

---

## Design Notes & Lessons Learned

These were the non-obvious issues resolved during bring-up — worth keeping in the README so they aren't rediscovered the hard way:

- **PX4 multi-vehicle SITL odometry is spawn-relative, not world-relative.** Each drone reports odometry relative to its own spawn point. Confirmed when the initial error matched `√(3² + 5²)`. Fix: add `spawn_ned` offsets to **every** consumer node.
- **ENU ↔ NED conversion:** `NED = (ENU_y, ENU_x, −ENU_z)`; display in RViz/Gazebo needs the reverse swap.
- **Analytic derivatives beat finite differences.** Yaw rate and velocity derivatives were switched from noisy finite differences to analytic values published on `twist.angular.z`.
- **Velocity direction must use the current heading `χ_i`,** not the commanded heading `χ_c` — the latter caused drones to fling outward.
- **No `max(0.0, …)` speed clamp.** It blocked reverse motion and froze drones away from their targets.
- **Leader speed scaling** was originally 30× too large (15.0 → 0.5).
- **Trajectory geometry must respect formation geometry** — see the turn-radius constraint above.
- **Small gains + large offsets = very slow convergence.** Gain tuning must be validated against its time-constant implications.

---

## Known Limitations

- Collision avoidance relies on quadrant/sector separation; densely populated formations impose restrictive leader turn-rate conditions (see paper Section III-C).
- Simulation parameters (speeds, gains) are scaled for the Gazebo world and differ from the paper's numerical study.
- Validated for a small swarm; scaling to `N` followers requires partitioning the plane into `N` disjoint sectors.

---

## Citation

If you use this implementation, please cite the original paper:

```bibtex
@inproceedings{basak2025virtual,
  title     = {Virtual Leader-based Multi-UAV Formation Path Following Guidance Using Vector Field},
  author    = {Basak, Subham and Ghosh, Satadal},
  booktitle = {2025 Eleventh Indian Control Conference (ICC)},
  year      = {2025},
  address   = {Bengaluru, India},
  doi       = {10.1109/ICC69100.2025.11372370}
}
```
