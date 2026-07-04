# ROS 2 PX4 Multi-UAV Swarm Formation Control

## Overview

This project implements a leader–follower multi-UAV formation control framework using **ROS 2 Humble**, **PX4 SITL**, and **Gazebo Classic**. A virtual leader generates predefined trajectories while follower UAVs maintain fixed relative offsets using a velocity-based proportional (P) controller. The framework demonstrates coordinated takeoff, offboard control, formation maintenance, and trajectory tracking in simulation.

This implementation serves as the baseline controller for future research on nonlinear guidance strategies such as Sliding Mode Control (SMC) and Vector Field Guidance.

---

## Features

- Multi-UAV simulation using PX4 SITL
- ROS 2 Humble architecture
- Virtual leader–follower formation
- Figure-8, circular and straight-line trajectories
- Velocity-based proportional controller
- Offboard control mode
- Real-time trajectory visualization
- Formation error logging
- CSV data generation for MATLAB analysis

---

## Software Stack

- Ubuntu 22.04
- ROS 2 Humble Hawksbill
- PX4 Autopilot v1.16
- Gazebo Classic
- Python
- px4_msgs
- XRCE-DDS

---

## Controller

The follower computes the desired formation position

```
Target Position = Leader Position + Desired Offset
```

The velocity command is generated using a proportional controller

```
v = Kp (Target Position − Current Position)
```

where

- Kp is the proportional gain
- velocity commands are sent to PX4 in Offboard mode

---

## Repository Structure

```
swarm_formation/

├── launch/
├── logs/
├── scripts/
├── swarm_formation/
│   ├── follower_controller.py
│   ├── formation_manager.py
│   ├── virtual_leader.py
│   ├── figure8_leader.py
│   └── ...
├── package.xml
├── setup.py
└── README.md
```

---

## Results

The implemented controller successfully demonstrates

- Stable coordinated takeoff
- Virtual leader trajectory generation
- Formation acquisition
- Formation maintenance
- Figure-8 trajectory tracking
- CSV logging for performance evaluation

---

## Current Limitations

- Sensitive to gain tuning
- Formation error increases during aggressive turns
- Limited disturbance rejection
- No robustness guarantees under uncertainties

---

## Future Work

- Sliding Mode Controller (SMC)
- Vector Field Guidance
- Lyapunov-based speed controller
- Collision avoidance
- Outdoor PX4 flight validation

---

## Author

Sai Palani Kumar G S

B.Tech Aerospace Engineering

Indian Institute of Technology Kharagpur

Research Internship – Multi-UAV Formation Control
