# Differential Drive Robot Simulator

A 2D simulation of a differential drive robot with P/PD/PID control for autonomous navigation to target waypoints.

![Robot Simulation](https://img.shields.io/badge/Python-3.7+-blue.svg)
![Pygame](https://img.shields.io/badge/Pygame-2.0+-green.svg)

## Overview

This simulator demonstrates a two-wheeled differential drive robot navigating to user-defined targets using different control strategies. The robot uses forward kinematics to compute motion and implements proportional (P), proportional-derivative (PD), and proportional-integral-derivative (PID) controllers for linear and angular velocity control.

## Features

- **Real-time 2D visualization** of robot motion and trajectory
- **Three controller modes:**
  - P linear + P angular (basic proportional control)
  - PD linear + P angular (adds damping to linear motion)
  - PD linear + PID angular (full PID for precise heading control)
- **Interactive target placement** via mouse clicks
- **Visual trajectory tracking** showing the robot's path
- **Real-time telemetry display** showing position, heading, and control commands

## Requirements

```txt
pygame>=2.0.0
```

Install dependencies:
```bash
pip install -r requirements.txt
```

Or directly:
```bash
pip install pygame
```

## Usage

Run the simulator:
```bash
python differential_drive_simulator.py
```

### Controls

| Key/Action | Function |
|------------|----------|
| **Left Click** | Place target waypoint at cursor position |
| **1** | Switch to P linear + P angular controller |
| **2** | Switch to PD linear + P angular controller |
| **3** | Switch to PD linear + PID angular controller (recommended) |
| **R** | Reset robot to origin (0, 0) |
| **Space** | Pause/unpause simulation |
| **ESC** | Quit application |

### Display Elements

- **Blue triangle**: Robot (pointing in heading direction)
- **Black line**: Heading indicator
- **Red circle with crosshair**: Target waypoint
- **Green trail**: Robot trajectory history
- **Origin crosshair**: World coordinate center (0, 0)

## How It Works

### Robot Kinematics

The differential drive robot uses two independently controlled wheels with the following kinematic model:

```
v = linear velocity (m/s)
ω = angular velocity (rad/s)
R = wheel radius
L = wheel base (distance between wheels)

ω_right = (2v + ωL) / (2R)
ω_left = (2v - ωL) / (2R)
```

Position is updated using:
```
x' = x + v·cos(θ)·dt
y' = y + v·sin(θ)·dt
θ' = θ + ω·dt
```

### Control Strategy

The controller computes linear and angular velocities based on:

1. **Distance error**: Euclidean distance to target
2. **Heading error**: Angular difference between current heading and desired heading toward target

#### Controller Modes

**Mode 1: P linear + P angular**
- Linear velocity: `v = Kp_d × distance`
- Angular velocity: `ω = Kp_θ × heading_error`
- Simple but may oscillate

**Mode 2: PD linear + P angular**
- Adds derivative term to dampen linear motion
- `v = Kp_d × distance + Kd_d × distance_rate`
- Smoother approach to target

**Mode 3: PD linear + PID angular** (Recommended)
- Full PID for angular control
- `ω = Kp_θ × error + Ki_θ × ∫error + Kd_θ × error_rate`
- Best performance with minimal overshoot

### Tuned Parameters

The default controller gains are pre-tuned for stable performance:

```python
kp_d = 0.9    # Linear proportional gain
kd_d = 0.6    # Linear derivative gain
kp_th = 3.0   # Angular proportional gain
ki_th = 0.2   # Angular integral gain
kd_th = 0.5   # Angular derivative gain
```

## Robot Specifications

| Parameter | Value | Units |
|-----------|-------|-------|
| Wheel radius | 0.03 | m |
| Wheel base | 0.18 | m |
| Max linear velocity | 0.6 | m/s |
| Max angular velocity | 3.0 | rad/s |
| Stop threshold | 0.035 | m |

## Customization

### Modify Controller Gains

Edit the gains in the `main()` function:

```python
controller.kp_d = 0.9   # Linear P gain
controller.kd_d = 0.6   # Linear D gain
controller.kp_th = 3.0  # Angular P gain
controller.ki_th = 0.2  # Angular I gain
controller.kd_th = 0.5  # Angular D gain
```

### Change Robot Dimensions

Modify constants at the top of the file:

```python
WHEEL_RADIUS = 0.03   # meters
WHEEL_BASE = 0.18     # meters
ROBOT_LENGTH = 0.25   # for visualization
ROBOT_WIDTH = 0.18    # for visualization
```

### Adjust Simulation Parameters

```python
FPS = 60              # Simulation frame rate
TRAJECTORY_MAX = 10000  # Max trajectory points stored
```

## Understanding the Display

The simulator shows:
- **Position**: Robot's (x, y) coordinates in meters
- **Heading (θ)**: Robot's orientation in radians
- **Distance to target**: Euclidean distance in meters
- **Heading error**: Angular error in radians
- **Command velocities**: Current v and ω being applied

## Learning Objectives

This simulator is useful for:
- Understanding differential drive kinematics
- Comparing P, PD, and PID control strategies
- Visualizing the effect of controller gains
- Learning about heading control and path tracking
- Experimenting with robot motion planning

## Troubleshooting

**Robot oscillates around target:**
- Reduce proportional gains (kp_d, kp_th)
- Increase derivative gains (kd_d, kd_th)

**Robot overshoots target:**
- Increase derivative gains for damping
- Reduce max velocities

**Robot turns too slowly:**
- Increase angular proportional gain (kp_th)
- Check max angular velocity limit

**Robot doesn't reach target:**
- Check stop threshold (default 0.035 m)
- Verify target is within workspace

## License

MIT License - feel free to use and modify for educational purposes.

## Contributing

Contributions are welcome! Consider adding:
- Obstacle avoidance
- Path planning algorithms (A*, RRT)
- Multiple waypoint sequences
- Sensor simulation (LIDAR, encoders)
- Different robot models (Ackermann, omnidirectional)

## Author

Created for robotics education and control systems demonstration.

---

**Enjoy experimenting with robot control! 🤖**