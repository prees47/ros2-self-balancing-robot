# Self-Balancing Robot — ROS 2 & Gazebo

A two-wheeled self-balancing robot simulated in **ROS 2 Jazzy** and **Gazebo**, stabilized by a **cascaded PID controller** and drivable in real time with keyboard teleoperation.

> 🎥 **Demo video:https://www.linkedin.com/feed/update/urn:li:activity:7510391201066397696/

## Overview

A two-wheeled inverted pendulum is inherently unstable: without continuous feedback it falls within a fraction of a second. This project implements a control system that keeps the robot upright, brings it to rest after disturbances, and accepts driving commands without losing balance.

## Key Features

- **Cascaded PID control**
  - *Inner loop:* regulates pitch angle from IMU feedback (angle + filtered gyro rate).
  - *Outer loop:* regulates translational velocity by adjusting the target lean angle, giving station-keeping instead of open-ended drift.
- **Signal filtering** — low-pass filters on gyro rate and odometry velocity remove sensor jitter that otherwise appears as chassis vibration.
- **Robust restart handling** — accumulated controller state (integrators, last command) is cleared automatically when the sensor stream is interrupted, so restarting the simulator alone does not cause a false start.
- **Safety features** — fall-angle cutoff, command saturation, slew-rate limiting, and integrator clamping.
- **Teleoperation integrated into the control loop** — keyboard commands are treated as a *requested velocity/turn rate* fed into the outer loop, not as a competing publisher on `/cmd_vel`. The balance controller remains the only node that commands the wheels.
- **Live telemetry** — pitch, target pitch, velocity, and motor command are printed at ~5 Hz for tuning and verification.

## Architecture

```
                 /teleop_cmd (Twist)
 keyboard_teleop ─────────────────────┐
                                      ▼
 /imu (Imu) ───────────────►  balance_controller  ───────►  /cmd_vel (Twist) ──► Gazebo diff-drive
 /odom (Odometry) ─────────►   outer loop: velocity → target pitch
                               inner loop: pitch PID → wheel command
```

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/imu` | `sensor_msgs/Imu` | in | orientation + angular rate |
| `/odom` | `nav_msgs/Odometry` | in | measured forward velocity |
| `/teleop_cmd` | `geometry_msgs/Twist` | in | requested speed / turn rate |
| `/cmd_vel` | `geometry_msgs/Twist` | out | wheel command to the simulator |

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim with `ros_gz_sim` and `ros_gz_bridge`
- Python 3.12

## Build

```bash
cd ~/ros2_ws/src
git clone <your-repo-url> self_balancing_robot
cd ~/ros2_ws
colcon build --packages-select self_balancing_robot
source install/setup.bash
```

## Run

Use three terminals (run `source ~/ros2_ws/install/setup.bash` in each):

```bash
# 1. Simulation
ros2 launch self_balancing_robot simulation.launch.py

# 2. Balance controller
ros2 run self_balancing_robot balance_controller

# 3. Keyboard teleop (click this terminal so it has focus)
ros2 run self_balancing_robot keyboard_teleop
```

### Keyboard controls

| Key | Action |
|---|---|
| `w` / `s` | increase forward / backward speed |
| `a` / `d` | turn left / right |
| `x` or `Space` | stop |
| `q` | quit teleop |

Speed and turn rate are capped conservatively (0.25 m/s, 1.0 rad/s) to stay within the range the balance loop was tuned for. Teleop commands expire after 1 s without a new message.

## Tuning

All calibration values sit in one block at the top of `balance_controller.py`:

| Parameter | Purpose | Working value |
|---|---|---|
| `invert_polarity` | sign convention between IMU and wheel command | `False` |
| `pitch_trim` | offset between sensor zero and true balance point (rad) | `0.00` |
| `kp_vel`, `ki_vel` | outer velocity loop gains | `0.2`, `0.01` |
| `kp_pitch`, `kd_pitch`, `ki_pitch` | inner pitch loop gains | `12.0`, `0.4`, `0.05` |
| `vel_filter_alpha` | velocity smoothing (smaller = smoother) | `0.15` |

Recommended order when re-tuning: **polarity → pitch trim → velocity loop**, changing one value at a time and watching the telemetry output.

## Results (simulation)

- Holds balance at rest with no drift.
- Recovers from pushes producing pitch excursions above 20° and settles to a standstill.
- Drives and turns under keyboard control while remaining balanced.

## Repository Structure

_Adjust to match your actual layout._

```
self_balancing_robot/
├── self_balancing_robot/
│   ├── balance_controller.py
│   └── keyboard_teleop.py
├── launch/
│   └── simulation.launch.py
├── urdf/
├── meshes/
├── package.xml
├── setup.py
└── README.md
```

## Roadmap

- Port the controller to physical hardware
- Add joystick / autonomous velocity commands
- Auto-tuning of PID gains



## Author

Soumik Dey — [LinkedIn](https://www.linkedin.com/in/soumik-dey-682284421/)
