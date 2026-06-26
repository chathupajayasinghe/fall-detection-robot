# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 workspace for an autonomous differential-drive robot running on a Raspberry Pi. Combines a SLAMTEC RPLIDAR A1 sensor with DC motor control and encoder-based odometry to do real-time SLAM via the SLAM Toolbox.

## Key Commands

```bash
# Source ROS 2 environment first (required every new shell)
source /opt/ros/humble/setup.bash
source install/setup.bash

# Build workspace
colcon build --symlink-install

# Full robot bringup (LIDAR + motors + SLAM)
ros2 launch robot_driver robot_bringup.launch.py

# Launch LIDAR only
ros2 launch rplidar_ros rplidar_a1_launch.py

# Record and save a SLAM map interactively
./save_slam_map.sh

# Lint checks (run from workspace root)
colcon test --packages-select robot_driver
colcon test-result --verbose
```

## Architecture

The workspace has two ROS 2 packages under `src/`:

### `robot_driver` (Python, `ament_python`)
Custom package for this robot. Two nodes:

- **MotorController** (`motor_controller.py`) — the primary runtime node. Subscribes `/cmd_vel` (Twist), drives left/right DC motors via `gpiozero` PWM (pins 12/5/6 left, 13/23/24 right), reads quadrature encoders (pins 17/27 left, 22/10 right), publishes `/odom` at 20 Hz, and broadcasts the `odom → base_link` TF. Key constants: `wheel_separation=0.20m`, `wheel_radius=0.0325m`, `ticks_per_rev=20`.
- **OdometryPublisher** (`odom_publisher.py`) — encoder-only odometry without motor control; useful for testing odometry in isolation.

The single launch file `launch/robot_bringup.launch.py` wires together:
1. RPLIDAR A1 on `/dev/ttyUSB0` @ 115200 baud
2. A static TF `base_link → laser` (0.1 m height offset)
3. `motor_controller` node
4. SLAM Toolbox `online_async_launch` using `~/slam_config.yaml`

### `rplidar_ros` (C++, `ament_cmake`)
Upstream SLAMTEC driver vendored into this repo. Publishes `/scan` (LaserScan). The bundled SDK (`sdk/`) handles serial/TCP/UDP transport internally — do not modify SDK files. LIDAR model-specific launch files live in `launch/` (A1, A2, A3, S1–S3, T1, C1).

### TF Frame Tree
```
map → odom → base_link → laser
       ↑           ↑
  SLAM Toolbox  MotorController
```

### SLAM Map Workflow
`save_slam_map.sh` orchestrates the full capture-to-file pipeline: launches the robot, starts a bag recording of `/map`, enables teleop keyboard control, then on Ctrl+C extracts the map as `~/maps/home_map.pgm` + `~/maps/home_map.yaml`.

## Hardware & Serial Access

The LIDAR appears as `/dev/ttyUSB0`. If the port is permission-denied, install the udev rules:

```bash
cd src/rplidar_ros
./scripts/create_udev_rules.sh
```

GPIO access for motors requires running as root or adding the user to the `gpio` group. The `gpiozero` library uses the `lgpio` pin factory (`GPIOZERO_PIN_FACTORY=lgpio`).

## SLAM Configuration

SLAM Toolbox parameters are read from `~/slam_config.yaml` (not tracked in this repo). Edit that file on the Pi to tune map resolution, update rate, and loop-closure settings.
