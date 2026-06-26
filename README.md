# Fall Detection Robot

A ROS 2-based autonomous robot that uses LIDAR and SLAM to navigate and detect falls. Built for Raspberry Pi with a differential-drive motor setup and SLAMTEC RPLIDAR A1.

## Hardware

| Component | Details |
|-----------|---------|
| Platform | Raspberry Pi |
| LIDAR | SLAMTEC RPLIDAR A1 (via `/dev/ttyUSB0`) |
| Motors | DC motors with PWM control via GPIO |
| Encoders | Quadrature encoders (20 ticks/rev) |
| Wheel separation | 0.20 m |
| Wheel radius | 0.0325 m |

## Software Stack

- **ROS 2** (Humble) — middleware
- **SLAM Toolbox** — real-time mapping and localization
- **gpiozero + lgpio** — GPIO motor control
- **rplidar_ros** — SLAMTEC LIDAR driver

## Packages

### `robot_driver`
Custom Python package for motor control and odometry.
- Subscribes to `/cmd_vel` (geometry_msgs/Twist) for velocity commands
- Publishes `/odom` (nav_msgs/Odometry) at 20 Hz
- Broadcasts `odom → base_link` TF transform

### `rplidar_ros`
Vendored SLAMTEC LIDAR driver (C++).
- Publishes `/scan` (sensor_msgs/LaserScan)
- Supports RPLIDAR A1/A2/A3/S/T/C series

## Getting Started

### Prerequisites
```bash
sudo apt install ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard
pip install gpiozero lgpio
```

### Serial port permissions
```bash
cd src/rplidar_ros
./scripts/create_udev_rules.sh
```

### Build
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Run

**Full robot bringup** (LIDAR + motors + SLAM):
```bash
ros2 launch robot_driver robot_bringup.launch.py
```

**LIDAR only:**
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```

**Record and save a SLAM map interactively:**
```bash
./save_slam_map.sh
```
Drive the robot with keyboard controls, then press `Ctrl+C` to save the map to `~/maps/`.

## TF Frame Tree

```
map → odom → base_link → laser
       ↑           ↑
  SLAM Toolbox  MotorController
```

## Configuration

SLAM Toolbox parameters are loaded from `~/slam_config.yaml` on the robot (not tracked in this repo). Edit that file to tune map resolution, update rate, and loop-closure settings.
