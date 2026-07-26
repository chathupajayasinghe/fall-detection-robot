# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 **Jazzy** workspace for an autonomous **tracked** (tank-tread) differential-drive robot
running on a Raspberry Pi — a fall-detection / elder-care assistant. A SLAMTEC RPLIDAR A1,
DC motors with quadrature encoders, and a PIR + RCWL movement sensor pair feed three
capabilities: SLAM mapping, Nav2 autonomous navigation, and a cloud-triggered welfare-check
mission driven by Firestore.

On the Pi the workspace lives at `~/fall-detection-robot`.

## Key Commands

```bash
# Source ROS 2 environment first (required every new shell)
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Build workspace
colcon build --symlink-install
colcon build --packages-select robot_driver   # single package

# Hardware bringup: LIDAR + motor controller + welfare sensor + static TF
ros2 launch robot_driver robot_bringup.launch.py

# Bringup with SLAM mapping enabled (slam_toolbox is OFF by default)
ros2 launch robot_driver robot_bringup.launch.py start_slam_toolbox:=true

# Autonomous navigation (run alongside bringup, in a second terminal)
ros2 launch robot_driver nav2_navigation.launch.py

# Localization only (map_server + AMCL)
ros2 launch robot_driver localization.launch.py

# Record and save a SLAM map interactively
./save_slam_map.sh

# Lint checks (run from workspace root)
colcon test --packages-select robot_driver
colcon test-result --verbose
```

## Architecture

Two ROS 2 packages under `src/`.

### `robot_driver` (Python, `ament_python`)

Custom package for this robot. **Six** nodes, all registered as console scripts in `setup.py`:

- **MotorController** (`motor_controller.py`) — the primary runtime node, started by bringup.
  Subscribes `/cmd_vel` (Twist), drives left/right motors via `gpiozero` PWM, reads quadrature
  encoders, publishes `/odom` at 20 Hz, and broadcasts the `odom → base_link` TF. A 0.5 s
  `/cmd_vel` watchdog stops the motors if commands stop arriving.
- **WelfareSensor** (`welfare_sensor.py`) — started by bringup. Publishes raw PIR/RCWL state on
  `/welfare/pir` and `/welfare/rcwl` at 5 Hz. On `/welfare_check` it runs a settle-then-sample
  window and publishes a JSON verdict on `/welfare_result`.
- **LidarDifferencing** (`lidar_differencing.py`) — person detection by scan differencing.
  Stores an empty-room reference scan (`~/maps/reference_scan.npy`) on `/save_reference_scan`;
  on `/find_person` it diffs the live `/scan` against the reference, clusters the new returns,
  filters to person-sized clusters, and publishes the centroid on `/person_location`.
- **FirestoreDispatcher** (`firestore_dispatcher.py`) — the mission orchestrator. Watches a
  Firestore `fall_alerts` collection for `pending` docs, then drives the sequence:
  navigate to the scan point → `/find_person` → navigate to an approach pose 0.8 m short of
  the person → `/welfare_check` → write the verdict back to Firestore.
- **WaypointPatrol** (`waypoint_patrol.py`) — standalone Nav2 waypoint loop. Not launched by
  any launch file; run manually for testing.
- **OdometryPublisher** (`odom_publisher.py`) — ⚠️ **legacy, do not use.** Encoder-only
  odometry superseded by MotorController. Its constants are stale (`wheel_radius 0.0325`,
  `ticks_per_rev 20`), its right encoder channels are swapped relative to MotorController, and
  it publishes to the same `/odom` topic — so running it produces confidently wrong data.
  Slated for deletion.

> **Nav2 is not started by bringup.** `robot_bringup.launch.py` owns hardware only.
> Navigation is a separate launch file so the robot can be brought up without it.

#### Hardware constants — measured, do not "correct" from datasheets

These live in `MotorController` and were established by physical measurement over weeks.
Several deliberately disagree with the spec sheets:

| Constant | Value | Why |
|---|---|---|
| `wheel_radius` | `0.027` m | Encoder calibration: 1343 and 1389 ticks over two 1.0 m tape-measured drives (~1360 ticks/m) |
| `wheel_separation` | `0.17` m | **Not** the physical 0.135 m. Inflated to compensate track scrub on tank treads — 90° commanded rotations were measuring 114–115° |
| `ticks_per_rev` | `235` | Verified by hand-rotating one wheel a full turn (221–235 across trials) |
| `MIN_PWM` | `0.35` | Motor deadband — below this duty cycle the motors buzz without turning |
| `RIGHT_TRIM` | `0.97` | The **only active** straightness compensation. Open-loop test measured 2.7 cm drift over 154 cm |
| `STRAIGHT_KP` | `0.0` | Closed-loop straightness corrector, **deliberately disabled** after a sign-flip bug made the robot pivot instead of driving straight. The direction clamp beside it guards that failure. Inert at KP=0 — do not delete it as dead code |
| rotation boost | `×1.5` | Extra duty on pure pivots to break track stiction, where there is no forward component to help |

#### GPIO pin map

| Function | Pins |
|---|---|
| Left motor | PWM `12`, IN1 `5`, IN2 `6` |
| Right motor | PWM `13`, IN1 `23`, IN2 `24` |
| Left encoder | A `17`, B `27` |
| Right encoder | A `10`, B `22` |
| PIR | `25` |
| RCWL | `16` |

Encoder channel **order matters** — `RotaryEncoder(10, 22)` and `RotaryEncoder(22, 10)` count
in opposite directions.

Both GPIO nodes open `lgpio` and free their pins *before* `gpiozero` claims them, so the nodes
survive a reboot or an unclean shutdown that left pins held. **That cleanup block must stay
first in each node's `__init__`.**

### `rplidar_ros` (C++, `ament_cmake`)

Upstream SLAMTEC driver vendored into this repo. Publishes `/scan` (LaserScan). The bundled
SDK (`sdk/`) handles serial/TCP/UDP transport internally — do not modify SDK files. LIDAR
model-specific launch files live in `launch/` (A1, A2, A3, S1–S3, T1, C1).

Bringup starts the driver directly as a `Node` (not via an include) so `respawn=True` can be
set — the A1 drops off USB occasionally and needs to come back on its own.

### ROS interface summary

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `Twist` | MotorController subscribes |
| `/odom` | `Odometry` | MotorController publishes |
| `/scan` | `LaserScan` | rplidar publishes |
| `/welfare/pir`, `/welfare/rcwl` | `Bool` | WelfareSensor publishes |
| `/welfare_check` | `Empty` | WelfareSensor subscribes |
| `/welfare_result` | `String` (JSON) | WelfareSensor publishes |
| `/save_reference_scan`, `/find_person` | `Empty` | LidarDifferencing subscribes |
| `/person_location` | `PointStamped` | LidarDifferencing publishes |
| `navigate_to_pose` | `NavigateToPose` action | Nav2 server; dispatcher + patrol are clients |

### TF Frame Tree

```
map → odom → base_link → laser
 ↑      ↑          ↑
 │  MotorController │
 │                 └── static_transform_publisher (bringup)
 └── slam_toolbox (mapping mode)  OR  AMCL (navigation mode)
```

The `base_link → laser` static transform is
`['-0.045', '0.0', '0.1', '3.14159', '0.0', '0.0']` — x, y, z, yaw, pitch, roll. The LIDAR sits
4.5 cm **behind** the base_link origin, 10 cm up, and is **mounted rotated 180°**, hence the
`3.14159` yaw. `lidar_differencing.py` applies the same 180° rotation itself (`+ math.pi`) when
converting beams to Cartesian.

### Configuration files

`src/robot_driver/config/`, installed to the package share directory:

- `slam_params.yaml` — slam_toolbox (mapping mode, 0.05 m resolution, 6 m max range)
- `amcl_params.yaml` — AMCL differential motion model + map_server
- `nav2_params.yaml` — full Nav2 stack tuning
- `navigate_no_spin.xml` — custom behaviour tree with the Spin/BackUp recovery nodes removed;
  they caused a collision/replan loop on this chassis

### SLAM Map Workflow

`save_slam_map.sh` orchestrates the full capture-to-file pipeline: launches the robot, starts a
bag recording of `/map`, enables teleop keyboard control, then on Ctrl+C extracts the map as
`~/maps/home_map.pgm` + `~/maps/home_map.yaml`.

## Hardware & Serial Access

The LIDAR is addressed as **`/dev/rplidar` at 460800 baud** — a udev symlink, not the raw
`/dev/ttyUSB0`. If the port is missing or permission-denied, install the udev rules:

```bash
cd src/rplidar_ros
./scripts/create_udev_rules.sh
```

GPIO access for motors requires running as root or adding the user to the `gpio` group. The
`gpiozero` library uses the `lgpio` pin factory (`GPIOZERO_PIN_FACTORY=lgpio`), set in code
before the `gpiozero` import in each GPIO-owning node.

## Firestore integration

`firestore_dispatcher.py` requires a service-account key. Both the credential path and the
project ID are currently hardcoded at the top of that file, and `SCAN_POINT` is an **unset
placeholder that must be configured per deployment room**. `firebase_admin` is a pip
dependency and is not declared in `package.xml`.
