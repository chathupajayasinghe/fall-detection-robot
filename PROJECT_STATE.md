PROJECT STATE — Assistive Fall-Detection Mobile Robot Student: Jayasinghe W.G.C.S.N. (ET/2020/045) · Supervisor: Dr. Laalitha Liyanage Capstone: ETEC 43018, University of Kelaniya · Last updated: 2026-07-28 (update after every major session)

SYSTEM OVERVIEW — 4 SUBSYSTEMS
Wearable: ESP32 + MPU6500, Edge Impulse ML fall model + 4-phase state machine (pre-fall, free-fall, impact, post-fall inactivity) — ✅ DONE
Caregiver app: Flutter, on Pixel 7 — ✅ DONE
Backend: Firebase/Firestore, project fall-detector-app-3319d. ESP32 posts fall alerts via REST; app receives in <3–5 s (measured) — ✅ DONE
ROS2 robot (navigation + person localization + welfare check) — 🔨 IN PROGRESS — navigation working, welfare sensors validated, integration pending

Response pipeline: wearable fall → Firestore → robot dispatcher → Nav2 navigates to scene → LIDAR differencing locates person → approach → PIR/RCWL welfare check → result to Firestore/app.

1.5 WEARABLE HARDWARE DETAILS
Component	GPIO Pin	Function
MPU6500 IMU	I2C (21/22)	Motion sensing
SSD1306 OLED	I2C (21/22)	Status display
FSR Wear Sensor	GPIO34 (Analog)	Wear detection interlock
Cancel Button	GPIO13	Cancel false alarms
Blue LED	GPIO16	Emergency alert indicator
Green LED	GPIO17	Heartbeat/status indicator
Buzzer	GPIO18	Audible pre-alert
Battery Monitor	GPIO33	Voltage divider for battery %
Slide Switch	Battery positive line	Master power cutoff

Power Chain:

3.7V Li-Po → Slide Switch → TP4056 Charger → MT3608 Boost (5V) → ESP32 5V Pin
1000µF electrolytic capacitor across 5V and GND (prevents WiFi brownouts)
Battery monitor: 2× 100kΩ voltage divider on GPIO33 (before boost converter)

Physical Build:

Electronics housed on a custom soldered matrix board
Device strapped to upper arm using a sport armband (adjustable, 5.5–7 inch phone holder style with zipper, elastic, and Velcro)
Total weight: approximately 150 g

OLED Display States:

STANDBY – Device not worn (FSR interlock open)
TRACKING – Device worn and monitoring
WARNING: SENDING IN Xs – Pre-alert countdown
PUSHED – Alert successfully sent
BATT: XX% WiFi: OK – System status

Wear-Compliance Interlock:

FSR threshold: 3800 (3-second debounce)
PHASE_UNWORN pauses all fall monitoring
Automatic resume when worn again
1.6 WEARABLE ML MODEL DETAILS
Platform: Edge Impulse (deployed as C++ library)
Model type: Neural Network classifier
Input: 6-axis IMU data (accelerometer + gyroscope) from MPU6500
Sampling rate: 100 Hz (10 ms per sample)
Window size: EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE / 6 samples
Output: Binary classification (fall / no fall)
Validation accuracy: 99.7%, Validation loss: 0.01
Dataset: Self-collected fall and ADL motion data
DSP block: Spectral analysis + feature extraction (Edge Impulse built-in)
Inference runs locally on ESP32 after impact detection by the state machine
1.7 FIREBASE DATA SCHEMA

Collection: fall_alerts Document fields written by ESP32 firmware (HTTP POST):

timestamp (string): Local time display (e.g., "2026-07-17 14:30:00")
createdAt (integer): ESP32 millis() at alert generation
isoTime (string): ISO 8601 with Sri Lanka offset (+05:30)
room (string): Room name from Wi-Fi RSSI localizer
status (string): "pending" on creation. The caregiver app does NOT write this field. Only firestore_dispatcher.py overwrites it, with one of six terminal values: confirmed_movement, confirmed_no_movement, confirmed_uncertain, no_person_found, welfare_timeout, navigation_failed.
message (string): Human-readable fall description

Fields written by the robot dispatcher (firestore_dispatcher.py, _finalize_alert):

status: one of the six terminal values above
welfareCheckedAt: server timestamp, written on EVERY finalisation path including the failure paths — it marks when the robot stopped working the alert, not when a sensor check succeeded
welfarePir / welfareRcwl: booleans, written only on the paths that actually ran a check, so the app can show which sensor the verdict rested on

Fields written by the caregiver app (firestore_service.dart, acknowledgeAlert):

acknowledgedAt: server timestamp
acknowledgedBy: caregiver email or uid
The app writes NOTHING else. The separation is deliberate — see hard-won facts.

1.8 WI-FI RSSI ROOM LOCALIZER
Runs on ESP32 wearable every 3 seconds
Reads WiFi.RSSI() and maps to room names using pre-calibrated thresholds
Thresholds: ≥ -65 dBm → bedroom, -65 to -80 dBm → kitchen, < -80 dBm → bathroom
⚠️ Prototype heuristic — designed for multi-room use; demo uses single room so RSSI differentiation is limited
Room name is included in the Firestore alert and displayed on the OLED
1.9 CAREGIVER APP (FLUTTER)
Platform: Flutter, tested on Google Pixel 7 (Android)
Authentication: Firebase Auth (email/password)
Alert reception: Firestore real-time listener on fall_alerts collection
Notification: Firebase Cloud Messaging (FCM) for background/terminated alerts
Screens: Login, Home (alert list), Alert Details, Profile
Emergency screen: Full-screen red alert with sound and vibration. The Acknowledge button writes acknowledgedAt + acknowledgedBy and deliberately does NOT touch status (fixed 2026-07-28 — see hard-won facts). The screen then follows the alert document live, so the caregiver watches the robot's welfare-check verdict arrive in real time.
Status: Installed, fully functional, receives alerts in <3–5 s (measured)
ROBOT HARDWARE

Chassis: metal tank-track, 2× JGA25-370 12 V 280 RPM encoder motors Physical dimensions: width 18 cm (with tank treads), length 20 cm, height 17 cm Sensor mast: RPLIDAR C1 mounted on elevated platform at approximately 15 cm above ground Driver: TB6612FNG · Computer: Raspberry Pi 5, Ubuntu 24.04, ROS2 Jazzy Battery: 2S2P 18650 (7.4 V nominal) LIDAR: RPLIDAR C1 (borrowed; original A1 DIED — motor spins, USB enumerates CP2102, serial times out SL_RESULT_OPERATION_TIMEOUT) Welfare sensors: PIR HC-SR501 (GPIO25, PRIMARY) + RCWL-0516 (GPIO16, secondary) — mounted, node validated (see welfare section below)

GPIO map: Left motor PWM=12 / AIN1=5 / AIN2=6 · Right PWM=13 / BIN1=23 / BIN2=24 · Left encoder A=17 / B=27 · Right encoder A=10 / B=22 (right pins swapped in code) · PIR OUT=25 (pin 22) · RCWL OUT=16 (pin 36) Odometry calibration (mean error 3.0% across 5 measured distances; best single run 0.2% over 1.2 m): ticks_per_rev=235 · wheel_radius=0.027 m · wheel_separation=0.17 m — effective, NOT physical: the physical track centre-to-centre is 0.135 m, inflated by 114.5/90 to compensate tank-track scrub (0.135 × 114.5/90 ≈ 0.172, rounded to 0.17). Do not "correct" it back to 0.135.

Welfare-check sensors (PIR + RCWL) — added 2026-07-15, commit 400af8d

Hardware:

PIR HC-SR501 (HW-416-B board): PRIMARY sensor. VCC→5V pin 4, GND→pin 9, OUT→GPIO25 (pin 22). Pots set to minimum (sensitivity + time delay). MD/L-H trigger pads left at factory (H, repeat trigger — fine for windowed sampling). 60 s warm-up after power-on — output untrustworthy before that. Angled slightly downward (target = person on floor at ~0.8 m).
RCWL-0516: SECONDARY only. VIN→5V pin 2, GND→pin 6, OUT→GPIO16 (pin 36). CDS/3V3 unconnected. Logo/smooth side = antenna, faces the person; 2–3 cm clear of metal.
⚠️ RCWL hard-won fact: all 3 modules capped at <1 m detection range (spec: 5–7 m) regardless of power source (Pi 5V or separate buck), grounding, orientation, or standoff — suspected clone batch. Hence PIR-primary architecture. Range still adequate for the 0.8 m welfare-check geometry, so kept as secondary channel.
Both mounted on nonmetal standoff at robot front, few cm apart.

Software (welfare_sensor.py, in robot_bringup.launch.py — bringup owns hardware):

Publishes /welfare/pir and /welfare/rcwl (Bool, 5 Hz) continuously.
Trigger: /welfare_check (Empty) → 2 s settle → 10 s sample (OR-latched) → /welfare_result (String, JSON): {"pir": bool, "rcwl": bool, "verdict": ...}.
Verdict (PIR-primary): PIR true → "movement" · both false → "no_movement" · PIR false + RCWL true → "uncertain" (RCWL not trusted alone) · within 60 s of node start → "warming_up".
Non-blocking timer state machine (no sleeps in callbacks).

Validation (on-robot, LIDAR spinning, motors idle):

All 4 verdict combinations verified, including no self-triggering of PIR from robot heat/vibration/LIDAR (empty room → "no_movement" ✅).

Dispatcher integration contract: after approach, publish /welfare_check, await /welfare_result, relay JSON verdict to Firestore.

2.5 ROBOT DISPATCHER (firestore_dispatcher.py)
Listens to Firestore fall_alerts collection in real-time (on_snapshot)
Detects new documents with status = "pending"
Navigates to a single SCAN_POINT dict constant. There is NO ROOM_COORDINATES dictionary — it was removed. The room field is still read off the alert and used for logging/display, but it does NOT route navigation.
Sends the goal via an rclpy ActionClient on the navigate_to_pose action — NOT nav2_simple_commander / BasicNavigator.goToPose()
Monitors navigation progress; succeeds only on STATUS_SUCCEEDED; counts failures per alert and gives up after 3 (MAX_NAV_FAILURES), writing navigation_failed
Integration: after arrival, triggers LIDAR differencing → approach 0.8 m short of the person → welfare check → reports result to Firestore

SCAN_POINT = {x: 1.5, y: 0.0} (firestore_dispatcher.py line 51) — a surveyed home-map coordinate that reached 10/10 in the straight-path navigation campaign.
⚠️ Environment-specific. These coordinates are only meaningful against the map they were surveyed on. Running against any other map — including the faculty demo room — sends the robot to an arbitrary point and the person search then runs from the wrong place. MUST be re-surveyed before any faculty-room demo.
⚠️ Coupled to lidar_differencing: the reference scan is compared to the live scan beam-by-beam BY INDEX, so it MUST be captured from this same pose. Re-surveying SCAN_POINT means re-capturing the reference scan.

For the single-room demonstration, the room field from the wearable is not used for navigation routing — all alerts navigate to SCAN_POINT regardless of the RSSI room name.

CRITICAL HARD-WON FACTS (do not rediscover these)
C1 LIDAR: baud 460800 (A1 was 115200). udev rule matches idVendor 10c4 / idProduct ea60 → /dev/rplidar. Crashes if scan_mode: 'Standard' is passed — scan_mode must be omitted entirely. Launch params must be correct types: serial_baudrate as int 460800, inverted/angle_compensate as booleans.
C1 mounted 180° rotated vs old A1 → fixed with yaw=3.14159 in base_link→laser static transform (in robot_bringup.launch.py). Any remount = re-check this.
Launch-file separation (the big bug, fixed): nav2_navigation.launch.py used to include robot_bringup → duplicate rplidar/motor nodes fought over the serial port, wedging the LIDAR driver mid-session (silent /scan death). Now: bringup owns ALL hardware; nav2_navigation launches ONLY the Nav2 stack. Always launch bringup FIRST. rplidar node has respawn=True, respawn_delay=2.0.
Motor deadband: PWM = MIN_PWM(0.35) + abs(speed)×(1−MIN_PWM) so Nav2's gentle velocities move the motors. MIN_PWM was calibrated on a fresh pack — degrades as voltage sags.
STRAIGHT_KP = 0.0 (straight-line PID disabled after a sign-flip bug pivoted the robot; a clamp now prevents corrections flipping wheel direction — but with KP=0 the whole correction block is currently inert). RIGHT_TRIM = 0.97 open-loop trim on the right motor is the only active straightness compensation; slight rightward drift in teleop is expected and Nav2 compensates. Pure in-place rotation gets a ×1.5 speed multiplier (angular_z≠0, linear_x=0) to overcome track friction — this is why slow pivots work. Re-enabling small KP with the clamp = possible future experiment; re-test the post-turn transition logic (was_turning guard) if you do.
Encoder warm-up / measurement artifact: encoders sometimes under-count on the FIRST drive after a power cycle. Two consecutive measured metres confirmed ~1360 ticks/m consistently. The alarming "3× error" was NOT real — it was a measurement artifact from comparing cumulative running totals against single-drive deltas. RULE: always measure with two consecutive deltas, never cumulative totals. This is what wheel_radius=0.027 rests on; an earlier Nav2-goal-based measurement suggested 0.055 and was wrong (contaminated by path curvature, goal tolerance and recovery behaviours).
Battery rule: <7.2 V = end of session. At ~7.1 V Nav2-speed motion becomes unreliable (creep → progress-checker 106 aborts) even when teleop still works. At 6.4 V the Pi browns out. Both "mystery" late-session failure cascades were battery. Consider a low-voltage buzzer on the balance connector.
Yaw goal tolerance = 3.14 (was 0.25): tank tracks cannot do precise slow pivots near obstacles — robot reaches positions but failed final-heading spins (repeated error 106). Now any arrival heading counts. Consequence: arrival heading is arbitrary — lidar_differencing must NOT assume person is in front (C1 sees 360°). CONFIRMED at 3.14 — an experiment at 0.5 did NOT help (the 106 aborts came back) and was reverted 2026-07-28. Do not retry tightening this; the chassis is the limit, not the tuning.
movement_time_allowance = 15.0 (was 10.0). rotate_to_heading_angular_vel=1.0, use_rotate_to_heading=true (already good).
2D LIDAR blind spots (document in thesis, manage in demos): floor-level obstacles (wires!), thin chair/desk legs, and transparent glass (bottle incident). Demo rule: clear floor.
Map–reality match is the #1 localization factor. Door state and furniture positions at map time must match run time. Door closed always. Robot once escaped through an open door while AMCL believed it was in-room (goal "SUCCEEDED" while physically in the living room).
AMCL procedures: startup = place robot at taped origin spot + publish /initialpose (0,0,0). Lost = ros2 service call /reinitialize_global_localization std_srvs/srv/Empty + slow driving (arcs, near distinctive features) until /amcl_pose covariance x,y < ~0.05–0.08. Verify with covariance numbers, not RViz. AMCL needs /scan before it publishes map→odom; /amcl_pose only publishes on motion updates (silence ≠ broken — check tf2_echo map odom).
RViz on laptop WSL2 freezes — root cause is the hotspot network (Message Filter queue-full drops, seconds-stale data), not graphics. export LIBGL_ALWAYS_SOFTWARE=1, saved ~/nav.rviz (Map Durability=Transient Local, LaserScan Reliability=Best Effort). Prefer terminal verification (covariance, tf2_echo) over RViz. Real fix: a router in the demo room.
Map saving: map_saver_cli needs -p map_subscribe_transient_local:=true or it fails with "Failed to spin map subscription".
Nav2 error codes seen: 106 = failed to make progress (usually battery, tight maneuver, or lost localization); 208 = planner can't reach goal (goal in occupied/inflated/unknown space).
Waypoint rules: ≥0.6 m from walls/obstacles (footprint + 0.25 inflation + ~10 cm AMCL error); approach furniture from open floor; every waypoint must be validated by an actual navigation run before trusting it.
Duplicate /base_link_to_laser existed (both launch files) — transforms were identical so harmless, but should be verified removed after the launch-file split.
Git pull can fail silently on the hotspot (Could not resolve host) while the launch continues with old code — after every pull, verify with git log --oneline -1 that HEAD matches the expected commit.
RCWL-0516 clone modules: see welfare sensor section — <1 m real range across 3 units, all power/grounding/orientation variations tested. PIR-primary architecture adopted as mitigation.
Wearable Li-Po below 3.0V → TP4056 refuses to charge. Use basic USB cable (not PD/smart cable). 950mAh charges in ~2 hrs.
CONFIRM_WINDOWS=2 failed with pole demo (second ML inference fails when pole bounces on mattress). Reduced to 1.
Firmware Firestore alert writes status = "pending" (was "detected") — changed 2026-07-28 in sketch_apr18a.ino line 253. ✅ RESOLVED, pipeline now matches end to end: firestore_dispatcher.py queries status == "pending" (line 144, re-checked at 153 and 226) and the Flutter app renders "pending" as an active alert (alert_status.dart lines 179-180 and 193). ⚠️ The deployed ESP32 keeps posting the old "detected" value until it is REFLASHED — reflash before any end-to-end drill. The app still handles both values, so an un-reflashed device still alarms in the app; it just never reaches the robot.
App acknowledge must never write status (fixed 2026-07-28): earlier builds overwrote status with "Attended" when the caregiver tapped Acknowledge. Because the dispatcher only picks up documents where status == "pending", a caregiver acknowledging BEFORE the robot picked up the alert silently cancelled the whole robot mission. Acknowledgement and robot verdict are independent facts and now live in independent fields (acknowledgedAt / acknowledgedBy). Old documents still carry the legacy "Attended" value and the app still renders it in the history list.
WORKFLOW & ENVIRONMENT

Code editing: Windows laptop, Claude Code at D:\Raspberry → git push Repo: github.com/chathupajayasinghe/fall-detection-robot (main) On Pi: cd ~/fall-detection-robot && git pull && colcon build --packages-select robot_driver && source install/setup.bash — then verify HEAD with git log --oneline -1 Pi login: et2020045 @ Robot.local (password = username — CHANGE before campus network). SSH over mobile hotspot: unstable, IP changes; use tmux (tmux new -s robot, reattach with tmux attach -t robot) so dropouts don't kill launches. VS Code remote terminals work; Ctrl+B tmux prefix may be intercepted — use multiple VS Code terminals instead. A Claude Code instance also runs ON the Pi (used sshpass; did the launch-file diagnosis).

Firmware repo (NEW, 2026-07-28): github.com/chathupajayasinghe/fall-detection-wearable-firmware (PRIVATE, branch main). First backup of the wearable firmware — it was previously unversioned, single copy on the laptop. Local working copy: D:\Projects\sketch_apr18a\. Credentials are NOT tracked: SECRET_SSID / SECRET_PASS / SECRET_API_KEY live in arduino_secrets.h, which is gitignored and was never committed. A fresh clone MUST copy arduino_secrets.h.example → arduino_secrets.h and fill in the real values, or the build fails on the missing include. Older sketches at D:\Project\falldetect\ and D:\Project\sketch_feb18a\ are still unversioned and still contain credentials inline.

App repo: github.com/chathupajayasinghe/fall-detector-app (PRIVATE, branch main). Local working copy: D:\StudioProjects\fall_detector_app\. Contains the caregiver app including the acknowledge fix (§1.9) and the robot welfare-check monitoring view — the emergency screen subscribes to the alert document so the caregiver follows the robot's progress through to its verdict. Firebase configuration is deliberately untracked (google-services.json, firebase_options.dart) — a fresh clone needs those restored before it will build.

Standard session startup:

Robot on taped origin spot, door closed, battery FULL (measure it)
ros2 launch robot_driver robot_bringup.launch.py (add start_slam_toolbox:=true only when mapping) — expect FOUR processes incl. welfare_sensor
Verify: ros2 topic hz /scan ≈ 10 Hz (if silent: unplug/replug C1 USB, check ls -l /dev/rplidar, relaunch)
ros2 launch robot_driver nav2_navigation.launch.py map:=$HOME/maps/<map>.yaml → wait for "Managed nodes are active"
Publish /initialpose → drive/verify covariance → work

Key files: src/robot_driver/robot_driver/motor_controller.py (motors+odometry+cmd_vel watchdog) · welfare_sensor.py (PIR+RCWL, /welfare_check → /welfare_result) · lidar_differencing.py (person finder: empty-room reference vs live scan → /person_location, triggered by /find_person) · firestore_dispatcher.py (Firestore alert → nav goal; SCAN_POINT) · launch/robot_bringup.launch.py · launch/nav2_navigation.launch.py · config/nav2_params.yaml

MAPS & WAYPOINTS

Home-room map: ~/maps/home_map_c1 (development only) Faculty room (DEMO room, 5.8 × 7.9 m): map at ~/maps/faculty_map.yaml (saved 2026-07-09). Must be committed to repo (maps/ folder) — may still exist only on the SD card. Faculty-room waypoints (from /amcl_pose; recorded in chat, STILL NOT COMMITTED and STILL NOT VALIDATED — first task):

home/origin: (−0.13, 0.06)
room_center: (2.31, −0.01) ← likely LIDAR-differencing scan point
ceiling_fan: (1.56, −1.07)
far_left_corner: (6.91, 1.56) — corners were problematic; re-validate
far_right_corner: (7.00, −2.63) — re-validate Orientation for all: z=0, w=1 (heading irrelevant with π yaw tolerance)

⚠️ None of these are yet loaded into firestore_dispatcher.py — it currently drives to the home-map SCAN_POINT {1.5, 0.0}. Re-surveying for the faculty room means updating SCAN_POINT AND re-capturing the lidar_differencing reference scan from the new pose.

CAPABILITIES DEMONSTRATED (qualitative — for thesis narrative context)

Navigation: autonomous point-to-point in both home room and faculty demo room; obstacle avoidance confirmed (paused at unseen bottle, resumed on removal); failure modes understood (208 = bad goal placement; 106 = tight maneuver / low battery / lost localization). LIDAR differencing: person-finder validated for direction/sign correctness (front/left/right) after the 180° fix (commit 4993f89); camera-free, uses empty-room reference vs live scan. Welfare check: dual-sensor PIR-primary + RCWL node validated on-robot; all verdict combinations correct; no PIR self-triggering from robot heat/vibration/LIDAR. Wearable: full 4-phase detection pipeline (freefall→impact→ML→stillness), FSR wear interlock, 10 s cancellable pre-alert, Firebase alert, OLED status. (Formal quantified test results live in the thesis draft, not here.)

Navigation trial results (home room): 20/20 trials succeeded, across TWO pathway types — straight-path and diagonal-path. Obstacle-avoidance and multi-waypoint pathway types are PENDING, not yet run. Against the proposal's committed obligation of 4 pathway types × ≥10 trials, 2 of the 4 types are complete. All 20 were run in the home room; the faculty demo room has not been trialled at this count.

6.5 FALL DEMONSTRATION PROCEDURE

Equipment:

Wearable attached to demonstration pole (1.2–1.5 m height)
Mattress or cushioned landing surface
Laptop running UDP listener (port 4444) for real-time logs
Mobile phone with caregiver app

Demo Steps:

Show OLED displaying "TRACKING" (FSR pressed, device armed)
Release pole — wearable falls freely onto mattress
Panel observes real-time UDP logs: Freefall → Impact → ML Inference (confidence ≥0.85) → Stillness → Pre-Alert countdown
OLED shows 10-second countdown with buzzer + blue LED
If not cancelled, Firestore alert sent
Caregiver app receives notification within 3–5 seconds

Key Points to Emphasize:

99.7% Edge Impulse model validation accuracy (live detection accuracy reported separately in results)
4-phase verification prevents false alarms
10-second user cancellation window
IMU-based detection at the wearable (no cameras anywhere in the system)
PROPOSAL / ACADEMIC STATE

Original signed proposal: fixed-sensor architecture (obsolete). Remade proposal (supervisor-requested) reflects real system: wearable detects, robot responds, single-room scope, dual welfare sensors robot-mounted, C1/TB6612FNG/Pi5/ROS2 Jazzy/Firebase. Examiner comments addressed: methodology added; robot added to MVP; targets: fall detection ≥90% (30 falls + 30 ADLs), alert <60 s, localization direction ≥80%; navigation spec: 4 pathway types × ≥10 trials, expected success 80–90%. NOTE: proposal wording should be updated to "dual-sensor welfare check (PIR primary + RCWL-0516)" before signing — verify deliverable 2, methodology Stage 4, equipment table, BOM (+~400 LKR PIR). Removed/parked promises: patrolling, whole-home coverage, fixed sensors, voice wear-reminders (future work), physical support handle (dead — verify it appears nowhere). ⚠️ Committed test obligations: the 30+30 fall test, 4×10 navigation trials (2 of 4 types now done, 20/20), localization tests — these MUST be run and reported. Integration and formal test campaigns in progress; see thesis draft for results.

9. SECURITY NOTES

Firebase API key and Wi-Fi credentials are NO LONGER hardcoded in the tracked firmware source (changed 2026-07-28). They are extracted to arduino_secrets.h, which is gitignored and was never committed — staged content was scanned for the literal values before the initial commit and was clean, so no live credential exists anywhere in the firmware repo history.

The values themselves are still live and still present locally in D:\Projects\sketch_apr18a\arduino_secrets.h:

SECRET_SSID / SECRET_PASS — the demo hotspot ("C_Pixel")
SECRET_API_KEY — Firebase Web API key for project fall-detector-app-3319d

REDACT before publishing thesis or public GitHub. If a firmware listing goes in the appendix, print arduino_secrets.h.example (placeholders), NOT arduino_secrets.h.
Older sketches at D:\Project\falldetect\ and D:\Project\sketch_feb18a\ still contain both credentials inline and are unversioned — clean or delete them before archiving the project.
Both GitHub repos (firmware, app) are PRIVATE. Rotate the Firebase key if either is ever made public or shared beyond the supervisor.
Pi login: et2020045 @ Robot.local, password = username — STILL UNCHANGED, change before joining the campus network.
