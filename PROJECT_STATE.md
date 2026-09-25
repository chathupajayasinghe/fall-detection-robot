PROJECT STATE — Assistive Fall-Detection Mobile Robot Student: Jayasinghe W.G.C.S.N. (ET/2020/045) · Supervisor: Dr. Laalitha Liyanage Capstone: ETEC 43018, University of Kelaniya · Last updated: 2026-09-25 (update after every major session)

SYSTEM OVERVIEW — 4 SUBSYSTEMS
Wearable: ESP32 + MPU6500, Edge Impulse ML fall model + 4-phase state machine (pre-fall, free-fall, impact, post-fall inactivity) — ✅ DONE
Caregiver app: Flutter, on Pixel 7 — ✅ DONE
Backend: Firebase/Firestore, project fall-detector-app-3319d. ESP32 posts fall alerts via REST; app receives in <3–5 s (measured) — ✅ DONE
ROS2 robot (navigation + person localization + welfare check) — 🔨 IN PROGRESS — navigation working in the faculty room on faculty_map_v2, person finder working in the empty faculty room (3/3 clean), welfare sensors validated, end-to-end integration pending (see SESSION 2026-09-25)

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
1.6a FALL-MODEL RETRAINING DATA COLLECTION (sessions 2026-08-17, 2026-08-30 and 2026-09-24)
Captured with tools\capture_windows.py over the UDP logger (DATA_COLLECTION_MODE true in SensorManager.cpp), written to D:\Projects\sketch_apr18a\data\ — which is GITIGNORED, so it exists in exactly one place. Backed up 2026-08-17 to D:\Projects\sketch_apr18a_data_backup_20260817\ (53 files, every file SHA256-verified against the source).
Dataset: 51 windows captured and indexed in data\index.csv. LABELLING COMPLETE — 48 FALL / 3 NOT_FALL, no blank rows; every index row matched to a file on disk by filename and back. The three NOT_FALL are window_011_conf_1.00_20260817_155839.csv (picking up the pole), window_003_conf_0.00_20260817_161152.csv and window_015_conf_0.75_20260817_161510.csv. Everything else is a pole drop and is labelled FALL, INCLUDING the low-confidence rows — those are real drops the model missed and are the most valuable rows in the set. Separate negatives batch still pending, target 40+.
⚠️ CONFIRMED FALSE POSITIVE of the deployed model: PICKING UP THE POLE scores 0.9961, far past FALL_CONFIDENCE (0.85, SensorManager.cpp:48), so the shipped build would raise a full alert on someone simply lifting the demo rig (window_011_conf_1.00_20260817_155839.csv, labelled NOT_FALL). This is the highest-value negative class for retraining: not a marginal case but a confident wrong answer, and trivially reproducible on demand in front of a panel.
The model's misses are NOT near-misses. Failed pole drops score 0.00–0.13, not merely below the 0.85 cutoff. The shipped model was trained on human falls onto a mattress; a weighted pole is a different distribution, so the failures are distributional rather than threshold tuning. CONSEQUENCE: do not try to fix pole-drop detection by lowering FALL_CONFIDENCE — there is nothing between 0.13 and 0.85 for a lower threshold to recover, and lowering it only harvests false positives like the pole-pickup case above.
⚠️ The wearable REBOOTS UNDER LOAD when the battery sags. The ESP32 restarted repeatedly across the ~40 minute capture session (operator counted four during the session; index.csv evidences two, via window-ID resets — a reboot that captures no window before the next reboot leaves no trace, so the log-side count is a floor, not a total). UDP window discard rate climbed to ~24% as the battery drained, against ~10% at full charge. RULE: charge the wearable to full immediately before any capture or demo session.
⚠️ Capture bookkeeping: window IDs restart at 1 on EVERY reboot (dcWindowId is monotonic per boot only, SensorManager.cpp:120), so data\index.csv contains repeated window_id values BY DESIGN. A window must be identified by its full filename, which carries a timestamp — never by window_id alone. Any script that keys on window_id will silently merge windows from different boots.
SESSION 2026-08-30 — NEGATIVES BATCH. 80 windows captured the same way (tools\capture_windows.py, DATA_COLLECTION_MODE true) into D:\Projects\sketch_apr18a\data\, and ALL 80 are labelled NOT_FALL. The whole session was deliberate non-fall motion — pole pickups, cane strikes, waves, and knocks against furniture — and no fall was performed at any point, so the label follows from the session itself rather than from a per-window judgement. Rows were matched to files by filename, never by window_id (see the bookkeeping rule above). This closes the "separate negatives batch still pending, target 40+" item at double the target.
⚠️ 37 of those 80 negatives score >= 0.85 — i.e. 37 CURRENT-MODEL FALSE POSITIVES on vigorous non-fall motion. The single pole-pickup false positive recorded above was not a one-off: close to half of ordinary vigorous handling of the rig clears FALL_CONFIDENCE (0.85) and would raise a full alert. This is the strongest available argument for retraining and the most valuable part of the negatives set.
Dataset after this session stood at 48 FALL / 83 NOT_FALL (131 windows) in data\index.csv.
Backup refreshed 2026-09-24 to D:\Projects\sketch_apr18a_data_backup_20260817\ — it now mirrors data\ in full, 163 files with data\.claude\ excluded, every file SHA256-verified against the source. The directory NAME still carries the original 2026-08-17 date; its CONTENTS are current.
⚠️ EDGE IMPULSE LABEL MAPPING: the Edge Impulse project's two classes are "fall" and "normal", NOT the FALL / NOT_FALL spelling used in data\index.csv. On upload, FALL maps to fall and NOT_FALL maps to normal. Uploading the index spelling verbatim would create two spurious extra classes and silently yield a 4-class model.
DELIVERED 2026-09-24 (was PENDING): the extra FALL windows on the FINAL DEMO RIG — the box attached to the pole, dropped onto a thin mattress — were captured; 30 of them, see the session block below. The 48 FALL windows above were BARE-POLE drops, a different mass distribution and a different impact signature from what the demo actually shows.
SESSION 2026-09-24 — FALLS ON THE FINAL DEMO RIG. 30 real falls captured with the rig the demo will actually use: the box attached to the pole, dropped onto a thin mattress. All 30 are labelled FALL, matched to files by filename and never by window_id.
⚠️ KEY RESULT — 29 of the 30 score >= 0.85 against the ORIGINAL, UNCHANGED model: 96.7% detection, with one miss at 0.54 (window_015_conf_0.54_20260924_175915.csv, a real fall the model scored low). This overturns the working assumption carried since 2026-08-17 that the shipped model cannot see these falls.
WHY the original model succeeds here having failed on the 2026-08-17 drops: the box on top makes the rig TOPPLE LIKE A BODY — it rotates about its base and lands with a body-like mass distribution, which is close to the human-fall data the model was trained on. The 2026-08-17 captures were BARE-POLE drops, which fall straight and strike with a concentrated impulse the model had never seen. The earlier 0.00–0.13 misses were therefore an artefact of the TEST RIG, not a deficiency of the model on the motion actually being demonstrated.
CONSEQUENCE: RETRAINING IS DEFERRED — now OPTIONAL rather than required. The original model detects the demo fall at 96.7%, so the retraining path (Edge Impulse upload, the label mapping above, reflash) is a nice-to-have and not a blocker for the demo.
Dataset now stands at 78 FALL / 83 NOT_FALL (161 windows) in data\index.csv: 48 bare-pole FALL and 3 NOT_FALL from 2026-08-17, 80 NOT_FALL from 2026-08-30, 30 final-rig FALL from 2026-09-24.
STILL OPEN: the 37 false positives from the 2026-08-30 negatives batch are untouched by this result. The original model's weakness is raising alerts on vigorous non-fall handling, not missing the demo fall — that, not detection, is what would justify retraining if it is ever picked up.
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
⚠️ STILL THE HOME-MAP VALUE as of 2026-09-25. The faculty-room scan point on faculty_map_v2 is VALIDATED at (2.31, −0.01) — several SUCCEEDED runs from the taped origin, stopping ~11–12 cm short, inside the 0.15 m goal tolerance. firestore_dispatcher.py must be changed to {x: 2.31, y: -0.01} before the end-to-end drill.
⚠️ Environment-specific. These coordinates are only meaningful against the map they were surveyed on. Running against any other map — including the faculty demo room — sends the robot to an arbitrary point and the person search then runs from the wrong place. MUST be re-surveyed before any faculty-room demo.
⚠️ Coupled to lidar_differencing: the reference scan is compared to the live scan by beam index, so it MUST be captured from this same pose. Re-surveying SCAN_POINT means re-capturing the reference scan. Since b86757c (2026-09-25) each live beam is compared against the MINIMUM of the reference within ±3° (REFERENCE_WINDOW_DEG = 3.0), which absorbs small arrival-heading differences — see SESSION 2026-09-25.

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
movement_time_allowance = 15.0 (was 10.0). rotate_to_heading_angular_vel=1.0. use_rotate_to_heading was actually FALSE in nav2_params.yaml (this line previously said "true (already good)" — that was wrong). With it false, a goal BEHIND the robot made RPP drive straight forward into the chair legs → error 208. Set to true in 5b035e7 (2026-09-25). ⚠️ So far tested only as a live parameter change; after the next FRESH Nav2 restart confirm with: ros2 param get /controller_server FollowPath.use_rotate_to_heading → True.
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
Wearable battery sag is a session-killer, not just a runtime limit: as the pack drains the ESP32 reboots under WiFi/inference load and UDP capture loss roughly doubles (~10% → ~24%). A mid-session reboot also resets the capture window-ID counter to 1. Charge to full before any capture or demo run — see §1.6a.
CONFIRM_WINDOWS=2 failed with the pole demo and was reduced to 1 — but the recorded reason ("second ML inference fails when the pole bounces on the mattress", implying a bounce produces a second window) was WRONG. Corrected 2026-08-17 from the 2026-08-12 capture log. runMLInference() calls sampleBurst(200 samples, 10 ms) which BLOCKS for a full 2.0 s, while a mattress rebound is over in 100–300 ms. So a bounce lands INSIDE the first window, never in a second one: 10 of the 29 reconstructed windows carry a second, lower peak at t=0.23–0.43 s, and two consecutive inferences are necessarily ≥2.0 s apart. The actual failure was a CONFIRMATION failure: with CONFIRM_WINDOWS=2 a positive inference immediately ran a second 2 s burst, which sampled an already-settled pole (every window's last 500 ms sits at ~1 g, stillness 56–100%), scored below FALL_CONFIDENCE, reset _consecutiveFallCount and fell back to PHASE_NORMAL — so the alert never fired. Consequence for any future retry: serial re-sampling can never confirm a transient impact, because the event is over before burst #2 begins. Multi-window confirmation would need the burst re-run against buffered history, not fresh samples. Corollary for capture bookkeeping: a bounce does NOT inflate the window count, so a window-count/drop-count mismatch must be explained some other way.
Firmware Firestore alert writes status = "pending" (was "detected") — changed 2026-07-28 in sketch_apr18a.ino line 253. ✅ RESOLVED, pipeline now matches end to end: firestore_dispatcher.py queries status == "pending" (line 144, re-checked at 153 and 226) and the Flutter app renders "pending" as an active alert (alert_status.dart lines 179-180 and 193). ⚠️ The deployed ESP32 keeps posting the old "detected" value until it is REFLASHED — reflash before any end-to-end drill. The app still handles both values, so an un-reflashed device still alarms in the app; it just never reaches the robot.
App acknowledge must never write status (fixed 2026-07-28): earlier builds overwrote status with "Attended" when the caregiver tapped Acknowledge. Because the dispatcher only picks up documents where status == "pending", a caregiver acknowledging BEFORE the robot picked up the alert silently cancelled the whole robot mission. Acknowledgement and robot verdict are independent facts and now live in independent fields (acknowledgedAt / acknowledgedBy). Old documents still carry the legacy "Attended" value and the app still renders it in the history list.
WORKFLOW & ENVIRONMENT

Code editing: Windows laptop, Claude Code at D:\Raspberry → git push Repo: github.com/chathupajayasinghe/fall-detection-robot (main) On Pi: cd ~/fall-detection-robot && git pull && colcon build --packages-select robot_driver && source install/setup.bash — then verify HEAD with git log --oneline -1 Pi login: et2020045 @ Robot.local (password = username — CHANGE before campus network). SSH over mobile hotspot: unstable, IP changes; use tmux (tmux new -s robot, reattach with tmux attach -t robot) so dropouts don't kill launches. VS Code remote terminals work; Ctrl+B tmux prefix may be intercepted — use multiple VS Code terminals instead. A Claude Code instance also runs ON the Pi (used sshpass; did the launch-file diagnosis).

Firmware repo (NEW, 2026-07-28): github.com/chathupajayasinghe/fall-detection-wearable-firmware (PRIVATE, branch main). First backup of the wearable firmware — it was previously unversioned, single copy on the laptop. Local working copy: D:\Projects\sketch_apr18a\. Credentials are NOT tracked: SECRET_SSID / SECRET_PASS / SECRET_API_KEY live in arduino_secrets.h, which is gitignored and was never committed. A fresh clone MUST copy arduino_secrets.h.example → arduino_secrets.h and fill in the real values, or the build fails on the missing include. Older sketches at D:\Project\falldetect\ and D:\Project\sketch_feb18a\ are still unversioned and still contain credentials inline.

⚠️ Firmware partition scheme (2026-08-09) — NOT IN THE REPO, lives only in Arduino IDE preferences. The firmware now REQUIRES Tools → Partition Scheme → "Huge APP (3MB No OTA/1MB SPIFFS)". The IDE default, "4MB with spiffs (1.2MB APP)", was already at 93% of the app partition BEFORE the data-collection code was added, so it now overflows and the build fails. Same failure class as arduino_secrets.h above: the setting is IDE state, not source, so it is invisible to git and travels with neither the repo nor a clone. If the IDE resets its preferences, or the sketch is opened on another machine or in a fresh workspace, it silently reverts to the default and the build fails with a partition-overflow error that names no cause anyone would connect to a menu setting — check this FIRST when a previously-building sketch stops fitting. With Huge APP selected the sketch sits at 39%, so there is ample headroom for the retrained model.

App repo: github.com/chathupajayasinghe/fall-detector-app (PRIVATE, branch main). Local working copy: D:\StudioProjects\fall_detector_app\. Contains the caregiver app including the acknowledge fix (§1.9) and the robot welfare-check monitoring view — the emergency screen subscribes to the alert document so the caregiver follows the robot's progress through to its verdict. Firebase configuration is deliberately untracked (google-services.json, firebase_options.dart) — a fresh clone needs those restored before it will build.

Standard session startup:

Robot on taped origin spot, door closed, battery FULL (measure it)
ros2 launch robot_driver robot_bringup.launch.py (add start_slam_toolbox:=true only when mapping) — expect FOUR processes incl. welfare_sensor
Verify: ros2 topic hz /scan ≈ 10 Hz (if silent: unplug/replug C1 USB, check ls -l /dev/rplidar, relaunch)
ros2 launch robot_driver nav2_navigation.launch.py map:=$HOME/maps/<map>.yaml → wait for "Managed nodes are active"
Publish /initialpose → drive/verify covariance → work
Person finder: lidar_differencing is NOT started by robot_bringup.launch.py — run it by hand in its own terminal: ros2 run robot_driver lidar_differencing (decision pending on adding it to bringup)

Key files: src/robot_driver/robot_driver/motor_controller.py (motors+odometry+cmd_vel watchdog) · welfare_sensor.py (PIR+RCWL, /welfare_check → /welfare_result) · lidar_differencing.py (person finder: empty-room reference vs live scan → /person_location, triggered by /find_person) · firestore_dispatcher.py (Firestore alert → nav goal; SCAN_POINT) · launch/robot_bringup.launch.py · launch/nav2_navigation.launch.py · config/nav2_params.yaml

MAPS & WAYPOINTS

Home-room map: ~/maps/home_map_c1 (development only).
Faculty room (DEMO room, 5.8 × 7.9 m): CURRENT MAP = faculty_map_v2 (remapped 2026-09-25). On the Pi at ~/maps/faculty_map_v2.yaml/.pgm; committed to the repo as maps/faculty_map_v2.* in 9bbc128. 161 × 123 cells @ 0.05 m, origin [-0.716, -3.837, 0], yaml image path is relative (faculty_map_v2.pgm). Launch: ros2 launch robot_driver nav2_navigation.launch.py map:=$HOME/maps/faculty_map_v2.yaml
The old ~/maps/faculty_map.yaml (2026-07-09) is SUPERSEDED — do not use it. It was never committed.
Initial pose: taped origin (0, 0), facing +x.

Room layout (faculty_map_v2): right half of the room has rows of table/chair legs; left and centre are open. The top wall is too close on the left side. Planned fall zone: ~1.5 m to the robot's right of the scan point, around (2.3, −1.5).

Faculty-room waypoints on faculty_map_v2:
scan point: (2.31, −0.01) — ✅ VALIDATED 2026-09-25 (several SUCCEEDED runs from origin; arrives ~11–12 cm short). NOT yet in firestore_dispatcher.py.
The other waypoints recorded 2026-07-09 (ceiling_fan (1.56, −1.07), far_left_corner (6.91, 1.56), far_right_corner (7.00, −2.63)) were measured on the OLD faculty_map and are NOT valid on v2 — re-survey if needed.

SESSION 2026-09-25 — FACULTY ROOM INTEGRATION (robot)
Map: room remapped → faculty_map_v2 (see MAPS & WAYPOINTS). Committed 9bbc128.
Navigation: scan point (2.31, −0.01) validated, several SUCCEEDED runs origin ↔ scan point, stopping ~11–12 cm short.
Bug fixed: use_rotate_to_heading was false → a goal behind the robot made RPP drive straight forward into chair legs (error 208). Fixed in 5b035e7 (see hard-won facts; still to confirm after a fresh Nav2 restart).
lidar_differencing is NOT in the bringup launch — it must be started by hand (ros2 run robot_driver lidar_differencing). /save_reference_scan and /find_person have no subscriber otherwise. Decision pending on adding it to robot_bringup.launch.py.
Reference scan: re-captured at the scan point with the room clear. 360 beams (1 beam = 1°), ~/maps/reference_scan.npy on the Pi, saved 12:57. Capture pose from a fresh AMCL update (ros2 service call /request_nomotion_update std_srvs/srv/Empty, then /amcl_pose; tf2_echo map base_link agreed): (2.197, 0.006), heading −10.8°. The old home-map reference (26 Jul) is kept on the Pi as ~/maps/reference_scan_home_20260726.npy. Lesson: ros2 topic echo --once /amcl_pose returns the LAST published pose, which can be minutes old if the robot has not moved — check header.stamp, or force a nomotion update first.
⚠️ Door ghost (empty room, before the fix): the door recess showed up as a false "person" (cluster 11–12 beams) because the arrival heading differs from the reference heading and the reference is compared by beam index. One recorded case: arrival heading +8.2° at (2.22, −0.11) vs reference −10.8° — a ~19° difference — gave a false cluster of 11 at base_link (−1.44, −2.94).
Fix: b86757c — each live beam is compared against the minimum of the reference within ±3° (REFERENCE_WINDOW_DEG = 3.0). Empty-room test after the fix: 3/3 clean (no false person).
⚠️ Open risk: ±3° is smaller than the ~19° heading difference seen in the case above. Record the arrival heading of every /find_person run; if a false person appears again, compare the arrival heading with −10.8° first.
Box test (fallen-person box, BEFORE the b86757c fix): wide face toward the robot → detected at base_link (0.34, −1.66), cluster 12. Narrow 20 cm face toward the robot → MISSED: ~7 beams, below MIN_CLUSTER_POINTS = 8, and span below MIN_PERSON_SIZE = 0.3 m. Size limits not changed yet — a proposal must accept the box's narrow face but reject examiners' legs (10–15 cm). Box test to be repeated after the fix.
⚠️ OPEN — navigation failure: the last return trip scan point → origin failed with repeated "collision ahead" and error 106, although earlier runs on the same route succeeded. Cause NOT found. The battery was low after ~3 h of running and was put on charge. Per the battery rule (<7.2 V = stop) and the 106 notes above, re-test on a full battery before looking for another cause; check localization covariance too.
Wearable: final demo rig 29/30 (96.7%) with the original model (see §1.6a). Pole pickup false alarm (0.9961) → press cancel during the demo.
Remaining work, in order: (1) repeat the box test after b86757c, several orientations including the 20 cm narrow face; (2) review cluster size / target selection in lidar_differencing.py and approve new size limits; (3) set SCAN_POINT in firestore_dispatcher.py to (2.31, −0.01); (4) decide whether lidar_differencing goes into the bringup launch; (5) re-test the return trip on a full battery; confirm use_rotate_to_heading after a fresh Nav2 restart; (6) end-to-end drill: wearable fall → dispatcher → robot → welfare check → app. Flash the wearable demo firmware from main with DATA_COLLECTION_MODE false (Huge APP partition).

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
⚠️ Known false alarm: PICKING UP THE POLE scores 0.9961. If it happens while setting up, press the CANCEL button within the 10 s countdown.
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
