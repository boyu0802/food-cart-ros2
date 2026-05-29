# Autonomous Food Cart — Build Log & Bring-up Guide

**FRC Team 6998.** An autonomous delivery cart that carries school lunches
between floors of a Taiwan school **by riding the normal passenger elevator** —
no dumbwaiter, no freight lift, no building modifications. It solves a real
problem: students currently haul heavy lunch crates up the stairs. (Also our
nationals science-fair entry.)

This is the single source of truth: what's built, how the cart rides the
elevator, the hardware quirks we hit, and the commands to bring everything back
up next time. Sections near the top are current; the **Subsystem reference**
near the bottom keeps the durable per-package detail.

---

## System at a glance

| Part | Notes |
|---|---|
| Compute | Orange Pi 5, Ubuntu 24.04 aarch64, kernel `6.1.0-1025-rockchip`. User `ros2`, workspace `~/ros2_ws`. |
| Software | **ROS 2 Jazzy** + Nav2 (MPPI) + slam_toolbox + foxglove_bridge |
| Drivetrain | FRC swerve on a **RoboRIO** (separate controller), commanded over NetworkTables |
| Pi ↔ RoboRIO link | `nt_bridge` — ROS topics ↔ NetworkTables (cmd_vel out; odom / IMU / Limelight / mission signals in) |
| LiDAR | Slamtec C1 (2D, `/scan`) |
| Depth camera | Intel RealSense D455, run **depth-only** → XYZ cloud → Nav2 voxel layer for off-plane obstacles |
| AprilTag perception | Limelight does **all** tag detection at ~90 FPS; the Pi consumes poses via the NT bridge |
| Door sensing | D455 depth ROI → median / lateral-std classifier (`door_state_detector`) |
| Network | Pi at `192.168.31.114` on `end1` |

### Packages

- **`cart_bringup`** — launch files, Nav2 / twist_mux / slam configs, saved maps, sensor bring-up.
- **`cart_elevator`** — the mission brain: state machine, docking, door detection, safe-to-enter gate, floor/direction estimators.
- **`nt_bridge`** — the Pi ↔ RoboRIO NetworkTables contract (see `src/nt_bridge/README.md`).
- **`cart_description`** — URDF / TF (real measured cart dimensions).
- **`rplidar_ros`** — vendored Slamtec C1 driver.

### Architecture

```
   [ Laptop ]                 [ Orange Pi ]                     [ RoboRIO ]
   Foxglove   <-- ws://-->    ROS 2 nodes       <-- NT4 -->     Swerve drive
                              (nav, slam,                       + Pigeon IMU
                               docking, mission)                + Limelight
```

All planning, localization, and mission logic run on the Pi. The RoboRIO is a
"dumb" motion executor — it reads `vx`, `vy`, `omega` from NetworkTables, drives
swerve, and publishes odom / IMU / Limelight tags + mission signals back.

---

## How the cart rides the elevator

The whole mission is a single **linear state machine**
(`cart_elevator/supervisor/mission.py`). Each non-wait state emits one action on
entry (drive a Nav2 goal, set a dock target, press a button, swap the map);
waits block on a single boolean. The full pickup→ride→dropoff sequence is 18
states:

```
IDLE → NAV_TO_PICKUP → DOCK_PICKUP → WAIT_LOADED
     → NAV_TO_HALLWAY_CALL → DOCK_HALLWAY_CALL → PRESS_CALL_BUTTON
     → WAIT_FOR_HALLWAY_DOOR_OPEN → NAV_INTO_CAB → DOCK_IN_CAB_BUTTON
     → PRESS_FLOOR_BUTTON → WAIT_FOR_FLOOR_REACHED → NAV_OUT_OF_CAB
     → RELOCALIZE_AT_FLOOR → NAV_TO_DROPOFF → DOCK_DROPOFF
     → WAIT_UNLOADED → DONE
```

`FAULT` is a sticky any-state→fault on dock-lost / nav-failed / press-failed
(fault-and-stop for now; no auto-recovery during the demo).

### Getting *into* the elevator

1. **Dock at the hallway call button** (`DOCK_HALLWAY_CALL`). There's an
   AprilTag on the elevator call panel; the Limelight reports its pose in the
   robot frame, and `dock_controller` closed-loop aligns X/Y/yaw until the
   cart's pneumatic presser is on the button.
2. **Press the call button** (`PRESS_CALL_BUTTON`). The supervisor publishes a
   string (`"call_up"` / `"call_down"`) to `/elevator/press_button`; the NT
   bridge forwards it; the RoboRIO drives the on-cart vertical lift to that
   button's setpoint and fires the pneumatic, then pulses `press_done` back.
3. **Wait for the door to open — safely** (`WAIT_FOR_HALLWAY_DOOR_OPEN`). The
   D455 watches the door through a calibrated depth ROI: a *closed* door is a
   flat panel ~0.7 m away; *open* is much deeper. The `safe_to_enter_gate` ANDs
   **door = OPEN ∧ elevator-direction = IDLE ∧ arrived-floor = ours ∧
   inside-clear** before it lets the cart move. (During bring-up, the immature
   direction/floor checks can be switched off so the gate keys on door-open
   alone — door is *always* required.)
4. **Drive into the cab** (`NAV_INTO_CAB`). A short Nav2 goal to the
   `into_cab_pose` recorded on that floor's map. The cab door is *narrow*, so
   this is the riskiest motion of the run.
5. **Dock at the in-cab button panel** (`DOCK_IN_CAB_BUTTON`). There's **no
   AprilTag inside the cab**, so instead of tag docking we fit the cab's side
   wall from the LiDAR scan (`dock/wall_fit.py`) and align the presser to the
   floor-button panel by wall geometry (target standoff + side offset).
6. **Press the destination floor button** (`PRESS_FLOOR_BUTTON`), same
   press→`press_done` handshake as the call button.

### Getting *out of* the elevator

7. **Wait until we've arrived** (`WAIT_FOR_FLOOR_REACHED`). Then the door opens
   on the destination floor.
8. **Drive out** (`NAV_OUT_OF_CAB`) to the `out_of_cab_pose` on the destination
   floor's map.
9. **Re-localize** (`RELOCALIZE_AT_FLOOR`) against that floor's saved map before
   continuing to the dropoff.

> **Known asymmetry we designed around:** auto door-detection is safe for
> *entering* (an open door at the call floor means the elevator arrived), but
> *not* for exiting in one continuous run — an open door right after boarding
> looks identical to an open door on arrival. Distinguishing them needs the
> floor detector or a close-then-reopen edge. Until that's hardened, the
> dropoff side is run as a separate sub-mission with the cart already at the
> destination floor.

### Bring-up is split into two single-floor tests

A full autonomous **floor 4 → floor 1** run needs Nav2 localized on *both*
floors and a mid-mission map swap (still unwired). To make progress without
that, the state machine has **sub-mission framing** — `first_state` and
`after_cab_exit` params let it start mid-sequence and stop after cab exit
*without adding new states*:

- **Test A (floor-4 map):** `first_state = DOCK_HALLWAY_CALL`, `after_cab_exit = DONE`.
  Exercises hallway-call tag dock → press → door-open detection → narrow-door
  entry → in-cab LiDAR wall dock → press floor button. *(`config/mission.yaml`
  is currently set to this.)*
- **Test B (floor-1 map):** `first_state = WAIT_FOR_FLOOR_REACHED`, cart placed
  physically in the cab → auto-detect door open → drive out → done. The only
  open door it can see is the arrival one, which sidesteps the asymmetry above.

---

## Current status & progress log

**Works in simulation, end-to-end.** `ros2 launch cart_elevator mission_sim.launch.py`
walks IDLE → DONE in ~24 s against fake Nav2 / dock / door / floor feeders.
42/42 unit tests pass. The whole stack follows a **pure-layer (ROS-free,
unit-tested) → ROS-wrapper → sim-feeder** split, so the decision logic is
testable without hardware.

**On real hardware:** sensors, navigation, docking, and door detection are up
and individually validated; the autonomous safe-gated ride is being brought up
floor-by-floor (see the two-test plan above).

### 2026-05-29 — real-hardware integration day (8 commits)

- Calibrated the door detector on the real D455 at **both** poses — hallway-wait
  (Test A) and in-cab (Test B). Key finding: an *open* cab reads **deep but
  jagged**, not flat, so the classifier's flatness gate had to be raised above
  the open noise and the median does the closed-vs-open call. `open_depth_m` is
  a deliberately *low* gate anchored on the ~0.3–0.7 m closed panel, because
  what the door reveals changes per floor.
- Added the **integrated mission launch** (`mission.launch.py`) and the
  `first_state` / `after_cab_exit` sub-mission knobs.
- Made the safe gate's direction/floor checks **optional** for bring-up
  (`require_direction` / `require_floor`) while those detectors mature.
- **D455 reconfigured to a depth-only XYZ cloud** feeding a Nav2 **voxel layer**
  for off-plane obstacles — the colored cloud was starving the USB link; plus an
  arm64 workaround that enables the pointcloud at runtime.
- **Velocity-pipeline fix:** pinned `twist_mux use_stamped:false` (the build
  defaulted to true and silently dropped every input → cart never moved).
- **Startup stagger** so ~19 nodes don't crash the Orange Pi on a cold boot.
- Added the floor-1 and floor-4 SLAM maps.

### 2026-05-24/26 — first real ride + cart calibration

- **First real elevator ride logged:** the z-accel fix held; the v4 fused floor
  estimator stayed robust where v3 drifted into a phantom floor.
- Measured real cart dimensions / swerve geometry into the URDF, masked the
  LiDAR corner-post sectors before Nav2, added a static-map localization mode for
  saved-map nav, and built the in-cab LiDAR wall-dock + floor-estimate-based
  relocalize.

### 2026-05-16/20 — the mission brain

- The 18-state machine, AprilTag docking controller + closed-loop sim, the door
  detector, the safe-to-enter gate, twist_mux arbitration, per-floor map-swap
  helper, the four floor detectors + v4 fusion, and the extended NT mission
  contract all landed in this stretch.

### What's still open

- **Field data for the mission:** real AprilTag IDs, recorded Nav2 poses
  (`into_cab_pose`, `out_of_cab_pose`, dock standoffs), and dock-gain tuning on
  the real swerve.
- **In-cab open-door thresholds** are fit to one floor's lobby — re-verify at a
  floor where the door opens onto something close.
- **Mid-mission map swap** (floor 4 → floor 1 in one run) is still unwired;
  that's why bring-up is split into Test A / Test B.
- **Elevator-direction detector** is still a placeholder — the safe gate runs
  with that check disabled for now.
- **RoboRIO-side NT contract** implementation (button-press pneumatic,
  mission-signal pulses, swerve odom/IMU/Limelight passthrough) — see the NT
  contract section below.

---

## Running it

Bring up sensors + navigation first, then the mission layer (this order avoids
the cold-boot spawn storm):

```bash
source ~/ros2_ws/install/setup.bash

# 1. sensors + nav stack
ros2 launch cart_bringup bringup.launch.py
#    isolate layers while debugging:
ros2 launch cart_bringup bringup.launch.py enable_nav2:=false      # description + sensors + slam only
ros2 launch cart_bringup bringup.launch.py enable_camera:=false    # no D455

# 2. mission layer (toggles default OFF; enable what you're testing)
ros2 launch cart_elevator mission.launch.py enable_door:=true enable_docks:=true
#    door_config defaults to door_hallway.yaml; use door_incab.yaml for Test B

# software-only end-to-end demo (no hardware)
ros2 launch cart_elevator mission_sim.launch.py   # then publish /mission/start
```

### Build & source

```bash
cd ~/ros2_ws && colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

---

## Hardware quirks (read this before debugging anything)

### D455 IMU does not work on this Pi
Driver fails with `No HID info provided, IMU is disabled` / `HID Motion Sensor
Failure!`. The rockchip kernel doesn't ship `hid-sensor-hub` /
`hid-sensor-custom`. We use the RoboRIO IMU (Pigeon) over the NT bridge instead.
**Do NOT** set `enable_gyro:=true` / `enable_accel:=true` on the realsense
launch — it just spams errors. (The launch already forces them off.)

### RealSense pointcloud param has a mangled namespace on arm64
On `ros-jazzy-realsense2-camera` 4.57.7 (aarch64), the runtime param is
`pointcloud__neon_.enable` (NEON variant), **not** `pointcloud.enable`, and the
launch arg does not reliably propagate. `sensors.launch.py` now runs the
`enable_d455_pointcloud` helper, which waits for the camera then sets it at
runtime. Manual equivalent:
```bash
ros2 param set /camera/camera pointcloud__neon_.enable true
```

### D455 is run depth-only
The full-res *colored* cloud starved the USB link (~0.6 Hz, bursty, UVC
`-EPIPE` in dmesg). Every stream except depth is stripped and we emit an
untextured XYZ cloud at `480x270x15`; bump `depth_module.depth_profile` if
range/detail is insufficient. This cloud feeds the Nav2 voxel layer.

### Slamtec C1 baud is 460800 (not 115200)
Default in `rplidar_c1_launch.py` is correct. On "operation timeout," check baud
and that the `/dev/rplidar` udev symlink resolves to the real `/dev/ttyUSB*`.

### Loopback multicast off → ROS2 discovery broken on localhost
Fixed by `/etc/systemd/system/lo-multicast.service` (runs `ip link set lo
multicast on` at boot). If two ROS2 processes on the Pi can't see each other,
check `ip link show lo` for the `MULTICAST` flag.

### RViz/Foxglove over the robot radio needs unicast peers
The OM5P radio drops multicast → DDS discovery fails across it, and the
discovery server is broken in this rmw/fastrtps build. Fix: unicast initial
peers via `FASTRTPS_DEFAULT_PROFILES_FILE` on **both** ends (helpers
`fastdds_peers.xml` + `use_unicast_peers.sh` in `~/`).

### USB WiFi (DWA-T185, RTL8822BU) needs a custom DKMS driver
Not in-kernel on this rockchip build; install the out-of-tree RTL8822BU DKMS
module or the adapter won't enumerate.

### `ros2 topic hz` lies for high-bandwidth topics
High-bandwidth image/cloud topics report a fraction of the true rate. Use the
matching `camera_info` topic for the real rate.

### UFW firewall is active
If a remote tool can't reach the Pi, check `sudo ufw status`. Foxglove port:
`sudo ufw allow 8765/tcp`.

---

## Connecting Foxglove from your laptop

1. Laptop on the **same network** as the Pi (a guest-network laptop can't see it).
2. Browser → https://studio.foxglove.dev → **Open connection**.
3. Pick **Foxglove WebSocket** (NOT Rosbridge — they look almost identical).
4. URL: `ws://192.168.31.114:8765` → Open.

**Panels:** 3D panel with `/scan` (lidar) + the D455 cloud; Image panel on the
depth topic; the door ROI debug image (`/elevator/door_roi_debug`) when tuning.
Harmless warnings: `inf` values on `/scan` (beams that hit nothing), and a
PointCloud2 length warning (driver declares full buffer, sends valid points
only).

---

## Pi ↔ RoboRIO NT contract

Exact keys/units live in `src/nt_bridge/config/nt_bridge.yaml` and
`src/nt_bridge/README.md`. Summary:

**Pi → NT (out):**
- `Nav/cmd/{vx,vy,omega,heartbeat,timestamp}` — velocity command (plain Twist; the whole pipeline is unstamped).
- `Pi/elevator/press_button` (string) — `"call_up"` / `"call_down"` / `"floor_N"`.

**NT → Pi (in):** rising-edge Bools are republished as one-shot ROS signals; polled at ~20 Hz so a ~20 ms FRC pulse can't slip through.
- `Robot/odom/*`, `Robot/imu/*` (incl. `accel_z`), `limelight/*` → `/limelight/tag`.
- `Robot/elevator/press_done` → `/elevator/press_done`.
- `Robot/mission/{start_pressed,loaded,unloaded}` → `/mission/{start,loaded,unloaded}`.

**RoboRIO still owes:** read `Pi/elevator/press_button`, drive the pneumatic
vertical lift to the per-button setpoint, fire/retract, pulse `press_done`;
joystick → `start_pressed`; intake → `loaded`/`unloaded`; subscribe `Nav/cmd/*`
and publish swerve odom + Pigeon IMU + Limelight passthrough.

---

## Subsystem reference

### Navigation — Nav2 + slam_toolbox

Nav2 runs the **MPPI** controller in **Omni** mode (swerve commands `vx`, `vy`,
`omega` independently; `PreferForwardCritic` disabled). Pipeline:

```
nav2 → cmd_vel_nav → velocity_smoother → cmd_vel_smooth ┐
dock_controller ──────────────────────→ /dock/cmd_vel ──┤ twist_mux → /cmd_vel → nt_bridge → NT4 → RoboRIO
```

`twist_mux` arbitrates navigation vs dock onto `/cmd_vel` (dock priority higher).
**`use_stamped` must stay false** across the chain, and never set `locks: {}`
(empty dict SIGABRTs twist_mux) — see `config/twist_mux.yaml`.

Localization runs slam_toolbox in **localization mode** against per-floor saved
maps (`maps/floor_N/map.{yaml,pgm,data,posegraph}`; the loc node needs
data+posegraph, not just the pgm). The local costmap carries a **voxel_layer**
fed by the D455 depth cloud for off-plane obstacles the 2D scan misses. Cart
dimensions, footprint, and the LiDAR corner-post scan mask are now real measured
values (2026-05-25), not the original placeholders.

### Floor detection — four detectors behind one message

All four publish `cart_elevator_msgs/FloorEstimate`; `floor_v4_fusion` consumes
them and emits the canonical `/elevator/floor`.

| Detector | Inputs | How it decides | Confidence |
|---|---|---|---|
| v1 AprilTag | `/limelight/tag` | tag id − `floor_tag_id_offset` = floor; scaled by tag area. Ground truth when visible. | 0.70–0.95 |
| v2 Time | `/imu` | accel_z deviation → motion start/stop; integrate speed×time to count floors. | ~0.60 |
| v3 Accel | `/imu` | estimate gravity bias when idle, double-integrate (a − g_bias), snap to nearest `floor_height_m`. | ~0.40 |
| v4 Fusion | v1/v2/v3 | priority + freshness: tag if fresh, else time, else accel, else last-known decaying. Explainable (not a Kalman filter). | inherits/decays |

Blind-test rigs (`fake_elevator_imu`, `fake_tag_publisher`) drive these from
scripted scenarios with no hardware. Retune `elevator_speed_m_s`,
`floor_height_m`, `floor_tag_id_offset`, and `*_max_age_s` in `config/floor.yaml`
against a timed real trip.

### Docking, door, safe gate

- **Docking** — one retargetable `dock_controller` (proportional X/Y/yaw on the
  tag pose in robot frame) serves every alignment; the supervisor retargets it
  via the `SetDockTarget` service. In-cab uses LiDAR wall-fitting instead of a
  tag (`dock/wall_fit.py`).
- **Door** — `door/door_classifier.py` (pure) takes ROI depth median +
  column-wise lateral-std + its rate, returns CLOSED/OPENING/OPEN/CLOSING.
  `door_state_detector.py` crops the depth ROI and feeds it. Two calibrated
  configs: `door_hallway.yaml` (Test A) and `door_incab.yaml` (Test B). Tune ROI
  live with `door_roi_overlay`; read medians with `door_std_probe`.
- **Safe gate** — `safe/safe_to_enter_gate.py` ANDs door=OPEN ∧ direction=IDLE ∧
  floor=target ∧ inside_clear (all fresh, above min confidence) into a single
  `/elevator/safe_to_enter`. `require_direction` / `require_floor` (default true)
  can be flipped off for bring-up; door is always required.

### Elevator-direction detection — HISTORICAL / superseded

> Built 2026-05-16: a digit-tracking detector (`direction_v1_digit` +
> `digit_match.py`) that read the floor digit off the in-cab LED display from a
> dedicated forward/up-facing webcam, verified against a recorded phone clip.
> The camera plan has since changed — single Limelight kept front-facing, **no
> second display webcam** — so this path is **not currently in the mission**.
> The code (digit match, video-replay rig, `direction_v3_fusion`) is kept for
> reference and possible reuse as a `floor_v5_display` signal, but `direction_v2_flow`
> stays a placeholder and the safe gate runs with the direction check disabled.
> Don't treat this as a live subsystem without re-confirming the camera plan.
