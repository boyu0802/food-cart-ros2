# Autonomous Food Cart — FRC Team 6998

An autonomous delivery cart that carries school lunches between floors of a
Taiwan school **by riding the normal passenger elevator** — no dumbwaiter, no
freight lift, no building modifications. It solves a real problem: students
currently haul heavy lunch crates up the stairs. This is also our nationals
science-fair entry (see [`docs/SCIENCE_FAIR_EXPERIMENTS.md`](docs/SCIENCE_FAIR_EXPERIMENTS.md)).

> **One-sentence thesis:** *How does sensor redundancy enable autonomy in
> environments where any single sensor is unreliable?*

---

## System at a glance

| Layer | What it is |
|---|---|
| Compute | Orange Pi 5, Ubuntu 24.04 aarch64, **ROS 2 Jazzy** |
| Drivetrain | FRC swerve on a **RoboRIO** (separate controller), commanded over NetworkTables |
| Pi ↔ RoboRIO link | `nt_bridge` — translates ROS topics ↔ NetworkTables (cmd_vel out; odom / IMU / Limelight / mission signals in) |
| Navigation | Nav2 + slam_toolbox, per-floor saved maps |
| Obstacle sensing | Slamtec C1 LiDAR (2D) + Intel RealSense D455 (depth-only XYZ cloud → Nav2 voxel layer) |
| AprilTag perception | Limelight does **all** tag detection at ~90 FPS; the Pi consumes poses via the NT bridge |
| Door sensing | D455 depth ROI → median/lateral-std classifier (`door_state_detector`) |

Detailed hardware, wiring, quirks, and bring-up commands live in
[`docs/SETUP.md`](docs/SETUP.md). This README is the high-level map and the
current-status log.

### Packages

- **`cart_bringup`** — launch files, Nav2 / twist_mux / slam configs, saved maps, sensor bring-up.
- **`cart_elevator`** — the mission brain: state machine, docking, door detection, safe-to-enter gate, floor/direction estimators.
- **`nt_bridge`** — the Pi ↔ RoboRIO NetworkTables contract.
- **`cart_description`** — URDF / TF (real measured cart dimensions).
- **`rplidar_ros`** — vendored LiDAR driver.

---

## How the cart rides the elevator

The whole mission is a single **linear state machine** (`cart_elevator/supervisor/mission.py`).
Each non-wait state emits one action on entry (drive a Nav2 goal, set a dock
target, press a button, swap the map); waits block on a single boolean. The
full pickup→ride→dropoff sequence is 18 states:

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
   flat panel ~0.7 m away; *open* is much deeper (the cab interior). The
   `safe_to_enter_gate` ANDs **door = OPEN ∧ elevator-direction = IDLE ∧
   arrived-floor = ours ∧ inside-clear** before it lets the cart move. (During
   bring-up, the immature direction/floor checks can be switched off so the
   gate keys on door-open alone — door is *always* required.)
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

## Current status (updated 2026-05-29)

**Works in simulation, end-to-end.** `ros2 launch cart_elevator mission_sim.launch.py`
walks IDLE → DONE in ~24 s against fake Nav2 / dock / door / floor feeders.
42/42 unit tests pass. The whole stack follows a **pure-layer (ROS-free,
unit-tested) → ROS-wrapper → sim-feeder** split, so the decision logic is
testable without hardware.

**On real hardware:** sensors, navigation, docking, and door detection are up
and individually validated; the autonomous safe-gated ride is being brought up
floor-by-floor (see the two-test plan above).

### Progress, past few days

- **2026-05-29 — real-hardware integration day (8 commits pushed):**
  - Calibrated the door detector on the real D455 at **both** poses — hallway-wait
    (Test A) and in-cab (Test B). Key finding: an *open* cab reads **deep but
    jagged**, not flat, so the classifier's "flatness" gate had to be raised
    above the open noise and the median does the closed-vs-open call. `open_depth_m`
    is a deliberately *low* gate anchored on the closed panel, because what the
    door reveals changes per floor.
  - Added the **integrated mission launch** (`mission.launch.py`) and the
    `first_state` / `after_cab_exit` sub-mission knobs.
  - Made the safe gate's direction/floor checks **optional** for bring-up
    (`require_direction` / `require_floor`) while those detectors mature.
  - **D455 reconfigured to a depth-only XYZ cloud** feeding a Nav2 **voxel
    layer** for off-plane obstacles — the colored cloud was starving the USB
    link; plus an arm64 workaround that enables the pointcloud at runtime.
  - **Velocity-pipeline fix:** pinned `twist_mux use_stamped:false` (the build
    defaulted to true and silently dropped every input → cart never moved).
  - **Startup stagger** so ~19 nodes don't crash the Orange Pi on a cold boot.
  - Added the floor-1 and floor-4 SLAM maps.
- **2026-05-25/26 — real-cart calibration:** measured cart dimensions / swerve
  geometry into the URDF, masked the LiDAR corner-post sectors before Nav2,
  added a static-map localization mode for saved-map nav, and built the in-cab
  LiDAR wall-dock + floor-estimate-based relocalize.
- **2026-05-24 — first real elevator ride logged:** the z-accel fix held; the
  v4 fused floor estimator stayed robust where v3 drifted into a phantom floor.
- **2026-05-18/20 — mission brain:** the 18-state machine, AprilTag docking
  controller + closed-loop sim, the door detector, the safe-to-enter gate, and
  the extended NT mission contract all landed.

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
- **RoboRIO-side NT contract** (button-press pneumatic, mission-signal pulses,
  swerve odom/IMU passthrough) — see [`docs/SETUP.md`](docs/SETUP.md).

---

## Running it

Bring up sensors + navigation first, then the mission layer (this order avoids
the cold-boot spawn storm):

```bash
# 1. sensors + nav stack
ros2 launch cart_bringup bringup.launch.py

# 2. mission layer (toggles default OFF; enable what you're testing)
ros2 launch cart_elevator mission.launch.py enable_door:=true enable_docks:=true

# software-only end-to-end demo (no hardware)
ros2 launch cart_elevator mission_sim.launch.py   # then publish /mission/start
```

See [`docs/SETUP.md`](docs/SETUP.md) for the full hardware bring-up, network
setup, RViz/Foxglove-over-radio notes, and the per-quirk workarounds.
