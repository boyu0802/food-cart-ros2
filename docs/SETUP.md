# Food Cart — Setup & Bring-up Notes

FRC Team 6998. Autonomous food cart delivery robot.

This doc is everything we've set up so far on the Orange Pi, the quirks we hit, and the commands to bring sensors + visualization back up next time.

---

## Hardware

| Part | Notes |
|---|---|
| Compute | Orange Pi 5, Ubuntu 24.04 aarch64, kernel `6.1.0-1025-rockchip` |
| Drivetrain | FRC swerve on RoboRIO (separate controller) |
| Floor / obstacle camera | Intel RealSense D455 (depth + RGB), planned to be mounted **angled downward** so it sees obstacles on the floor (people's feet, skateboards, curbs). Because it points down, it cannot also be the camera that watches the elevator hall display. |
| Elevator-display camera | **TBD — likely a separate USB color webcam** mounted higher up, pointed forward/up at the elevator panel. The digit-tracking detector only needs RGB (no depth), so any cheap color webcam that ROS can read via `usb_cam` / `v4l2_camera` should work. The current code reads from the generic `/image` topic, so it doesn't care which physical device produces the frames. |
| LiDAR | Slamtec C1 |
| AprilTag perception | Limelight (does all AprilTag detection at ~90 FPS; Pi consumes detections via NT bridge) |
| Network | Pi at `192.168.31.114` on `end1` |

User on Pi: `ros2`. Workspace: `~/ros2_ws`.

**On cameras:** at the start of the project we assumed one D455 would do everything. After thinking about mount angles, we don't think that's workable: the obstacle-avoidance use case wants the depth camera pointed *down* at the floor, and the elevator-panel use case wants a camera pointed *up and forward* at the wall display. Two cameras (D455 looking down + a cheap RGB webcam looking up) is the current plan, and is what the elevator-direction code is written against (any color image stream on `/image`). This is not finalised — see "What's next" if we ever revisit the single-camera idea.

---

## Software stack

- **ROS 2 Jazzy** on the Pi
- **foxglove_bridge** for visualization (laptop talks to Pi over WebSocket)
- **rplidar_ros** for the C1 lidar
- **realsense2_camera** for the D455
- **nt_bridge** (our package) — bridges ROS2 ↔ NetworkTables for the RoboRIO
- **cart_description** (our package) — URDF / xacro for the cart
- **cart_bringup** (our package) — launch + config for the full nav stack
- **slam_toolbox** — online async mapping (configured, not yet driven on a real robot)
- **nav2** — full navigation stack with MPPI controller (configured, not yet driven)

Everything is scaffolded with PLACEHOLDER values; we'll re-tune once the chassis is built and the RoboRIO is wired.

---

## Architecture

```
   [ Laptop ]                 [ Orange Pi ]                     [ RoboRIO ]
   Foxglove   <-- ws://-->    ROS2 nodes        <-- NT4 -->     Swerve drive
                              (nav, slam,
                               sensors)
```

- All planning + localization on the Pi.
- RoboRIO is a "dumb" motion executor — reads `vx`, `vy`, `omega` from NetworkTables and drives swerve.
- See `~/ros2_ws/src/nt_bridge/README.md` for the exact NT4 keys.

---

## Hardware quirks (read this before debugging anything)

### D455 IMU does not work on this Pi
Driver fails with `No HID info provided, IMU is disabled` / `HID Motion Sensor Failure!`. The rockchip kernel doesn't ship `hid-sensor-hub` / `hid-sensor-custom`. Fix would require rebuilding the kernel — not worth it. We'll use the RoboRIO IMU (NavX/Pigeon) over the NT bridge instead.

**Do NOT** add `enable_gyro:=true` or `enable_accel:=true` to the realsense launch — it just spams errors.

### Slamtec C1 baud is 460800 (not 115200)
Default in `rplidar_c1_launch.py` is correct. If you see "operation timeout," check baud and that the `/dev/rplidar` udev symlink resolves to the actual `/dev/ttyUSB*`.

### Loopback multicast was off → ROS2 discovery broken on localhost
Fixed by `/etc/systemd/system/lo-multicast.service` which runs `ip link set lo multicast on` at boot. If two ROS2 processes on the Pi can't see each other, check `ip link show lo` for the `MULTICAST` flag.

### RealSense pointcloud param has a weird namespace
On the arm64 build of `ros-jazzy-realsense2-camera` 4.57.7, the runtime param is `pointcloud__neon_.enable` (NEON-optimized variant), not `pointcloud.enable`. The launch arg `pointcloud.enable:=true` does **not** reliably propagate. Set it at runtime:
```
ros2 param set /camera/camera pointcloud__neon_.enable true
```

### `ros2 topic hz` lies for high-bandwidth topics
Color image (1280x720 @ 30 Hz ≈ 80 MB/s) shows up as ~4 Hz; pointcloud (~37 MB/s) shows up as ~6 Hz. The camera is actually running at 30 Hz. Use the matching `camera_info` topic for true rate.

### UFW firewall is active
Probably wasn't blocking anything for us so far, but if a remote tool can't reach the Pi, check `sudo ufw status`. To open the foxglove port if needed: `sudo ufw allow 8765/tcp`.

---

## Bring-up commands

### Lidar
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rplidar_ros rplidar_c1_launch.py
```
Publishes `/scan` (`sensor_msgs/LaserScan`).

### D455 camera (with pointcloud)
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py
# in another terminal, after it's running:
ros2 param set /camera/camera pointcloud__neon_.enable true
```
Publishes:
- `/camera/camera/color/image_raw` — RGB
- `/camera/camera/depth/image_rect_raw` — depth
- `/camera/camera/depth/color/points` — colored pointcloud (after enabling)

### Foxglove bridge (visualization)
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
Listens on `0.0.0.0:8765`.

---

## Connecting Foxglove from your laptop

1. Make sure the laptop is on the **same Wi-Fi network** as the Pi (we hit this — laptop on guest network couldn't see the Pi).
2. Browser → https://studio.foxglove.dev
3. Click **Open connection**
4. Pick **Foxglove WebSocket** (NOT Rosbridge — they look almost identical)
5. URL: `ws://192.168.31.114:8765`
6. Click Open

### What to add in Foxglove
- **3D panel** → enable `/scan` (lidar) and `/camera/camera/depth/color/points` (pointcloud). Drag-orbit to see the 3D depth.
- **Image panel** → topic `/camera/camera/color/image_raw` for the RGB feed.

### Known display warnings (harmless)
- *"N infinity/invalid values detected"* on `/scan` — normal. Lidar returns `inf` for beams that hit nothing within range.
- *"PointCloud2 data length is less than height×row_step"* on `/camera/.../points` — driver declares full buffer size in the header but only sends valid points. Foxglove still renders it.

---

## Nav stack scaffold (built 2026-05-16)

Two new packages, designed so the *whole* navigation pipeline runs end-to-end the moment real hardware is ready. Placeholders are everywhere — every value that depends on the physical robot is tagged `# PLACEHOLDER` in YAML or pulled out into a `xacro:property` at the top of the URDF.

### Package tree

```
~/ros2_ws/src/
├── cart_description/       # URDF / xacro for the cart
│   ├── urdf/cart.urdf.xacro
│   ├── launch/description.launch.py    # robot_state_publisher
│   ├── rviz/                           # (empty, for later)
│   ├── package.xml         # ament_cmake
│   └── CMakeLists.txt      # installs share/{urdf,launch,rviz}
│
├── cart_bringup/           # launch + config for the full nav stack
│   ├── config/
│   │   ├── slam_toolbox.yaml
│   │   └── nav2.yaml
│   ├── launch/
│   │   ├── description.launch.py  # (defined in cart_description, re-included)
│   │   ├── sensors.launch.py      # lidar + D455 (no IMU)
│   │   ├── slam.launch.py         # slam_toolbox async mapping
│   │   ├── nav2.launch.py         # full nav2 stack
│   │   └── bringup.launch.py      # composes everything
│   ├── maps/                          # (empty, for saved maps later)
│   ├── package.xml         # ament_cmake
│   └── CMakeLists.txt      # installs share/{launch,config,maps}
│
├── nt_bridge/              # NT4 ↔ ROS2 bridge (built, not yet tested vs RoboRIO)
└── rplidar_ros/            # Slamtec C1 driver (third-party, .gitignored)
```

### cart_description — URDF

`urdf/cart.urdf.xacro` defines this TF tree:

```
base_footprint           (ground projection, REP-105 root for nav)
└── base_link            (chassis center, lifted by wheel_radius)
    ├── laser            (Slamtec C1 mount — name 'laser' matches rplidar_ros default)
    ├── camera_link      (D455 — driver publishes internal optical frames from here)
    └── imu_link         (RoboRIO Pigeon/NavX — not the D455 IMU)
```

All 6 placeholder values are `xacro:property` at the top of the file:

| Property | Default | Tune when |
|---|---|---|
| `chassis_length / width / height` | 0.60 / 0.50 / 0.30 m | Chassis built |
| `chassis_mass` | 20.0 kg | Cart weighed |
| `wheel_radius` | 0.075 m | Swerve modules picked |
| `lidar_xyz` | top center, +2 cm above chassis | Lidar mounted |
| `camera_xyz` | front edge, mid height | D455 mounted |
| `imu_xyz` | center, identity | RoboRIO placed |

`launch/description.launch.py` runs `robot_state_publisher` (and `joint_state_publisher`, harmless with no joints). It re-renders the xacro every launch so editing the URDF doesn't require a rebuild.

Smoke-checked: `check_urdf` parses it cleanly with the tree shown above.

### cart_bringup — launch + config

Five launch files, layered so each layer can be brought up and debugged on its own.

#### `launch/sensors.launch.py`

Wraps the existing third-party launches:
- `rplidar_ros/launch/rplidar_c1_launch.py` (publishes `/scan`)
- `realsense2_camera/launch/rs_launch.py` with `enable_gyro:=false enable_accel:=false` (per the hardware quirk above) and `enable_color`, `enable_depth`, `align_depth.enable`, `pointcloud.enable` all true. Reminder: pointcloud arg doesn't propagate on the arm64 build — set `pointcloud__neon_.enable` at runtime.

Toggles: `enable_lidar`, `enable_camera`.

#### `launch/slam.launch.py` + `config/slam_toolbox.yaml`

Online async mapping. Key choices:

- `mode: mapping` (not localization — no saved map yet)
- `map_frame: map`, `odom_frame: odom`, `base_frame: base_link` → matches the URDF and what nt_bridge publishes
- `max_laser_range: 12.0` → Slamtec C1 spec
- `resolution: 0.05` m/cell (PLACEHOLDER)
- `minimum_travel_distance: 0.2 m`, `minimum_travel_heading: 0.2 rad` (PLACEHOLDER — these are the "how far must the robot move before adding a new scan?" knobs; tune once we know cart top speed)
- Loop closure on (`do_loop_closing: true`) with default thresholds

slam_toolbox is the *only* thing publishing `map → odom` while we're mapping. `nt_bridge` publishes `odom → base_link` (sits at identity until RoboRIO is live).

#### `launch/nav2.launch.py` + `config/nav2.yaml`

Full Nav2 with seven lifecycle-managed nodes:

| Node | Plugin |
|---|---|
| `controller_server` | `nav2_mppi_controller::MPPIController` (FollowPath) |
| `planner_server` | `nav2_navfn_planner::NavfnPlanner` (GridBased, Dijkstra) |
| `smoother_server` | `nav2_smoother::SimpleSmoother` |
| `behavior_server` | spin, backup, drive_on_heading, wait, assisted_teleop |
| `bt_navigator` | bundled default BT |
| `waypoint_follower` | wait_at_waypoint |
| `velocity_smoother` | rate-limits `cmd_vel_nav → cmd_vel` |

We deliberately skipped `map_server` and `amcl` — slam_toolbox is publishing the live map and `map → odom` TF itself. When we switch to pure-localization mode later, we'll add them.

MPPI is configured for **omni** motion (`motion_model: "Omni"`) since swerve can independently command `vx`, `vy`, `omega`. `PreferForwardCritic` is disabled for the same reason. Critic mix: Constraint, Cost, Goal, GoalAngle, PathAlign, PathFollow, PathAngle.

Velocity-smoother remap chain: `controller_server` publishes to `cmd_vel_nav`, `velocity_smoother` reads that and publishes the smoothed result to `cmd_vel`, which `nt_bridge` then forwards as NT4 `Nav/cmd/{vx,vy,omega}` to the RoboRIO.

```
nav2 → cmd_vel_nav → velocity_smoother → cmd_vel → nt_bridge → NT4 → RoboRIO
```

Placeholders in `nav2.yaml` (grep `# PLACEHOLDER`):
- Footprint polygon (currently `[[±0.30, ±0.25]]` matching chassis defaults)
- `inflation_radius` (0.45 local, 0.55 global)
- `vx_max / vy_max / wz_max` and matching mins
- `ax_max / ay_max / az_max` (MPPI accel constraints)
- `xy_goal_tolerance / yaw_goal_tolerance`
- `velocity_smoother` max accel/decel arrays
- `behavior_server` rotational vel/accel

#### `launch/bringup.launch.py`

Composes everything. Six `enable_*` toggles let you isolate any layer:

```bash
ros2 launch cart_bringup bringup.launch.py
ros2 launch cart_bringup bringup.launch.py enable_nav2:=false               # just description + sensors + slam
ros2 launch cart_bringup bringup.launch.py enable_sensors:=false enable_slam:=false  # nav2 against a recorded bag
ros2 launch cart_bringup bringup.launch.py enable_nt_bridge:=false          # no RoboRIO hardware
```

Toggles: `enable_description`, `enable_sensors`, `enable_nt_bridge`, `enable_slam`, `enable_nav2`, `enable_foxglove`.

### Build & source

```bash
cd ~/ros2_ws && colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

Both new packages built clean on first try (5.98 s total). The five launch files all import without error; `check_urdf` confirms the xacro produces a well-formed tree.

### Where to retune for the real robot

When the chassis is built and the RoboRIO is wired, in order:

1. **`cart_description/urdf/cart.urdf.xacro`** — set `chassis_*`, `wheel_radius`, `lidar_xyz`, `camera_xyz`, `imu_xyz` from real measurements.
2. **`cart_bringup/config/nav2.yaml`** — update the footprint polygon to match `chassis_*`; set `vx_max`/`vy_max`/`wz_max` from measured top speed; set `ax_max`/`ay_max` from measured / safe accel.
3. **`cart_bringup/config/slam_toolbox.yaml`** — usually fine as-is; revisit `minimum_travel_*` if cart moves much faster or slower than expected.

---

## Elevator interaction stack (built 2026-05-16)

Two more packages, plus a Limelight extension to `nt_bridge`. Goal: ride a building elevator autonomously — track which floor we're on, recognise the up/down arrow on the display, and dock to an AprilTag inside the car.

The first deliverable is **floor detection**, with four interchangeable algorithms behind a single message type so we can A/B them against the same data.

### `cart_elevator_msgs` — shared message types

- `TagDetection.msg` — one AprilTag from the Limelight (`tid`, `ta`, `tx`, `ty`, `targetpose_robotspace`). Republished into ROS by `nt_bridge`, consumed by anything that needs tags.
- `FloorEstimate.msg` — `{floor, confidence, source, moving, direction}`. All four floor detectors emit this type, so the fusion node and downstream consumers treat them interchangeably.

`ament_cmake` package (msgs must be C++/CMake even when consumed from Python).

### `nt_bridge` Limelight extension

Added a fourth flow:

```
NT  limelight/{tv,tid,tx,ty,ta,targetpose_robotspace}
  -> ROS2  cart_elevator_msgs/TagDetection  on  /limelight/tag
```

Subscribes via `pyntcore` to the Limelight default table, downsamples to `tag_publish_rate` (default 20 Hz — Limelight runs ~90 FPS, no consumer needs that), and converts the `[x, y, z, pitch_deg, yaw_deg, roll_deg]` array Limelight publishes into a proper `geometry_msgs/Pose`. When no tag is visible (`tv != 1`) it publishes `tag_id = -1` so consumers can still see liveness.

New params (all in `config/nt_bridge.yaml`): `enable_limelight_in`, `tag_topic`, `nt_limelight_table`, `tag_publish_rate`, `tag_frame_id`. Toggle off cleanly by setting `enable_limelight_in: false`.

Per [project-perception-arch], the Pi never runs `apriltag_ros` itself — the Limelight does all detection, the bridge just forwards.

### `cart_elevator` — floor detection

```
cart_elevator/
├── cart_elevator/
│   ├── floor/
│   │   ├── floor_v1_apriltag.py    # tag id -> floor number, high confidence
│   │   ├── floor_v2_time.py        # detect motion start/stop, count floors by elapsed time
│   │   ├── floor_v3_accel.py       # detect motion, integrate accel twice for distance
│   │   └── floor_v4_fusion.py      # priority+freshness fusion of v1/v2/v3
│   └── test_helpers/
│       ├── fake_imu.py             # synthetic /imu with scripted elevator trips
│       └── fake_tag.py             # synthetic /limelight/tag with scripted tag visibility
├── config/floor.yaml               # one YAML, all four detectors + the two fakes
└── launch/floor_blind_test.launch.py
```

All four detectors publish `cart_elevator_msgs/FloorEstimate` on their own topic (`/elevator/floor_v1_apriltag`, `…v2_time`, `…v3_accel`); the fusion node consumes those and emits the canonical `/elevator/floor`.

#### Algorithm summary

| Detector | Inputs | How it decides | Confidence |
|---|---|---|---|
| v1 AprilTag | `/limelight/tag` | Tag id − `floor_tag_id_offset` (default 100) = floor number; confidence scaled by tag area. Ground truth when visible. | 0.70 – 0.95 |
| v2 Time | `/imu` | Watch accel_z deviation to detect motion start/stop; integrate (elevator_speed × elapsed) to count floors. | ~0.60 |
| v3 Accel | `/imu` | Estimate gravity bias when idle, integrate (a − g_bias) twice for distance, snap to nearest `floor_height_m`. | ~0.40 |
| v4 Fusion | the three above | Priority + freshness: tag if fresh, else time, else accel, else last-known with decaying confidence. Not a Kalman filter — explainable on a science-fair poster. | inherits / decays |

#### Test helpers (no robot required)

- `fake_elevator_imu` synthesizes a `/imu` stream with scripted elevator trips. Scenario string is a comma-separated list like `"+3,p5,-1"` — `+3` = up 3 floors, `p5` = 5-second pause, `-1` = down 1 floor. Models accel/cruise/decel/idle phases at `accel_mss = 1.0 m/s²` by default; adds Gaussian noise and an optional constant bias so v3 actually has to estimate the bias.
- `fake_tag_publisher` publishes a scripted `/limelight/tag` stream. Scenario string is `"<t>:<tag_id>:<area>; ..."` — at `t=2s` show tag 101 at area 0.04, at `t=8s` switch to tag 103, etc. Between entries it publishes `tag_id = -1` so it looks like the Limelight when no tag is in view.

#### Blind-test launch

```bash
ros2 launch cart_elevator floor_blind_test.launch.py
# or override the scripted trips:
ros2 launch cart_elevator floor_blind_test.launch.py imu_scenario:='+2,p10,-2' \
    tag_scenario:='2:101:0.04; 16:103:0.05'
```

Brings up both fakes + all four detectors. Open Foxglove or use `ros2 topic echo` on the five `/elevator/...` topics to compare detectors against identical synthetic input. This is the rig we'll use to tune confidences and thresholds before the cart exists; the same detectors run against real `/imu` and `/limelight/tag` later by simply not launching the fakes.

#### Unit tests

`test/test_floor_logic.py` directly calls the four detectors' callbacks with crafted messages and asserts internal state — no spinning executor, no network. Run with:

```bash
cd ~/ros2_ws && colcon test --packages-select cart_elevator
colcon test-result --verbose
```

### Elevator-direction detection — digit-tracking pipeline (2026-05-16, working)

> **Status:** primary detector implemented and verified end-to-end against a recorded phone video of floors 1→4. Optical-flow fallback deliberately deferred. Not yet tested against the real building's elevator with the real camera mounted on the cart.

#### Motivation

To autonomously ride an elevator, the cart needs to answer two related questions every frame:

1. **Which floor are we currently on?** — used to know when to exit the car.
2. **Is the car moving, and in which direction?** — used to know we boarded the right elevator, and as a sanity check against the IMU-based floor stack.

A wall-mounted LED display in the elevator car answers both questions simultaneously: it shows a big floor digit (1, 2, 3, …) and an animated up/down arrow above it. We had originally planned to read the *arrow* (since direction is what `cart_elevator_msgs/ElevatorDirection` carries), but on reviewing a phone video of the real display we realised the digit is a much better signal:

- The digit is large, high-contrast, and present every frame — the arrow is animated, so any single frame may catch it mid-blink.
- Reading the digit gives us the absolute floor number **for free**, which the arrow can't.
- Digit recognition with a 4-template IoU match is trivially fast (microseconds per frame) and needs zero training data — we bootstrap the templates from the video itself.

So the v1 detector watches the **digit**, not the arrow. The arrow-based optical-flow detector (v2) is kept as a scaffold for very short hops where the digit might not change before the doors open, but we've decided to only implement it if measurements prove it's needed.

#### Pipeline (per frame)

Implemented in [`cart_elevator/direction/digit_match.py`](../src/cart_elevator/cart_elevator/direction/digit_match.py) — all four steps are pure functions so they can be unit-tested and reused by future tools (a `floor_v5_display` detector would call the same helpers).

```
   raw BGR frame
        │
        ▼
   find_housing(bgr) ────────► (x, y, w, h) of the dark display panel
        │                      (largest dark, vertically-elongated region
        │                       that contains warm-bright LED pixels)
        ▼
   warm_mask(bgr)  ───────────► binary mask of lit amber / red LEDs
        │                      (two HSV ranges around hue=0 to cover the
        │                       full red→amber wrap-around)
        ▼
   extract_digit(mask, housing) ──► 32×48 normalized binary crop of the
        │                          digit alone. Picks the largest lit
        │                          connected component whose centroid is
        │                          in the LOWER HALF of the housing — the
        │                          arrow lives in the upper half and is
        │                          deliberately discarded.
        ▼
   classify_digit(crop, templates) ──► (digit, IoU_score, margin)
                                     IoU vs each digit_<N>.png template,
                                     return the best plus the gap to the
                                     runner-up (margin). Both must clear
                                     thresholds (default score≥0.45,
                                     margin≥0.05) to be trusted.
```

The numeric tuning constants (`HOUSING_*`, HSV ranges, `MIN_DIGIT_AREA`) are module-level constants at the top of `digit_match.py`. They are tuned for the lighting + camera angle of the recorded phone clip; **expect to re-tune them when the real elevator-display camera is mounted on the cart**, especially HSV thresholds (different webcam = different white balance) and the housing size constraints (different mount distance = different bbox size on the image).

#### The detector node — `direction_v1_digit`

Lives in [`cart_elevator/direction/direction_v1_digit.py`](../src/cart_elevator/cart_elevator/direction/direction_v1_digit.py). Subscribes to `/image`, runs the pipeline above on every frame, and maintains a sliding 5-second window of confident digit observations. Each timer tick (10 Hz) it classifies the current direction from the window:

| Window contents | Reported direction | Confidence |
|---|---|---|
| All observations are the same digit | `IDLE` | 0.75 |
| Last digit > first digit (and not equal) | `UP` | 0.55 + 0.10 × obs, capped 0.95 |
| Last digit < first digit | `DOWN` | 0.55 + 0.10 × obs, capped 0.95 |
| Endpoints equal but window is non-uniform (went up then back) | `UNKNOWN` | 0.0 |
| Fewer than 2 observations | `UNKNOWN` | 0.0 |

Confidence grows with the number of observations — a single 1→2 transition is much weaker evidence than 1→2→3→4. The confidence ceiling of 0.95 is so v3_fusion can still prefer a corroborating signal if one shows up.

**Known weirdness: the loop-back wraparound.** Our recorded clip loops 1→2→3→4→(rewind)→1→2…. When the file rewinds, the detector sees a 4→1 transition, which is "decreasing", which it reports as `DOWN` with high confidence. That is *correct* given what it sees — don't be fooled by it during testing into thinking the detector is broken. On real hardware this never happens; the video-replay loop is the only context where it occurs.

#### The fusion node — `direction_v3_fusion`

Unchanged from the original scaffold. Takes `/elevator/direction_v1_digit` and `/elevator/direction_v2_flow` (currently always UNKNOWN) and emits `/elevator/direction` with this policy:

- Both fresh + agree → that direction, confidence = max(v1, v2)
- Both fresh + disagree → `UNKNOWN`, confidence = 0.5 × min(v1, v2)
- Only one fresh → pass through with confidence -= 0.1
- Neither fresh → `UNKNOWN`, 0.0

Freshness window is `max_age_s: 1.0` from the YAML.

Because v2 is currently a placeholder that never reports anything other than `UNKNOWN`, fusion is effectively a v1 pass-through with a 0.1 confidence haircut. That's fine — the API is in place for when v2 comes online.

#### Test rig — video replay + live debug overlay

Two new test helpers, both runnable without any cart hardware:

1. **`video_replay`** ([`test_helpers/video_replay.py`](../src/cart_elevator/cart_elevator/test_helpers/video_replay.py)) — opens an mp4 with OpenCV's VideoCapture, publishes each frame as `sensor_msgs/Image` (rgb8) on `/image` at the file's native framerate, and rewinds to frame 0 on EOF (configurable). This is our deterministic fixture: every run sees the exact same input, so detector behavior is reproducible.

2. **`direction_debug_view`** ([`test_helpers/direction_debug_view.py`](../src/cart_elevator/cart_elevator/test_helpers/direction_debug_view.py)) — pops an OpenCV `imshow` window that draws, on top of each replayed frame:
   - The detected housing bounding box (green)
   - A 6×-zoomed view of the extracted digit crop, top-right
   - Top-left: per-frame digit, IoU score, margin — green if confident, yellow if rejected
   - A short rolling history strip of recent classifications
   - Bottom-left: the currently-published direction + confidence bar, colored by direction (green UP, red DOWN, gray IDLE, yellow UNKNOWN)

This is the panel you actually watch to know the pipeline is working — text logs alone won't tell you whether `find_housing` picked the right bbox.

#### Procedure: how to reproduce the verification

1. **Build the workspace** (only needed after pulling fresh code or editing setup.py):
   ```bash
   cd ~/ros2_ws && colcon build --symlink-install --packages-select cart_elevator cart_elevator_msgs
   source ~/ros2_ws/install/setup.bash
   ```
2. **Run the video-driven test launch**:
   ```bash
   ros2 launch cart_elevator direction_video_test.launch.py
   ```
   This starts four nodes: `video_replay` (publishing `test/data/elevator_1_to_4_up.mp4` on `/image`), `direction_v1_digit`, `direction_v3_fusion`, and `direction_debug_view`.
3. **Watch the OpenCV window.** Expect: the green housing bbox locks onto the display almost immediately; the extracted-digit zoom in the top-right shows recognisable shapes of 1, 2, 3, 4 as the elevator ascends; the bottom-left direction label sits on `UP` with confidence climbing from 0.55 toward 0.95 as more observations accumulate; at the loop point (when the mp4 rewinds) it briefly flips to `DOWN` then back to `UP`.
4. **Inspect the topic** with `ros2 topic echo --no-arr /elevator/direction_v1_digit`. You should see ~10 Hz messages with `direction: 1` (UP), `confidence ≈ 0.95`, `source: 1` (DIGIT_TRACK).
5. **To run without the GUI** (e.g. over SSH):
   ```bash
   ros2 launch cart_elevator direction_video_test.launch.py show:=false
   ```

#### Templates and the bootstrap script

The four digit templates live at `src/cart_elevator/data/digit_templates/digit_{1,2,3,4}.png` (32×48 binary PNGs). They were generated by scanning through the recorded phone video, running the housing + warm-mask + extract pipeline, and saving the first clean crop of each digit. The bootstrap script that did this work was prototyped at `/tmp/bootstrap_digit_templates.py` and **was not committed** — it's listed under "What's next" because we'll need it again when we add floors outside 1–4 or re-shoot the templates from the cart-mounted camera.

#### Why we deleted `direction_v1_framediff.py`

The original v1 was a generic frame-differencing approach: look at the arrow ROI, watch which pixels light up, decide UP vs DOWN from the centroid trend. After looking at real frames it became clear (a) the arrow animation is complex enough that a single still can land on a near-empty phase that's ambiguous, and (b) tracking the digit is strictly more informative because it also yields the floor number. We replaced framediff with digit-tracking and renamed the source constant in `ElevatorDirection.msg` from `SOURCE_FRAMEDIFF` to `SOURCE_DIGIT_TRACK` (kept at numeric value 1 so wire-compat doesn't matter).

#### The arrow-flow detector (`direction_v2_flow`) is still a scaffold

This is intentional. The plan agreed on 2026-05-16 is: only fill it in if digit-tracking proves insufficient — specifically if there's a real-elevator scenario where the doors open before the digit has time to change (a "short hop" between adjacent floors at a fast elevator). On the recorded clip, digit changes are visible ~1.5 s into each leg, well before a typical door-open event, so the fallback may never be needed. If we do implement it, the existing scaffold already has the topic plumbing and YAML params wired up.

#### Files at a glance

```
src/cart_elevator/
├── cart_elevator/direction/
│   ├── direction_v1_digit.py        # PRIMARY detector
│   ├── direction_v2_flow.py         # PLACEHOLDER fallback
│   ├── direction_v3_fusion.py       # fuses v1 + v2
│   └── digit_match.py               # find_housing / warm_mask / extract_digit / classify_digit / load_templates
├── cart_elevator/test_helpers/
│   ├── video_replay.py              # mp4 → /image
│   ├── direction_debug_view.py      # cv2.imshow live overlay
│   └── fake_direction_image.py      # synthetic marching-block (used by direction_blind_test)
├── data/digit_templates/
│   └── digit_{1,2,3,4}.png          # 32×48 binary IoU templates
├── test/data/
│   └── elevator_1_to_4_up.mp4       # 3.4 MB phone capture, floors 1→4 ascending
└── launch/
    ├── direction_video_test.launch.py  # video_replay + v1 + v3 + debug overlay
    └── direction_blind_test.launch.py  # synthetic image + v2 + v3 (plumbing only)
```

Dependencies added to `cart_elevator/package.xml`: `python3-numpy`, `python3-opencv`.

### Where to retune for the real elevator

| File | Field | When |
|---|---|---|
| `config/floor.yaml` → `floor_v2_time` | `elevator_speed_m_s`, `floor_height_m` | Time one trip in the actual building |
| `config/floor.yaml` → `floor_v3_accel` | `floor_height_m`, `bias_window_s` | Same trip; verify gravity bias settles within bias window |
| `config/floor.yaml` → `floor_v1_apriltag` | `floor_tag_id_offset` | Decide tag-id ↔ floor-number convention in the building |
| `config/floor.yaml` → `floor_v4_fusion` | `*_max_age_s` | After observing real detector cadences |
| `cart_elevator/direction/digit_match.py` | `HOUSING_*` constants, HSV ranges | Once the elevator-display camera is mounted on the cart and we see how the real display looks at the real distance + lighting |
| `data/digit_templates/digit_*.png` | the template images themselves | Re-bootstrap from a clip taken with the cart-mounted camera, not the phone. Phone-shot templates may not generalise to a different camera's pixel response |
| `config/floor.yaml` → `direction_v1_digit` | `min_score`, `min_margin`, `history_window_s` | After capturing more clips (up trip, down trip, idle, off-spec floors) and seeing where false-rejects vs false-accepts live |

---

## What's next

**Done since the last entry** (2026-05-18, brought current):

- [x] **Direction detector — digit-tracking** (`direction_v1_digit`) — implemented and verified against recorded video. See "Elevator-direction detection" above for full procedure. Optical-flow fallback (`direction_v2_flow`) intentionally deferred until measurements prove it's needed.

**No robot needed (do these now):**

- [ ] **AprilTag alignment / final-approach docking controller — NOT STARTED.** This is the biggest open piece on the elevator side. The Limelight publishes `targetpose_robotspace` for any visible tag (a `geometry_msgs/Pose`), and `nt_bridge` already republishes that as `cart_elevator_msgs/TagDetection` on `/limelight/tag`. What we still need to build:
  - A docking node that subscribes to `/limelight/tag`, picks the target tag by id (e.g. an in-car tag near the door, or a tag on the hallway wall outside the elevator), and emits `geometry_msgs/Twist` on `cmd_vel` to drive the cart to a target offset pose relative to that tag.
  - A simple controller: probably proportional on lateral error (tag_x), longitudinal error (tag_z minus desired stand-off), and yaw error (tag yaw). Start with three independent P loops, only add I/D if the simulator (next bullet) shows offset bias.
  - A closed-loop simulator for the controller: extend `fake_tag_publisher` so the tag pose responds to commanded `cmd_vel` (integrate position over time), letting us tune gains without the cart.
  - Define the "I'm docked" termination condition (within X cm laterally, Y cm longitudinally, Z degrees in yaw — pick values that match the elevator door's clear width).
  - Open question: does docking happen *before* boarding (align in the hallway to enter cleanly) or *inside* the car (align with an in-car tag to be at a known pose for exit)? Probably both — same controller, different target tag id.
- [ ] **Door-state detector** (open / closing / closed). Most likely option: D455 depth-crop of the door region — when the door is open, the average depth in that crop jumps from "wall distance" to "inside-car distance". Same shape as floor detection: multiple algorithms behind one message type, blind-test launch.
- [ ] **Safe-to-enter gate**: AND together `door == open`, free-space inside the car (D455 depth, after the elevator-display camera has confirmed `direction == IDLE` and floor matches target), and direction confidence. One boolean `/elevator/safe_to_enter`.
- [ ] **Elevator BT (behaviour tree) node**: sequences `approach hallway tag → press call button (manual for now) → wait for door → wait for IDLE+correct floor → align to in-car tag → enter → wait for door close + ride → wait for IDLE+target floor → align to door tag → exit`. Drive it against the fake helpers (extended with door + arrow scenarios) before any real cart hardware exists.
- [ ] **More elevator-display fixtures** (waiting on you to capture from a phone):
  - `elevator_4_to_1_down.mp4` — descending trip so we can validate DOWN without relying on the loop-rewind artefact.
  - `elevator_idle_floor*.mp4` — ~15 s of a stationary display so we can validate IDLE.
  - A clip that exposes digits outside 1–4 (e.g. a building with floors 0, B1, 5+) — so we can extend the template set.
- [ ] **Restore digit-template bootstrap script in-repo.** Currently only existed at `/tmp/bootstrap_digit_templates.py`, which is gone. Re-create at `src/cart_elevator/scripts/bootstrap_digit_templates.py`; it should accept a video path and floor-range bins and write `data/digit_templates/digit_<N>.png`.
- [ ] **Recorded-bag tests for floor stack**: record one real elevator trip on a phone IMU app, replay through v2/v3 to validate before the cart is built.
- [ ] **End-to-end blind-test assertion**: extend `test/test_floor_logic.py` (or add `test/test_blind_e2e.py`) so it spawns the blind-test launch in a subprocess, lets it run for ~20 s with a known scenario, and asserts `/elevator/floor` settled on the expected floor.
- [ ] **`floor_v5_display` detector**: a fifth floor estimator that reuses `digit_match.py` to read the absolute floor from the elevator-display camera. Same code we already wrote — gives us a high-confidence floor signal without an AprilTag.

**Camera / mounting decisions (decide before cart assembly):**

- [ ] **Confirm the elevator-display camera.** Current plan: separate USB webcam pointed forward/up, distinct from the D455 (which will look down for floor obstacles). Pick the actual webcam model, confirm it works with `usb_cam`/`v4l2_camera` on the rockchip kernel, mount-test against the in-car display from a realistic cart-height position.
- [ ] **Re-shoot digit templates from the chosen webcam** once mounted. Phone-shot templates may not survive a sensor change.

**Needs the robot / RoboRIO (defer):**

- [ ] Plug in real chassis / mount dimensions (update xacro properties).
- [ ] Test `nt_bridge` against the RoboRIO (or a sim NT server).
- [ ] First mapping run with `slam_toolbox` once the robot can move.
- [ ] Tune MPPI critics against measured top speed / accel.
- [ ] D455-based obstacle layer for small / moving / low-profile obstacles — this is *also* the obstacle-avoidance use case that drove the "D455 looks down" mounting decision in the Hardware table.
