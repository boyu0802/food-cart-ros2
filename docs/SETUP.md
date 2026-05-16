# Food Cart — Setup & Bring-up Notes

FRC Team 6998. Autonomous food cart delivery robot.

This doc is everything we've set up so far on the Orange Pi, the quirks we hit, and the commands to bring sensors + visualization back up next time.

---

## Hardware

| Part | Notes |
|---|---|
| Compute | Orange Pi 5, Ubuntu 24.04 aarch64, kernel `6.1.0-1025-rockchip` |
| Drivetrain | FRC swerve on RoboRIO (separate controller) |
| Camera | Intel RealSense D455 (depth + RGB) |
| LiDAR | Slamtec C1 |
| Network | Pi at `192.168.31.114` on `end1` |

User on Pi: `ros2`. Workspace: `~/ros2_ws`.

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

### Where to retune for the real elevator

| File | Field | When |
|---|---|---|
| `config/floor.yaml` → `floor_v2_time` | `elevator_speed_m_s`, `floor_height_m` | Time one trip in the actual building |
| `config/floor.yaml` → `floor_v3_accel` | `floor_height_m`, `bias_window_s` | Same trip; verify gravity bias settles within bias window |
| `config/floor.yaml` → `floor_v1_apriltag` | `floor_tag_id_offset` | Decide tag-id ↔ floor-number convention in the building |
| `config/floor.yaml` → `floor_v4_fusion` | `*_max_age_s` | After observing real detector cadences |

---

## What's next

**No robot needed (do these now):**

- [ ] **Direction-arrow recognizer** for the elevator's animated display. ROI + frame-differencing or sparse optical flow. Outputs `cart_elevator_msgs/ElevatorDirection {direction, confidence}`. Test with recorded video.
- [ ] **Door-state detector**: open / closing / closed via depth-image variance or a simple AprilTag-on-door trick. Same shape as floor detection — multiple algorithms behind one message type, blind-test launch.
- [ ] **Safe-to-enter gate**: combine door-state + free-space inside the car (D455 depth crop) + floor confidence into one boolean `/elevator/safe_to_enter`.
- [ ] **Elevator BT node**: a Nav2 BT plugin (or standalone state machine) that sequences `approach → wait for door → enter → ride → exit`. Drive it against the fake_* helpers extended with door + arrow scenarios.
- [ ] **Docking controller**: P-controller from `/limelight/tag` → `/cmd_vel` for final-cm alignment to a tag. Closed-loop test with `fake_tag_publisher` + an odom integrator.
- [ ] **Recorded-bag tests for floor stack**: record one real elevator trip on a phone IMU app or any IMU we can borrow, replay through v2/v3 to validate before the cart is built.
- [ ] **End-to-end blind-test assertion**: extend `test/test_floor_logic.py` (or add `test/test_blind_e2e.py`) so it spawns the blind-test launch in a subprocess, lets it run for ~20 s with a known scenario, and asserts `/elevator/floor` settled on the expected floor.

**Needs the robot / RoboRIO (defer):**

- [ ] Plug in real chassis / mount dimensions (update xacro properties).
- [ ] Test `nt_bridge` against the RoboRIO (or a sim NT server).
- [ ] First mapping run with `slam_toolbox` once the robot can move.
- [ ] Tune MPPI critics against measured top speed / accel.
- [ ] D455-based obstacle layer for small / moving / low-profile obstacles.
