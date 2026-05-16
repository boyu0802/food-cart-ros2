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

## What's next

- [ ] Plug in real chassis / mount dimensions (update xacro properties)
- [ ] Test `nt_bridge` against the RoboRIO (or a sim NT server)
- [ ] First mapping run with `slam_toolbox` once the robot can move
- [ ] Tune MPPI critics against measured top speed / accel
- [ ] D455-based obstacle layer for small / moving / low-profile obstacles
