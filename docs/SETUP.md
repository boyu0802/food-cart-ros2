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

Planned (not done yet): `slam_toolbox`, `nav2`, MPPI controller.

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

## State of the workspace

```
~/ros2_ws/src/
├── nt_bridge/      # our NT4 ↔ ROS2 bridge (built, not yet tested against real RoboRIO)
└── rplidar_ros/    # Slamtec C1 driver
```

Build: `cd ~/ros2_ws && colcon build --symlink-install`
Source: `source ~/ros2_ws/install/setup.bash`

---

## What's next

- [ ] Test `nt_bridge` against the RoboRIO (or a sim NT server)
- [ ] URDF + static TF tree so lidar and camera frames are aligned in Foxglove
- [ ] `slam_toolbox` for live mapping
- [ ] `nav2` with MPPI controller
- [ ] D455-based obstacle layer for small / moving / low-profile obstacles
