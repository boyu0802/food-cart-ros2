# Team 6998 Food Cart — Gazebo Simulation Plan

## Architecture

```
Docker Compose (Windows 11 / Docker Desktop WSL2)
┌─────────────────────────────────────────────────────────────┐
│  sim service        │  nav service       │  foxglove svc   │
│  Gazebo Harmonic    │  Nav2 + SLAM       │  foxglove_bridge│
│  school world SDF   │  cart_elevator     │  :8765          │
│  ros_gz_bridge      │  mission FSM       │                 │
│  elevator plugin    │  door/floor detect │                 │
│  actor plugin       │  safe gate + dock  │                 │
└─────────────────────┴────────────────────┴─────────────────┘
    ↕ Gz Transport          ↕ ROS2 DDS           ↕ WebSocket
    ros_gz_bridge ──────────┘                     │
                                           Foxglove Studio
                                           (Windows browser)
```

## What Gets Built

### 1. Docker Setup (`simulation/`)
- `Dockerfile` — ROS2 Jazzy + Gazebo Harmonic + Nav2 + slam_toolbox + foxglove_bridge + ros_gz packages, builds workspace
- `docker-compose.yml` — 3 services: `sim`, `nav`, `foxglove`
- Volume mounts existing `src/` into the container

### 2. Gazebo World (`simulation/worlds/school.sdf`)
Two-floor school building with:
- **Floor 4**: Kitchen area (pickup) + hallway + elevator lobby with call panel
- **Floor 1**: Lobby/hallway + classroom area (dropoff)
- **Elevator shaft**: Connecting floors 1-4 (3.8m per floor = 11.4m shaft)
  - Moving platform (prismatic joint, 1.1 m/s cruise, 1.0 m/s² accel)
  - Sliding doors on each floor (separate prismatic joints)
  - Interior: 1.4m x 1.4m cab
- **Hallway obstacles**: Benches, trash cans, lockers, pillars
- **Dynamic actors**: 4-6 animated walking people
  - 2 patrol hallways
  - 1 stands IN the elevator cab periodically (tests unhandled scenario)
  - 1 walks across elevator door path during opening

### 3. Robot Model (`simulation/models/food_cart/`)
SDF model matching the existing URDF exactly:
- 0.725m square chassis, 0.628m tall, 20kg
- Omnidirectional drive plugin (gz-sim-mecanum-drive or custom)
- Slamtec C1 lidar: 360°, 720 beams, 12m range, 10Hz
- D455 depth camera: 87°x58° FOV, 480x270, 15Hz, depth-only point cloud
- IMU sensor at base_link

### 4. Elevator Controller Plugin (`simulation/plugins/elevator_system/`)
Gazebo system plugin that:
- Listens to ROS2 `/elevator/call` and `/elevator/goto_floor` topics
- Drives the platform prismatic joint with trapezoidal velocity profile
- Opens/closes doors with realistic timing (2s open, 5s hold, 2s close)
- Publishes `/elevator/actual_floor`, `/elevator/door_state`, `/elevator/accel_z`
- Models: door obstruction detection, travel delays, idle-at-floor behavior
- **People-in-cab scenario**: actor inside prevents safe entry — tests the gap in the real project's logic

### 5. Simulation Launch (`simulation/launch/sim.launch.py`)
- Spawns Gazebo with the school world
- Spawns the robot model
- Starts ros_gz_bridge for all topic mappings:
  - `/scan`, `/camera/depth/image_rect_raw`, `/camera/depth/color/points`, `/imu`
  - `/cmd_vel` → Gazebo drive plugin
  - `/odom` ← Gazebo odometry
  - TF tree (map→odom→base_link→laser/camera_link/imu_link)
- Starts foxglove_bridge on port 8765

### 6. Navigation Launch (`simulation/launch/nav_sim.launch.py`)
- Runs the real nav2.yaml from `cart_bringup/config/` with `use_sim_time: true`
- Runs slam_toolbox in mapping mode
- Runs cart_elevator mission stack (supervisor, door detector, safe gate, floor detector)
- Runs the scan_sector_filter (masking cart structure from simulated lidar)

### 7. Test Scenarios (`simulation/scenarios/`)
Python scripts that orchestrate test runs:
- `test_full_mission.py` — Full floor 4→1 delivery
- `test_elevator_with_person.py` — Person blocks elevator entry
- `test_hallway_crowd.py` — Dense pedestrian traffic
- `test_door_timing.py` — Door closes while robot is entering
- `test_nav_recovery.py` — Obstacle blocks planned path

## Key Constants Used (from project files)

| Constant | Value | Source |
|----------|-------|--------|
| Chassis | 0.725m × 0.725m × 0.628m | cart.urdf.xacro |
| Mass | 20 kg | cart.urdf.xacro |
| Ground clearance | 0.042m | cart.urdf.xacro |
| Lidar height | 0.193m AGL | cart.urdf.xacro |
| Camera position | (0.321, -0.051, 0.674)m AGL | cart.urdf.xacro |
| Camera pitch | 10° down (0.1745 rad) | cart.urdf.xacro |
| Lidar yaw | 180° (rear-facing 0°) | cart.urdf.xacro |
| vx/vy max | 3.0 m/s | nav2.yaml |
| wz max | 1.8 rad/s | nav2.yaml |
| ax/ay max | 1.0 m/s² | nav2.yaml |
| Floor height | 3.8m | floor.yaml |
| Elevator speed | 1.1 m/s | floor.yaml |
| Elevator accel | 1.0 m/s² | floor.yaml |
| Cab interior | ~1.4m × 1.4m | fake_cab_sim.py |
| Dock standoff | 0.35-0.60m | mission.yaml |
| Inflation radius | 0.6m | nav2.yaml |
| Costmap resolution | 0.05m | nav2.yaml |
| n_floors | 5 | floor.yaml |

## Iterative Review Plan
After first implementation:
1. **Review 1**: Check all constants match source files, verify topic names, fix SDF syntax
2. **Review 2**: Verify physics (elevator timing, robot dynamics, sensor noise models)
3. **Review 3**: Test scenario coverage, edge cases, Docker build reproducibility
