# nt_bridge

NetworkTables 4 <-> ROS2 bridge for Team 6998 autonomous food cart.

The Orange Pi runs ROS2 (Nav2 + slam_toolbox). The RoboRIO drives the swerve.
This node is the contract between them, over NT4.

## Topics & NT entries (the contract)

**Pi publishes (RoboRIO subscribes)** — table `Nav/cmd`:

| NT key       | Type   | Units      | Meaning                                |
|--------------|--------|------------|----------------------------------------|
| `vx`         | double | m/s        | Forward velocity, robot frame          |
| `vy`         | double | m/s        | Strafe velocity, robot frame           |
| `omega`      | double | rad/s      | Yaw rate                               |
| `heartbeat`  | int64  | -          | Monotonic counter, increments each cmd |
| `timestamp`  | double | s          | Pi ROS time when this command was set  |

Watchdog: if no `/cmd_vel` arrives for `cmd_vel_max_age` (0.5 s default), the
bridge publishes `(0, 0, 0)` so the swerve stops. RoboRIO should also stop if
`heartbeat` stops incrementing.

**RoboRIO publishes (Pi subscribes)** — table `Robot/odom`:

| NT key   | Type   | Units | Meaning                                |
|----------|--------|-------|----------------------------------------|
| `x`      | double | m     | Pose x, odom frame                     |
| `y`      | double | m     | Pose y, odom frame                     |
| `theta`  | double | rad   | Heading, odom frame                    |
| `vx`     | double | m/s   | Body-frame forward velocity            |
| `vy`     | double | m/s   | Body-frame strafe velocity             |
| `omega`  | double | rad/s | Yaw rate                               |

Computed on RoboRIO from swerve module states + gyro.

**RoboRIO publishes (Pi subscribes)** — table `Robot/imu`:

| NT key      | Type   | Units    | Meaning                              |
|-------------|--------|----------|--------------------------------------|
| `yaw`       | double | rad      | Gyro yaw (NavX / Pigeon)             |
| `yaw_rate`  | double | rad/s    | Gyro yaw rate                        |
| `accel_x`   | double | m/s^2    | Linear accel, body frame             |
| `accel_y`   | double | m/s^2    | Linear accel, body frame             |

## Frame conventions

ROS2 REP-103: x forward, y left, z up, yaw CCW positive.
RoboRIO code must follow the same convention or convert before publishing.

## Run

```bash
ros2 launch nt_bridge nt_bridge.launch.py
```

Edit `config/nt_bridge.yaml` to change topic names, rates, or override the
server address.
