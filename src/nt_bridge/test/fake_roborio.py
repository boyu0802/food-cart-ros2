#!/usr/bin/env python3
"""Fake RoboRIO for testing nt_bridge end-to-end without real hardware.

Acts as an NT4 server on 127.0.0.1:5810 and:
  - subscribes to Nav/cmd/{vx,vy,omega,heartbeat,timestamp}
    (what the Pi/bridge publishes) and prints what it sees;
  - publishes mock Robot/odom/* describing a robot driving a slow circle, and
    Robot/imu/* with corresponding yaw / yaw_rate — so we can verify the
    bridge converts those into /odom and /imu on the ROS2 side.

Run alongside the bridge:
    Term A:  python3 ~/ros2_ws/src/nt_bridge/test/fake_roborio.py
    Term B:  ros2 run nt_bridge nt_bridge_node --ros-args -p server_address:=127.0.0.1
    Term C:  ros2 topic echo /odom --once
             ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist \\
                   '{linear: {x: 0.5, y: 0.0}, angular: {z: 0.3}}'
"""

import math
import time

import ntcore


def main() -> None:
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startServer(listen_address='127.0.0.1')
    print('[fake_roborio] NT4 server listening on 127.0.0.1:5810')

    # ---- subscribe to what the Pi publishes ----
    cmd = inst.getTable('Nav/cmd')
    vx_s = cmd.getDoubleTopic('vx').subscribe(float('nan'))
    vy_s = cmd.getDoubleTopic('vy').subscribe(float('nan'))
    omega_s = cmd.getDoubleTopic('omega').subscribe(float('nan'))
    hb_s = cmd.getIntegerTopic('heartbeat').subscribe(-1)
    ts_s = cmd.getDoubleTopic('timestamp').subscribe(float('nan'))

    # ---- publish what RoboRIO would (mock robot driving a circle) ----
    odom = inst.getTable('Robot/odom')
    odom_x = odom.getDoubleTopic('x').publish()
    odom_y = odom.getDoubleTopic('y').publish()
    odom_theta = odom.getDoubleTopic('theta').publish()
    odom_vx = odom.getDoubleTopic('vx').publish()
    odom_vy = odom.getDoubleTopic('vy').publish()
    odom_omega = odom.getDoubleTopic('omega').publish()

    imu = inst.getTable('Robot/imu')
    imu_yaw = imu.getDoubleTopic('yaw').publish()
    imu_yaw_rate = imu.getDoubleTopic('yaw_rate').publish()
    imu_ax = imu.getDoubleTopic('accel_x').publish()
    imu_ay = imu.getDoubleTopic('accel_y').publish()
    imu_az = imu.getDoubleTopic('accel_z').publish()

    # circle params: radius 2 m, angular rate 0.5 rad/s
    R, w = 2.0, 0.5
    t0 = time.monotonic()
    last_print = 0.0

    print('[fake_roborio] publishing mock odom/imu; printing Pi cmd 5 Hz')
    try:
        while True:
            t = time.monotonic() - t0
            theta = w * t

            odom_x.set(R * math.cos(theta) - R)   # start at origin
            odom_y.set(R * math.sin(theta))
            odom_theta.set(theta)
            odom_vx.set(-R * w * math.sin(theta))
            odom_vy.set(R * w * math.cos(theta))
            odom_omega.set(w)

            imu_yaw.set(theta)
            imu_yaw_rate.set(w)
            imu_ax.set(0.0)
            imu_ay.set(R * w * w)  # centripetal accel magnitude
            imu_az.set(9.81)       # flat ground: just gravity, no vertical motion

            now = time.monotonic()
            if now - last_print >= 0.2:
                last_print = now
                print(
                    f'[Pi->RoboRIO] vx={vx_s.get():+.3f}  vy={vy_s.get():+.3f}  '
                    f'omega={omega_s.get():+.3f}  hb={hb_s.get()}  '
                    f'ts={ts_s.get():.3f}'
                )

            time.sleep(0.02)  # 50 Hz odom publish
    except KeyboardInterrupt:
        print('\n[fake_roborio] stopping')
    finally:
        inst.stopServer()


if __name__ == '__main__':
    main()
