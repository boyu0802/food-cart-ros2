#!/usr/bin/env python3
"""floor_v2_time — SECONDARY floor detector.

Detects elevator trip start/stop from IMU accel_z. While moving, estimates
floor change from elapsed time * calibrated elevator_speed_m_s / floor_height_m.

Assumptions:
- IMU is mounted upright; linear_acceleration.z is the world-vertical accel
  (= gravity + elevator accel).
- Cart is stationary relative to the elevator car (i.e., we're inside it).
- Elevator speed is roughly constant during the cruise phase. Trip duration
  approximates floors_traveled * floor_height / speed for trips that include
  at least some cruise (>1 floor).

Output: FloorEstimate on /elevator/floor_v2_time. Confidence ~0.6 while
moving (drifting estimate), ~0.8 when stationary at a snapped floor.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from cart_elevator_msgs.msg import FloorEstimate

GRAVITY = 9.81


class FloorV2Time(Node):
    def __init__(self) -> None:
        super().__init__('floor_v2_time')

        self.declare_parameters(namespace='', parameters=[
            ('imu_topic', '/imu'),
            ('output_topic', '/elevator/floor_v2_time'),
            ('starting_floor', 1),
            ('elevator_speed_m_s', 1.5),   # typical commercial elevator
            ('ramp_time_s', 1.5),          # accel & decel each take ~this long
            ('floor_height_m', 3.5),       # typical floor-to-floor spacing
            ('move_threshold_mss', 0.30),  # |accel_z - g| above this = "active"
            ('start_samples', 10),         # ~0.1s at 100Hz
            ('stop_samples', 100),         # ~1.0s at 100Hz
            ('publish_rate_hz', 5.0),
        ])
        gp = lambda n: self.get_parameter(n).value
        self.imu_topic = gp('imu_topic')
        self.output_topic = gp('output_topic')
        self.starting_floor = int(gp('starting_floor'))
        self.speed = float(gp('elevator_speed_m_s'))
        self.ramp_time = float(gp('ramp_time_s'))
        self.floor_h = float(gp('floor_height_m'))
        self.move_thr = float(gp('move_threshold_mss'))
        self.start_samples = int(gp('start_samples'))
        self.stop_samples = int(gp('stop_samples'))
        publish_rate = float(gp('publish_rate_hz'))

        # State
        self.current_floor = self.starting_floor
        self.is_moving = False
        self.direction = 0          # -1 / 0 / +1
        self.trip_start_t = 0.0
        self.active_streak = 0
        self.idle_streak = 0
        self.last_initial_signed_dev = 0.0  # for direction inference
        # Trip is only considered complete after we've observed an accel
        # pulse opposite to the start direction (i.e. the decel phase).
        # Without this, cruise (a_z ~ 0) gets mistaken for trip-end.
        self.saw_opposite_pulse = False

        self.create_subscription(Imu, self.imu_topic, self._on_imu, 50)
        self.pub = self.create_publisher(FloorEstimate, self.output_topic, 10)
        self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f'floor_v2_time ready (start={self.starting_floor}, '
            f'speed={self.speed} m/s, floor_h={self.floor_h} m)')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_imu(self, msg: Imu) -> None:
        signed_dev = msg.linear_acceleration.z - GRAVITY
        active_now = abs(signed_dev) > self.move_thr

        if not self.is_moving:
            if active_now:
                self.active_streak += 1
                if self.active_streak >= self.start_samples:
                    self.is_moving = True
                    self.trip_start_t = self._now_s()
                    self.direction = 1 if signed_dev > 0 else -1
                    self.active_streak = 0
                    self.idle_streak = 0
                    self.saw_opposite_pulse = False
                    self.get_logger().info(
                        f'trip start: dir={"UP" if self.direction>0 else "DOWN"}')
            else:
                self.active_streak = 0
        else:
            # currently moving
            if not active_now:
                self.idle_streak += 1
                if self.idle_streak >= self.stop_samples and self.saw_opposite_pulse:
                    duration = self._now_s() - self.trip_start_t - (
                        self.stop_samples / 100.0)  # subtract the idle window
                    duration = max(0.0, duration)
                    distance = self.speed * max(0.0, duration - self.ramp_time)
                    floors_changed = max(1, int(round(distance / self.floor_h)))
                    self.current_floor += self.direction * floors_changed
                    self.get_logger().info(
                        f'trip end: duration={duration:.2f}s, '
                        f'distance={distance:.2f}m, floors={floors_changed}, '
                        f'now at {self.current_floor}')
                    self.is_moving = False
                    self.direction = 0
                    self.idle_streak = 0
                    self.saw_opposite_pulse = False
            else:
                self.idle_streak = 0
                # Opposite-sign active pulse = decel phase observed.
                if signed_dev * self.direction < 0:
                    self.saw_opposite_pulse = True

    def _publish(self) -> None:
        out = FloorEstimate()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.source = FloorEstimate.SOURCE_TIME
        out.moving = self.is_moving
        out.direction = int(self.direction)

        if self.is_moving:
            # rolling estimate while in motion
            elapsed = self._now_s() - self.trip_start_t
            est_floors = elapsed * self.speed / self.floor_h
            out.floor = self.current_floor + int(round(self.direction * est_floors))
            out.confidence = 0.55
        else:
            out.floor = self.current_floor
            out.confidence = 0.80

        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FloorV2Time()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
