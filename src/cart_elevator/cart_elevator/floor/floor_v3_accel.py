#!/usr/bin/env python3
"""floor_v3_accel — TERTIARY floor detector (software accel integration).

Double-integrates IMU accel_z to recover vertical velocity and position
during an elevator trip. At motion end, snaps the integrated position to
the nearest multiple of floor_height_m and updates the current floor.

Note: the IMU itself (Pigeon 2 via NT bridge, or D455) only publishes raw
linear_acceleration — neither hardware integrates accel to vertical
position. This node does the double-integration in software here on the Pi.

Compared to floor_v2_time, this DOESN'T assume a calibrated constant
elevator speed — it derives the trip distance from physics. But it suffers
from accel bias drift, so we estimate the bias during stationary periods
and subtract it during motion. Even so, accuracy degrades on long trips.

Bias estimation: average (accel_z - g) over the last bias_window_s seconds
of stationary data. Reset velocity + position to zero at the start of each
trip; this prevents inter-trip drift accumulation.

Stationarity gate: a sample is admitted to the bias estimator only when
the variance of (accel_z - g) over the last bias_var_window_n samples is
below bias_var_thr_mss2. This is independent of the bias magnitude — a
constant-but-large bias still has near-zero variance, so the estimator
keeps learning even at biases that would saturate a magnitude-threshold
gate. Replaces the older `|signed_dev_raw| < move_thr` gate, which broke
once the bias itself exceeded `move_thr` because every stationary sample
then looked like motion.

Optional correction input (`correction_topic`): when v4_fusion sees a
fresh AprilTag and judges the cart stationary, it publishes the
authoritative floor here. On receipt — *only if v3 itself is also
stationary* — we snap `current_floor` to the corrected value and back out
an implied bias error from the residual of the most recent trip
(Δz ≈ ½·b·T² ⇒ b ≈ 2·Δz/T²). The trip metadata used for this lives in
`last_trip_*`. v3 with no correction stream still works as the pure-IMU
baseline used in the bias sweep.
"""

import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from cart_elevator_msgs.msg import FloorEstimate

GRAVITY = 9.81


class FloorV3Accel(Node):
    def __init__(self) -> None:
        super().__init__('floor_v3_accel')

        self.declare_parameters(namespace='', parameters=[
            ('imu_topic', '/imu'),
            ('output_topic', '/elevator/floor_v3_accel'),
            ('starting_floor', 1),
            ('floor_height_m', 3.5),
            ('move_threshold_mss', 0.30),
            ('v_threshold_ms', 0.20),
            ('start_samples', 10),
            ('stop_samples', 100),
            ('bias_window_s', 5.0),
            ('bias_var_thr_mss2', 0.01),
            ('bias_var_window_n', 25),
            ('correction_topic', '/elevator/floor_v3_accel/correction'),
            ('correction_max_floor_delta', 2),
            ('publish_rate_hz', 5.0),
        ])
        gp = lambda n: self.get_parameter(n).value
        self.imu_topic = gp('imu_topic')
        self.output_topic = gp('output_topic')
        self.starting_floor = int(gp('starting_floor'))
        self.floor_h = float(gp('floor_height_m'))
        self.move_thr = float(gp('move_threshold_mss'))
        self.v_thr = float(gp('v_threshold_ms'))
        self.start_samples = int(gp('start_samples'))
        self.stop_samples = int(gp('stop_samples'))
        self.bias_window = float(gp('bias_window_s'))
        self.var_thr = float(gp('bias_var_thr_mss2'))
        self.var_window_n = int(gp('bias_var_window_n'))
        self.correction_max_delta = int(gp('correction_max_floor_delta'))
        publish_rate = float(gp('publish_rate_hz'))

        # State
        self.current_floor = self.starting_floor
        self.is_moving = False
        self.direction = 0
        self.active_streak = 0
        self.idle_streak = 0
        self.bias_z = 0.0
        self.bias_samples: deque = deque(maxlen=int(self.bias_window * 100))
        self.var_buf: deque = deque(maxlen=self.var_window_n)

        # Integrators (only valid during a trip)
        self.v_z = 0.0
        self.z = 0.0
        self.last_t = None

        # Most-recent trip metadata, for the residual-based bias correction
        # below. Set when a trip ends; consumed if a correction arrives soon
        # after. trip_start_t is set when a trip begins.
        self.trip_start_t: float | None = None
        self.last_trip_z = 0.0
        self.last_trip_duration_s = 0.0

        self.create_subscription(Imu, self.imu_topic, self._on_imu, 50)
        self.create_subscription(
            FloorEstimate, gp('correction_topic'), self._on_correction, 10)
        self.pub = self.create_publisher(FloorEstimate, self.output_topic, 10)
        self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f'floor_v3_accel ready (start={self.starting_floor}, '
            f'floor_h={self.floor_h} m)')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_imu(self, msg: Imu) -> None:
        now = self._now_s()
        signed_dev_raw = msg.linear_acceleration.z - GRAVITY
        self.var_buf.append(signed_dev_raw)

        # Variance-gated bias estimator. A magnitude gate (the old approach)
        # rejects stationary samples once `bias > move_thr`, freezing the
        # estimate at 0 and causing linear position drift. Variance over a
        # short rolling window is independent of the bias magnitude — a
        # constant-but-large bias still has ~zero variance, so we keep
        # admitting samples and the running mean converges to the true bias.
        if (not self.is_moving
                and len(self.var_buf) >= self.var_window_n
                and statistics.pvariance(self.var_buf) < self.var_thr):
            self.bias_samples.append(signed_dev_raw)
            if self.bias_samples:
                self.bias_z = sum(self.bias_samples) / len(self.bias_samples)

        # bias-corrected vertical acceleration
        a_z = signed_dev_raw - self.bias_z
        active_now = abs(a_z) > self.move_thr

        if not self.is_moving:
            if active_now:
                self.active_streak += 1
                if self.active_streak >= self.start_samples:
                    self.is_moving = True
                    self.direction = 1 if a_z > 0 else -1
                    self.active_streak = 0
                    self.idle_streak = 0
                    self.v_z = 0.0
                    self.z = 0.0
                    self.last_t = now
                    self.trip_start_t = now
                    self.get_logger().info(
                        f'trip start: dir={"UP" if self.direction>0 else "DOWN"}, '
                        f'bias={self.bias_z:.4f} m/s^2')
            else:
                self.active_streak = 0
        else:
            # Integrate
            if self.last_t is not None:
                dt = now - self.last_t
                if 0.0 < dt < 0.5:  # guard against gaps / startup
                    self.v_z += a_z * dt
                    self.z += self.v_z * dt
            self.last_t = now

            # Use integrated velocity (not raw accel) for stop detection.
            # During cruise a_z ~ 0 but v_z stays at cruise speed, so this
            # bridges the cruise phase that would otherwise look like a stop.
            stopped_now = abs(self.v_z) < self.v_thr
            if stopped_now:
                self.idle_streak += 1
                if self.idle_streak >= self.stop_samples:
                    # On stop: snap integrated z to nearest floor multiple.
                    # Sign of z is positive when we moved up.
                    floors_changed = int(round(self.z / self.floor_h))
                    if floors_changed == 0 and self.direction != 0:
                        floors_changed = self.direction  # at least 1 floor if trip happened
                    self.current_floor += floors_changed
                    self.get_logger().info(
                        f'trip end: integrated z={self.z:.2f}m, '
                        f'floors_changed={floors_changed}, now at {self.current_floor}')
                    # Preserve the trip's metadata for residual-based bias
                    # correction if a tag-driven correction arrives soon after.
                    self.last_trip_z = self.z
                    self.last_trip_duration_s = (
                        now - self.trip_start_t if self.trip_start_t else 0.0)
                    self.is_moving = False
                    self.direction = 0
                    self.idle_streak = 0
                    self.v_z = 0.0
                    self.z = 0.0
                    self.last_t = None
                    self.trip_start_t = None
            else:
                self.idle_streak = 0

    def _on_correction(self, msg: FloorEstimate) -> None:
        # Tag-driven correction: snap current_floor and back out an implied
        # bias error from the last trip's residual. Two safety gates:
        # (1) only correct when v3 itself is stationary (don't yank mid-trip),
        # (2) reject corrections whose floor delta exceeds correction_max_delta
        # — those are more likely a stale or wrongly-routed tag than a real
        # bias drift of that magnitude.
        if self.is_moving:
            return
        delta = self.current_floor - int(msg.floor)
        if abs(delta) > self.correction_max_delta:
            self.get_logger().warning(
                f'rejecting correction: |delta|={abs(delta)} > '
                f'{self.correction_max_delta} (v3={self.current_floor}, '
                f'tag={int(msg.floor)})')
            return
        if delta != 0 and self.last_trip_duration_s > 0.5:
            # Δz overshoot = delta * floor_h. For constant bias over trip
            # duration T: Δz ≈ ½·b·T² ⇒ b ≈ 2·Δz/T². Add directly (blend=1):
            # one good correction should fully heal the bias, subsequent
            # corrections refine.
            residual_z = delta * self.floor_h
            implied_bias = 2.0 * residual_z / (self.last_trip_duration_s ** 2)
            old_bias = self.bias_z
            self.bias_z += implied_bias
            # Re-seed the bias_samples buffer with the corrected estimate so
            # the running mean doesn't immediately drag bias_z back.
            self.bias_samples.clear()
            self.bias_samples.append(self.bias_z)
            self.get_logger().info(
                f'correction: floor {self.current_floor} -> {int(msg.floor)} '
                f'(delta={delta}), bias {old_bias:.4f} -> {self.bias_z:.4f} '
                f'(from trip T={self.last_trip_duration_s:.2f}s)')
        else:
            self.get_logger().info(
                f'correction: floor {self.current_floor} -> {int(msg.floor)} '
                f'(delta=0 or no recent trip, position only)')
        self.current_floor = int(msg.floor)

    def _publish(self) -> None:
        out = FloorEstimate()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.source = FloorEstimate.SOURCE_ACCEL
        out.moving = self.is_moving
        out.direction = int(self.direction)

        if self.is_moving:
            # rolling estimate from integrator
            est_floors = self.z / self.floor_h
            out.floor = self.current_floor + int(round(est_floors))
            # confidence falls off with trip duration (drift)
            out.confidence = 0.40
        else:
            out.floor = self.current_floor
            out.confidence = 0.60

        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FloorV3Accel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
