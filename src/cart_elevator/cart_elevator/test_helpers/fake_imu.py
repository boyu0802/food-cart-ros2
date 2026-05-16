#!/usr/bin/env python3
"""fake_elevator_imu — publishes a synthetic /imu stream for an elevator trip.

Models a trip as four phases:
  ACCEL   — constant accel a for t_accel seconds (sign = direction)
  CRUISE  — constant velocity v = a * t_accel
  DECEL   — constant accel -a for t_accel seconds
  IDLE    — stationary

Cruise time is derived from total distance = num_floors * floor_height_m.
Accel/decel each contribute 0.5 * a * t_accel^2 of distance.

Use this to drive floor_v2_time, floor_v3_accel, and floor_v4_fusion
without a real cart or RoboRIO. Multiple back-to-back trips can be
scripted via the 'scenario' parameter.

Scenario string format (comma-separated trips):
  "<floors>,<floors>,..."   e.g. "+3,-1,+2"
where '+' = up, '-' = down. A 'p' character inserts a pause between trips,
e.g. "+1,p5,+1" = up 1 floor, 5s pause, up 1 floor.
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


GRAVITY = 9.81


class FakeElevatorImu(Node):
    def __init__(self) -> None:
        super().__init__('fake_elevator_imu')

        self.declare_parameters(namespace='', parameters=[
            ('imu_topic', '/imu'),
            ('frame_id', 'imu_link'),
            ('publish_rate_hz', 100.0),
            ('accel_mss', 1.0),
            ('floor_height_m', 3.5),
            ('pre_idle_s', 2.0),
            ('inter_trip_idle_s', 3.0),
            ('post_idle_s', 5.0),
            ('noise_std_mss', 0.02),
            ('bias_mss', 0.0),
            ('scenario', '+3'),  # default: up 3 floors
            ('loop', False),
        ])
        gp = lambda n: self.get_parameter(n).value

        self.frame_id = gp('frame_id')
        self.rate = float(gp('publish_rate_hz'))
        self.a = float(gp('accel_mss'))
        self.floor_h = float(gp('floor_height_m'))
        self.pre_idle = float(gp('pre_idle_s'))
        self.inter_idle = float(gp('inter_trip_idle_s'))
        self.post_idle = float(gp('post_idle_s'))
        self.noise = float(gp('noise_std_mss'))
        self.bias = float(gp('bias_mss'))
        self.loop = bool(gp('loop'))

        self.pub = self.create_publisher(Imu, gp('imu_topic'), 50)
        self.schedule = self._build_schedule(gp('scenario'))
        self.dt = 1.0 / self.rate
        self.t = 0.0
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'fake_elevator_imu: scenario={gp("scenario")}, '
            f'total={self.schedule[-1][0]:.1f}s')

    def _build_schedule(self, scenario: str):
        """Return list of (end_time, accel_z) keyframes.
        Each tuple says 'until this time, publish accel_z = a + GRAVITY'.
        accel_z stored here is the *true* vertical acceleration excluding gravity
        (so 0.0 = stationary). We add GRAVITY in _tick().
        """
        schedule = []
        t = 0.0
        # initial pause
        t += self.pre_idle
        schedule.append((t, 0.0))

        items = [s.strip() for s in scenario.split(',') if s.strip()]
        first = True
        for it in items:
            if not first:
                t += self.inter_idle
                schedule.append((t, 0.0))
            first = False

            if it.startswith('p'):
                # explicit pause: 'p5' = 5s pause
                pause_s = float(it[1:])
                t += pause_s
                schedule.append((t, 0.0))
                continue

            sign = +1 if it.startswith('+') else -1
            n = int(it[1:]) if it[0] in '+-' else int(it)
            distance = n * self.floor_h
            t_accel = math.sqrt(abs(distance) / self.a) if distance < (
                self.a * (1.0) ** 2) else None
            # Use trapezoidal profile: accel for t_accel, cruise at v=a*t_accel
            # We pick t_accel so accel+decel = 0.5*a*t_accel^2 * 2 <= |distance|
            # then cruise fills the rest. Cap t_accel at 1.5s for realism.
            t_accel = min(1.5, math.sqrt(abs(distance) / self.a))
            v_cruise = self.a * t_accel
            d_accel = 0.5 * self.a * t_accel * t_accel
            d_cruise = abs(distance) - 2 * d_accel
            t_cruise = max(0.0, d_cruise / v_cruise)

            # Phase 1: accel
            t += t_accel
            schedule.append((t, sign * self.a))
            # Phase 2: cruise
            t += t_cruise
            schedule.append((t, 0.0))
            # Phase 3: decel
            t += t_accel
            schedule.append((t, -sign * self.a))

        # final pause
        t += self.post_idle
        schedule.append((t, 0.0))
        return schedule

    def _accel_at(self, t: float) -> float:
        for end_t, a in self.schedule:
            if t < end_t:
                return a
        return 0.0  # past end

    def _tick(self) -> None:
        if self.t > self.schedule[-1][0]:
            if self.loop:
                self.t = 0.0
            else:
                # keep publishing idle so subscribers see we stayed stopped
                pass

        # synthetic accel_z = elevator accel + gravity + bias + noise
        import random
        a_true = self._accel_at(self.t)
        noise = random.gauss(0.0, self.noise) if self.noise > 0 else 0.0
        a_z = a_true + GRAVITY + self.bias + noise

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = a_z
        # mark x,y as unknown, z as measured
        LARGE = 1e6
        msg.linear_acceleration_covariance = [
            LARGE, 0.0, 0.0,
            0.0, LARGE, 0.0,
            0.0, 0.0, 0.05,
        ]
        # mark angular vel & orientation as unknown for this fake
        msg.angular_velocity_covariance = [LARGE, 0.0, 0.0, 0.0, LARGE, 0.0, 0.0, 0.0, LARGE]
        msg.orientation_covariance = [LARGE, 0.0, 0.0, 0.0, LARGE, 0.0, 0.0, 0.0, LARGE]
        self.pub.publish(msg)
        self.t += self.dt


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeElevatorImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
