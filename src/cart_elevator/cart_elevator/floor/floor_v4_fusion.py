#!/usr/bin/env python3
"""floor_v4_fusion — fuses v1 (AprilTag), v2 (time), v3 (accel) into one estimate.

This is NOT a Kalman filter. The fusion is a small explicit decision tree
that's easy to defend on a science-fair poster:

  1. Build a candidate list from sources that are (a) fresh and (b) whose
     floor lies in [1, n_floors]. The range clamp is what stops v2's
     saturated +8 estimate from polluting the output once the IMU bias
     pushes it past the building.
  2. If v1 (AprilTag) is a candidate, use it. AprilTag IDs are
     Hamming-coded, so an ID misread is effectively impossible; the only
     failure mode is stale reads, which the freshness check already
     filtered out. When v1 wins AND v3 is fresh AND v3 is stationary, we
     also publish v1's floor back to v3 as a correction (see B3 in
     floor_v3_accel — v3 will snap and re-derive its bias).
  3. Else if both v2 and v3 are candidates and disagree by >1 floor, pick
     whichever is closer to last_fusion (fallback: starting_floor).
     Heuristic, but better than fixed priority when one detector has
     visibly drifted.
  4. Else if both are candidates and agree (within 1 floor), prefer the
     one with higher confidence — ties go to v3 (physics over heuristic).
  5. Else fall through to whichever candidate exists.

If no candidate exists, we coast on last_fusion with linearly-decaying
confidence until something fresh shows up.
"""

import rclpy
from rclpy.node import Node
from cart_elevator_msgs.msg import FloorEstimate


class FloorV4Fusion(Node):
    def __init__(self) -> None:
        super().__init__('floor_v4_fusion')

        self.declare_parameters(namespace='', parameters=[
            ('v1_topic', '/elevator/floor_v1_apriltag'),
            ('v2_topic', '/elevator/floor_v2_time'),
            ('v3_topic', '/elevator/floor_v3_accel'),
            ('output_topic', '/elevator/floor'),
            ('correction_out_topic', '/elevator/floor_v3_accel/correction'),
            ('tag_max_age_s', 1.5),
            ('time_max_age_s', 5.0),
            ('accel_max_age_s', 5.0),
            ('publish_rate_hz', 5.0),
            ('n_floors', 5),
            ('starting_floor', 1),
        ])
        gp = lambda n: self.get_parameter(n).value
        self.tag_max_age = float(gp('tag_max_age_s'))
        self.time_max_age = float(gp('time_max_age_s'))
        self.accel_max_age = float(gp('accel_max_age_s'))
        self.n_floors = int(gp('n_floors'))
        self.starting_floor = int(gp('starting_floor'))
        publish_rate = float(gp('publish_rate_hz'))

        self.last_v1: FloorEstimate | None = None
        self.last_v2: FloorEstimate | None = None
        self.last_v3: FloorEstimate | None = None
        self.last_fusion: FloorEstimate | None = None

        self.create_subscription(FloorEstimate, gp('v1_topic'), self._on_v1, 10)
        self.create_subscription(FloorEstimate, gp('v2_topic'), self._on_v2, 10)
        self.create_subscription(FloorEstimate, gp('v3_topic'), self._on_v3, 10)
        self.pub = self.create_publisher(FloorEstimate, gp('output_topic'), 10)
        self.correction_pub = self.create_publisher(
            FloorEstimate, gp('correction_out_topic'), 10)
        self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f'floor_v4_fusion ready (n_floors={self.n_floors}, '
            f'start={self.starting_floor})')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_s(msg: FloorEstimate) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_v1(self, msg: FloorEstimate) -> None: self.last_v1 = msg
    def _on_v2(self, msg: FloorEstimate) -> None: self.last_v2 = msg
    def _on_v3(self, msg: FloorEstimate) -> None: self.last_v3 = msg

    def _fresh(self, msg: FloorEstimate | None, max_age: float) -> bool:
        if msg is None:
            return False
        return (self._now_s() - self._stamp_s(msg)) < max_age

    def _in_range(self, floor: int) -> bool:
        return 1 <= int(floor) <= self.n_floors

    def _publish(self) -> None:
        out = FloorEstimate()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.source = FloorEstimate.SOURCE_FUSION

        # 1. Candidate list: fresh AND in-building.
        candidates: dict[str, FloorEstimate] = {}
        if self._fresh(self.last_v1, self.tag_max_age) and self._in_range(self.last_v1.floor):
            candidates['v1'] = self.last_v1
        if self._fresh(self.last_v2, self.time_max_age) and self._in_range(self.last_v2.floor):
            candidates['v2'] = self.last_v2
        if self._fresh(self.last_v3, self.accel_max_age) and self._in_range(self.last_v3.floor):
            candidates['v3'] = self.last_v3

        # No fresh in-range source — coast on last fusion with decaying conf.
        if not candidates:
            if self.last_fusion is not None:
                age = self._now_s() - self._stamp_s(self.last_fusion)
                out.floor = self.last_fusion.floor
                out.moving = False
                out.direction = 0
                out.confidence = max(0.0, self.last_fusion.confidence - 0.05 * age)
                self.last_fusion = out
                self.pub.publish(out)
            return

        chosen: FloorEstimate

        # 2. Tag wins. Also push it to v3 as a correction when v3 is
        # stationary — v3 will snap its floor + back-derive its bias.
        if 'v1' in candidates:
            chosen = candidates['v1']
            v3 = candidates.get('v3')
            if v3 is not None and not v3.moving:
                corr = FloorEstimate()
                corr.header.stamp = self.get_clock().now().to_msg()
                corr.header.frame_id = 'map'
                corr.source = FloorEstimate.SOURCE_APRILTAG
                corr.floor = chosen.floor
                corr.confidence = chosen.confidence
                corr.moving = False
                corr.direction = 0
                self.correction_pub.publish(corr)

        # 3-4. v2 and v3 both present.
        elif 'v2' in candidates and 'v3' in candidates:
            v2_msg = candidates['v2']
            v3_msg = candidates['v3']
            if abs(int(v2_msg.floor) - int(v3_msg.floor)) > 1:
                # Disagreement: pick whichever is closer to last fusion.
                ref = (int(self.last_fusion.floor)
                       if self.last_fusion is not None else self.starting_floor)
                chosen = (v2_msg if abs(int(v2_msg.floor) - ref)
                          < abs(int(v3_msg.floor) - ref) else v3_msg)
            else:
                # Agreement-ish: prefer higher confidence; tie-break v3
                # (physics > heuristic).
                chosen = (v3_msg if v3_msg.confidence >= v2_msg.confidence
                          else v2_msg)

        # 5. Single non-tag source.
        elif 'v2' in candidates:
            chosen = candidates['v2']
        else:
            chosen = candidates['v3']

        out.floor = chosen.floor
        out.confidence = chosen.confidence
        out.moving = chosen.moving
        out.direction = chosen.direction
        self.last_fusion = out
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FloorV4Fusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
