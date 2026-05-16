#!/usr/bin/env python3
"""floor_v4_fusion — fuses v1 (AprilTag), v2 (time), v3 (accel) into one estimate.

Strategy: priority + freshness, NOT a Kalman filter. AprilTag is treated as
ground-truth-when-fresh; time and accel are dead-reckoning fallbacks that
get used only when the tag estimate is stale. This is intentionally simple
so it's explainable on a science-fair poster.

Decision rule per output cycle:
  1. If v1 sample exists and age < tag_max_age_s   -> publish v1 (boosted confidence)
  2. Else if v2 sample exists and age < time_max_age_s -> publish v2
  3. Else if v3 sample exists and age < accel_max_age_s -> publish v3
  4. Else publish last_known with degraded confidence
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
            ('tag_max_age_s', 1.5),
            ('time_max_age_s', 5.0),
            ('accel_max_age_s', 5.0),
            ('publish_rate_hz', 5.0),
        ])
        gp = lambda n: self.get_parameter(n).value
        self.tag_max_age = float(gp('tag_max_age_s'))
        self.time_max_age = float(gp('time_max_age_s'))
        self.accel_max_age = float(gp('accel_max_age_s'))
        publish_rate = float(gp('publish_rate_hz'))

        self.last_v1: FloorEstimate | None = None
        self.last_v2: FloorEstimate | None = None
        self.last_v3: FloorEstimate | None = None
        self.last_fusion: FloorEstimate | None = None

        self.create_subscription(FloorEstimate, gp('v1_topic'), self._on_v1, 10)
        self.create_subscription(FloorEstimate, gp('v2_topic'), self._on_v2, 10)
        self.create_subscription(FloorEstimate, gp('v3_topic'), self._on_v3, 10)
        self.pub = self.create_publisher(FloorEstimate, gp('output_topic'), 10)
        self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info('floor_v4_fusion ready')

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

    def _publish(self) -> None:
        out = FloorEstimate()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.source = FloorEstimate.SOURCE_FUSION

        if self._fresh(self.last_v1, self.tag_max_age):
            chosen = self.last_v1
            out.confidence = chosen.confidence  # already high
        elif self._fresh(self.last_v2, self.time_max_age):
            chosen = self.last_v2
            out.confidence = chosen.confidence
        elif self._fresh(self.last_v3, self.accel_max_age):
            chosen = self.last_v3
            out.confidence = chosen.confidence
        elif self.last_fusion is not None:
            # All sources stale — degrade confidence over time
            age = self._now_s() - self._stamp_s(self.last_fusion)
            out.floor = self.last_fusion.floor
            out.moving = False
            out.direction = 0
            out.confidence = max(0.0, self.last_fusion.confidence - 0.05 * age)
            self.last_fusion = out
            self.pub.publish(out)
            return
        else:
            # nothing at all yet
            return

        out.floor = chosen.floor
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
