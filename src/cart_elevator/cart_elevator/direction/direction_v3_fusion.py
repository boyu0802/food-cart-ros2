#!/usr/bin/env python3
"""direction_v3_fusion — fuses v1 (frame-diff) and v2 (optical flow)
into one ElevatorDirection on /elevator/direction.

Strategy: agreement-weighted vote, NOT a Kalman filter. Same spirit as
floor_v4_fusion.
  - Both fresh + agree            -> publish that direction, conf = max
  - Both fresh + disagree         -> publish UNKNOWN, conf = 0.5 *
                                     min(conf_v1, conf_v2)
  - Only one fresh                -> pass through, conf -= 0.1
  - Neither fresh                 -> publish UNKNOWN, conf = 0.0
"""

import rclpy
from rclpy.node import Node
from cart_elevator_msgs.msg import ElevatorDirection


class DirectionV3Fusion(Node):
    def __init__(self) -> None:
        super().__init__('direction_v3_fusion')

        self.declare_parameters(namespace='', parameters=[
            ('v1_topic', '/elevator/direction_v1_digit'),
            ('v2_topic', '/elevator/direction_v2_flow'),
            ('output_topic', '/elevator/direction'),
            ('max_age_s', 1.0),
            ('publish_rate_hz', 10.0),
        ])
        gp = lambda n: self.get_parameter(n).value
        self.max_age = float(gp('max_age_s'))

        self.last_v1: ElevatorDirection | None = None
        self.last_v2: ElevatorDirection | None = None

        self.create_subscription(
            ElevatorDirection, gp('v1_topic'), self._on_v1, 10)
        self.create_subscription(
            ElevatorDirection, gp('v2_topic'), self._on_v2, 10)
        self.pub = self.create_publisher(
            ElevatorDirection, gp('output_topic'), 10)

        self.create_timer(1.0 / float(gp('publish_rate_hz')), self._publish)
        self.get_logger().info('direction_v3_fusion ready')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_s(msg: ElevatorDirection) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_v1(self, msg: ElevatorDirection) -> None: self.last_v1 = msg
    def _on_v2(self, msg: ElevatorDirection) -> None: self.last_v2 = msg

    def _fresh(self, msg: ElevatorDirection | None) -> bool:
        if msg is None:
            return False
        return (self._now_s() - self._stamp_s(msg)) < self.max_age

    def _publish(self) -> None:
        out = ElevatorDirection()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'camera_color_optical_frame'
        out.source = ElevatorDirection.SOURCE_FUSION

        v1_ok = self._fresh(self.last_v1)
        v2_ok = self._fresh(self.last_v2)

        if v1_ok and v2_ok:
            if self.last_v1.direction == self.last_v2.direction:
                out.direction = self.last_v1.direction
                out.confidence = max(
                    self.last_v1.confidence, self.last_v2.confidence)
            else:
                out.direction = ElevatorDirection.DIRECTION_UNKNOWN
                out.confidence = 0.5 * min(
                    self.last_v1.confidence, self.last_v2.confidence)
        elif v1_ok:
            out.direction = self.last_v1.direction
            out.confidence = max(0.0, self.last_v1.confidence - 0.1)
        elif v2_ok:
            out.direction = self.last_v2.direction
            out.confidence = max(0.0, self.last_v2.confidence - 0.1)
        else:
            out.direction = ElevatorDirection.DIRECTION_UNKNOWN
            out.confidence = 0.0

        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DirectionV3Fusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
