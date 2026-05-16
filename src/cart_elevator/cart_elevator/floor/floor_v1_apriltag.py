#!/usr/bin/env python3
"""floor_v1_apriltag — PRIMARY floor detector.

Subscribes to /limelight/tag (cart_elevator_msgs/TagDetection) — the Limelight
republishes one AprilTag detection at a time via nt_bridge.

Tags with id >= floor_tag_id_offset are interpreted as floor markers:
    floor = tag_id - floor_tag_id_offset
e.g. offset=100 -> tag 101 = floor 1, tag 102 = floor 2, ...

Publishes FloorEstimate on /elevator/floor_v1_apriltag every time we see a
valid floor tag. High confidence (the tag *is* the ground truth).
"""

import rclpy
from rclpy.node import Node
from cart_elevator_msgs.msg import TagDetection, FloorEstimate


class FloorV1AprilTag(Node):
    def __init__(self) -> None:
        super().__init__('floor_v1_apriltag')

        self.declare_parameters(namespace='', parameters=[
            ('tag_topic', '/limelight/tag'),
            ('output_topic', '/elevator/floor_v1_apriltag'),
            ('floor_tag_id_offset', 100),
            ('min_area', 0.005),      # ignore tags <0.5% of image (too far)
            ('full_conf_area', 0.05), # area at which we report ceiling confidence
            ('confidence_floor', 0.70),
            ('confidence_ceil', 0.95),
        ])
        gp = lambda n: self.get_parameter(n).value
        self.offset = int(gp('floor_tag_id_offset'))
        self.min_area = float(gp('min_area'))
        self.full_area = float(gp('full_conf_area'))
        self.cf = float(gp('confidence_floor'))
        self.cc = float(gp('confidence_ceil'))

        self.create_subscription(TagDetection, gp('tag_topic'), self._on_tag, 10)
        self.pub = self.create_publisher(FloorEstimate, gp('output_topic'), 10)
        self.get_logger().info(
            f'floor_v1_apriltag ready (offset={self.offset}, min_area={self.min_area})')

    def _on_tag(self, msg: TagDetection) -> None:
        if msg.tag_id < self.offset:
            return
        if msg.area < self.min_area:
            return

        floor = int(msg.tag_id - self.offset)
        conf = self.cf + (self.cc - self.cf) * min(1.0, msg.area / self.full_area)

        out = FloorEstimate()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.floor = floor
        out.confidence = conf
        out.source = FloorEstimate.SOURCE_APRILTAG
        out.moving = False  # tag is at a stationary floor
        out.direction = 0
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FloorV1AprilTag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
