#!/usr/bin/env python3
"""fake_tag_publisher — publishes scripted TagDetection messages.

Mimics what nt_bridge will publish once it's wired to the Limelight.
Use this to test floor_v1_apriltag without a real Limelight + RoboRIO.

Scenario format (semicolon-separated frames):
  "<delay_s>:<tag_id>:<area>;..."

Example:
  "2:101:0.04; 5:102:0.05; 8:103:0.06"
  -> after 2s publish tag 101, after 5s tag 102, after 8s tag 103.

Times are absolute from node start. To represent 'no tag visible', use
tag_id = -1. Areas not specified default to 0.03.
"""

import rclpy
from rclpy.node import Node
from cart_elevator_msgs.msg import TagDetection


class FakeTagPublisher(Node):
    def __init__(self) -> None:
        super().__init__('fake_tag_publisher')

        self.declare_parameters(namespace='', parameters=[
            ('tag_topic', '/limelight/tag'),
            ('frame_id', 'limelight'),
            ('publish_rate_hz', 10.0),
            ('scenario', '2:101:0.04; 5:102:0.05'),
        ])
        gp = lambda n: self.get_parameter(n).value
        self.frame_id = gp('frame_id')
        self.rate = float(gp('publish_rate_hz'))

        self.pub = self.create_publisher(TagDetection, gp('tag_topic'), 10)
        self.events = self._parse(gp('scenario'))
        self.current_tag = -1
        self.current_area = 0.0
        self.t = 0.0
        self.dt = 1.0 / self.rate
        self.event_idx = 0
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'fake_tag_publisher: {len(self.events)} events scripted')

    def _parse(self, scenario: str):
        events = []
        for frag in scenario.split(';'):
            frag = frag.strip()
            if not frag:
                continue
            parts = [p.strip() for p in frag.split(':')]
            t = float(parts[0])
            tid = int(parts[1])
            area = float(parts[2]) if len(parts) > 2 else 0.03
            events.append((t, tid, area))
        events.sort()
        return events

    def _tick(self) -> None:
        # advance through events
        while (self.event_idx < len(self.events)
               and self.events[self.event_idx][0] <= self.t):
            _, tid, area = self.events[self.event_idx]
            self.current_tag = tid
            self.current_area = area
            self.event_idx += 1
            self.get_logger().info(f'tag visible: id={tid}, area={area:.3f}')

        msg = TagDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.tag_id = int(self.current_tag)
        msg.area = float(self.current_area)
        msg.tx_deg = 0.0
        msg.ty_deg = 0.0
        msg.has_pose = False
        self.pub.publish(msg)
        self.t += self.dt


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeTagPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
