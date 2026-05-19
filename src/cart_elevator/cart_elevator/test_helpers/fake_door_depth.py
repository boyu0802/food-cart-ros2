#!/usr/bin/env python3
"""fake_door_depth — synthesizes a depth Image that exercises door_state_detector.

Drives a scripted sequence: CLOSED -> OPENING -> OPEN -> CLOSING -> CLOSED
so the detector node can be validated without the real D455 / real
elevator. Cycle period defaults to 12 s (3 s per phase).

Image is 16UC1 (D455 native format) so the detector parses it the same
way it parses real frames.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


WIDTH = 640
HEIGHT = 480

ROI_X = 200
ROI_Y = 120
ROI_W = 240
ROI_H = 160


def _make_depth_frame(phase_s: float,
                      cycle_s: float,
                      closed_depth_mm: int,
                      open_depth_mm: int) -> np.ndarray:
    """Return a HxW uint16 depth frame in mm."""
    frame = np.full((HEIGHT, WIDTH), open_depth_mm, dtype=np.uint16)

    # Phase boundaries inside the cycle:
    q = cycle_s / 4.0
    if phase_s < q:
        # CLOSED: door fully covers the ROI columns at closed_depth.
        frame[ROI_Y:ROI_Y + ROI_H, ROI_X:ROI_X + ROI_W] = closed_depth_mm
    elif phase_s < 2 * q:
        # OPENING: door panel retracting from the right; fraction of
        # ROI width still covered drops from 1 -> 0 over q seconds.
        retract = (phase_s - q) / q   # 0..1
        cover_w = int(ROI_W * (1.0 - retract))
        if cover_w > 0:
            frame[ROI_Y:ROI_Y + ROI_H, ROI_X:ROI_X + cover_w] = closed_depth_mm
        # The uncovered part stays at open_depth (already set above).
    elif phase_s < 3 * q:
        # OPEN: whole ROI is the open-cab depth (already set).
        pass
    else:
        # CLOSING: panel sliding back in from the right.
        progress = (phase_s - 3 * q) / q   # 0..1
        cover_w = int(ROI_W * progress)
        if cover_w > 0:
            frame[ROI_Y:ROI_Y + ROI_H, ROI_X:ROI_X + cover_w] = closed_depth_mm
    return frame


class FakeDoorDepth(Node):
    def __init__(self) -> None:
        super().__init__('fake_door_depth')

        self.declare_parameters(namespace='', parameters=[
            ('topic', '/camera/camera/depth/image_rect_raw'),
            ('rate_hz', 15.0),
            ('cycle_s', 12.0),
            ('closed_depth_mm', 350),   # 0.35 m — typical D455-to-door at standoff
            ('open_depth_mm', 2200),    # 2.2 m — into the cab
        ])
        gp = lambda n: self.get_parameter(n).value

        self.cycle_s = float(gp('cycle_s'))
        self.closed_mm = int(gp('closed_depth_mm'))
        self.open_mm = int(gp('open_depth_mm'))

        self.pub = self.create_publisher(Image, gp('topic'), 5)
        period = 1.0 / float(gp('rate_hz'))
        self.create_timer(period, self._tick)
        self.t0 = self._now_s()

        self.get_logger().info(
            f'fake_door_depth ready (cycle={self.cycle_s}s, '
            f'closed={self.closed_mm}mm, open={self.open_mm}mm)')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        phase = math.fmod(self._now_s() - self.t0, self.cycle_s)
        frame = _make_depth_frame(phase, self.cycle_s, self.closed_mm, self.open_mm)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_depth_optical_frame'
        msg.height = HEIGHT
        msg.width = WIDTH
        msg.encoding = '16UC1'
        msg.is_bigendian = 0
        msg.step = WIDTH * 2
        msg.data = frame.tobytes()
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeDoorDepth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
