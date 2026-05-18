#!/usr/bin/env python3
"""fake_direction_image — synthesizes a /image stream of a fake elevator
display so we can blind-test direction_v1/v2 without real video.

What it generates: a small black image with a single bright "block" that
marches up or down every frame, wrapping around. Direction is scripted
by the same scenario syntax as fake_elevator_imu:

  "+3,p5,-1"   -> march up for 3 floors-worth of time, pause 5s, down 1 floor

Floor count is fake (we don't simulate physics here); we just want the
direction sign to switch at scripted times so the detectors are
verifiable.

This is intentionally a stand-in for real elevator-display video. Once
we have actual recordings we'll feed those through a video-replay node
instead.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class FakeDirectionImage(Node):
    def __init__(self) -> None:
        super().__init__('fake_direction_image')

        self.declare_parameters(namespace='', parameters=[
            ('image_topic', '/image'),
            ('frame_id', 'camera_color_optical_frame'),
            ('publish_rate_hz', 30.0),
            ('image_w', 320),
            ('image_h', 240),
            # Block pixels — bright square that marches.
            ('block_size_px', 20),
            ('block_speed_px_per_frame', 4.0),
            # Per-trip seconds. Each "floor" in scenario string = this long.
            ('seconds_per_floor', 6.0),
            ('idle_pause_s', 3.0),
            ('scenario', '+3'),
            ('loop', False),
        ])
        gp = lambda n: self.get_parameter(n).value

        self.frame_id = gp('frame_id')
        self.rate = float(gp('publish_rate_hz'))
        self.w = int(gp('image_w'))
        self.h = int(gp('image_h'))
        self.block_size = int(gp('block_size_px'))
        self.block_speed = float(gp('block_speed_px_per_frame'))
        self.sec_per_floor = float(gp('seconds_per_floor'))
        self.idle_pause = float(gp('idle_pause_s'))
        self.loop = bool(gp('loop'))

        self.pub = self.create_publisher(Image, gp('image_topic'), 10)
        self.schedule = self._build_schedule(gp('scenario'))
        self.t = 0.0
        self.block_y = self.h / 2.0
        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'fake_direction_image: scenario={gp("scenario")}, '
            f'total={self.schedule[-1][0]:.1f}s')

    def _build_schedule(self, scenario: str):
        """Return list of (end_time, sign) keyframes.
        sign in {-1, 0, +1}. 0 = idle.
        """
        out = []
        t = 0.0
        for tok in scenario.split(','):
            tok = tok.strip()
            if not tok:
                continue
            if tok.startswith('p'):
                dur = float(tok[1:])
                t += dur
                out.append((t, 0))
                continue
            sign = 1 if tok.startswith('+') else -1
            n = int(tok[1:]) if tok[0] in '+-' else int(tok)
            t += abs(n) * self.sec_per_floor
            out.append((t, sign))
            t += self.idle_pause
            out.append((t, 0))
        if not out:
            out.append((10.0, 0))
        return out

    def _sign_at(self, t: float) -> int:
        for end, sign in self.schedule:
            if t < end:
                return sign
        if self.loop:
            return self._sign_at(t % self.schedule[-1][0])
        return 0

    def _tick(self) -> None:
        sign = self._sign_at(self.t)
        # NOTE: image y increases downward. A "going up" arrow on a real
        # display has marching blocks that travel upward (toward y=0).
        # So sign = +1 -> block_y decreases.
        self.block_y -= sign * self.block_speed
        # wrap
        if self.block_y < 0:
            self.block_y += self.h
        if self.block_y >= self.h:
            self.block_y -= self.h

        frame = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        if sign != 0:
            y0 = int(self.block_y - self.block_size / 2) % self.h
            y1 = y0 + self.block_size
            x0 = self.w // 2 - self.block_size // 2
            x1 = x0 + self.block_size
            if y1 <= self.h:
                frame[y0:y1, x0:x1] = 255
            else:
                # wrap-around: split top + bottom
                frame[y0:self.h, x0:x1] = 255
                frame[0:y1 - self.h, x0:x1] = 255

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = self.h
        msg.width = self.w
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = self.w * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)

        self.t += self.dt


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeDirectionImage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
