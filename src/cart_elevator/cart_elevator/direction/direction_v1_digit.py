#!/usr/bin/env python3
"""direction_v1_digit -- PRIMARY direction detector.

Watches the elevator hall display via the in-car camera, reads the floor
DIGIT each frame (template-match against digit_<N>.png templates), and
publishes the direction implied by the digit sequence over time:

  digit increasing across the window  -> UP
  digit decreasing                    -> DOWN
  digit stable                        -> IDLE
  no recent confident reading         -> UNKNOWN

This is intentionally NOT looking at the arrow animation -- the digit is
ground truth and gives us the absolute floor for free. The arrow-based
detector (direction_v2_flow) remains a placeholder fallback for very
short hops where the digit doesn't change in time.

Inputs:
  /image (sensor_msgs/Image, bgr8 / rgb8 / mono8)

Outputs:
  /elevator/direction_v1_digit (cart_elevator_msgs/ElevatorDirection)
"""

from __future__ import annotations

from collections import deque
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from cart_elevator_msgs.msg import ElevatorDirection

from cart_elevator.direction.digit_match import (
    classify_digit, extract_digit, find_housing, load_templates, warm_mask,
)


class DirectionV1Digit(Node):
    def __init__(self) -> None:
        super().__init__('direction_v1_digit')

        default_tpl = str(
            Path(get_package_share_directory('cart_elevator'))
            / 'data' / 'digit_templates')

        self.declare_parameters(namespace='', parameters=[
            ('image_topic', '/image'),
            ('output_topic', '/elevator/direction_v1_digit'),
            ('template_dir', default_tpl),
            # IoU thresholds for trusting a digit classification.
            ('min_score', 0.45),
            ('min_margin', 0.05),
            # Sliding window over which we infer direction.
            ('history_window_s', 5.0),
            # Min observations in the window before we will report non-UNKNOWN.
            ('min_observations', 2),
            # Confidence ceiling/floor.
            ('confidence_idle', 0.75),
            ('confidence_moving_base', 0.55),
            ('confidence_moving_step', 0.10),
            ('publish_rate_hz', 10.0),
        ])
        gp = lambda n: self.get_parameter(n).value

        self.min_score = float(gp('min_score'))
        self.min_margin = float(gp('min_margin'))
        self.history_window = float(gp('history_window_s'))
        self.min_obs = int(gp('min_observations'))
        self.c_idle = float(gp('confidence_idle'))
        self.c_base = float(gp('confidence_moving_base'))
        self.c_step = float(gp('confidence_moving_step'))

        tpl_dir = gp('template_dir')
        self.templates = load_templates(tpl_dir)
        if not self.templates:
            self.get_logger().error(
                f'no digit templates found in {tpl_dir!r} -- node will run '
                f'but always emit UNKNOWN')
        else:
            self.get_logger().info(
                f'loaded {len(self.templates)} digit templates: '
                f'{sorted(self.templates.keys())}')

        self.history: deque[tuple[float, int]] = deque()

        self.create_subscription(Image, gp('image_topic'), self._on_image, 5)
        self.pub = self.create_publisher(
            ElevatorDirection, gp('output_topic'), 10)
        self.create_timer(1.0 / float(gp('publish_rate_hz')), self._publish)
        self.get_logger().info('direction_v1_digit ready')

    @staticmethod
    def _image_to_bgr(msg: Image) -> np.ndarray | None:
        """Tiny in-tree BGR converter so we don't have to depend on cv_bridge."""
        h, w = msg.height, msg.width
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        enc = msg.encoding.lower()
        if enc in ('bgr8',):
            return buf.reshape(h, w, 3)
        if enc in ('rgb8',):
            return cv2.cvtColor(buf.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
        if enc in ('mono8',):
            return cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
        return None

    def _on_image(self, msg: Image) -> None:
        bgr = self._image_to_bgr(msg)
        if bgr is None:
            self.get_logger().warn(
                f'unsupported image encoding {msg.encoding!r}', throttle_duration_sec=5.0)
            return

        housing = find_housing(bgr)
        if housing is None:
            return
        crop = extract_digit(warm_mask(bgr), housing)
        if crop is None:
            return
        result = classify_digit(crop, self.templates)
        if result is None:
            return
        digit, score, margin = result
        if score < self.min_score or margin < self.min_margin:
            return

        t = self.get_clock().now().nanoseconds * 1e-9
        self.history.append((t, digit))
        # Prune.
        cutoff = t - self.history_window
        while self.history and self.history[0][0] < cutoff:
            self.history.popleft()

    def _classify_direction(self) -> tuple[int, float]:
        """Return (direction, confidence) from the current history."""
        if len(self.history) < self.min_obs:
            return ElevatorDirection.DIRECTION_UNKNOWN, 0.0

        digits = [d for _, d in self.history]
        first, last = digits[0], digits[-1]
        unique = set(digits)

        if first == last and len(unique) == 1:
            return ElevatorDirection.DIRECTION_IDLE, self.c_idle

        if last > first:
            conf = min(0.95, self.c_base + self.c_step * len(digits))
            return ElevatorDirection.DIRECTION_UP, conf
        if last < first:
            conf = min(0.95, self.c_base + self.c_step * len(digits))
            return ElevatorDirection.DIRECTION_DOWN, conf

        # first == last but window contains other digits (e.g. went up then back) --
        # don't trust either direction.
        return ElevatorDirection.DIRECTION_UNKNOWN, 0.0

    def _publish(self) -> None:
        # Prune again on publish so stale observations don't keep us "alive".
        t = self.get_clock().now().nanoseconds * 1e-9
        cutoff = t - self.history_window
        while self.history and self.history[0][0] < cutoff:
            self.history.popleft()

        direction, confidence = self._classify_direction()

        out = ElevatorDirection()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'camera_color_optical_frame'
        out.source = ElevatorDirection.SOURCE_DIGIT_TRACK
        out.direction = direction
        out.confidence = confidence
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DirectionV1Digit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
