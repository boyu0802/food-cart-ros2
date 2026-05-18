#!/usr/bin/env python3
"""direction_debug_view -- live visualizer for the digit-tracking detector.

Subscribes to the same /image stream that direction_v1_digit consumes,
re-runs the digit pipeline locally so it can draw the intermediate state
(housing bbox, extracted digit crop, classification score), and overlays
the current direction estimate read off /elevator/direction_v1_digit.

Pops a cv2.imshow window. Press 'q' (or ctrl-c the launch) to quit.
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
    TPL_H, TPL_W,
    classify_digit, extract_digit, find_housing, load_templates, warm_mask,
)


DIR_LABEL = {
    ElevatorDirection.DIRECTION_UNKNOWN: 'UNKNOWN',
    ElevatorDirection.DIRECTION_IDLE: 'IDLE',
    ElevatorDirection.DIRECTION_UP: 'UP',
    ElevatorDirection.DIRECTION_DOWN: 'DOWN',
}

DIR_COLOR = {
    ElevatorDirection.DIRECTION_UNKNOWN: (60, 200, 220),    # yellow
    ElevatorDirection.DIRECTION_IDLE: (180, 180, 180),      # gray
    ElevatorDirection.DIRECTION_UP: (80, 220, 80),          # green
    ElevatorDirection.DIRECTION_DOWN: (80, 80, 230),        # red
}


class DirectionDebugView(Node):
    def __init__(self) -> None:
        super().__init__('direction_debug_view')

        default_tpl = str(
            Path(get_package_share_directory('cart_elevator'))
            / 'data' / 'digit_templates')

        self.declare_parameters(namespace='', parameters=[
            ('image_topic', '/image'),
            ('direction_topic', '/elevator/direction_v1_digit'),
            ('template_dir', default_tpl),
            ('window_name', 'direction_v1_digit (debug)'),
            ('history_len', 30),
        ])
        gp = lambda n: self.get_parameter(n).value

        self.templates = load_templates(gp('template_dir'))
        self.window = gp('window_name')
        self.digit_history: deque[int | None] = deque(maxlen=int(gp('history_len')))

        self.last_dir = ElevatorDirection()
        self.last_dir.direction = ElevatorDirection.DIRECTION_UNKNOWN
        self.last_dir.confidence = 0.0

        self.create_subscription(Image, gp('image_topic'), self._on_image, 5)
        self.create_subscription(
            ElevatorDirection, gp('direction_topic'), self._on_direction, 10)

        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, 960, 720)
        self.get_logger().info(
            f'direction_debug_view ready -- showing "{self.window}". '
            f'Click the window and press q to quit.')

    @staticmethod
    def _image_to_bgr(msg: Image) -> np.ndarray | None:
        h, w = msg.height, msg.width
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        enc = msg.encoding.lower()
        if enc == 'bgr8':
            return buf.reshape(h, w, 3).copy()
        if enc == 'rgb8':
            return cv2.cvtColor(buf.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
        if enc == 'mono8':
            return cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
        return None

    def _on_direction(self, msg: ElevatorDirection) -> None:
        self.last_dir = msg

    def _on_image(self, msg: Image) -> None:
        bgr = self._image_to_bgr(msg)
        if bgr is None:
            return
        canvas = bgr.copy()

        housing = find_housing(bgr)
        digit, score, margin = None, 0.0, 0.0
        crop = None
        if housing is not None:
            x, y, w, h = housing
            cv2.rectangle(canvas, (x, y), (x + w, y + h), (80, 220, 80), 3)
            cv2.putText(canvas, 'housing', (x, max(0, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 220, 80), 2)

            crop = extract_digit(warm_mask(bgr), housing)
            if crop is not None:
                result = classify_digit(crop, self.templates)
                if result is not None:
                    digit, score, margin = result
                    self.digit_history.append(digit)
                else:
                    self.digit_history.append(None)
            else:
                self.digit_history.append(None)
        else:
            self.digit_history.append(None)

        self._draw_overlay(canvas, digit, score, margin, crop)

        cv2.imshow(self.window, canvas)
        # 1 ms wait -- required to actually flush the window event loop.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('quit key pressed, shutting down')
            rclpy.shutdown()

    def _draw_overlay(self, canvas: np.ndarray,
                      digit: int | None, score: float, margin: float,
                      crop: np.ndarray | None) -> None:
        H, W = canvas.shape[:2]

        # Top-left: per-frame digit classification
        cv2.rectangle(canvas, (0, 0), (360, 110), (0, 0, 0), -1)
        if digit is not None:
            txt = f'frame digit: {digit}   score {score:.2f}   margin {margin:.2f}'
            color = (80, 220, 80) if score >= 0.45 and margin >= 0.05 else (60, 200, 220)
            cv2.putText(canvas, txt, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        else:
            cv2.putText(canvas, 'frame digit: (none)', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 200, 220), 2)

        # History strip
        cv2.putText(canvas, 'recent:', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)
        hx = 90
        for d in self.digit_history:
            s = '-' if d is None else str(d)
            c = (180, 180, 180) if d is None else (80, 220, 80)
            cv2.putText(canvas, s, (hx, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 2)
            hx += 14
            if hx > 350:
                break

        # Bottom-left: published direction
        direction = int(self.last_dir.direction)
        label = DIR_LABEL.get(direction, str(direction))
        color = DIR_COLOR.get(direction, (220, 220, 220))
        conf = float(self.last_dir.confidence)
        cv2.rectangle(canvas, (0, H - 110), (360, H), (0, 0, 0), -1)
        cv2.putText(canvas, f'DIRECTION: {label}', (10, H - 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv2.putText(canvas, f'confidence: {conf:.2f}', (10, H - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        # Confidence bar
        bar_w = int(340 * max(0.0, min(1.0, conf)))
        cv2.rectangle(canvas, (10, H - 18), (10 + 340, H - 8), (60, 60, 60), 1)
        cv2.rectangle(canvas, (10, H - 18), (10 + bar_w, H - 8), color, -1)

        # Top-right: extracted digit crop, scaled up so you can see what the
        # classifier is matching against.
        if crop is not None:
            zoom = 6
            patch = cv2.resize(crop, (TPL_W * zoom, TPL_H * zoom),
                               interpolation=cv2.INTER_NEAREST)
            patch_bgr = cv2.cvtColor(patch, cv2.COLOR_GRAY2BGR)
            ph, pw = patch_bgr.shape[:2]
            x0, y0 = W - pw - 10, 10
            cv2.rectangle(canvas, (x0 - 4, y0 - 4),
                          (x0 + pw + 4, y0 + ph + 4), (80, 220, 80), 2)
            canvas[y0:y0 + ph, x0:x0 + pw] = patch_bgr
            cv2.putText(canvas, 'extracted digit',
                        (x0, y0 + ph + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 220, 80), 1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DirectionDebugView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
