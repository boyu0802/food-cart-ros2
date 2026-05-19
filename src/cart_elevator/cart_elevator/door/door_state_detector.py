#!/usr/bin/env python3
"""door_state_detector — turns a D455 depth ROI into /elevator/door_state.

Subscribes:
  /camera/camera/depth/image_rect_raw  (sensor_msgs/Image, 16UC1 mm)
Publishes:
  /elevator/door_state                 (cart_elevator_msgs/DoorState) at
                                       publish_rate_hz.

The depth math + state decision lives in `door_classifier.py` (pure
functions, unit-testable). This node only handles the ROS plumbing:
crop the door ROI out of each frame, reduce to (median, lateral_std)
scalars, smooth, derive d(lateral_std)/dt for the moving-door check.

ROI is configured in pixel coords (x0,y0,w,h). It must contain the
door panel area when the cart is parked at the standoff pose; pick it
once with `rqt_image_view` after you've docked, write it into the
yaml, and don't touch it again.

If the depth subscription has been silent longer than depth_max_age_s,
the node publishes UNKNOWN so the supervisor doesn't act on stale data.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cart_elevator_msgs.msg import DoorState

from cart_elevator.door.door_classifier import (
    DoorThresholds, classify,
)


class DoorStateDetector(Node):
    def __init__(self) -> None:
        super().__init__('door_state_detector')

        self.declare_parameters(namespace='', parameters=[
            ('depth_topic', '/camera/camera/depth/image_rect_raw'),
            ('state_topic', '/elevator/door_state'),
            # ROI in pixels (top-left + size).
            ('roi_x', 200),
            ('roi_y', 120),
            ('roi_w', 240),
            ('roi_h', 160),
            # Depth unit conversion: D455 publishes 16UC1 in millimeters.
            ('depth_scale_m', 0.001),
            # Classifier thresholds (mirror DoorThresholds defaults).
            ('closed_depth_m', 0.50),
            ('open_depth_m', 1.50),
            ('lateral_std_flat_m', 0.05),
            ('moving_dlat_thr_per_s', 0.10),
            # Smoothing + freshness.
            ('lateral_std_lpf_alpha', 0.4),   # 0..1, higher = faster response
            ('depth_max_age_s', 0.5),
            ('publish_rate_hz', 10.0),
        ])
        gp = lambda n: self.get_parameter(n).value

        self.roi = (int(gp('roi_x')), int(gp('roi_y')),
                    int(gp('roi_w')), int(gp('roi_h')))
        self.depth_scale_m = float(gp('depth_scale_m'))
        self.depth_max_age_s = float(gp('depth_max_age_s'))
        self.alpha = float(gp('lateral_std_lpf_alpha'))

        self.thr = DoorThresholds(
            closed_depth_m=float(gp('closed_depth_m')),
            open_depth_m=float(gp('open_depth_m')),
            lateral_std_flat_m=float(gp('lateral_std_flat_m')),
            moving_dlat_thr_per_s=float(gp('moving_dlat_thr_per_s')),
        )

        self.last_median_m: float | None = None
        self.last_lateral_std_m: float = 0.0
        self.last_filtered_lat_std: float | None = None
        self.last_filt_t: float | None = None
        self.last_dlat_per_s: float = 0.0
        self.last_depth_t: float | None = None

        self.create_subscription(Image, gp('depth_topic'), self._on_depth, 10)
        self.pub = self.create_publisher(DoorState, gp('state_topic'), 10)
        self.create_timer(1.0 / float(gp('publish_rate_hz')), self._tick)

        self.get_logger().info(
            f'door_state_detector ready (ROI={self.roi}, scale={self.depth_scale_m})')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_depth(self, msg: Image) -> None:
        # 16UC1 expected. If anything else shows up (e.g. 32FC1), bail
        # cleanly — better to publish UNKNOWN than to misread bytes.
        if msg.encoding != '16UC1':
            self.get_logger().warn_once(
                f'unexpected depth encoding {msg.encoding!r}, expected 16UC1')
            return

        x0, y0, w, h = self.roi
        if (x0 < 0 or y0 < 0 or x0 + w > msg.width or y0 + h > msg.height):
            self.get_logger().warn_once(
                f'ROI {self.roi} out of image {msg.width}x{msg.height}')
            return

        # Reconstruct just the rows we need, no full-frame copy.
        row_stride = msg.step  # bytes per row
        buf = msg.data
        depth_mm = np.empty((h, w), dtype=np.uint16)
        for r in range(h):
            start = (y0 + r) * row_stride + x0 * 2
            depth_mm[r, :] = np.frombuffer(
                buf, dtype=np.uint16, count=w, offset=start)

        # Mask out the D455 zero-depth-invalid pixels before stats.
        valid = depth_mm > 0
        if not valid.any():
            return

        depth_m = depth_mm.astype(np.float32) * self.depth_scale_m
        median_m = float(np.median(depth_m[valid]))

        # Lateral std: per-column median, then std over columns. Robust
        # to small holes in the depth image.
        col_med = np.zeros(w, dtype=np.float32)
        for c in range(w):
            col_valid = valid[:, c]
            if col_valid.any():
                col_med[c] = float(np.median(depth_m[col_valid, c]))
            else:
                col_med[c] = median_m
        lateral_std_m = float(np.std(col_med))

        # Low-pass + numeric derivative for d(lateral_std)/dt.
        now = self._now_s()
        if self.last_filtered_lat_std is None or self.last_filt_t is None:
            self.last_filtered_lat_std = lateral_std_m
            self.last_dlat_per_s = 0.0
        else:
            dt = max(1e-3, now - self.last_filt_t)
            filt_new = (self.alpha * lateral_std_m
                        + (1.0 - self.alpha) * self.last_filtered_lat_std)
            self.last_dlat_per_s = (filt_new - self.last_filtered_lat_std) / dt
            self.last_filtered_lat_std = filt_new

        self.last_filt_t = now
        self.last_median_m = median_m
        self.last_lateral_std_m = lateral_std_m
        self.last_depth_t = now

    def _tick(self) -> None:
        out = DoorState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'camera_depth_optical_frame'

        stale = (self.last_depth_t is None
                 or (self._now_s() - self.last_depth_t) > self.depth_max_age_s)
        if stale or self.last_median_m is None:
            out.state = DoorState.UNKNOWN
            out.confidence = 0.0
            out.median_depth_m = float('nan')
            self.pub.publish(out)
            return

        cls = classify(self.last_median_m,
                       self.last_filtered_lat_std or self.last_lateral_std_m,
                       self.last_dlat_per_s,
                       self.thr)
        out.state = cls.state
        out.confidence = cls.confidence
        out.median_depth_m = float(self.last_median_m)
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DoorStateDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
