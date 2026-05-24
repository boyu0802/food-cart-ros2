#!/usr/bin/env python3
"""scan_sector_filter — mask fixed angular sectors out of a LaserScan.

The cart's four corner posts sit in the 360 deg plane of the centered C1
lidar, so each post (a) returns a constant short-range hit and (b)
shadows everything behind it. Left raw, those hits get baked into the
slam_toolbox map and stamped as phantom obstacles hugging the footprint
in both Nav2 costmaps — the robot then refuses to move or fires pointless
recoveries.

This node replaces the readings inside each configured sector with NaN
("no information") so downstream consumers neither MARK them as obstacles
nor RAYTRACE-CLEAR through them. The sectors are fixed in the `laser`
frame because the posts are rigidly mounted, so a static mask is the
right tool — and the masked beams are the price: those bearings become
blind spots (see the safety note in scan_filter.yaml).

Sectors are [lo_deg, hi_deg] pairs in the lidar frame, ROS convention:
0 deg = +x (forward), CCW positive, wrapped to (-180, 180]. A pair with
lo > hi is treated as wrapping through +/-180 deg.

Subscribes:  input_topic  (sensor_msgs/LaserScan, default /scan_raw)
Publishes:   output_topic (sensor_msgs/LaserScan, default /scan)

QoS mirrors the sensor-data profile the rplidar driver publishes with, so
slam_toolbox and the costmaps see the same contract they did when wired
straight to the driver.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def _norm_vec(a: np.ndarray) -> np.ndarray:
    """Wrap an array of radians to (-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))


class ScanSectorFilter(Node):
    def __init__(self) -> None:
        super().__init__('scan_sector_filter')

        self.declare_parameters(namespace='', parameters=[
            ('input_topic', '/scan_raw'),
            ('output_topic', '/scan'),
            # Flat [lo0, hi0, lo1, hi1, ...] in degrees, lidar frame.
            ('mask_sectors_deg', [
                33.8, 45.8,      # front-left post
                -45.8, -33.8,    # front-right post
                134.2, 146.2,    # rear-left post
                -146.2, -134.2,  # rear-right post
            ]),
        ])
        gp = lambda n: self.get_parameter(n).value

        flat = [float(v) for v in gp('mask_sectors_deg')]
        if len(flat) % 2 != 0:
            raise ValueError(
                'mask_sectors_deg must contain an even number of values '
                '(lo, hi degree pairs)')
        # Store normalized (lo, hi) radian pairs.
        self._sectors = [
            (float(_norm_vec(np.array(math.radians(flat[i])))),
             float(_norm_vec(np.array(math.radians(flat[i + 1])))))
            for i in range(0, len(flat), 2)
        ]

        self._mask: np.ndarray | None = None
        self._cached_key = None    # (angle_min, angle_increment, n)

        out_topic = gp('output_topic')
        self.pub = self.create_publisher(
            LaserScan, out_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            LaserScan, gp('input_topic'), self._on_scan, qos_profile_sensor_data)

        pretty = [(round(math.degrees(lo), 1), round(math.degrees(hi), 1))
                  for lo, hi in self._sectors]
        self.get_logger().info(
            f"scan_sector_filter ready: {gp('input_topic')} -> {out_topic}, "
            f"masking {len(self._sectors)} sector(s) {pretty} (deg)")

    def _build_mask(self, msg: LaserScan) -> np.ndarray:
        n = len(msg.ranges)
        angles = _norm_vec(msg.angle_min + np.arange(n) * msg.angle_increment)
        mask = np.zeros(n, dtype=bool)
        for lo, hi in self._sectors:
            if lo <= hi:
                mask |= (angles >= lo) & (angles <= hi)
            else:  # sector wraps through +/-pi
                mask |= (angles >= lo) | (angles <= hi)
        return mask

    def _on_scan(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        key = (msg.angle_min, msg.angle_increment, n)
        if key != self._cached_key:
            self._mask = self._build_mask(msg)
            self._cached_key = key
            masked = int(self._mask.sum())
            pct = 100.0 * masked / max(n, 1)
            self.get_logger().info(
                f'mask rebuilt for {n}-beam scan: {masked} beams masked '
                f'({pct:.1f}% of FOV blind)')

        ranges = np.asarray(msg.ranges, dtype=np.float32)
        ranges[self._mask] = float('nan')
        msg.ranges = ranges.tolist()
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanSectorFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
