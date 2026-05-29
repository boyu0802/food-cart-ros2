#!/usr/bin/env python3
"""door_std_probe — print median + lateral_std over the door ROI at 2 Hz.

Uses the SAME math as door_state_detector so the lateral_std it prints is
the exact number the classifier's flatness gate sees. Run it, then open
and close the door and read off lateral_std for each state — that tells us
where to set lateral_std_flat_m. Calibration tool, not part of the mission.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DoorStdProbe(Node):
    def __init__(self) -> None:
        super().__init__('door_std_probe')
        self.declare_parameters(namespace='', parameters=[
            ('depth_topic', '/camera/camera/depth/image_rect_raw'),
            ('roi_x', 104), ('roi_y', 58), ('roi_w', 170), ('roi_h', 194),
            ('depth_scale_m', 0.001),
        ])
        gp = lambda n: self.get_parameter(n).value
        self.roi = (int(gp('roi_x')), int(gp('roi_y')),
                    int(gp('roi_w')), int(gp('roi_h')))
        self.scale = float(gp('depth_scale_m'))
        self.latest = None
        self.create_subscription(Image, gp('depth_topic'), self._on_depth, 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(f'door_std_probe ready, ROI={self.roi}')

    def _on_depth(self, msg: Image) -> None:
        if msg.encoding != '16UC1':
            return
        x0, y0, w, h = self.roi
        if x0 + w > msg.width or y0 + h > msg.height:
            self.latest = ('ROI OUT OF BOUNDS', None)
            return
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width)[y0:y0 + h, x0:x0 + w].astype(np.float32) * self.scale
        valid = d > 0
        if not valid.any():
            self.latest = ('no valid pixels', None)
            return
        median_m = float(np.median(d[valid]))
        col_med = np.array([float(np.median(d[valid[:, c], c]))
                            if valid[:, c].any() else median_m
                            for c in range(w)], dtype=np.float32)
        self.latest = (median_m, float(np.std(col_med)))

    def _tick(self) -> None:
        if self.latest is None:
            return
        med, std = self.latest
        if std is None:
            self.get_logger().info(str(med))
        else:
            self.get_logger().info(
                f'median={med:.2f} m   lateral_std={std:.3f} m')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DoorStdProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
