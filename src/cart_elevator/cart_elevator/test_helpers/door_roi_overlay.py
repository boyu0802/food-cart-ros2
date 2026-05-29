#!/usr/bin/env python3
"""door_roi_overlay — live ROI calibration helper for the door detector.

Subscribes to the D455 depth image, colourises it so it's actually visible
in Foxglove, draws the current door ROI rectangle on top, and republishes
as a normal bgr8 image. The ROI box and the median depth inside it are
burned into the picture, so you can watch the box move as you retune.

The four ROI params are DYNAMIC — change them live with e.g.

    ros2 param set /door_roi_overlay roi_x 160
    ros2 param set /door_roi_overlay roi_h 130

and the box jumps on the next frame, no restart. When the box frames the
door opening, copy roi_x/roi_y/roi_w/roi_h into config/door.yaml.

The rectangle is GREEN when the ROI fits inside the image and RED when it
spills out of bounds (the real detector refuses to run an out-of-bounds
ROI, so red == the detector would publish nothing).

This is a calibration tool, not part of the mission. Run it standalone:

    python3 door_roi_overlay.py
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DoorRoiOverlay(Node):
    def __init__(self) -> None:
        super().__init__('door_roi_overlay')

        self.declare_parameters(namespace='', parameters=[
            ('depth_topic', '/camera/camera/depth/image_rect_raw'),
            ('out_topic', '/elevator/door_roi_debug'),
            # Mirror door.yaml so you tune the same numbers you'll paste back.
            ('roi_x', 200),
            ('roi_y', 120),
            ('roi_w', 240),
            ('roi_h', 160),
            ('depth_scale_m', 0.001),
            # Depth values above this (m) saturate the colour map. Set near
            # the door's expected open distance so contrast is useful.
            ('viz_max_m', 4.0),
        ])
        gp = lambda n: self.get_parameter(n).value
        self.roi_x = int(gp('roi_x'))
        self.roi_y = int(gp('roi_y'))
        self.roi_w = int(gp('roi_w'))
        self.roi_h = int(gp('roi_h'))
        self.depth_scale_m = float(gp('depth_scale_m'))
        self.viz_max_m = float(gp('viz_max_m'))

        self.bridge = CvBridge()
        self.add_on_set_parameters_callback(self._on_set_params)

        self.create_subscription(Image, gp('depth_topic'), self._on_depth, 10)
        self.pub = self.create_publisher(Image, gp('out_topic'), 10)
        self.get_logger().info(
            f"door_roi_overlay up. View {gp('out_topic')} in Foxglove. "
            f"Tune with: ros2 param set /door_roi_overlay roi_x <n>")

    def _on_set_params(self, params):
        for p in params:
            if p.name in ('roi_x', 'roi_y', 'roi_w', 'roi_h'):
                setattr(self, p.name, int(p.value))
            elif p.name == 'viz_max_m':
                self.viz_max_m = float(p.value)
        return SetParametersResult(successful=True)

    def _on_depth(self, msg: Image) -> None:
        if msg.encoding != '16UC1':
            self.get_logger().warn(
                f'expected 16UC1, got {msg.encoding!r}', once=True)
            return

        depth_mm = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width)
        depth_m = depth_mm.astype(np.float32) * self.depth_scale_m

        # Colourise: clip to viz_max, scale to 0..255, JET map. Invalid
        # (zero) pixels render as the near end — fine for a viz.
        norm = np.clip(depth_m / max(self.viz_max_m, 1e-3), 0.0, 1.0)
        vis = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)

        x0, y0, w, h = self.roi_x, self.roi_y, self.roi_w, self.roi_h
        in_bounds = (x0 >= 0 and y0 >= 0
                     and x0 + w <= msg.width and y0 + h <= msg.height)
        colour = (0, 255, 0) if in_bounds else (0, 0, 255)
        cv2.rectangle(vis, (x0, y0), (x0 + w, y0 + h), colour, 2)

        # Median depth inside the (clamped) ROI, ignoring invalid zeros.
        cx0, cy0 = max(0, x0), max(0, y0)
        cx1, cy1 = min(msg.width, x0 + w), min(msg.height, y0 + h)
        label = f'roi=({x0},{y0},{w},{h})  img={msg.width}x{msg.height}'
        if cx1 > cx0 and cy1 > cy0:
            crop = depth_m[cy0:cy1, cx0:cx1]
            valid = crop[crop > 0]
            med = float(np.median(valid)) if valid.size else float('nan')
            label2 = f'median={med:.2f} m  {"" if in_bounds else "OUT OF BOUNDS"}'
        else:
            label2 = 'ROI fully off-image'
        cv2.putText(vis, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(vis, label2, (6, 38), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, colour, 1, cv2.LINE_AA)

        out = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        out.header = msg.header
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DoorRoiOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
