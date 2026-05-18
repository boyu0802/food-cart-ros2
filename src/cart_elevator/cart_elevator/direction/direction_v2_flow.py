#!/usr/bin/env python3
"""direction_v2_flow — detect elevator-display arrow direction by
sparse optical flow inside an ROI of the camera feed.

Idea (will be tuned once we have real video):
  1. Crop a rectangle around the display.
  2. Detect a small fixed grid of points inside the ROI (or use
     goodFeaturesToTrack on the first frame).
  3. Lucas-Kanade pyramidal flow between consecutive frames.
  4. The animated arrow drags features vertically. Median dy across
     trackable points is the direction signal: dy<0 -> up (image y
     increases downward, but a marching-up arrow drags features upward
     so the sign convention will be confirmed against real footage),
     dy>0 -> down, |dy| small -> idle.

Inputs:
  /image  (sensor_msgs/Image)

Outputs:
  /elevator/direction_v2_flow  (cart_elevator_msgs/ElevatorDirection)

PLACEHOLDER algorithm — internals filled in after viewing real frames.
The node structure, params, and message shape are final.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cart_elevator_msgs.msg import ElevatorDirection


class DirectionV2Flow(Node):
    def __init__(self) -> None:
        super().__init__('direction_v2_flow')

        self.declare_parameters(namespace='', parameters=[
            ('image_topic', '/camera/camera/color/image_raw'),
            ('output_topic', '/elevator/direction_v2_flow'),
            ('roi_x', 0),
            ('roi_y', 0),
            ('roi_w', 200),
            ('roi_h', 200),
            # Pixels of median dy per frame above which we trust the sign.
            ('flow_threshold_px', 0.5),  # PLACEHOLDER
            # Median window over which we smooth dy.
            ('smoothing_window', 5),
            ('confidence_idle', 0.50),
            ('confidence_moving', 0.85),
            ('publish_rate_hz', 10.0),
        ])
        gp = lambda n: self.get_parameter(n).value

        self.create_subscription(Image, gp('image_topic'), self._on_image, 5)
        self.pub = self.create_publisher(
            ElevatorDirection, gp('output_topic'), 10)

        self._prev_gray = None
        self._features = None
        self._dy_history: list[float] = []

        self.create_timer(1.0 / float(gp('publish_rate_hz')), self._publish)
        self.get_logger().info('direction_v2_flow scaffold ready')

    def _on_image(self, msg: Image) -> None:
        # TODO(tune-from-video): convert to gray, crop ROI, run
        # calcOpticalFlowPyrLK, append median dy to history.
        self._prev_gray = msg

    def _publish(self) -> None:
        out = ElevatorDirection()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'camera_color_optical_frame'
        out.source = ElevatorDirection.SOURCE_OPTICAL_FLOW

        out.direction = ElevatorDirection.DIRECTION_UNKNOWN
        out.confidence = 0.0
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DirectionV2Flow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
