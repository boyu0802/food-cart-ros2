#!/usr/bin/env python3
"""video_replay -- publish a recorded mp4 as sensor_msgs/Image on /image.

Used to drive direction_v1_digit (and any future vision detector) with the
recorded elevator clip so we have a deterministic, ground-truth fixture
that doesn't depend on the real camera being attached.

Loops by default. When the video ends we rewind to frame 0.
"""

from __future__ import annotations

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class VideoReplay(Node):
    def __init__(self) -> None:
        super().__init__('video_replay')

        self.declare_parameters(namespace='', parameters=[
            ('video_path', ''),
            ('image_topic', '/image'),
            ('frame_id', 'camera_color_optical_frame'),
            ('loop', True),
            # If >0, override the file's native framerate.
            ('publish_rate_hz', 0.0),
        ])
        gp = lambda n: self.get_parameter(n).value

        path = gp('video_path')
        if not path:
            raise RuntimeError(
                'video_replay requires video_path parameter')

        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise RuntimeError(f'video_replay could not open {path!r}')

        native_fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        override = float(gp('publish_rate_hz'))
        self.rate = override if override > 0 else native_fps

        self.loop = bool(gp('loop'))
        self.frame_id = gp('frame_id')
        self.image_topic = gp('image_topic')

        self.pub = self.create_publisher(Image, self.image_topic, 10)
        self.create_timer(1.0 / self.rate, self._tick)

        total = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.get_logger().info(
            f'video_replay: {path} -- {total} frames @ {self.rate:.2f} Hz '
            f'-> {self.image_topic} (loop={self.loop})')

    def _tick(self) -> None:
        ok, bgr = self.cap.read()
        if not ok:
            if not self.loop:
                self.get_logger().info('video_replay: end of file, stopping timer')
                self.destroy_timer(self.timers[0])
                return
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, bgr = self.cap.read()
            if not ok:
                self.get_logger().error('video_replay: rewind failed')
                return

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = h
        msg.width = w
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = np.ascontiguousarray(rgb).tobytes()
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VideoReplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
