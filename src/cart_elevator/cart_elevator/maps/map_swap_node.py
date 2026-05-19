#!/usr/bin/env python3
"""map_swap_node — swaps Nav2's active map + re-initializes AMCL on
floor transitions.

Triggered by the supervisor publishing the destination floor on
/map/swap_floor. Sequence (each step best-effort, fail-soft):

  1. Look up per-floor map file from `maps:` param (yaml dict).
     If the file exists, call /map_server/load_map with that path.
     If it doesn't exist (per-floor maps not yet recorded), log a
     warning and proceed — the cart will keep whatever map is loaded.
  2. Wait for the floor-N hallway AprilTag detection on /limelight/tag
     (tag id = floor_tag_id_offset + floor). Tag-loss timeout is
     configurable; if we never see it, publish current_floor anyway
     with a warning so the BT doesn't deadlock.
  3. Convert the tag pose to a robot pose in the new map frame and
     publish /initialpose so AMCL snaps to that pose.
  4. Publish /map/current_floor (Int32). The supervisor advances
     out of RELOCALIZE_AT_FLOOR only when this matches its target.

Designed to work today (with maps missing) AND later (when each
floor's map is recorded). Add new map files in the yaml; no code
change needed.
"""

import math
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped

from cart_elevator_msgs.msg import TagDetection


def _quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class MapSwapNode(Node):
    def __init__(self) -> None:
        super().__init__('map_swap')

        self.declare_parameters(namespace='', parameters=[
            ('swap_topic', '/map/swap_floor'),
            ('current_floor_topic', '/map/current_floor'),
            ('tag_topic', '/limelight/tag'),
            ('initialpose_topic', '/initialpose'),
            ('load_map_service', '/map_server/load_map'),
            ('floor_tag_id_offset', 100),
            # Map file paths per floor. Empty / missing -> skip load.
            # ROS yaml lacks dict params; the launch should pass a flat
            # list per floor as separate keys like map_floor_1: "path".
            ('map_floor_1', ''),
            ('map_floor_2', ''),
            ('map_floor_3', ''),
            ('map_floor_4', ''),
            ('map_floor_5', ''),
            ('relocalize_timeout_s', 8.0),
            ('initialpose_xy_cov', 0.25),
            ('initialpose_yaw_cov', 0.10),
        ])
        gp = lambda n: self.get_parameter(n).value

        self.floor_tag_offset = int(gp('floor_tag_id_offset'))
        self.relocalize_timeout_s = float(gp('relocalize_timeout_s'))
        self.xy_cov = float(gp('initialpose_xy_cov'))
        self.yaw_cov = float(gp('initialpose_yaw_cov'))

        self.map_paths = {
            n: gp(f'map_floor_{n}') for n in range(1, 6)
        }

        # Pending swap state. While set, _on_tag will fire the
        # initialpose republish on the first matching detection.
        self._pending_floor: Optional[int] = None
        self._pending_deadline_s: float = 0.0

        self.create_subscription(Int32, gp('swap_topic'), self._on_swap, 5)
        self.create_subscription(TagDetection, gp('tag_topic'), self._on_tag, 20)

        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, gp('initialpose_topic'), 5)
        self.current_floor_pub = self.create_publisher(
            Int32, gp('current_floor_topic'), 5)

        # map_server/load_map service. Imported lazily so the node can
        # boot even if nav2_msgs isn't installed during sim.
        try:
            from nav2_msgs.srv import LoadMap
            self._load_map_type = LoadMap
            self._load_map_client = self.create_client(LoadMap,
                                                       gp('load_map_service'))
        except ImportError:
            self._load_map_type = None
            self._load_map_client = None
            self.get_logger().warn(
                'nav2_msgs not available — map load will be skipped')

        # 10 Hz watchdog: enforce the relocalize timeout so a missing
        # hallway tag doesn't deadlock the BT forever.
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'map_swap ready (timeout={self.relocalize_timeout_s:.1f}s)')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_swap(self, msg: Int32) -> None:
        floor = int(msg.data)
        self.get_logger().info(f'swap requested -> floor {floor}')

        # Step 1: load map for the new floor, if a path is set + exists.
        path = self.map_paths.get(floor, '')
        if not path:
            self.get_logger().warn(
                f'no map file configured for floor {floor} '
                '— skipping load (re-localize only)')
        elif not os.path.isfile(path):
            self.get_logger().warn(
                f'map file for floor {floor} does not exist: {path!r} '
                '— skipping load')
        elif self._load_map_client is None:
            self.get_logger().warn(
                'map load skipped (LoadMap srv type unavailable)')
        else:
            self._load_map(path)

        # Step 2: arm the wait for the floor-N hallway tag.
        self._pending_floor = floor
        self._pending_deadline_s = self._now_s() + self.relocalize_timeout_s
        self.get_logger().info(
            f'waiting for tag id={self.floor_tag_offset + floor} '
            f'to re-init AMCL (timeout {self.relocalize_timeout_s:.1f}s)')

    def _load_map(self, path: str) -> None:
        req = self._load_map_type.Request()
        req.map_url = path
        if not self._load_map_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                f'map_server/load_map not available — leaving current map')
            return
        fut = self._load_map_client.call_async(req)
        fut.add_done_callback(self._on_load_map_done)

    def _on_load_map_done(self, future) -> None:
        try:
            resp = future.result()
            # nav2_msgs/LoadMap.result values: 0=success, 1=map_does_not_exist,
            # 2=invalid_map_data, 3=invalid_map_metadata, 255=undefined.
            if resp.result == 0:
                self.get_logger().info('map loaded successfully')
            else:
                self.get_logger().error(
                    f'load_map returned result={resp.result} '
                    '(non-success — see nav2_msgs/LoadMap)')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'load_map exception: {e}')

    def _on_tag(self, msg: TagDetection) -> None:
        if self._pending_floor is None:
            return
        expected_id = self.floor_tag_offset + self._pending_floor
        if msg.tag_id != expected_id or not msg.has_pose:
            return

        # Tag pose is given in the robot frame: where the tag is in
        # front of us. To re-init AMCL we need where the ROBOT is in
        # the map frame. We assume each floor's map was recorded with
        # the floor-N tag at the map origin (0,0,0). Then the robot
        # pose in map = inverse(tag_in_robot).
        tx = msg.pose_in_robot_frame.position.x
        ty = msg.pose_in_robot_frame.position.y
        q = msg.pose_in_robot_frame.orientation
        tag_yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)

        # inverse of (translate then rotate) -> apply -rot then -trans
        c, s = math.cos(-tag_yaw), math.sin(-tag_yaw)
        rx = -(c * tx - s * ty)
        ry = -(s * tx + c * ty)
        robot_yaw_in_map = -tag_yaw

        self._publish_initialpose(rx, ry, robot_yaw_in_map)

        # Announce success.
        m = Int32()
        m.data = int(self._pending_floor)
        self.current_floor_pub.publish(m)
        self.get_logger().info(
            f'AMCL re-init done -> floor {self._pending_floor} '
            f'@ map ({rx:+.2f}, {ry:+.2f}, {math.degrees(robot_yaw_in_map):+.1f}deg)')

        self._pending_floor = None
        self._pending_deadline_s = 0.0

    def _tick(self) -> None:
        if self._pending_floor is None:
            return
        if self._now_s() < self._pending_deadline_s:
            return
        # Timeout: announce current_floor anyway so the BT can proceed,
        # but warn loudly. AMCL was NOT re-initialized — Nav2 will be
        # working on stale localization until the cart actually sees
        # the tag (or the operator manually sets initialpose).
        floor = int(self._pending_floor)
        self.get_logger().error(
            f'relocalize timeout — no floor-{floor} tag seen in '
            f'{self.relocalize_timeout_s:.1f}s. AMCL NOT re-initialized.')
        m = Int32()
        m.data = floor
        self.current_floor_pub.publish(m)
        self._pending_floor = None
        self._pending_deadline_s = 0.0

    def _publish_initialpose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # AMCL covariance layout is 6x6 (x, y, z, roll, pitch, yaw),
        # row-major.
        cov = [0.0] * 36
        cov[0]  = self.xy_cov   # xx
        cov[7]  = self.xy_cov   # yy
        cov[35] = self.yaw_cov  # yaw-yaw
        msg.pose.covariance = cov
        self.initpose_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapSwapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
