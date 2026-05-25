#!/usr/bin/env python3
"""map_swap_node — swaps Nav2's active map + re-initializes AMCL on
floor transitions.

Triggered by the supervisor publishing the destination floor on
/map/swap_floor (in RELOCALIZE_AT_FLOOR, right after the cart drives out
of the cab). Sequence (each step best-effort, fail-soft):

  1. Look up the per-floor map file from the `map_floor_N` params and,
     if it exists, call /map_server/load_map with that path. Missing /
     unset -> log a warning and keep the current map.
  2. Re-localize AMCL to that floor. We DON'T wait to see an AprilTag:
     the cart often can't see the hallway tag on exit. Instead we trust
     the FLOOR NODE — when /elevator/floor (FloorEstimate, robust to tag
     dropout via the v4 fusion) confirms we're on the destination floor,
     we snap AMCL to the KNOWN cab-exit pose for that floor (the elevator
     door is a fixed coordinate in each floor's map). AMCL + the lidar
     scan refine from there. If the floor node doesn't confirm within
     `relocalize_timeout_s`, we proceed on the commanded floor anyway so
     the BT can't deadlock — but we still re-init to the cab-exit pose
     (unlike the old node, which left AMCL un-initialized on timeout).
  3. Publish /map/current_floor (Int32). The supervisor advances out of
     RELOCALIZE_AT_FLOOR only when this matches its target.

Per-floor cab-exit poses go in the `cab_exit_floor_N: [x, y, yaw]`
params (map frame). A floor with no pose configured logs a loud warning
and skips the AMCL re-init for that floor — fill these in once measured.
"""

import math
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped

from cart_elevator_msgs.msg import FloorEstimate


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class MapSwapNode(Node):
    def __init__(self) -> None:
        super().__init__('map_swap')

        self.declare_parameters(namespace='', parameters=[
            ('swap_topic', '/map/swap_floor'),
            ('current_floor_topic', '/map/current_floor'),
            ('floor_topic', '/elevator/floor'),
            ('initialpose_topic', '/initialpose'),
            ('load_map_service', '/map_server/load_map'),
            # Map file path per floor. Empty / missing -> skip load.
            ('map_floor_1', ''),
            ('map_floor_2', ''),
            ('map_floor_3', ''),
            ('map_floor_4', ''),
            ('map_floor_5', ''),
            # Known cab-exit pose per floor, [x, y, yaw] in the map frame.
            # Empty/!=3 -> not yet measured (AMCL re-init skipped, warns).
            ('cab_exit_floor_1', []),
            ('cab_exit_floor_2', []),
            ('cab_exit_floor_3', []),
            ('cab_exit_floor_4', []),
            ('cab_exit_floor_5', []),
            # Min FloorEstimate confidence to accept as "we are on floor N".
            ('floor_confirm_confidence', 0.5),
            ('relocalize_timeout_s', 8.0),
            ('initialpose_xy_cov', 0.25),
            ('initialpose_yaw_cov', 0.10),
        ])
        gp = lambda n: self.get_parameter(n).value

        self.confirm_conf = float(gp('floor_confirm_confidence'))
        self.relocalize_timeout_s = float(gp('relocalize_timeout_s'))
        self.xy_cov = float(gp('initialpose_xy_cov'))
        self.yaw_cov = float(gp('initialpose_yaw_cov'))

        self.map_paths = {n: gp(f'map_floor_{n}') for n in range(1, 6)}
        # Parse cab-exit poses; keep only floors with a full [x, y, yaw].
        self.cab_exit = {}
        for n in range(1, 6):
            v = list(gp(f'cab_exit_floor_{n}') or [])
            if len(v) == 3:
                self.cab_exit[n] = (float(v[0]), float(v[1]), float(v[2]))

        # Pending swap state + the latest floor-node estimate.
        self._pending_floor: Optional[int] = None
        self._pending_deadline_s: float = 0.0
        self._last_floor_est: Optional[FloorEstimate] = None

        self.create_subscription(Int32, gp('swap_topic'), self._on_swap, 5)
        self.create_subscription(
            FloorEstimate, gp('floor_topic'), self._on_floor, 20)

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

        # 10 Hz watchdog enforces the relocalize timeout.
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'map_swap ready (floors with cab-exit pose: '
            f'{sorted(self.cab_exit)}, timeout={self.relocalize_timeout_s:.1f}s)')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_swap(self, msg: Int32) -> None:
        floor = int(msg.data)
        self.get_logger().info(f'swap requested -> floor {floor}')

        # Step 1: load the map for the new floor if a path is set + exists.
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

        # Step 2: arm the floor-node-confirmed relocalize. Commit now if
        # the floor node already agrees we're there.
        self._pending_floor = floor
        self._pending_deadline_s = self._now_s() + self.relocalize_timeout_s
        if self._floor_confirmed(floor):
            self._commit(floor, confirmed=True)
        else:
            self.get_logger().info(
                f'waiting for floor node to confirm floor {floor} '
                f'(conf >= {self.confirm_conf:.2f}, timeout '
                f'{self.relocalize_timeout_s:.1f}s)')

    def _floor_confirmed(self, floor: int) -> bool:
        e = self._last_floor_est
        return (e is not None and int(e.floor) == int(floor)
                and float(e.confidence) >= self.confirm_conf)

    def _on_floor(self, msg: FloorEstimate) -> None:
        self._last_floor_est = msg
        if self._pending_floor is not None and self._floor_confirmed(
                self._pending_floor):
            self._commit(self._pending_floor, confirmed=True)

    def _tick(self) -> None:
        if self._pending_floor is None:
            return
        if self._now_s() < self._pending_deadline_s:
            return
        # Timeout: the floor node never confirmed. Proceed on the
        # COMMANDED floor — we still re-init AMCL to its cab-exit pose
        # (better than leaving localization stale), just without the
        # floor-node cross-check.
        self.get_logger().warn(
            f'floor node did not confirm floor {self._pending_floor} in '
            f'{self.relocalize_timeout_s:.1f}s — proceeding on commanded floor')
        self._commit(self._pending_floor, confirmed=False)

    def _commit(self, floor: int, confirmed: bool) -> None:
        pose = self.cab_exit.get(floor)
        if pose is None:
            self.get_logger().error(
                f'no cab-exit pose configured for floor {floor} — AMCL NOT '
                're-initialized (set cab_exit_floor_%d). Publishing '
                'current_floor anyway so the mission can proceed.' % floor)
        else:
            x, y, yaw = pose
            self._publish_initialpose(x, y, yaw)
            self.get_logger().info(
                f'AMCL re-init -> floor {floor} @ map '
                f'({x:+.2f}, {y:+.2f}, {math.degrees(yaw):+.1f}deg) '
                f'[{"floor-node confirmed" if confirmed else "timeout fallback"}]')

        m = Int32()
        m.data = int(floor)
        self.current_floor_pub.publish(m)
        self._pending_floor = None
        self._pending_deadline_s = 0.0

    def _load_map(self, path: str) -> None:
        req = self._load_map_type.Request()
        req.map_url = path
        if not self._load_map_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                'map_server/load_map not available — leaving current map')
            return
        fut = self._load_map_client.call_async(req)
        fut.add_done_callback(self._on_load_map_done)

    def _on_load_map_done(self, future) -> None:
        try:
            resp = future.result()
            # nav2_msgs/LoadMap.result: 0=success, 1=does_not_exist,
            # 2=invalid_data, 3=invalid_metadata, 255=undefined.
            if resp.result == 0:
                self.get_logger().info('map loaded successfully')
            else:
                self.get_logger().error(
                    f'load_map returned result={resp.result} '
                    '(non-success — see nav2_msgs/LoadMap)')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'load_map exception: {e}')

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
        # AMCL covariance is 6x6 (x, y, z, roll, pitch, yaw), row-major.
        cov = [0.0] * 36
        cov[0] = self.xy_cov    # xx
        cov[7] = self.xy_cov    # yy
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
