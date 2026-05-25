#!/usr/bin/env python3
"""wall_dock_controller — docks to the in-cab button panel using lidar
wall-fitting instead of an AprilTag (there's no tag inside the cab).

Sibling of dock_controller.py: it publishes the SAME Twist on
/dock/cmd_vel and the SAME DockStatus on /dock/status (so twist_mux and
the supervisor treat it identically), but it derives the pose error from
fitted cab walls (cart_elevator.dock.wall_fit) rather than a tag pose.
The control law itself is the shared dock_control.control().

Geometry (see wall_fit): the FRONT wall (panel) gives standoff distance
(e_x) and squareness (e_yaw); a perpendicular SIDE wall gives lateral
(e_y). If no usable side wall is visible — the likely outcome given the
~24% blind lidar FOV — lateral is held and we rely on entry placement.
That gap is exactly what the in-cab repeatability experiment measures.

States mirror DockStatus: DISABLED / WAIT / TRACK / ALIGNED / LOST.
Enable/disable at runtime via a std_srvs/SetBool service (no custom msg
needed); the standalone test launch boots it enabled via `autostart`.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

from cart_elevator_msgs.msg import DockStatus
from cart_elevator.dock.dock_control import DockGains, control, aligned
from cart_elevator.dock import wall_fit


class WallDockController(Node):
    def __init__(self) -> None:
        super().__init__('wall_dock_controller')

        self.declare_parameters(namespace='', parameters=[
            ('scan_topic', '/scan'),
            ('cmd_topic', '/dock/cmd_vel'),
            ('status_topic', '/dock/status'),
            ('enable_srv', '/dock/wall/enable'),
            ('autostart', False),
            # --- target geometry ---
            ('standoff_x', 0.30),      # desired distance to the panel wall (m)
            ('side', 'auto'),          # 'left' | 'right' | 'auto' | 'none'
            ('side_target', 0.0),      # desired distance to the side wall (m)
            ('front_max_theta', 0.6),  # reject "front" if more crooked (rad)
            # --- wall fitting ---
            ('range_min', 0.15),
            ('range_max', 4.0),
            ('ransac_threshold', 0.03),
            ('min_inliers', 12),
            # --- shared dock control gains ---
            ('kp_x', 0.8), ('kp_y', 0.8), ('kp_yaw', 1.2),
            ('dead_x', 0.02), ('dead_y', 0.02), ('dead_yaw', 0.035),
            ('max_vx', 0.4), ('max_vy', 0.3), ('max_omega', 1.0),
            # --- timing / latching ---
            ('lost_timeout', 0.5),
            ('align_frames', 8),
            ('control_rate_hz', 20.0),
            ('status_rate_hz', 10.0),
        ])
        gp = lambda n: self.get_parameter(n).value

        self.standoff_x = float(gp('standoff_x'))
        self.side = str(gp('side')).lower()
        self.side_target = float(gp('side_target'))
        self.front_max_theta = float(gp('front_max_theta'))
        self.range_min = float(gp('range_min'))
        self.range_max = float(gp('range_max'))
        self.ransac_threshold = float(gp('ransac_threshold'))
        self.min_inliers = int(gp('min_inliers'))
        self.lost_timeout = float(gp('lost_timeout'))
        self.align_frames = int(gp('align_frames'))

        self.gains = DockGains(
            kp_x=float(gp('kp_x')), kp_y=float(gp('kp_y')),
            kp_yaw=float(gp('kp_yaw')),
            dead_x=float(gp('dead_x')), dead_y=float(gp('dead_y')),
            dead_yaw=float(gp('dead_yaw')),
            max_vx=float(gp('max_vx')), max_vy=float(gp('max_vy')),
            max_omega=float(gp('max_omega')),
        )

        self.enabled = bool(gp('autostart'))
        self.state = DockStatus.WAIT if self.enabled else DockStatus.DISABLED
        self.last_valid_t = None
        self.align_count = 0
        self._last_err = (0.0, 0.0, 0.0)
        self._last_cmd = (0.0, 0.0, 0.0)

        self.create_subscription(LaserScan, gp('scan_topic'), self._on_scan, 10)
        self.pub = self.create_publisher(Twist, gp('cmd_topic'), 10)
        self.status_pub = self.create_publisher(DockStatus, gp('status_topic'), 10)
        self.create_service(SetBool, gp('enable_srv'), self._on_enable)
        self.create_timer(1.0 / float(gp('control_rate_hz')), self._tick)
        self.create_timer(1.0 / float(gp('status_rate_hz')), self._publish_status)

        self.get_logger().info(
            f'wall_dock_controller ready (state={self._state_name()}, '
            f'standoff_x={self.standoff_x:.2f} m, side={self.side}'
            + (f', side_target={self.side_target:.2f} m' if self.side != 'none'
               else '') + ')')

    # ----- enable / disable -----

    def _on_enable(self, req: SetBool.Request,
                   resp: SetBool.Response) -> SetBool.Response:
        self.enabled = bool(req.data)
        if self.enabled:
            self.state = DockStatus.WAIT
            self.last_valid_t = None
            self.align_count = 0
            self.get_logger().info('wall dock enabled')
        else:
            self.state = DockStatus.DISABLED
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().info('wall dock disabled')
        resp.success = True
        resp.message = ''
        return resp

    # ----- scan -> wall fit -> error -----

    def _pick_side(self, walls: dict):
        """Pick the side wall the configured `side` asks for."""
        if self.side == 'none':
            return None
        if self.side in ('left', 'right'):
            return walls[self.side]
        # 'auto': use whichever side wall has more support.
        cands = [s for s in (walls['left'], walls['right']) if s is not None]
        return max(cands, key=lambda s: s.n_points) if cands else None

    def _on_scan(self, msg: LaserScan) -> None:
        if not self.enabled:
            return

        rmin = max(self.range_min, msg.range_min)
        rmax = min(self.range_max, msg.range_max)
        pts = wall_fit.scan_to_points(
            msg.ranges, msg.angle_min, msg.angle_increment, rmin, rmax)

        walls = wall_fit.classify_walls(
            pts, threshold=self.ransac_threshold, min_inliers=self.min_inliers)
        front = walls['front']

        # No usable front wall this frame -> let the lost-timeout handle it.
        if front is None or abs(front.theta) > self.front_max_theta:
            return

        side = self._pick_side(walls)
        side_target = self.side_target if side is not None else None
        err = wall_fit.pose_from_walls(front, side, self.standoff_x, side_target)
        vx, vy, omega = control(err, self.gains)

        self._last_err = (err.e_x, err.e_y, err.e_yaw)
        self._last_cmd = (vx, vy, omega)
        self.last_valid_t = self.get_clock().now()

        if aligned(err, self.gains):
            self.align_count += 1
            if (self.align_count >= self.align_frames
                    and self.state != DockStatus.ALIGNED):
                self.state = DockStatus.ALIGNED
                self.get_logger().info(
                    f'ALIGNED (e_x={err.e_x:+.3f} m, e_y={err.e_y:+.3f} m, '
                    f'e_yaw={math.degrees(err.e_yaw):+.1f} deg)')
        else:
            self.align_count = 0
            if self.state != DockStatus.TRACK:
                self.state = DockStatus.TRACK

    # ----- control tick -----

    def _tick(self) -> None:
        if not self.enabled:
            # Stay silent on /dock/cmd_vel while disabled — the tag dock
            # shares this topic, and a stream of zeros from us would fight
            # whichever controller is active. One zero is already sent on
            # the disable transition (_on_enable).
            return

        now = self.get_clock().now()
        if (self.last_valid_t is None
                or (now - self.last_valid_t).nanoseconds * 1e-9 > self.lost_timeout):
            if self.last_valid_t is None:
                self.state = DockStatus.WAIT
            elif self.state != DockStatus.LOST:
                self.state = DockStatus.LOST
                self.get_logger().warn(
                    f'no usable cab wall > {self.lost_timeout:.2f}s, holding zero')
            self._publish(0.0, 0.0, 0.0)
            return

        if self.state == DockStatus.ALIGNED:
            self._publish(0.0, 0.0, 0.0)
            return

        self._publish(*self._last_cmd)

    def _publish(self, vx: float, vy: float, omega: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(omega)
        self.pub.publish(msg)

    def _publish_status(self) -> None:
        m = DockStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state = self.state
        m.target_tag_id = -1   # no tag — wall-fit mode
        m.e_x, m.e_y, m.e_yaw = self._last_err
        self.status_pub.publish(m)

    def _state_name(self) -> str:
        return {
            DockStatus.DISABLED: 'DISABLED',
            DockStatus.WAIT: 'WAIT',
            DockStatus.TRACK: 'TRACK',
            DockStatus.ALIGNED: 'ALIGNED',
            DockStatus.LOST: 'LOST',
        }.get(self.state, '?')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WallDockController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
