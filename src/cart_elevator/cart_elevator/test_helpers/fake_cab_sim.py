#!/usr/bin/env python3
"""fake_cab_sim — closed-loop simulator for the lidar wall-dock controller.

Integrates a flat-ground swerve robot from /dock/cmd_vel inside a
rectangular elevator cab, and publishes a synthetic /scan (LaserScan)
of the cab walls as seen from the cart. Mirrors fake_dock_sim, but for
walls instead of an AprilTag.

The cab geometry + the blind-FOV mask are parameters, so you can recreate
the real cab once measured and watch whether wall-fitting still lands the
pusher within tolerance (the in-cab repeatability experiment).

    ros2 launch cart_elevator wall_dock_test.launch.py robot_x0:=-0.25 \
        robot_y0:=0.18 robot_yaw0:=0.15
    ros2 topic echo /dock/status     # watch e_x/e_y/e_yaw -> 0, state -> ALIGNED
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from cart_elevator.test_helpers.cab_model import cab_scan_ranges


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class FakeCabSim(Node):
    def __init__(self) -> None:
        super().__init__('fake_cab_sim')

        self.declare_parameters(namespace='', parameters=[
            ('cmd_topic', '/dock/cmd_vel'),
            ('scan_topic', '/scan'),
            ('frame_id', 'laser'),
            # Cab interior bounds (m): x_back, x_front, y_right, y_left.
            # Default ~1.4 m square cab centered at origin; panel on +x.
            ('x_back', -0.7), ('x_front', 0.7),
            ('y_right', -0.7), ('y_left', 0.7),
            # Cart start pose in the cab (m, m, rad).
            ('robot_x0', -0.25), ('robot_y0', 0.18), ('robot_yaw0', 0.15),
            # Lidar model.
            ('n_beams', 360),
            ('angle_min', -math.pi),
            ('range_min', 0.12),
            ('range_max', 6.0),
            ('noise_m', 0.01),
            # Flat [start_deg, end_deg, ...] body-frame blind sectors.
            ('blind_sectors_deg', []),
            # Timing.
            ('sim_rate_hz', 50.0),
            ('scan_rate_hz', 15.0),
            ('cmd_timeout', 0.5),
            ('log_rate_hz', 2.0),
        ])
        gp = lambda n: self.get_parameter(n).value

        self.frame_id = gp('frame_id')
        self.bounds = (float(gp('x_back')), float(gp('x_front')),
                       float(gp('y_right')), float(gp('y_left')))
        self.n_beams = int(gp('n_beams'))
        self.angle_min = float(gp('angle_min'))
        self.angle_inc = (2.0 * math.pi) / self.n_beams
        self.range_min = float(gp('range_min'))
        self.range_max = float(gp('range_max'))
        self.noise = float(gp('noise_m'))
        self.blind = [float(v) for v in gp('blind_sectors_deg')]
        self.cmd_timeout = float(gp('cmd_timeout'))

        self.rx = float(gp('robot_x0'))
        self.ry = float(gp('robot_y0'))
        self.ryaw = float(gp('robot_yaw0'))
        self.vx = self.vy = self.omega = 0.0
        self.last_cmd_t = None

        import random
        self._rng = random.Random(0)

        self.create_subscription(Twist, gp('cmd_topic'), self._on_cmd, 10)
        self.pub = self.create_publisher(LaserScan, gp('scan_topic'), 10)

        self.sim_dt = 1.0 / float(gp('sim_rate_hz'))
        self.create_timer(self.sim_dt, self._integrate)
        self.create_timer(1.0 / float(gp('scan_rate_hz')), self._publish_scan)
        self.create_timer(1.0 / float(gp('log_rate_hz')), self._log)

        self.get_logger().info(
            f'fake_cab_sim ready  cab={self.bounds}  '
            f'start=({self.rx:.2f},{self.ry:.2f},{math.degrees(self.ryaw):.0f}deg)  '
            f'beams={self.n_beams} blind={self.blind}')

    def _on_cmd(self, msg: Twist) -> None:
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.omega = float(msg.angular.z)
        self.last_cmd_t = self.get_clock().now()

    def _integrate(self) -> None:
        now = self.get_clock().now()
        if (self.last_cmd_t is None
                or (now - self.last_cmd_t).nanoseconds * 1e-9 > self.cmd_timeout):
            return
        c, s = math.cos(self.ryaw), math.sin(self.ryaw)
        vx_w = c * self.vx - s * self.vy
        vy_w = s * self.vx + c * self.vy
        self.rx += vx_w * self.sim_dt
        self.ry += vy_w * self.sim_dt
        self.ryaw = wrap(self.ryaw + self.omega * self.sim_dt)

    def _publish_scan(self) -> None:
        ranges = cab_scan_ranges(
            self.rx, self.ry, self.ryaw, self.bounds,
            self.angle_min, self.angle_inc, self.n_beams,
            noise=self.noise, rng=self._rng, blind_pairs=self.blind)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_min + self.angle_inc * (self.n_beams - 1)
        msg.angle_increment = self.angle_inc
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = [float(r) for r in ranges]
        self.pub.publish(msg)

    def _log(self) -> None:
        d_front = self.bounds[1] - self.rx
        self.get_logger().info(
            f'pose=({self.rx:+.3f}, {self.ry:+.3f}, '
            f'{math.degrees(self.ryaw):+.1f}deg)  dist_to_panel={d_front:.3f} m')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeCabSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
