#!/usr/bin/env python3
"""fake_dock_sim — closed-loop simulator for the AprilTag docking controller.

Integrates a flat-ground swerve robot from incoming /dock/cmd_vel, holds a
static tag at a fixed world pose, and publishes the tag's pose in the robot's
body frame as cart_elevator_msgs/TagDetection on /limelight/tag.

Models a Limelight FOV cone: when the tag is behind the robot, outside the
horizontal half-angle, or past max_range, the sim publishes tag_id=-1 with
has_pose=False so the controller's lost-tag failsafe is exercisable.

This is intentionally separate from fake_tag (the scripted scenario player) so
the two stay independently useful.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from cart_elevator_msgs.msg import TagDetection


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quat_zw(yaw: float) -> tuple[float, float]:
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class FakeDockSim(Node):
    def __init__(self) -> None:
        super().__init__('fake_dock_sim')

        self.declare_parameters(namespace='', parameters=[
            ('cmd_topic', '/dock/cmd_vel'),
            ('tag_topic', '/limelight/tag'),
            ('frame_id', 'limelight'),
            ('tag_id', 101),
            # Robot start pose in world (x_m, y_m, yaw_rad).
            ('robot_x0', 0.0),
            ('robot_y0', 0.0),
            ('robot_yaw0', 0.0),
            # Tag pose in world. Default: 1.5m ahead, 0.3m to robot's left,
            # facing the robot (yaw = pi in world means tag's +x points to -x).
            ('tag_x', 1.5),
            ('tag_y', 0.3),
            ('tag_yaw', math.pi),
            # FOV model.
            ('fov_half_deg', 30.0),
            ('max_range_m', 6.0),
            # Sim timing.
            ('sim_rate_hz', 50.0),
            ('publish_rate_hz', 20.0),
            ('cmd_timeout', 0.5),
        ])
        gp = lambda n: self.get_parameter(n).value

        self.frame_id = gp('frame_id')
        self.tag_id = int(gp('tag_id'))
        self.fov_half = math.radians(float(gp('fov_half_deg')))
        self.max_range = float(gp('max_range_m'))
        self.cmd_timeout = float(gp('cmd_timeout'))

        self.rx = float(gp('robot_x0'))
        self.ry = float(gp('robot_y0'))
        self.ryaw = float(gp('robot_yaw0'))

        self.tx_w = float(gp('tag_x'))
        self.ty_w = float(gp('tag_y'))
        self.tyaw_w = float(gp('tag_yaw'))

        self.vx_body = 0.0
        self.vy_body = 0.0
        self.omega = 0.0
        self.last_cmd_t = None

        self.create_subscription(Twist, gp('cmd_topic'), self._on_cmd, 10)
        self.pub = self.create_publisher(TagDetection, gp('tag_topic'), 10)

        self.sim_dt = 1.0 / float(gp('sim_rate_hz'))
        self.create_timer(self.sim_dt, self._integrate)
        self.create_timer(1.0 / float(gp('publish_rate_hz')), self._publish)

        self.get_logger().info(
            f'fake_dock_sim ready  robot=({self.rx:.2f},{self.ry:.2f},{self.ryaw:.2f}) '
            f'tag=({self.tx_w:.2f},{self.ty_w:.2f},{self.tyaw_w:.2f}) '
            f'fov=+/-{math.degrees(self.fov_half):.1f}deg')

    def _on_cmd(self, msg: Twist) -> None:
        self.vx_body = float(msg.linear.x)
        self.vy_body = float(msg.linear.y)
        self.omega = float(msg.angular.z)
        self.last_cmd_t = self.get_clock().now()

    def _integrate(self) -> None:
        now = self.get_clock().now()
        # Fail-safe: drop velocities if no recent cmd.
        if (self.last_cmd_t is None
                or (now - self.last_cmd_t).nanoseconds * 1e-9 > self.cmd_timeout):
            vx_b, vy_b, w = 0.0, 0.0, 0.0
        else:
            vx_b, vy_b, w = self.vx_body, self.vy_body, self.omega

        c, s = math.cos(self.ryaw), math.sin(self.ryaw)
        vx_w = c * vx_b - s * vy_b
        vy_w = s * vx_b + c * vy_b

        self.rx += vx_w * self.sim_dt
        self.ry += vy_w * self.sim_dt
        self.ryaw = wrap(self.ryaw + w * self.sim_dt)

    def _tag_in_robot(self) -> tuple[float, float, float, float, float]:
        """Return tag pose in robot body frame: (x, y, yaw, range, bearing)."""
        dx = self.tx_w - self.rx
        dy = self.ty_w - self.ry
        c, s = math.cos(self.ryaw), math.sin(self.ryaw)
        # Inverse rotation by ryaw.
        tag_x_b = c * dx + s * dy
        tag_y_b = -s * dx + c * dy
        tag_yaw_b = wrap(self.tyaw_w - self.ryaw)
        rng = math.hypot(tag_x_b, tag_y_b)
        bearing = math.atan2(tag_y_b, tag_x_b)
        return tag_x_b, tag_y_b, tag_yaw_b, rng, bearing

    def _publish(self) -> None:
        tag_x_b, tag_y_b, tag_yaw_b, rng, bearing = self._tag_in_robot()

        msg = TagDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        in_fov = (abs(bearing) < self.fov_half and rng < self.max_range
                  and tag_x_b > 0.0)
        if not in_fov:
            msg.tag_id = -1
            msg.area = 0.0
            msg.tx_deg = 0.0
            msg.ty_deg = 0.0
            msg.has_pose = False
            self.pub.publish(msg)
            return

        msg.tag_id = self.tag_id
        # Crude area model: tag apparent size ~ 1/range^2 (no clipping needed).
        msg.area = max(0.0, min(1.0, 0.05 / (rng * rng)))
        msg.tx_deg = math.degrees(bearing)
        msg.ty_deg = 0.0
        msg.has_pose = True
        msg.pose_in_robot_frame.position.x = tag_x_b
        msg.pose_in_robot_frame.position.y = tag_y_b
        msg.pose_in_robot_frame.position.z = 0.0
        qz, qw = yaw_to_quat_zw(tag_yaw_b)
        msg.pose_in_robot_frame.orientation.x = 0.0
        msg.pose_in_robot_frame.orientation.y = 0.0
        msg.pose_in_robot_frame.orientation.z = qz
        msg.pose_in_robot_frame.orientation.w = qw
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeDockSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
