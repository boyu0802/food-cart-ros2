#!/usr/bin/env python3
"""dock_controller — drives the cart to a fixed offset from a chosen AprilTag.

One controller serves every alignment use case in the mission: kitchen
pickup, hallway elevator button-panel, in-cab button-panel, dropoff
marker. Switch targets at runtime via the SetDockTarget service.

Subscribes:
  <tag_topic> (cart_elevator_msgs/TagDetection)  default /limelight/tag
Publishes:
  <cmd_topic> (geometry_msgs/Twist)              default /dock/cmd_vel
  <status_topic> (cart_elevator_msgs/DockStatus) default /dock/status
Service:
  <set_target_srv> (cart_elevator_msgs/SetDockTarget)
                                                 default /dock/set_target

States: DISABLED / WAIT / TRACK / ALIGNED / LOST. DISABLED is the
initial state on boot — the supervisor (or a launch override) must
call SetDockTarget(enabled=true, ...) before this node will drive.
The previous behaviour of "load a target from yaml on boot" is kept as
a fallback for the standalone dock_test launch via the
`autostart_target` parameter.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from cart_elevator_msgs.msg import TagDetection, DockStatus
from cart_elevator_msgs.srv import SetDockTarget
from cart_elevator.dock.dock_control import (
    DockGains, DockTarget, pose_error, control, aligned,
)


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


class DockController(Node):
    def __init__(self) -> None:
        super().__init__('dock_controller')

        self.declare_parameters(namespace='', parameters=[
            ('tag_topic', '/limelight/tag'),
            ('cmd_topic', '/dock/cmd_vel'),
            ('status_topic', '/dock/status'),
            ('set_target_srv', '/dock/set_target'),
            # Boot-time target. If autostart_target is true the node
            # comes up enabled with these values. Useful for the
            # standalone dock_test sim where no supervisor is running.
            ('autostart_target', True),
            ('target_tag_id', 101),
            ('standoff_x', 0.6),
            ('standoff_y', 0.0),
            ('facing_yaw', math.pi),
            ('kp_x', 0.8),
            ('kp_y', 0.8),
            ('kp_yaw', 1.2),
            ('dead_x', 0.02),
            ('dead_y', 0.02),
            ('dead_yaw', 0.035),
            ('max_vx', 0.4),
            ('max_vy', 0.3),
            ('max_omega', 1.0),
            ('lost_timeout', 0.5),
            ('align_frames', 8),
            ('control_rate_hz', 20.0),
            ('status_rate_hz', 10.0),
        ])
        gp = lambda n: self.get_parameter(n).value

        self.tag_topic = gp('tag_topic')
        self.lost_timeout = float(gp('lost_timeout'))
        self.align_frames = int(gp('align_frames'))

        # Gains are global to the node (you can change them via a param
        # set if needed). Target identity + standoff + deadbands are
        # runtime-mutable via SetDockTarget.
        self.target_id = int(gp('target_tag_id'))
        self.target = DockTarget(
            standoff_x=float(gp('standoff_x')),
            standoff_y=float(gp('standoff_y')),
            facing_yaw=float(gp('facing_yaw')),
        )
        self.gains = DockGains(
            kp_x=float(gp('kp_x')), kp_y=float(gp('kp_y')),
            kp_yaw=float(gp('kp_yaw')),
            dead_x=float(gp('dead_x')), dead_y=float(gp('dead_y')),
            dead_yaw=float(gp('dead_yaw')),
            max_vx=float(gp('max_vx')), max_vy=float(gp('max_vy')),
            max_omega=float(gp('max_omega')),
        )

        self.enabled = bool(gp('autostart_target'))
        self.last_valid_t = None
        self.align_count = 0
        self.state = DockStatus.WAIT if self.enabled else DockStatus.DISABLED
        self._last_err = (0.0, 0.0, 0.0)
        self._last_vx = self._last_vy = self._last_omega = 0.0

        self.create_subscription(TagDetection, self.tag_topic, self._on_tag, 10)
        self.pub = self.create_publisher(Twist, gp('cmd_topic'), 10)
        self.status_pub = self.create_publisher(DockStatus, gp('status_topic'), 10)
        self.create_service(SetDockTarget, gp('set_target_srv'), self._on_set_target)
        self.create_timer(1.0 / float(gp('control_rate_hz')), self._tick)
        self.create_timer(1.0 / float(gp('status_rate_hz')), self._publish_status)

        self.get_logger().info(
            f'dock_controller ready (state={self._state_name()}, '
            f'target_tag_id={self.target_id}, '
            f'standoff=({self.target.standoff_x:.2f}, {self.target.standoff_y:.2f}) m)')

    # ----- service: retarget at runtime -----

    def _on_set_target(self, req: SetDockTarget.Request,
                       resp: SetDockTarget.Response) -> SetDockTarget.Response:
        if not req.enabled:
            self.enabled = False
            self.state = DockStatus.DISABLED
            self.last_valid_t = None
            self.align_count = 0
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().info('dock target cleared (disabled)')
            resp.ok = True
            resp.message = ''
            return resp

        # Bounds-check before committing.
        if req.tag_id < 0:
            resp.ok = False
            resp.message = f'invalid tag_id {req.tag_id}'
            return resp

        self.target_id = int(req.tag_id)
        self.target = DockTarget(
            standoff_x=float(req.standoff_x),
            standoff_y=float(req.standoff_y),
            facing_yaw=float(req.facing_yaw),
        )
        # Deadbands are part of the per-target spec; gains stay global.
        self.gains = DockGains(
            kp_x=self.gains.kp_x, kp_y=self.gains.kp_y, kp_yaw=self.gains.kp_yaw,
            dead_x=float(req.dead_x), dead_y=float(req.dead_y),
            dead_yaw=float(req.dead_yaw),
            max_vx=self.gains.max_vx, max_vy=self.gains.max_vy,
            max_omega=self.gains.max_omega,
        )
        self.enabled = True
        self.last_valid_t = None
        self.align_count = 0
        self.state = DockStatus.WAIT
        self.get_logger().info(
            f'dock retargeted: tag_id={self.target_id}, '
            f'standoff=({self.target.standoff_x:.2f}, {self.target.standoff_y:.2f}), '
            f'facing_yaw={self.target.facing_yaw:.2f}, '
            f'dead=({self.gains.dead_x:.3f}, {self.gains.dead_y:.3f}, {self.gains.dead_yaw:.3f})')
        resp.ok = True
        resp.message = ''
        return resp

    # ----- tag callback + control tick -----

    def _on_tag(self, msg: TagDetection) -> None:
        if not self.enabled:
            return
        if msg.tag_id != self.target_id or not msg.has_pose:
            return

        tag_x = msg.pose_in_robot_frame.position.x
        tag_y = msg.pose_in_robot_frame.position.y
        q = msg.pose_in_robot_frame.orientation
        tag_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        err = pose_error(tag_x, tag_y, tag_yaw, self.target)
        vx, vy, omega = control(err, self.gains)

        self._last_err = (err.e_x, err.e_y, err.e_yaw)
        self._last_vx, self._last_vy, self._last_omega = vx, vy, omega
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

    def _tick(self) -> None:
        if not self.enabled:
            self._publish(0.0, 0.0, 0.0)
            return

        now = self.get_clock().now()
        if (self.last_valid_t is None
                or (now - self.last_valid_t).nanoseconds * 1e-9 > self.lost_timeout):
            if self.last_valid_t is None:
                self.state = DockStatus.WAIT
            elif self.state != DockStatus.LOST:
                self.state = DockStatus.LOST
                self.get_logger().warn(
                    f'tag {self.target_id} lost > {self.lost_timeout:.2f}s, '
                    'holding zero')
            self._publish(0.0, 0.0, 0.0)
            return

        if self.state == DockStatus.ALIGNED:
            self._publish(0.0, 0.0, 0.0)
            return

        self._publish(self._last_vx, self._last_vy, self._last_omega)

    def _publish(self, vx: float, vy: float, omega: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = omega
        self.pub.publish(msg)

    def _publish_status(self) -> None:
        m = DockStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state = self.state
        m.target_tag_id = int(self.target_id) if self.enabled else -1
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
    node = DockController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
