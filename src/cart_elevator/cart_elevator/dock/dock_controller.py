#!/usr/bin/env python3
"""dock_controller — drives the cart to a fixed offset from a chosen AprilTag.

Subscribes:
  /limelight/tag (cart_elevator_msgs/TagDetection)
Publishes:
  /dock/cmd_vel (geometry_msgs/Twist)

Only acts on detections whose tag_id matches target_tag_id and that have
has_pose=True. On tag-loss for longer than lost_timeout, publishes zero.
When the pose error is inside the deadband for align_frames in a row, the
node enters ALIGNED state and holds zero velocity.

The output topic is intentionally /dock/cmd_vel (not /cmd_vel) so this can
co-exist with Nav2 during bring-up; a higher-level supervisor decides which
source feeds nt_bridge.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from cart_elevator_msgs.msg import TagDetection
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
            ('target_tag_id', 101),
            # Target pose (tag-in-robot when docked).
            ('standoff_x', 0.6),
            ('standoff_y', 0.0),
            ('facing_yaw', math.pi),
            # Gains.
            ('kp_x', 0.8),
            ('kp_y', 0.8),
            ('kp_yaw', 1.2),
            ('dead_x', 0.02),
            ('dead_y', 0.02),
            ('dead_yaw', 0.035),
            ('max_vx', 0.4),
            ('max_vy', 0.3),
            ('max_omega', 1.0),
            # Behavior.
            ('lost_timeout', 0.5),     # s of no valid detection -> zero vel
            ('align_frames', 8),       # consecutive in-deadband frames -> ALIGNED
            ('control_rate_hz', 20.0),
        ])
        gp = lambda n: self.get_parameter(n).value

        self.tag_topic = gp('tag_topic')
        self.target_id = int(gp('target_tag_id'))
        self.lost_timeout = float(gp('lost_timeout'))
        self.align_frames = int(gp('align_frames'))

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

        self.last_valid_t = None  # ROS time of last in-target pose
        self.align_count = 0
        self.state = 'WAIT'  # WAIT | TRACK | ALIGNED | LOST

        self.create_subscription(TagDetection, self.tag_topic, self._on_tag, 10)
        self.pub = self.create_publisher(Twist, gp('cmd_topic'), 10)
        self.create_timer(1.0 / float(gp('control_rate_hz')), self._tick)

        self._last_vx = self._last_vy = self._last_omega = 0.0
        self.get_logger().info(
            f'dock_controller ready (target_tag_id={self.target_id}, '
            f'standoff=({self.target.standoff_x:.2f}, {self.target.standoff_y:.2f}) m)')

    def _on_tag(self, msg: TagDetection) -> None:
        if msg.tag_id != self.target_id or not msg.has_pose:
            return

        tag_x = msg.pose_in_robot_frame.position.x
        tag_y = msg.pose_in_robot_frame.position.y
        q = msg.pose_in_robot_frame.orientation
        tag_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        err = pose_error(tag_x, tag_y, tag_yaw, self.target)
        vx, vy, omega = control(err, self.gains)

        self._last_vx, self._last_vy, self._last_omega = vx, vy, omega
        self.last_valid_t = self.get_clock().now()

        if aligned(err, self.gains):
            self.align_count += 1
            if self.align_count >= self.align_frames and self.state != 'ALIGNED':
                self.state = 'ALIGNED'
                self.get_logger().info(
                    f'ALIGNED (e_x={err.e_x:+.3f} m, e_y={err.e_y:+.3f} m, '
                    f'e_yaw={math.degrees(err.e_yaw):+.1f} deg)')
        else:
            self.align_count = 0
            if self.state != 'TRACK':
                self.state = 'TRACK'

    def _tick(self) -> None:
        now = self.get_clock().now()

        if (self.last_valid_t is None
                or (now - self.last_valid_t).nanoseconds * 1e-9 > self.lost_timeout):
            if self.state != 'LOST':
                if self.last_valid_t is None:
                    self.state = 'WAIT'
                else:
                    self.state = 'LOST'
                    self.get_logger().warn(
                        f'tag {self.target_id} lost > {self.lost_timeout:.2f}s, '
                        'holding zero')
            self._publish(0.0, 0.0, 0.0)
            return

        if self.state == 'ALIGNED':
            self._publish(0.0, 0.0, 0.0)
            return

        self._publish(self._last_vx, self._last_vy, self._last_omega)

    def _publish(self, vx: float, vy: float, omega: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = omega
        self.pub.publish(msg)


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
