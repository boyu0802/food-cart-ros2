#!/usr/bin/env python3
"""safe_to_enter_gate -- AND gate over the four boarding preconditions.

Subscribes to:
  /elevator/door_state     (cart_elevator_msgs/DoorState)
  /elevator/direction      (cart_elevator_msgs/ElevatorDirection)
  /elevator/floor          (cart_elevator_msgs/FloorEstimate)
  /elevator/inside_clear   (std_msgs/Bool) — future "no obstacle inside
                           the cab" check. Until the detector exists,
                           defaults to True so the gate doesn't block
                           on the missing signal.

Publishes:
  /elevator/safe_to_enter  (std_msgs/Bool) at publish_rate_hz

All four conditions must hold AND be fresh (within max_age_s):
  - door state == OPEN with confidence >= min_confidence
  - direction == IDLE with confidence >= min_confidence
  - floor == target_floor (param, set per trip by the supervisor)
  - inside_clear is True

Stale data on any input fails closed (publishes False). The throttled
logger prints the *specific* reason it's not safe so the BT logs are
diagnosable.

Ported from an earlier worktree branch. evaluate() is exposed as a
pure synchronous method so test_safe_gate.py can crank state and
assert decisions without spinning an executor.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from cart_elevator_msgs.msg import DoorState, ElevatorDirection, FloorEstimate


class SafeToEnterGate(Node):
    def __init__(self) -> None:
        super().__init__('safe_to_enter_gate')

        self.declare_parameters(namespace='', parameters=[
            ('door_topic', '/elevator/door_state'),
            ('direction_topic', '/elevator/direction'),
            ('floor_topic', '/elevator/floor'),
            ('inside_clear_topic', '/elevator/inside_clear'),
            ('output_topic', '/elevator/safe_to_enter'),
            # Topic the supervisor publishes on to retarget the gate
            # between mission phases (hallway WAIT uses starting floor,
            # in-cab WAIT uses destination floor).
            ('target_floor_topic', '/safe_to_enter/target_floor'),
            ('target_floor', 1),
            ('max_age_s', 2.0),
            ('min_confidence', 0.50),
            ('publish_rate_hz', 5.0),
            # If no /elevator/inside_clear publisher exists, assume clear.
            ('inside_clear_default', True),
            # Drop the direction / floor preconditions from the AND. Lets a
            # bring-up run gate on DOOR==OPEN alone while the direction
            # detector (placeholder) and floor detector aren't trustworthy.
            # Door is always required. Keep both True for the real mission.
            ('require_direction', True),
            ('require_floor', True),
        ])
        gp = lambda n: self.get_parameter(n).value
        self.target_floor = int(gp('target_floor'))
        self.max_age = float(gp('max_age_s'))
        self.min_conf = float(gp('min_confidence'))
        self.inside_default = bool(gp('inside_clear_default'))
        self.require_direction = bool(gp('require_direction'))
        self.require_floor = bool(gp('require_floor'))

        self.last_door: DoorState | None = None
        self.last_dir: ElevatorDirection | None = None
        self.last_floor: FloorEstimate | None = None
        self.last_inside_clear: bool | None = None
        self.last_inside_t: float = 0.0

        self.create_subscription(DoorState, gp('door_topic'),
                                 self._on_door, 10)
        self.create_subscription(ElevatorDirection, gp('direction_topic'),
                                 self._on_direction, 10)
        self.create_subscription(FloorEstimate, gp('floor_topic'),
                                 self._on_floor, 10)
        self.create_subscription(Bool, gp('inside_clear_topic'),
                                 self._on_inside, 10)
        self.create_subscription(Int32, gp('target_floor_topic'),
                                 self._on_target_floor, 10)
        self.pub = self.create_publisher(Bool, gp('output_topic'), 10)
        self.create_timer(1.0 / float(gp('publish_rate_hz')), self._publish)
        self.get_logger().info(
            f'safe_to_enter_gate ready (target_floor={self.target_floor})')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_s(msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_door(self, msg): self.last_door = msg
    def _on_direction(self, msg): self.last_dir = msg
    def _on_floor(self, msg): self.last_floor = msg

    def _on_inside(self, msg):
        self.last_inside_clear = bool(msg.data)
        self.last_inside_t = self._now_s()

    def _on_target_floor(self, msg: Int32) -> None:
        new = int(msg.data)
        if new != self.target_floor:
            self.get_logger().info(
                f'target_floor: {self.target_floor} -> {new}')
            self.target_floor = new

    def _fresh(self, msg, max_age=None) -> bool:
        if msg is None:
            return False
        max_age = self.max_age if max_age is None else max_age
        return (self._now_s() - self._stamp_s(msg)) < max_age

    def evaluate(self) -> tuple[bool, str]:
        """Pure decision logic; returns (safe, reason)."""
        if not self._fresh(self.last_door):
            return False, 'door stale or missing'
        if (self.last_door.state != DoorState.OPEN
                or self.last_door.confidence < self.min_conf):
            return False, (f'door state={self.last_door.state} '
                           f'conf={self.last_door.confidence:.2f}')

        if self.require_direction:
            if not self._fresh(self.last_dir):
                return False, 'direction stale or missing'
            if (self.last_dir.direction != ElevatorDirection.DIRECTION_IDLE
                    or self.last_dir.confidence < self.min_conf):
                return False, (f'direction={self.last_dir.direction} '
                               f'conf={self.last_dir.confidence:.2f}')

        if self.require_floor:
            if not self._fresh(self.last_floor):
                return False, 'floor stale or missing'
            if self.last_floor.floor != self.target_floor:
                return False, (f'floor={self.last_floor.floor} '
                               f'!= target={self.target_floor}')
            if self.last_floor.confidence < self.min_conf:
                return False, f'floor conf={self.last_floor.confidence:.2f}'

        if not self._current_inside_clear():
            return False, 'inside not clear'
        return True, 'all preconditions met'

    def _current_inside_clear(self) -> bool:
        if self.last_inside_clear is None:
            return self.inside_default
        age = self._now_s() - self.last_inside_t
        if age > self.max_age:
            return self.inside_default
        return self.last_inside_clear

    def _publish(self) -> None:
        safe, reason = self.evaluate()
        msg = Bool()
        msg.data = safe
        self.pub.publish(msg)
        if safe:
            self.get_logger().info('SAFE -- all preconditions met',
                                   throttle_duration_sec=2.0)
        else:
            self.get_logger().info(f'not safe: {reason}',
                                   throttle_duration_sec=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeToEnterGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
