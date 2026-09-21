#!/usr/bin/env python3
"""
Elevator Controller Node for Gazebo Simulation

Fixes vs previous version:
- Uses ROS clock (sim-time-aware) instead of time.time()
- Publishes cart_elevator_msgs types so supervisor receives them correctly
- Adds /elevator/press_button subscriber -> /elevator/press_done after 1.5s
- Adds /elevator/inside_clear publisher (True while door is OPEN at floor)
- Adds door auto-close after hold_time_s
- Fixes infinite door-loop bug (door close -> re-trigger move at same floor)
- Replaces fragile 10Hz modulo check with a dedicated publish timer
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64, String, Int32, Bool
import math
from enum import Enum
from dataclasses import dataclass
from typing import Optional

try:
    from cart_elevator_msgs.msg import DoorState as DoorStateMsg
    from cart_elevator_msgs.msg import FloorEstimate, ElevatorDirection
    _HAVE_CART_MSGS = True
except ImportError:
    _HAVE_CART_MSGS = False


# DoorState int constants matching mission.py
DOOR_UNKNOWN  = 0
DOOR_CLOSED   = 1
DOOR_OPENING  = 2
DOOR_OPEN     = 3
DOOR_CLOSING  = 4


class _DS(Enum):
    OPEN    = DOOR_OPEN
    CLOSED  = DOOR_CLOSED
    OPENING = DOOR_OPENING
    CLOSING = DOOR_CLOSING


class _Dir(Enum):
    UP   =  1
    DOWN = -1
    IDLE =  0


@dataclass
class ElevatorConfig:
    floor_height_m:     float = 3.8
    elevator_speed_m_s: float = 1.1
    elevator_accel_m_s2:float = 1.0
    n_floors:           int   = 5
    door_open_time_s:   float = 2.0
    door_hold_time_s:   float = 5.0
    door_close_time_s:  float = 2.0
    door_slide_m:       float = 0.65
    press_done_delay_s: float = 1.5   # simulated pneumatic press time


class ElevatorController(Node):
    def __init__(self):
        super().__init__('elevator_controller')
        self.cfg = ElevatorConfig()
        self._cbg = ReentrantCallbackGroup()

        # --- motion state ---
        self.current_floor    = 1
        self.target_floor: Optional[int] = None
        self.door_state       = _DS.CLOSED
        self.direction        = _Dir.IDLE
        self.pos_m            = 0.0
        self.vel_m_s          = 0.0
        self.accel_m_s2       = 0.0

        # --- trajectory ---
        self._traj_active     = False
        self._traj_t0         = 0.0
        self._traj_pos0       = 0.0
        self._traj_posf       = 0.0
        self._traj_dur        = 0.0
        self._traj_t_accel    = 0.0
        self._traj_t_cruise   = 0.0
        self._traj_v_peak     = 0.0

        # --- door animation ---
        self._door_active     = False
        self._door_t0         = 0.0
        self._door_dur        = 0.0
        self._door_target     = _DS.CLOSED
        self._door_floor      = 1
        # hold timer: set when door finishes opening
        self._door_hold_t0    = 0.0
        self._door_holding    = False

        # --- press state ---
        self._press_t0        = 0.0
        self._press_pending   = False

        # ======= Publishers =======
        # Gazebo joint commands
        self.platform_cmd_pub = self.create_publisher(
            Float64, '/model/elevator_cab/joint/elevator_joint/0/cmd_pos', 10)
        self.door_f1_left_pub = self.create_publisher(
            Float64, '/model/elevator_door_f1/joint/door_f1_left_joint/0/cmd_pos', 10)
        self.door_f1_right_pub = self.create_publisher(
            Float64, '/model/elevator_door_f1/joint/door_f1_right_joint/0/cmd_pos', 10)
        self.door_f4_left_pub = self.create_publisher(
            Float64, '/model/elevator_door_f4/joint/door_f4_left_joint/0/cmd_pos', 10)
        self.door_f4_right_pub = self.create_publisher(
            Float64, '/model/elevator_door_f4/joint/door_f4_right_joint/0/cmd_pos', 10)

        # ROS state (cart_elevator_msgs if available, fallback String/Int)
        if _HAVE_CART_MSGS:
            self._door_pub = self.create_publisher(DoorStateMsg, '/elevator/door_state', 10)
            self._floor_pub = self.create_publisher(FloorEstimate, '/elevator/floor', 10)
            self._dir_pub = self.create_publisher(ElevatorDirection, '/elevator/direction', 10)
        else:
            self._door_pub = self.create_publisher(Int32, '/elevator/door_state', 10)
            self._floor_pub = self.create_publisher(Int32, '/elevator/floor', 10)
            self._dir_pub = self.create_publisher(Int32, '/elevator/direction', 10)
        self._accel_pub = self.create_publisher(Float64, '/elevator/accel_z', 10)
        self._press_done_pub = self.create_publisher(Bool, '/elevator/press_done', 10)
        self._inside_clear_pub = self.create_publisher(Bool, '/elevator/inside_clear', 10)
        self._safe_pub = self.create_publisher(Bool, '/elevator/safe_to_enter', 10)

        # ======= Subscribers =======
        self.create_subscription(
            Int32, '/elevator/call', self._on_call, 10, callback_group=self._cbg)
        self.create_subscription(
            Int32, '/elevator/goto_floor', self._on_goto, 10, callback_group=self._cbg)
        self.create_subscription(
            String, '/elevator/press_button', self._on_press, 10, callback_group=self._cbg)

        # ======= Timers =======
        self._update_timer = self.create_timer(0.01, self._update, callback_group=self._cbg)
        self._pub_timer = self.create_timer(0.1, self._publish_state, callback_group=self._cbg)

        self.get_logger().info(
            f'Elevator controller ready: {self.cfg.n_floors} floors, '
            f'{self.cfg.floor_height_m}m/floor, '
            f'v={self.cfg.elevator_speed_m_s} a={self.cfg.elevator_accel_m_s2}')

    # =============== helpers ===============

    def _now_sec(self) -> float:
        """Sim-time-aware clock."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _floor_to_pos(self, floor: int) -> float:
        return (floor - 1) * self.cfg.floor_height_m

    def _pos_to_floor(self, pos: float) -> int:
        f = round(pos / self.cfg.floor_height_m) + 1
        return max(1, min(self.cfg.n_floors, f))

    # =============== callbacks ===============

    def _on_call(self, msg: Int32):
        t = msg.data
        if t < 1 or t > self.cfg.n_floors:
            return
        self.get_logger().info(f'Call to floor {t}')
        self._begin_move(t)

    def _on_goto(self, msg: Int32):
        t = msg.data
        if t < 1 or t > self.cfg.n_floors:
            return
        self.get_logger().info(f'Goto floor {t}')
        self._begin_move(t)

    def _on_press(self, msg: String):
        """Handle button press commands from supervisor."""
        button = msg.data
        self.get_logger().info(f'Press button: {button}')
        # Simulate mechanical press delay then publish press_done
        self._press_pending = True
        self._press_t0 = self._now_sec()
        # If it's a floor button, also trigger goto
        if button.startswith('floor_'):
            try:
                floor_num = int(button.split('_')[1])
                self._begin_move(floor_num)
            except (ValueError, IndexError):
                pass
        elif button == 'call_up' or button == 'call_down':
            # Call button just arms — elevator is already at this floor
            # or needs to come here. For sim, call to current floor.
            self._begin_move(self.current_floor)

    # =============== motion ===============

    def _begin_move(self, target: int):
        if self._traj_active:
            self.get_logger().warn('Move already in progress, ignoring')
            return

        # Close doors first if open
        if self.door_state != _DS.CLOSED:
            self._door_holding = False
            self._start_door(_DS.CLOSED, self.current_floor)
            self.target_floor = target
            return

        target_pos = self._floor_to_pos(target)
        distance = abs(target_pos - self.pos_m)

        if distance < 0.01:
            # Already here — just open doors (no re-trigger loop)
            if self.door_state == _DS.CLOSED:
                self._start_door(_DS.OPEN, target)
            return

        self.target_floor = target
        self.direction = _Dir.UP if target_pos > self.pos_m else _Dir.DOWN

        v = self.cfg.elevator_speed_m_s
        a = self.cfg.elevator_accel_m_s2
        d_ramp = 0.5 * v * v / a

        if distance < 2.0 * d_ramp:
            v_peak = math.sqrt(a * distance)
            t_accel = v_peak / a
            t_cruise = 0.0
        else:
            v_peak = v
            t_accel = v / a
            t_cruise = (distance - 2.0 * d_ramp) / v

        self._traj_t_accel  = t_accel
        self._traj_t_cruise = t_cruise
        self._traj_v_peak   = v_peak
        self._traj_dur      = t_accel + t_cruise + t_accel
        self._traj_t0       = self._now_sec()
        self._traj_pos0     = self.pos_m
        self._traj_posf     = target_pos
        self._traj_active   = True

        self.get_logger().info(
            f'Move floor {self.current_floor}->{target} '
            f'dist={distance:.2f}m dur={self._traj_dur:.2f}s')

    def _update_trajectory(self, now: float):
        elapsed = now - self._traj_t0
        sign = 1.0 if self.direction == _Dir.UP else -1.0
        a = self.cfg.elevator_accel_m_s2
        t_a = self._traj_t_accel
        t_c = self._traj_t_cruise
        vp  = self._traj_v_peak

        if elapsed >= self._traj_dur:
            self.pos_m = self._traj_posf
            self.vel_m_s = 0.0
            self.accel_m_s2 = 0.0
            self._traj_active = False
            self.direction = _Dir.IDLE
            self.current_floor = self.target_floor
            cmd = Float64(); cmd.data = self.pos_m
            self.platform_cmd_pub.publish(cmd)
            self.get_logger().info(f'Arrived floor {self.current_floor}')
            self._start_door(_DS.OPEN, self.current_floor)
            return

        if elapsed < t_a:
            self.accel_m_s2 = a * sign
            self.vel_m_s = a * elapsed * sign
            disp = 0.5 * a * elapsed * elapsed
        elif elapsed < t_a + t_c:
            self.accel_m_s2 = 0.0
            self.vel_m_s = vp * sign
            disp = 0.5 * a * t_a * t_a + vp * (elapsed - t_a)
        else:
            t_d = elapsed - t_a - t_c
            self.accel_m_s2 = -a * sign
            self.vel_m_s = (vp - a * t_d) * sign
            disp = 0.5 * a * t_a * t_a + vp * t_c + vp * t_d - 0.5 * a * t_d * t_d

        self.pos_m = self._traj_pos0 + disp * sign
        self.current_floor = self._pos_to_floor(self.pos_m)
        cmd = Float64(); cmd.data = self.pos_m
        self.platform_cmd_pub.publish(cmd)

    # =============== doors ===============

    def _start_door(self, target: _DS, floor: int):
        self._door_active   = True
        self._door_t0       = self._now_sec()
        self._door_target   = target
        self._door_floor    = floor
        self._door_holding  = False
        if target == _DS.OPEN:
            self._door_dur  = self.cfg.door_open_time_s
            self.door_state = _DS.OPENING
        else:
            self._door_dur  = self.cfg.door_close_time_s
            self.door_state = _DS.CLOSING

    def _update_door(self, now: float):
        elapsed = now - self._door_t0

        # Hold phase: door is fully open, waiting before auto-close
        if self._door_holding:
            if now - self._door_t0 >= self.cfg.door_hold_time_s:
                self._door_holding = False
                self._start_door(_DS.CLOSED, self._door_floor)
            return

        progress = min(1.0, elapsed / self._door_dur)
        if self._door_target == _DS.OPEN:
            door_pos = progress * self.cfg.door_slide_m
        else:
            door_pos = (1.0 - progress) * self.cfg.door_slide_m

        left = Float64(); left.data = -door_pos
        right = Float64(); right.data = door_pos
        if self._door_floor == 1:
            self.door_f1_left_pub.publish(left)
            self.door_f1_right_pub.publish(right)
        elif self._door_floor == 4:
            self.door_f4_left_pub.publish(left)
            self.door_f4_right_pub.publish(right)

        if progress >= 1.0:
            self._door_active = False
            self.door_state = self._door_target

            if self.door_state == _DS.OPEN:
                # Start hold timer; do NOT immediately re-trigger move
                self._door_holding = True
                self._door_t0 = now
            elif self.door_state == _DS.CLOSED and self.target_floor is not None:
                # Doors closed after a pending move request
                target = self.target_floor
                self.target_floor = None
                self._begin_move(target)

    # =============== main update ===============

    def _update(self):
        now = self._now_sec()
        if self._traj_active:
            self._update_trajectory(now)
        if self._door_active or self._door_holding:
            self._update_door(now)
        # Press done after delay
        if self._press_pending and (now - self._press_t0) >= self.cfg.press_done_delay_s:
            self._press_pending = False
            msg = Bool(); msg.data = True
            self._press_done_pub.publish(msg)
            self.get_logger().info('press_done published')

    # =============== state publish ===============

    def _publish_state(self):
        if _HAVE_CART_MSGS:
            d = DoorStateMsg()
            d.state = self.door_state.value
            self._door_pub.publish(d)

            f = FloorEstimate()
            f.floor = self.current_floor
            f.confidence = 0.0 if self._traj_active else 1.0
            self._floor_pub.publish(f)

            dr = ElevatorDirection()
            dr.direction = self.direction.value
            self._dir_pub.publish(dr)
        else:
            d = Int32(); d.data = self.door_state.value
            self._door_pub.publish(d)
            f = Int32(); f.data = self.current_floor
            self._floor_pub.publish(f)
            dr = Int32(); dr.data = self.direction.value
            self._dir_pub.publish(dr)

        az = Float64(); az.data = self.accel_m_s2
        self._accel_pub.publish(az)

        # inside_clear: True when door is open and cab is not moving
        clear = Bool()
        clear.data = (self.door_state == _DS.OPEN and not self._traj_active)
        self._inside_clear_pub.publish(clear)

        # safe_to_enter: simulates what safe_to_enter_gate computes in real stack
        # (door OPEN + direction IDLE + not moving)
        safe = Bool()
        safe.data = (
            self.door_state == _DS.OPEN
            and self.direction == _Dir.IDLE
            and not self._traj_active
        )
        self._safe_pub.publish(safe)


def main(args=None):
    rclpy.init(args=args)
    node = ElevatorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
