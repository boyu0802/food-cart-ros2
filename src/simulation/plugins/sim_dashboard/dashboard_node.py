#!/usr/bin/env python3
"""
Sim Dashboard Node

Terminal dashboard for monitoring and controlling the simulation.
- Shows live elevator state, mission state, robot velocity, door state
- Lets you edit key variables via ROS parameters at runtime
- Logs timestamped events to a file

Run:  ros2 run simulation sim_dashboard
Edit: ros2 param set /sim_dashboard elevator_speed_m_s 0.5
Log:  tail -f /tmp/sim_dashboard.log
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32, Float64
from geometry_msgs.msg import Twist
from datetime import datetime


LOG_FILE = '/tmp/sim_dashboard.log'


class SimDashboard(Node):
    def __init__(self):
        super().__init__('sim_dashboard')

        # ── editable parameters (ros2 param set /sim_dashboard <name> <val>) ──
        self.declare_parameter('elevator_speed_m_s',  1.1)
        self.declare_parameter('elevator_accel_m_s2', 1.0)
        self.declare_parameter('door_hold_time_s',    5.0)
        self.declare_parameter('door_open_time_s',    2.0)
        self.declare_parameter('door_close_time_s',   2.0)
        self.declare_parameter('target_floor',        4)
        self.declare_parameter('auto_call_floor',     0)   # 0 = disabled

        # ── state ──
        self._door_state   = 'UNKNOWN'
        self._floor        = 0
        self._direction    = 'UNKNOWN'
        self._mission_state= 'UNKNOWN'
        self._safe_to_enter= False
        self._inside_clear = False
        self._press_done   = False
        self._lin_x        = 0.0
        self._ang_z        = 0.0
        self._accel_z      = 0.0
        self._events: list[str] = []

        # ── subscriptions ──
        self.create_subscription(Int32,  '/elevator/door_state',    self._on_door,    10)
        self.create_subscription(Int32,  '/elevator/floor',         self._on_floor,   10)
        self.create_subscription(Int32,  '/elevator/direction',     self._on_dir,     10)
        self.create_subscription(String, '/mission/state',          self._on_mission, 10)
        self.create_subscription(Bool,   '/elevator/safe_to_enter', self._on_safe,    10)
        self.create_subscription(Bool,   '/elevator/inside_clear',  self._on_clear,   10)
        self.create_subscription(Bool,   '/elevator/press_done',    self._on_press,   10)
        self.create_subscription(Twist,  '/cmd_vel',                self._on_vel,     10)
        self.create_subscription(Float64,'/elevator/accel_z',       self._on_accel,   10)

        # ── publishers (manual injection) ──
        self._call_pub  = self.create_publisher(Int32, '/elevator/call',       10)
        self._goto_pub  = self.create_publisher(Int32, '/elevator/goto_floor', 10)
        self._start_pub = self.create_publisher(Bool,  '/mission/start',       10)

        # ── param-forward timer: push editable params to elevator node ──
        self._param_pub = self.create_publisher(String, '/sim_dashboard/params', 10)
        self.create_timer(1.0, self._forward_params)

        # ── display timer ──
        self.create_timer(0.5, self._render)

        self._log_file = open(LOG_FILE, 'a')
        self._log('=== Dashboard started ===')
        self.get_logger().info(f'Dashboard running. Log: {LOG_FILE}')
        self.get_logger().info('Edit params: ros2 param set /sim_dashboard <param> <value>')

    # ── door state int -> label ──
    _DOOR_LABELS = {1: 'CLOSED', 2: 'OPENING', 3: 'OPEN', 4: 'CLOSING'}
    _DIR_LABELS  = {1: 'UP', -1: 'DOWN', 0: 'IDLE'}

    # ── callbacks ──

    def _on_door(self, msg):
        label = self._DOOR_LABELS.get(msg.data, str(msg.data))
        if label != self._door_state:
            self._log(f'door_state: {self._door_state} -> {label}')
        self._door_state = label

    def _on_floor(self, msg):
        v = msg.data
        if v != self._floor:
            self._log(f'floor: {self._floor} -> {v}')
        self._floor = v

    def _on_dir(self, msg):
        label = self._DIR_LABELS.get(msg.data, str(msg.data))
        if label != self._direction:
            self._log(f'direction: {self._direction} -> {label}')
        self._direction = label

    def _on_mission(self, msg):
        if msg.data != self._mission_state:
            self._log(f'mission_state: {self._mission_state} -> {msg.data}')
        self._mission_state = msg.data

    def _on_safe(self, msg):  self._safe_to_enter = msg.data
    def _on_clear(self, msg): self._inside_clear  = msg.data

    def _on_press(self, msg):
        if msg.data:
            self._log('press_done received')
        self._press_done = msg.data

    def _on_vel(self, msg):
        self._lin_x = msg.linear.x
        self._ang_z = msg.angular.z

    def _on_accel(self, msg):
        self._accel_z = msg.data

    # ── param forwarding ──

    def _forward_params(self):
        # auto_call_floor: if nonzero, publish a call then reset to 0
        auto = self.get_parameter('auto_call_floor').value
        if auto and auto > 0:
            msg = Int32(); msg.data = int(auto)
            self._call_pub.publish(msg)
            self._log(f'auto_call -> floor {auto}')
            self.set_parameters([
                rclpy.parameter.Parameter(
                    'auto_call_floor',
                    rclpy.parameter.Parameter.Type.INTEGER, 0)
            ])

    # ── render ──

    def _render(self):
        door_color = {
            'OPEN': '\033[92m', 'CLOSED': '\033[90m',
            'OPENING': '\033[93m', 'CLOSING': '\033[93m',
        }.get(self._door_state, '')
        reset = '\033[0m'

        lines = [
            '\033[2J\033[H',
            '╔══════════════════════════════════════╗',
            '║       SIM DASHBOARD  (Ctrl-C exit)   ║',
            '╠══════════════════════════════════════╣',
            f'║  Floor        : {self._floor:<4}                  ║',
            f'║  Direction    : {self._direction:<10}            ║',
            f'║  Door         : {door_color}{self._door_state:<10}{reset}            ║',
            f'║  Accel Z      : {self._accel_z:+.3f} m/s²           ║',
            '╠══════════════════════════════════════╣',
            f'║  Mission      : {self._mission_state:<22}║',
            f'║  safe_to_enter: {"YES" if self._safe_to_enter else "no ":<4}  '
            f'inside_clear: {"YES" if self._inside_clear else "no "}  ║',
            f'║  press_done   : {"YES" if self._press_done else "no "}                    ║',
            '╠══════════════════════════════════════╣',
            f'║  cmd_vel lin  : {self._lin_x:+.3f} m/s             ║',
            f'║  cmd_vel ang  : {self._ang_z:+.3f} rad/s           ║',
            '╠══════════════════════════════════════╣',
            '║  EDITABLE PARAMS (ros2 param set)    ║',
        ]
        param_names = [
            'elevator_speed_m_s', 'elevator_accel_m_s2',
            'door_hold_time_s', 'door_open_time_s',
            'door_close_time_s', 'target_floor', 'auto_call_floor',
        ]
        for name in param_names:
            val = self.get_parameter(name).value
            lines.append(f'║  {name:<22}: {str(val):<8}  ║')
        lines += [
            '╠══════════════════════════════════════╣',
            '║  RECENT EVENTS                       ║',
        ]
        for ev in self._events[-6:]:
            lines.append(f'║  {ev[:38]:<38}║')
        lines.append('╚══════════════════════════════════════╝')
        print('\n'.join(lines), end='', flush=True)

    # ── logging ──

    def _log(self, msg: str):
        ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        line = f'[{ts}] {msg}'
        self._events.append(line)
        if len(self._events) > 200:
            self._events = self._events[-200:]
        self._log_file.write(line + '\n')
        self._log_file.flush()

    def destroy_node(self):
        self._log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
