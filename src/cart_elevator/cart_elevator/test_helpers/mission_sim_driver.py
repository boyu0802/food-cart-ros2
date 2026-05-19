#!/usr/bin/env python3
"""mission_sim_driver — fakes every external input so the supervisor
can walk the full mission without hardware.

Replaces Nav2 + AprilTag + RoboRIO + door sensor + load button with a
scripted sequence:
  - Listens to /mission/state. On each NEW state, performs the
    appropriate canned response after a small delay.
  - For Nav2: instead of running the real action server, watches
    /mission/state and, when a NAV_* state is entered, calls back
    nav_succeeded by publishing on /mission/_test_nav_ok. (The
    supervisor would normally use Nav2's action result; in sim we
    flip the flag from the outside by publishing a dummy NavigateToPose
    action server that always succeeds.)
  - For the dock: publishes /dock/status ALIGNED after a delay.
  - For door: publishes /elevator/door_state OPEN after a delay.
  - For floor: publishes the target floor to /elevator/floor at high
    confidence after the in-cab press_done.
  - Press: echoes /elevator/press_button -> /elevator/press_done after
    a short delay.

This is a demo / smoke harness, NOT something that ships with the
real cart. Run it with cart_supervisor for a full mission walkthrough
in ~30 seconds.
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32

from cart_elevator_msgs.msg import (
    DoorState, FloorEstimate, ElevatorDirection, DockStatus,
)
from cart_elevator_msgs.srv import SetDockTarget
from nav2_msgs.action import NavigateToPose


class MissionSimDriver(Node):
    def __init__(self) -> None:
        super().__init__('mission_sim_driver')

        self.declare_parameters(namespace='', parameters=[
            ('nav_delay_s', 1.0),
            ('dock_delay_s', 1.0),
            ('press_delay_s', 0.5),
            ('door_open_delay_s', 1.0),
            ('floor_settle_delay_s', 1.5),
            ('target_floor', 4),
        ])
        gp = lambda n: self.get_parameter(n).value
        self.nav_delay = float(gp('nav_delay_s'))
        self.dock_delay = float(gp('dock_delay_s'))
        self.press_delay = float(gp('press_delay_s'))
        self.door_open_delay = float(gp('door_open_delay_s'))
        self.floor_settle_delay = float(gp('floor_settle_delay_s'))
        self.target_floor = int(gp('target_floor'))

        self._state = 'UNKNOWN'
        self._scheduled: list = []
        self._last_press: str = ''

        # Wires.
        self.create_subscription(String, '/mission/state', self._on_state, 10)
        self.create_subscription(String, '/elevator/press_button',
                                 self._on_press, 10)
        self.create_subscription(Int32, '/map/swap_floor',
                                 self._on_swap_floor, 5)

        self.dock_pub = self.create_publisher(DockStatus, '/dock/status', 5)
        self.door_pub = self.create_publisher(DoorState, '/elevator/door_state', 5)
        self.floor_pub = self.create_publisher(FloorEstimate, '/elevator/floor', 5)
        self.dir_pub = self.create_publisher(ElevatorDirection,
                                             '/elevator/direction', 5)
        self.press_done_pub = self.create_publisher(Bool,
                                                    '/elevator/press_done', 5)
        self.current_floor_pub = self.create_publisher(Int32,
                                                       '/map/current_floor', 5)
        self.loaded_pub = self.create_publisher(Bool, '/mission/loaded', 5)
        self.unloaded_pub = self.create_publisher(Bool, '/mission/unloaded', 5)

        # Fake Nav2 action server that succeeds after nav_delay.
        self._nav_server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose',
            execute_callback=self._fake_nav_execute)

        # Fake /dock/set_target service. The supervisor calls this
        # before each dock phase; in sim we just acknowledge and let
        # the scripted dock_aligned publisher do its thing.
        self._dock_srv = self.create_service(
            SetDockTarget, '/dock/set_target', self._fake_set_target)

        # Periodic background publishers so the supervisor's snapshot
        # is always fresh (these match a "nothing happening" world).
        self.create_timer(0.5, self._publish_background)
        self.create_timer(0.1, self._service_schedule)

        self.get_logger().info(
            f'mission_sim_driver ready (target_floor={self.target_floor})')

    # ---- background "world is calm" publishers ----

    def _publish_background(self) -> None:
        # Hallway/lobby default: door closed, no elevator arriving yet,
        # floor 1 by default.
        ds = DoorState()
        ds.header.stamp = self.get_clock().now().to_msg()
        ds.state = DoorState.CLOSED
        ds.confidence = 0.9
        ds.median_depth_m = 0.30
        self.door_pub.publish(ds)

        ed = ElevatorDirection()
        ed.header.stamp = self.get_clock().now().to_msg()
        ed.direction = 0
        ed.confidence = 0.0
        self.dir_pub.publish(ed)

    # ---- state change -> schedule a canned response ----

    def _on_state(self, msg: String) -> None:
        if msg.data == self._state:
            return
        prev = self._state
        self._state = msg.data
        self.get_logger().info(f'sim observed: {prev} -> {self._state}')
        self._react(self._state)

    def _react(self, state: str) -> None:
        now = self._now_s()
        if state == 'DOCK_PICKUP' or state == 'DOCK_HALLWAY_CALL' \
                or state == 'DOCK_IN_CAB_BUTTON' or state == 'DOCK_DROPOFF':
            self._schedule(now + self.dock_delay, self._publish_dock_aligned)
        elif state == 'WAIT_LOADED':
            self._schedule(now + self.press_delay,
                           lambda: self._publish_bool(self.loaded_pub))
        elif state == 'WAIT_UNLOADED':
            self._schedule(now + self.press_delay,
                           lambda: self._publish_bool(self.unloaded_pub))
        elif state == 'WAIT_FOR_HALLWAY_DOOR_OPEN':
            self._schedule(now + self.door_open_delay, self._publish_door_open_up)
        elif state == 'WAIT_FOR_FLOOR_REACHED':
            self._schedule(now + self.floor_settle_delay,
                           self._publish_floor_arrived)

    def _on_press(self, msg: String) -> None:
        self._last_press = msg.data
        self.get_logger().info(f'sim: fake-pressing button {msg.data!r}')
        self._schedule(self._now_s() + self.press_delay,
                       lambda: self._publish_bool(self.press_done_pub))

    def _on_swap_floor(self, msg: Int32) -> None:
        # Fake the map_swap_node ack: after a short delay, publish
        # /map/current_floor with the requested floor.
        floor = int(msg.data)
        self.get_logger().info(f'sim: fake map swap -> floor {floor}')
        self._schedule(self._now_s() + 0.5,
                       lambda f=floor: self._publish_int(self.current_floor_pub, f))

    # ---- canned publishers ----

    def _publish_dock_aligned(self) -> None:
        m = DockStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state = DockStatus.ALIGNED
        self.dock_pub.publish(m)

    def _publish_door_open_up(self) -> None:
        ds = DoorState()
        ds.header.stamp = self.get_clock().now().to_msg()
        ds.state = DoorState.OPEN
        ds.confidence = 0.9
        ds.median_depth_m = 2.0
        self.door_pub.publish(ds)
        ed = ElevatorDirection()
        ed.header.stamp = self.get_clock().now().to_msg()
        ed.direction = 1
        ed.confidence = 0.9
        self.dir_pub.publish(ed)

    def _publish_floor_arrived(self) -> None:
        f = FloorEstimate()
        f.header.stamp = self.get_clock().now().to_msg()
        f.floor = self.target_floor
        f.confidence = 0.9
        f.source = FloorEstimate.SOURCE_FUSION
        self.floor_pub.publish(f)
        ds = DoorState()
        ds.header.stamp = self.get_clock().now().to_msg()
        ds.state = DoorState.OPEN
        ds.confidence = 0.9
        ds.median_depth_m = 2.0
        self.door_pub.publish(ds)

    def _publish_bool(self, pub) -> None:
        b = Bool()
        b.data = True
        pub.publish(b)

    def _publish_int(self, pub, value: int) -> None:
        m = Int32()
        m.data = int(value)
        pub.publish(m)

    # ---- fake Nav2 action execute ----

    def _fake_set_target(self, req, resp):
        # Echo whatever the supervisor asked for, always OK.
        if req.enabled:
            self.get_logger().info(
                f'sim: dock target set tag_id={req.tag_id}')
        else:
            self.get_logger().info('sim: dock target cleared')
        resp.ok = True
        resp.message = ''
        return resp

    def _fake_nav_execute(self, goal_handle):
        # Sync callback that blocks for nav_delay then succeeds. Must
        # be sync (rclpy actions don't provide an asyncio loop) and
        # requires a MultiThreadedExecutor so the blocking doesn't
        # starve subscriptions / timers — see main() below.
        import time
        time.sleep(self.nav_delay)
        goal_handle.succeed()
        return NavigateToPose.Result()

    # ---- helpers ----

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _schedule(self, t: float, fn) -> None:
        self._scheduled.append((t, fn))

    def _service_schedule(self) -> None:
        now = self._now_s()
        due, remaining = [], []
        for t, fn in self._scheduled:
            (due if t <= now else remaining).append((t, fn))
        self._scheduled = remaining
        for _, fn in due:
            try:
                fn()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'scheduled task raised: {e}')


def main(args=None) -> None:
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = MissionSimDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
