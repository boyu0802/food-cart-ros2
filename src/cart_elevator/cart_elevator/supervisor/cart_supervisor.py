#!/usr/bin/env python3
"""cart_supervisor — ROS wrapper around supervisor/mission.py.

Brain (Mission state machine) lives in mission.py and is unit-tested
ROS-free. This module is the body: subscribes to the topics that
build a Snapshot, calls Mission.step() every tick_period, and
dispatches each Action via:

  NAV_GOAL          -> Nav2 NavigateToPose action client
  SET_DOCK_TARGET   -> /dock/set_target service (per-target yaml)
  CLEAR_DOCK_TARGET -> SetDockTarget(enabled=false)
  PRESS_BUTTON      -> /elevator/press_button std_msgs/String; wait for
                       /elevator/press_done std_msgs/Bool from NT bridge
  SWAP_MAP          -> std_msgs/Int32 on /map/swap_floor + an
                       /initialpose republish; map_server load + AMCL
                       re-init wiring is a TODO once per-floor maps exist
  MISSION_CMD       -> std_msgs/String on /mission/cmd ("load" | "unload"
                       | "stow") for the RoboRIO-side lift+pusher.
                       Independent of the pneumatic press path, so can
                       be in flight simultaneously with PRESS_BUTTON.

Inputs (Snapshot):
  /elevator/door_state    -> DoorState   -> snap.door_state
  /elevator/floor         -> FloorEstimate -> current_floor, floor_confidence
  /elevator/direction     -> ElevatorDirection -> elevator_direction
  /dock/status            -> DockStatus  -> dock_aligned, dock_lost
  /mission/start          -> std_msgs/Bool  -> start_pressed (one-shot latch)
  /mission/loaded         -> std_msgs/Bool  -> loaded
  /mission/unloaded       -> std_msgs/Bool  -> unloaded
  /elevator/press_done    -> std_msgs/Bool  -> press_done (one-shot)
  /map/current_floor      -> std_msgs/Int32 -> current_map_floor
  /elevator/safe_to_enter -> std_msgs/Bool  -> safe_to_enter
  Nav2 action goal callback -> nav_succeeded / nav_failed

Each one-shot input (start, loaded, unloaded, press_done) is consumed
when the Mission advances past the WAIT state — i.e. we LATCH the
flag, then clear it once it triggers a transition. Prevents an old
"loaded=true" from accidentally re-triggering loading the next cycle.

Hold-to-run / restart (2026-05-20):
  /mission/enable  std_msgs/Bool, *level* — supervisor only ticks while
    True. On a True->False transition we cancel any in-flight Nav2 goal
    so the controller doesn't keep trying to drive into something while
    we're paused. On False->True we re-issue the entry actions of the
    current state (only idempotent ones: NAV_GOAL, SET_DOCK_TARGET,
    SET_SAFE_TARGET). MISSION_CMD and PRESS_BUTTON are NOT re-issued —
    the Rio side is independently gated on the same operator button and
    will resume its own sequence.
  /mission/restart std_msgs/Bool, *edge* — cancel in-flight Nav2,
    clear all one-shot latches on the Snapshot, reset Mission to IDLE.
  require_enable parameter — default True. Set False for the sim
    driver so it doesn't have to publish /mission/enable to advance.
"""

from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String, Int32  # noqa: F401  (Int32 used below)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from cart_elevator_msgs.msg import (
    DoorState, FloorEstimate, ElevatorDirection, DockStatus,
)
from cart_elevator_msgs.srv import SetDockTarget
from nav2_msgs.action import NavigateToPose
from cart_elevator.supervisor.mission import (
    Mission, MissionConfig, Snapshot, Action, ActionKind, State,
)


# Action kinds that are safe to re-fire on a disable->enable resume.
# NAV_GOAL re-plans, SET_DOCK_TARGET re-arms the controller, SET_SAFE_TARGET
# re-tells the safe gate which floor to watch. The others have side effects
# (Rio sequences, NT pulses) that we do NOT want to repeat.
_RESUMABLE_ACTION_KINDS = frozenset({
    ActionKind.NAV_GOAL,
    ActionKind.SET_DOCK_TARGET,
    ActionKind.CLEAR_DOCK_TARGET,
    ActionKind.SET_SAFE_TARGET,
})

import math


def _yaw_to_quat(yaw: float):
    # Z-axis quaternion (cart is planar).
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class CartSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('cart_supervisor')

        self.declare_parameters(namespace='', parameters=[
            ('tick_rate_hz', 5.0),
            ('target_floor', 4),
            ('starting_floor', 1),
            ('pickup_tag_id', 200),
            ('hallway_panel_tag_id', 210),
            ('in_cab_panel_tag_id', 220),
            ('dropoff_tag_id', 230),
            # Per-target standoff + deadbands.
            ('pickup_standoff_x', 0.60), ('pickup_standoff_y', 0.0),
            ('pickup_facing_yaw', math.pi),
            ('pickup_dead_x', 0.03), ('pickup_dead_y', 0.03),
            ('pickup_dead_yaw', 0.05),
            ('panel_standoff_x', 0.35), ('panel_standoff_y', 0.0),
            ('panel_facing_yaw', math.pi),
            ('panel_dead_x', 0.015), ('panel_dead_y', 0.015),
            ('panel_dead_yaw', 0.025),
            ('dropoff_standoff_x', 0.60), ('dropoff_standoff_y', 0.0),
            ('dropoff_facing_yaw', math.pi),
            ('dropoff_dead_x', 0.03), ('dropoff_dead_y', 0.03),
            ('dropoff_dead_yaw', 0.05),
            # Nav2 approach poses — (x, y, yaw) tuples flattened to lists.
            ('pickup_pose', [0.0, 0.0, 0.0]),
            ('hallway_pose', [0.0, 0.0, 0.0]),
            ('into_cab_pose', [0.0, 0.0, 0.0]),
            ('out_of_cab_pose', [0.0, 0.0, 0.0]),
            ('dropoff_pose', [0.0, 0.0, 0.0]),
            # Hold-to-run: when True (default), the supervisor only ticks
            # while /mission/enable is True. Set False for the sim driver
            # so mission_sim doesn't have to publish enable to advance.
            ('require_enable', True),
        ])
        gp = lambda n: self.get_parameter(n).value

        cfg = MissionConfig(
            target_floor=int(gp('target_floor')),
            starting_floor=int(gp('starting_floor')),
            pickup_tag_id=int(gp('pickup_tag_id')),
            hallway_panel_tag_id=int(gp('hallway_panel_tag_id')),
            in_cab_panel_tag_id=int(gp('in_cab_panel_tag_id')),
            dropoff_tag_id=int(gp('dropoff_tag_id')),
            pickup_approach_pose=tuple(['map'] + list(gp('pickup_pose'))),
            hallway_approach_pose=tuple(['map'] + list(gp('hallway_pose'))),
            into_cab_pose=tuple(['map'] + list(gp('into_cab_pose'))),
            out_of_cab_pose=tuple(['map'] + list(gp('out_of_cab_pose'))),
            dropoff_approach_pose=tuple(['map'] + list(gp('dropoff_pose'))),
        )
        self.cfg = cfg
        self.mission = Mission(cfg)

        # Per-tag-id -> (standoff_x, standoff_y, facing_yaw, dead_x, dead_y, dead_yaw).
        # The mission only emits tag_id in SET_DOCK_TARGET payloads;
        # the supervisor looks up the rest here.
        self._target_spec = {
            cfg.pickup_tag_id: (float(gp('pickup_standoff_x')),
                                float(gp('pickup_standoff_y')),
                                float(gp('pickup_facing_yaw')),
                                float(gp('pickup_dead_x')),
                                float(gp('pickup_dead_y')),
                                float(gp('pickup_dead_yaw'))),
            cfg.hallway_panel_tag_id: (float(gp('panel_standoff_x')),
                                       float(gp('panel_standoff_y')),
                                       float(gp('panel_facing_yaw')),
                                       float(gp('panel_dead_x')),
                                       float(gp('panel_dead_y')),
                                       float(gp('panel_dead_yaw'))),
            cfg.in_cab_panel_tag_id: (float(gp('panel_standoff_x')),
                                      float(gp('panel_standoff_y')),
                                      float(gp('panel_facing_yaw')),
                                      float(gp('panel_dead_x')),
                                      float(gp('panel_dead_y')),
                                      float(gp('panel_dead_yaw'))),
            cfg.dropoff_tag_id: (float(gp('dropoff_standoff_x')),
                                 float(gp('dropoff_standoff_y')),
                                 float(gp('dropoff_facing_yaw')),
                                 float(gp('dropoff_dead_x')),
                                 float(gp('dropoff_dead_y')),
                                 float(gp('dropoff_dead_yaw'))),
        }

        # ---- Snapshot fields (latched booleans + most-recent values) ----
        self.snap = Snapshot(target_floor=cfg.target_floor)
        self._nav_in_flight = False
        self._nav_goal_handle = None   # for cancellation on pause / restart

        # ---- Hold-to-run state ----
        self._require_enable = bool(gp('require_enable'))
        # Start disabled when require_enable is True. The bridge publishes
        # the latched current value shortly after we subscribe, so we'll
        # pick up the real state if the operator already has the button held.
        self._enabled = (not self._require_enable)
        self._prev_enabled = self._enabled

        # ---- Subscriptions feeding the Snapshot ----
        self.create_subscription(DoorState, '/elevator/door_state',
                                 self._on_door_state, 10)
        self.create_subscription(FloorEstimate, '/elevator/floor',
                                 self._on_floor, 10)
        self.create_subscription(ElevatorDirection, '/elevator/direction',
                                 self._on_direction, 10)
        self.create_subscription(DockStatus, '/dock/status',
                                 self._on_dock_status, 10)
        self.create_subscription(Bool, '/mission/start', self._on_start, 10)
        self.create_subscription(Bool, '/mission/loaded', self._on_loaded, 10)
        self.create_subscription(Bool, '/mission/unloaded', self._on_unloaded, 10)
        self.create_subscription(Bool, '/elevator/press_done',
                                 self._on_press_done, 10)
        self.create_subscription(Int32, '/map/current_floor',
                                 self._on_current_floor, 10)
        self.create_subscription(Bool, '/elevator/safe_to_enter',
                                 self._on_safe_to_enter, 10)
        # Enable is latched (transient_local) on the bridge side; match
        # the QoS so we receive the most-recent value on subscribe.
        enable_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, '/mission/enable',
                                 self._on_enable, enable_qos)
        self.create_subscription(Bool, '/mission/restart',
                                 self._on_restart, 10)

        # ---- Action / service / publisher clients for the side effects ----
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.dock_target_client = self.create_client(SetDockTarget, '/dock/set_target')
        self.press_pub = self.create_publisher(String, '/elevator/press_button', 5)
        # /mission/cmd carries "load" | "unload" | "stow" to the
        # RoboRIO-side lift+pusher (parallel mechanism to the
        # pneumatic press, so this and /elevator/press_button can be
        # in flight at the same time).
        self.mission_cmd_pub = self.create_publisher(String, '/mission/cmd', 5)
        self.swap_map_pub = self.create_publisher(Int32, '/map/swap_floor', 5)
        # Retarget the safe gate before each boarding WAIT phase.
        self.safe_target_pub = self.create_publisher(
            Int32, '/safe_to_enter/target_floor', 5)
        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 5)
        self.state_pub = self.create_publisher(String, '/mission/state', 5)

        self.create_timer(1.0 / float(gp('tick_rate_hz')), self._tick)
        self.get_logger().info(
            f'cart_supervisor ready (target_floor={cfg.target_floor}, '
            f'state={self.mission.state.value})')

    # ----- snapshot inputs -----

    def _on_door_state(self, msg: DoorState) -> None:
        self.snap.door_state = int(msg.state)

    def _on_floor(self, msg: FloorEstimate) -> None:
        self.snap.current_floor = int(msg.floor)
        self.snap.floor_confidence = float(msg.confidence)

    def _on_direction(self, msg: ElevatorDirection) -> None:
        # ElevatorDirection.direction: -1 / 0 / +1 (down/unknown/up).
        self.snap.elevator_direction = int(msg.direction)

    def _on_dock_status(self, msg: DockStatus) -> None:
        self.snap.dock_aligned = (msg.state == DockStatus.ALIGNED)
        self.snap.dock_lost = (msg.state == DockStatus.LOST)

    def _on_start(self, msg: Bool) -> None:
        if msg.data:
            self.snap.start_pressed = True

    def _on_loaded(self, msg: Bool) -> None:
        if msg.data:
            self.snap.loaded = True

    def _on_unloaded(self, msg: Bool) -> None:
        if msg.data:
            self.snap.unloaded = True

    def _on_press_done(self, msg: Bool) -> None:
        if msg.data:
            self.snap.press_done = True

    def _on_current_floor(self, msg: Int32) -> None:
        # Published by map_swap_node after AMCL is re-initialized for
        # the new floor. Drives the RELOCALIZE_AT_FLOOR transition.
        self.snap.current_map_floor = int(msg.data)

    def _on_safe_to_enter(self, msg: Bool) -> None:
        # Continuous boolean from safe_to_enter_gate. Level-triggered:
        # we don't latch — the supervisor advances exactly when this
        # is true at tick time, regardless of how long it's been so.
        self.snap.safe_to_enter = bool(msg.data)

    def _on_enable(self, msg: Bool) -> None:
        # Level-triggered: True while operator holds the dead-man button.
        # The actual edge handling (cancel-on-pause, re-issue-on-resume)
        # lives in _tick where we have the mission state to act on.
        self._enabled = bool(msg.data)

    def _on_restart(self, msg: Bool) -> None:
        if not msg.data:
            return
        # Cancel any in-flight side effects, clear latches, reset state.
        # This fires even while disabled — the operator can restart from
        # a frozen state without first re-enabling.
        self.get_logger().warn('mission: RESTART pressed -> IDLE')
        self._cancel_nav_goal()
        self._clear_dock_target()
        # Wipe all one-shot latches AND level-triggered safe gate cache —
        # nothing should carry across a restart.
        self.snap = Snapshot(target_floor=self.cfg.target_floor)
        self.mission.reset()

    # ----- tick -----

    def _tick(self) -> None:
        # Handle enable edges first; they may abort the rest of this tick.
        if self._require_enable and self._enabled != self._prev_enabled:
            if self._enabled:
                # Resume: re-issue idempotent entry actions of the
                # current state (cancelled nav goals, cleared dock
                # targets) so the controllers pick up where we paused.
                resume_actions = [
                    a for a in self.mission.entry_actions_for_current_state()
                    if a.kind in _RESUMABLE_ACTION_KINDS
                ]
                if resume_actions:
                    self.get_logger().info(
                        f'mission: RESUMED in {self.mission.state.value} — '
                        f're-issuing {[a.kind.value for a in resume_actions]}')
                    for a in resume_actions:
                        self._dispatch(a)
                else:
                    self.get_logger().info(
                        f'mission: RESUMED in {self.mission.state.value}')
            else:
                # Pause: cancel in-flight Nav2 so the controller doesn't
                # keep trying to drive while we're frozen. Rio is gated
                # on its own copy of the dead-man button, so /cmd_vel will
                # stop being actuated regardless — this just keeps the
                # action server from timing out on a long pause.
                self.get_logger().info(
                    f'mission: PAUSED in {self.mission.state.value}')
                self._cancel_nav_goal()
            self._prev_enabled = self._enabled

        # While disabled, publish current state for observability but
        # don't tick the machine or dispatch anything.
        if self._require_enable and not self._enabled:
            s = String()
            s.data = self.mission.state.value
            self.state_pub.publish(s)
            return

        prev_state = self.mission.state
        new_state, actions = self.mission.step(self.snap)

        # Publish current state so it's observable.
        s = String()
        s.data = new_state.value
        self.state_pub.publish(s)

        if new_state != prev_state:
            self.get_logger().info(
                f'mission: {prev_state.value} -> {new_state.value}')
            for action in actions:
                self._dispatch(action)
            # Clear one-shot latches so they don't re-fire next tick.
            self._consume_latches(prev_state)

    def _consume_latches(self, just_left: State) -> None:
        # The thing that *caused* the transition is the latch we clear.
        if just_left == State.IDLE:
            self.snap.start_pressed = False
        if just_left == State.WAIT_LOADED:
            self.snap.loaded = False
        if just_left == State.WAIT_UNLOADED:
            self.snap.unloaded = False
        if just_left in (State.PRESS_CALL_BUTTON, State.PRESS_FLOOR_BUTTON):
            self.snap.press_done = False
        if just_left in (State.NAV_TO_PICKUP, State.NAV_TO_HALLWAY_CALL,
                         State.NAV_INTO_CAB, State.NAV_OUT_OF_CAB,
                         State.NAV_TO_DROPOFF):
            self.snap.nav_succeeded = False
            self.snap.nav_failed = False
            self._nav_in_flight = False
        if just_left in (State.DOCK_PICKUP, State.DOCK_HALLWAY_CALL,
                         State.DOCK_IN_CAB_BUTTON, State.DOCK_DROPOFF):
            # dock_aligned / dock_lost are level-triggered from DockStatus,
            # nothing to clear.
            pass

    # ----- action dispatch -----

    def _dispatch(self, action: Action) -> None:
        if action.kind == ActionKind.NONE:
            return
        if action.kind == ActionKind.NAV_GOAL:
            self._send_nav_goal(action.payload['pose'])
        elif action.kind == ActionKind.SET_DOCK_TARGET:
            self._send_dock_target(action.payload['tag_id'])
        elif action.kind == ActionKind.CLEAR_DOCK_TARGET:
            self._clear_dock_target()
        elif action.kind == ActionKind.PRESS_BUTTON:
            self._press_button(action.payload['button'])
        elif action.kind == ActionKind.SWAP_MAP:
            self._swap_map(action.payload['floor'])
        elif action.kind == ActionKind.SET_SAFE_TARGET:
            self._set_safe_target(action.payload['floor'])
        elif action.kind == ActionKind.MISSION_CMD:
            self._publish_mission_cmd(action.payload['cmd'])

    def _send_nav_goal(self, pose) -> None:
        frame_id, x, y, yaw = pose
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qx, qy, qz, qw = _yaw_to_quat(float(yaw))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(
                'Nav2 NavigateToPose action server not available — faulting')
            self.snap.nav_failed = True
            return
        self._nav_in_flight = True
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self._on_nav_goal_response)
        self.get_logger().info(
            f'nav: goal sent ({frame_id} {x:.2f},{y:.2f},{yaw:.2f})')

    def _on_nav_goal_response(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('nav: goal rejected')
            self.snap.nav_failed = True
            return
        # Hold the handle so a pause/restart can cancel it.
        self._nav_goal_handle = handle
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(self._on_nav_result)

    def _cancel_nav_goal(self) -> None:
        if self._nav_goal_handle is None:
            return
        self.get_logger().info('nav: cancelling in-flight goal')
        # Fire-and-forget — the cancel response itself isn't actionable.
        self._nav_goal_handle.cancel_goal_async()
        self._nav_goal_handle = None
        self._nav_in_flight = False
        # Clear stale nav latches so we don't think a cancelled goal
        # succeeded on the next tick.
        self.snap.nav_succeeded = False
        self.snap.nav_failed = False

    def _on_nav_result(self, future) -> None:
        status = future.result().status
        # status == STATUS_SUCCEEDED (4) means we made it.
        if status == 4:
            self.snap.nav_succeeded = True
            self.get_logger().info('nav: succeeded')
        else:
            self.snap.nav_failed = True
            self.get_logger().error(f'nav: failed (status={status})')

    def _send_dock_target(self, tag_id: int) -> None:
        spec = self._target_spec.get(int(tag_id))
        if spec is None:
            self.get_logger().error(
                f'no dock target spec for tag_id={tag_id} — faulting')
            self.snap.dock_lost = True
            return
        sox, soy, fyaw, dx, dy, dyaw = spec
        req = SetDockTarget.Request()
        req.enabled = True
        req.tag_id = int(tag_id)
        req.standoff_x = float(sox)
        req.standoff_y = float(soy)
        req.facing_yaw = float(fyaw)
        req.dead_x = float(dx)
        req.dead_y = float(dy)
        req.dead_yaw = float(dyaw)
        if not self.dock_target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                'dock /set_target service not available — faulting')
            self.snap.dock_lost = True
            return
        fut = self.dock_target_client.call_async(req)
        fut.add_done_callback(self._on_set_target_response)
        self.get_logger().info(f'dock: set target tag_id={tag_id}')

    def _clear_dock_target(self) -> None:
        req = SetDockTarget.Request()
        req.enabled = False
        if not self.dock_target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('dock /set_target service not available to clear')
            return
        self.dock_target_client.call_async(req)

    def _on_set_target_response(self, future) -> None:
        try:
            resp = future.result()
            if not resp.ok:
                self.get_logger().error(f'dock retarget rejected: {resp.message}')
                self.snap.dock_lost = True
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'dock retarget exception: {e}')
            self.snap.dock_lost = True

    def _press_button(self, button: str) -> None:
        msg = String()
        msg.data = button
        self.press_pub.publish(msg)
        self.get_logger().info(f'press: {button} (waiting on /elevator/press_done)')

    def _publish_mission_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.mission_cmd_pub.publish(msg)
        self.get_logger().info(f'mission_cmd: {cmd}')

    def _set_safe_target(self, floor: int) -> None:
        # Tell the safe gate which floor to expect. Also pre-clear the
        # cached safe_to_enter so the BT doesn't advance on a stale
        # True from the *previous* WAIT phase (where the gate was
        # checking a different floor).
        self.snap.safe_to_enter = False
        m = Int32()
        m.data = int(floor)
        self.safe_target_pub.publish(m)
        self.get_logger().info(f'safe gate: target_floor -> {floor}')

    def _swap_map(self, floor: int) -> None:
        # Publish the target floor; map_swap_node owns the rest:
        # load the per-floor map (if a file exists), re-init AMCL
        # from the floor-N hallway tag, and ack on /map/current_floor.
        # The BT advances only when that ack arrives — no self-ack.
        m = Int32()
        m.data = int(floor)
        self.swap_map_pub.publish(m)
        self.get_logger().info(
            f'map: swap requested -> floor {floor} '
            '(waiting for map_swap_node ack on /map/current_floor)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
