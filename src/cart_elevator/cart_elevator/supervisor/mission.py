"""Pure mission-state-machine for the cart's lunch-delivery routine.

ROS-free so we can unit-test transition logic with crafted inputs the
same way we test `dock_control.py` and `door_classifier.py`. The wrapper
node (`cart_supervisor.py`) does the topic plumbing and dispatches the
side-effects (set dock target, publish a Nav2 goal, fire the pneumatic
NT key) that each state requests.

The state machine intentionally stays simple — explicit enum + a single
`step()` that takes a Snapshot of the world and returns (next_state,
Action). No threads, no async, no behavior-tree library. We can port to
py_trees later if branching gets complicated; for now the linear demo
mission fits in one switch.

Mission outline (hardcoded for the science-fair demo):
  IDLE
    -> wait for start trigger
  NAV_TO_PICKUP
    -> Nav2 goal at the kitchen tag's approach pose
  DOCK_PICKUP
    -> dock_controller targeting kitchen pickup tag; wait for ALIGNED
  WAIT_LOADED
    -> wait for the 'loaded' signal (operator button for now)
  NAV_TO_HALLWAY_CALL
    -> Nav2 goal in front of the hallway elevator button panel
  DOCK_HALLWAY_CALL
    -> dock_controller targeting hallway button-panel tag
  PRESS_CALL_BUTTON
    -> send 'press_call_up' NT command to RoboRIO; wait for press_done
  WAIT_FOR_HALLWAY_DOOR_OPEN
    -> watch /elevator/door_state for OPEN; sanity-check direction
       detector confirms an UP car arrived
  NAV_INTO_CAB
    -> Nav2 short trip into the cab
  DOCK_IN_CAB_BUTTON
    -> lidar wall-dock to the in-cab button panel (no tag inside the cab)
  PRESS_FLOOR_BUTTON
    -> fire pneumatic with floor=target_floor
  WAIT_FOR_FLOOR_REACHED
    -> floor_v4 fusion says current_floor == target_floor AND
       door_state goes OPEN
  NAV_OUT_OF_CAB
    -> drive out
  RELOCALIZE_AT_FLOOR
    -> swap map to the target-floor map; re-init AMCL from the
       hallway tag at that floor
  NAV_TO_DROPOFF
    -> Nav2 to drop-off marker
  DOCK_DROPOFF
    -> final align
  WAIT_UNLOADED
    -> wait for 'unloaded' signal
  RETURN_*
    -> mirror the above going down
  DONE

Each non-WAIT state issues exactly one Action on entry; WAIT states
issue no Action but flip on a Snapshot field. Transitions are pure: no
clock, no random — drives unit-testability.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class State(Enum):
    IDLE = 'IDLE'
    NAV_TO_PICKUP = 'NAV_TO_PICKUP'
    DOCK_PICKUP = 'DOCK_PICKUP'
    WAIT_LOADED = 'WAIT_LOADED'
    NAV_TO_HALLWAY_CALL = 'NAV_TO_HALLWAY_CALL'
    DOCK_HALLWAY_CALL = 'DOCK_HALLWAY_CALL'
    PRESS_CALL_BUTTON = 'PRESS_CALL_BUTTON'
    WAIT_FOR_HALLWAY_DOOR_OPEN = 'WAIT_FOR_HALLWAY_DOOR_OPEN'
    NAV_INTO_CAB = 'NAV_INTO_CAB'
    DOCK_IN_CAB_BUTTON = 'DOCK_IN_CAB_BUTTON'
    PRESS_FLOOR_BUTTON = 'PRESS_FLOOR_BUTTON'
    WAIT_FOR_FLOOR_REACHED = 'WAIT_FOR_FLOOR_REACHED'
    NAV_OUT_OF_CAB = 'NAV_OUT_OF_CAB'
    RELOCALIZE_AT_FLOOR = 'RELOCALIZE_AT_FLOOR'
    NAV_TO_DROPOFF = 'NAV_TO_DROPOFF'
    DOCK_DROPOFF = 'DOCK_DROPOFF'
    WAIT_UNLOADED = 'WAIT_UNLOADED'
    DONE = 'DONE'
    FAULT = 'FAULT'   # any unrecoverable error lands here


# --- Snapshot of the world that the wrapper hands in each step() ---

@dataclass
class Snapshot:
    start_pressed: bool = False
    loaded: bool = False
    unloaded: bool = False
    dock_aligned: bool = False
    dock_lost: bool = False         # tag lost > timeout in current dock target
    nav_succeeded: bool = False
    nav_failed: bool = False
    press_done: bool = False
    press_failed: bool = False
    door_state: int = 0             # mirrors DoorState.msg constants
    current_floor: int = 1
    floor_confidence: float = 0.0
    elevator_direction: int = 0     # -1 down / 0 unknown / +1 up
    current_map_floor: int = 1      # which floor's map AMCL is currently using
    target_floor: int = 1           # set by mission config
    # /elevator/safe_to_enter from the safe gate. Single bool that ANDs
    # (door=OPEN ∧ direction=IDLE ∧ floor=expected ∧ inside_clear) with
    # freshness. Drives both WAIT_FOR_HALLWAY_DOOR_OPEN and
    # WAIT_FOR_FLOOR_REACHED — the supervisor retargets the gate
    # between phases via the SET_SAFE_TARGET action.
    safe_to_enter: bool = False


# --- Action the wrapper should execute when entering a state ---

class ActionKind(Enum):
    NONE = 'NONE'
    NAV_GOAL = 'NAV_GOAL'           # send a goal pose to Nav2
    SET_DOCK_TARGET = 'SET_DOCK_TARGET'
    CLEAR_DOCK_TARGET = 'CLEAR_DOCK_TARGET'
    SET_WALL_DOCK = 'SET_WALL_DOCK'  # enable/disable the lidar in-cab wall dock
    PRESS_BUTTON = 'PRESS_BUTTON'   # NT key to RoboRIO
    SWAP_MAP = 'SWAP_MAP'
    SET_SAFE_TARGET = 'SET_SAFE_TARGET'  # retarget safe_to_enter_gate
    MISSION_CMD = 'MISSION_CMD'     # Pi/mission/cmd to RoboRIO
                                     # ("load" | "unload" | "stow")


@dataclass
class Action:
    kind: ActionKind = ActionKind.NONE
    # Generic payload bag — wrapper interprets per-kind.
    payload: dict = field(default_factory=dict)


# DoorState constants mirrored here so the pure layer doesn't import the msg.
DOOR_UNKNOWN = 0
DOOR_CLOSED  = 1
DOOR_OPENING = 2
DOOR_OPEN    = 3
DOOR_CLOSING = 4


@dataclass
class MissionConfig:
    target_floor: int = 4
    # Floor the cart starts on (kitchen). Used to retarget the safe
    # gate during the hallway WAIT — we're waiting for the elevator
    # to arrive at THIS floor, not the destination.
    starting_floor: int = 1
    # Tag IDs. Real values come from yaml at deploy time.
    pickup_tag_id: int = 200
    hallway_panel_tag_id: int = 210
    in_cab_panel_tag_id: int = 220
    dropoff_tag_id: int = 230
    # Nav2 goal poses, expressed as (frame_id, x, y, yaw). The wrapper
    # turns these into PoseStamped messages.
    pickup_approach_pose: tuple = ('map', 0.0, 0.0, 0.0)
    hallway_approach_pose: tuple = ('map', 0.0, 0.0, 0.0)
    into_cab_pose: tuple = ('map', 0.0, 0.0, 0.0)
    out_of_cab_pose: tuple = ('map', 0.0, 0.0, 0.0)
    dropoff_approach_pose: tuple = ('map', 0.0, 0.0, 0.0)


class Mission:
    """Linear state machine. Call step() with a fresh Snapshot each tick.

    Each call returns the (possibly new) state + a single Action to
    execute *if* the state changed this tick. Re-entering the same state
    yields Action(NONE).
    """

    def __init__(self, cfg: MissionConfig) -> None:
        self.cfg = cfg
        self.state = State.IDLE
        self._entry_action: Optional[Action] = None

    def step(self, snap: Snapshot) -> tuple[State, list['Action']]:
        """Advance one tick. Returns the new state + a list of actions
        to dispatch in order. Most states emit zero or one action;
        NAV_TO_HALLWAY_CALL emits two (stow the intake, then nav)."""
        prev = self.state
        self.state = self._next(snap)
        if self.state == prev:
            return self.state, []
        return self.state, self._on_enter(self.state, snap)

    def reset(self) -> None:
        """Drop the mission back to IDLE. Called by the wrapper on a
        restart-button rising edge. The wrapper is responsible for also
        cancelling any in-flight side effects (Nav2 goal, dock target)
        and clearing one-shot latches on the Snapshot — this method
        only resets the pure-layer state."""
        self.state = State.IDLE
        self._entry_action = None

    def entry_actions_for_current_state(self) -> list['Action']:
        """Re-emit the on-enter actions for the current state without
        ticking. The wrapper uses this on a disable->enable transition
        to re-issue Nav2 goals or dock targets that were cancelled
        during the pause."""
        return self._on_enter(self.state, Snapshot())

    def _next(self, s: Snapshot) -> State:
        cur = self.state
        # Faults are sticky until reset.
        if cur == State.FAULT or cur == State.DONE:
            return cur

        # Generic dock-loss / nav-failed fault routing — applies in any
        # state that's waiting on dock or nav. We choose to fault rather
        # than retry; retry policy is a follow-up question.
        if cur in (State.DOCK_PICKUP, State.DOCK_HALLWAY_CALL,
                   State.DOCK_IN_CAB_BUTTON, State.DOCK_DROPOFF):
            if s.dock_lost:
                return State.FAULT
        if cur in (State.NAV_TO_PICKUP, State.NAV_TO_HALLWAY_CALL,
                   State.NAV_INTO_CAB, State.NAV_OUT_OF_CAB,
                   State.NAV_TO_DROPOFF):
            if s.nav_failed:
                return State.FAULT
        if cur in (State.PRESS_CALL_BUTTON, State.PRESS_FLOOR_BUTTON):
            if s.press_failed:
                return State.FAULT

        if cur == State.IDLE:
            return State.NAV_TO_PICKUP if s.start_pressed else cur

        if cur == State.NAV_TO_PICKUP:
            return State.DOCK_PICKUP if s.nav_succeeded else cur
        if cur == State.DOCK_PICKUP:
            return State.WAIT_LOADED if s.dock_aligned else cur
        if cur == State.WAIT_LOADED:
            return State.NAV_TO_HALLWAY_CALL if s.loaded else cur

        if cur == State.NAV_TO_HALLWAY_CALL:
            return State.DOCK_HALLWAY_CALL if s.nav_succeeded else cur
        if cur == State.DOCK_HALLWAY_CALL:
            return State.PRESS_CALL_BUTTON if s.dock_aligned else cur
        if cur == State.PRESS_CALL_BUTTON:
            return State.WAIT_FOR_HALLWAY_DOOR_OPEN if s.press_done else cur

        if cur == State.WAIT_FOR_HALLWAY_DOOR_OPEN:
            # Safe gate ANDs door=OPEN, direction=IDLE, floor=starting,
            # inside_clear=True with freshness. One bool — no inline
            # reproduction of the AND here.
            return State.NAV_INTO_CAB if s.safe_to_enter else cur

        if cur == State.NAV_INTO_CAB:
            return State.DOCK_IN_CAB_BUTTON if s.nav_succeeded else cur
        if cur == State.DOCK_IN_CAB_BUTTON:
            return State.PRESS_FLOOR_BUTTON if s.dock_aligned else cur
        if cur == State.PRESS_FLOOR_BUTTON:
            return State.WAIT_FOR_FLOOR_REACHED if s.press_done else cur

        if cur == State.WAIT_FOR_FLOOR_REACHED:
            # Same safe gate, retargeted to the destination floor by
            # the SET_SAFE_TARGET action when we entered this state.
            return State.NAV_OUT_OF_CAB if s.safe_to_enter else cur

        if cur == State.NAV_OUT_OF_CAB:
            return State.RELOCALIZE_AT_FLOOR if s.nav_succeeded else cur
        if cur == State.RELOCALIZE_AT_FLOOR:
            return (State.NAV_TO_DROPOFF
                    if s.current_map_floor == self.cfg.target_floor else cur)

        if cur == State.NAV_TO_DROPOFF:
            return State.DOCK_DROPOFF if s.nav_succeeded else cur
        if cur == State.DOCK_DROPOFF:
            return State.WAIT_UNLOADED if s.dock_aligned else cur
        if cur == State.WAIT_UNLOADED:
            return State.DONE if s.unloaded else cur

        return cur

    def _on_enter(self, st: State, s: Snapshot) -> list['Action']:
        c = self.cfg
        # Map state -> the list of actions to fire when we enter it.
        # Most states emit one; NAV_TO_HALLWAY_CALL emits stow + nav
        # because the cart has just finished loading and the lift+pusher
        # has to retract before we drive.
        if st == State.NAV_TO_PICKUP:
            return [Action(ActionKind.NAV_GOAL,
                           {'pose': c.pickup_approach_pose})]
        if st == State.DOCK_PICKUP:
            return [Action(ActionKind.SET_DOCK_TARGET,
                           {'tag_id': c.pickup_tag_id})]
        if st == State.WAIT_LOADED:
            # Cart is parked at the loading station — tell RoboRIO to
            # run the lift+pusher cycle. The Rio-side intake machine
            # eventually pulses Robot/mission/loaded=true.
            return [Action(ActionKind.MISSION_CMD, {'cmd': 'load'})]
        if st == State.NAV_TO_HALLWAY_CALL:
            # Retract the lift+pusher first ("stow"), then drive away.
            # Both fire in the same tick — the Rio handles the stow
            # during the early phase of the drive.
            return [Action(ActionKind.MISSION_CMD, {'cmd': 'stow'}),
                    Action(ActionKind.NAV_GOAL,
                           {'pose': c.hallway_approach_pose})]
        if st == State.DOCK_HALLWAY_CALL:
            return [Action(ActionKind.SET_DOCK_TARGET,
                           {'tag_id': c.hallway_panel_tag_id})]
        if st == State.PRESS_CALL_BUTTON:
            return [Action(ActionKind.PRESS_BUTTON,
                           {'button': 'call_up'})]
        if st == State.WAIT_FOR_HALLWAY_DOOR_OPEN:
            # Retarget the safe gate to the floor we're standing on
            # (we're waiting for a car to arrive HERE).
            return [Action(ActionKind.SET_SAFE_TARGET,
                           {'floor': c.starting_floor})]
        if st == State.NAV_INTO_CAB:
            return [Action(ActionKind.NAV_GOAL,
                           {'pose': c.into_cab_pose})]
        if st == State.DOCK_IN_CAB_BUTTON:
            # No AprilTag inside the cab — align to the button panel with
            # the lidar wall-dock instead. Enabling it disables the tag
            # dock (they share /dock/cmd_vel + /dock/status).
            return [Action(ActionKind.SET_WALL_DOCK, {'enabled': True})]
        if st == State.PRESS_FLOOR_BUTTON:
            return [Action(ActionKind.PRESS_BUTTON,
                           {'button': f'floor_{c.target_floor}'})]
        if st == State.WAIT_FOR_FLOOR_REACHED:
            # Floor button is pressed; we're riding now. Release the wall
            # dock (it held us on the panel through the press) so it stops
            # driving, and retarget the safe gate to the destination floor.
            return [Action(ActionKind.SET_WALL_DOCK, {'enabled': False}),
                    Action(ActionKind.SET_SAFE_TARGET,
                           {'floor': c.target_floor})]
        if st == State.NAV_OUT_OF_CAB:
            return [Action(ActionKind.NAV_GOAL,
                           {'pose': c.out_of_cab_pose})]
        if st == State.RELOCALIZE_AT_FLOOR:
            return [Action(ActionKind.SWAP_MAP,
                           {'floor': c.target_floor})]
        if st == State.NAV_TO_DROPOFF:
            return [Action(ActionKind.NAV_GOAL,
                           {'pose': c.dropoff_approach_pose})]
        if st == State.DOCK_DROPOFF:
            return [Action(ActionKind.SET_DOCK_TARGET,
                           {'tag_id': c.dropoff_tag_id})]
        if st == State.WAIT_UNLOADED:
            # Cart parked at drop-off — tell RoboRIO to run the
            # lift+pusher in reverse. Rio pulses Robot/mission/
            # unloaded=true when done.
            return [Action(ActionKind.MISSION_CMD, {'cmd': 'unload'})]
        # DONE/FAULT/IDLE intentionally do nothing on entry.
        return []
