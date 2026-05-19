"""Pure-state-machine tests for the cart supervisor mission."""

from cart_elevator.supervisor.mission import (
    Mission, MissionConfig, State, ActionKind, Snapshot,
)


def _cfg(target_floor: int = 4) -> MissionConfig:
    return MissionConfig(
        target_floor=target_floor,
        pickup_tag_id=200,
        hallway_panel_tag_id=210,
        in_cab_panel_tag_id=220,
        dropoff_tag_id=230,
    )


def test_starts_in_idle_no_action_on_entry():
    m = Mission(_cfg())
    assert m.state == State.IDLE
    st, act = m.step(Snapshot())
    assert st == State.IDLE
    assert act.kind == ActionKind.NONE


def test_start_press_advances_to_nav_to_pickup_with_nav_goal_action():
    m = Mission(_cfg())
    st, act = m.step(Snapshot(start_pressed=True))
    assert st == State.NAV_TO_PICKUP
    assert act.kind == ActionKind.NAV_GOAL


def test_dock_aligned_at_pickup_moves_to_wait_loaded():
    m = Mission(_cfg())
    m.state = State.DOCK_PICKUP
    st, act = m.step(Snapshot(dock_aligned=True))
    assert st == State.WAIT_LOADED
    # Wait states emit no action.
    assert act.kind == ActionKind.NONE


def test_dock_lost_during_dock_state_faults():
    m = Mission(_cfg())
    m.state = State.DOCK_HALLWAY_CALL
    st, _ = m.step(Snapshot(dock_lost=True))
    assert st == State.FAULT


def test_press_call_button_action_fired_on_entry():
    m = Mission(_cfg())
    m.state = State.DOCK_HALLWAY_CALL
    st, act = m.step(Snapshot(dock_aligned=True))
    assert st == State.PRESS_CALL_BUTTON
    assert act.kind == ActionKind.PRESS_BUTTON
    assert act.payload['button'] == 'call_up'


def test_press_floor_button_uses_target_floor_in_payload():
    m = Mission(_cfg(target_floor=3))
    m.state = State.DOCK_IN_CAB_BUTTON
    st, act = m.step(Snapshot(dock_aligned=True))
    assert st == State.PRESS_FLOOR_BUTTON
    assert act.payload['button'] == 'floor_3'


def test_hallway_door_open_waits_for_safe_to_enter():
    m = Mission(_cfg())
    m.state = State.WAIT_FOR_HALLWAY_DOOR_OPEN
    # Safe gate hasn't said yes yet.
    st, _ = m.step(Snapshot(safe_to_enter=False))
    assert st == State.WAIT_FOR_HALLWAY_DOOR_OPEN
    # Gate flips.
    st, _ = m.step(Snapshot(safe_to_enter=True))
    assert st == State.NAV_INTO_CAB


def test_wait_for_hallway_door_open_emits_set_safe_target_for_starting_floor():
    cfg = MissionConfig(target_floor=4, starting_floor=1,
                        pickup_tag_id=200, hallway_panel_tag_id=210,
                        in_cab_panel_tag_id=220, dropoff_tag_id=230)
    m = Mission(cfg)
    m.state = State.PRESS_CALL_BUTTON
    st, act = m.step(Snapshot(press_done=True))
    assert st == State.WAIT_FOR_HALLWAY_DOOR_OPEN
    assert act.kind == ActionKind.SET_SAFE_TARGET
    assert act.payload['floor'] == 1


def test_wait_for_floor_reached_emits_set_safe_target_for_destination():
    cfg = MissionConfig(target_floor=4, starting_floor=1,
                        pickup_tag_id=200, hallway_panel_tag_id=210,
                        in_cab_panel_tag_id=220, dropoff_tag_id=230)
    m = Mission(cfg)
    m.state = State.PRESS_FLOOR_BUTTON
    st, act = m.step(Snapshot(press_done=True))
    assert st == State.WAIT_FOR_FLOOR_REACHED
    assert act.kind == ActionKind.SET_SAFE_TARGET
    assert act.payload['floor'] == 4


def test_floor_reached_waits_for_safe_to_enter():
    m = Mission(_cfg(target_floor=4))
    m.state = State.WAIT_FOR_FLOOR_REACHED
    st, _ = m.step(Snapshot(safe_to_enter=False))
    assert st == State.WAIT_FOR_FLOOR_REACHED
    st, _ = m.step(Snapshot(safe_to_enter=True))
    assert st == State.NAV_OUT_OF_CAB


def test_swap_map_action_on_relocalize_state():
    m = Mission(_cfg(target_floor=4))
    m.state = State.NAV_OUT_OF_CAB
    st, act = m.step(Snapshot(nav_succeeded=True))
    assert st == State.RELOCALIZE_AT_FLOOR
    assert act.kind == ActionKind.SWAP_MAP
    assert act.payload['floor'] == 4


def test_relocalize_blocks_until_map_floor_matches():
    m = Mission(_cfg(target_floor=4))
    m.state = State.RELOCALIZE_AT_FLOOR
    st, _ = m.step(Snapshot(current_map_floor=1))
    assert st == State.RELOCALIZE_AT_FLOOR
    st, _ = m.step(Snapshot(current_map_floor=4))
    assert st == State.NAV_TO_DROPOFF


def test_full_happy_path_reaches_done():
    m = Mission(_cfg(target_floor=4))
    seq = [
        Snapshot(start_pressed=True),               # IDLE -> NAV_TO_PICKUP
        Snapshot(nav_succeeded=True),               # -> DOCK_PICKUP
        Snapshot(dock_aligned=True),                # -> WAIT_LOADED
        Snapshot(loaded=True),                      # -> NAV_TO_HALLWAY_CALL
        Snapshot(nav_succeeded=True),               # -> DOCK_HALLWAY_CALL
        Snapshot(dock_aligned=True),                # -> PRESS_CALL_BUTTON
        Snapshot(press_done=True),                  # -> WAIT_FOR_HALLWAY_DOOR_OPEN
        Snapshot(safe_to_enter=True),               # -> NAV_INTO_CAB
        Snapshot(nav_succeeded=True),               # -> DOCK_IN_CAB_BUTTON
        Snapshot(dock_aligned=True),                # -> PRESS_FLOOR_BUTTON
        Snapshot(press_done=True),                  # -> WAIT_FOR_FLOOR_REACHED
        Snapshot(safe_to_enter=True),               # -> NAV_OUT_OF_CAB
        Snapshot(nav_succeeded=True),               # -> RELOCALIZE_AT_FLOOR
        Snapshot(current_map_floor=4),              # -> NAV_TO_DROPOFF
        Snapshot(nav_succeeded=True),               # -> DOCK_DROPOFF
        Snapshot(dock_aligned=True),                # -> WAIT_UNLOADED
        Snapshot(unloaded=True),                    # -> DONE
    ]
    for snap in seq:
        m.step(snap)
    assert m.state == State.DONE


def test_fault_is_sticky():
    m = Mission(_cfg())
    m.state = State.FAULT
    st, _ = m.step(Snapshot(start_pressed=True, nav_succeeded=True))
    assert st == State.FAULT
