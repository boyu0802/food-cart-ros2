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


def _kinds(acts):
    return [a.kind for a in acts]


def test_starts_in_idle_no_action_on_entry():
    m = Mission(_cfg())
    assert m.state == State.IDLE
    st, acts = m.step(Snapshot())
    assert st == State.IDLE
    assert acts == []


def test_start_press_advances_to_nav_to_pickup_with_nav_goal_action():
    m = Mission(_cfg())
    st, acts = m.step(Snapshot(start_pressed=True))
    assert st == State.NAV_TO_PICKUP
    assert _kinds(acts) == [ActionKind.NAV_GOAL]


def test_dock_aligned_at_pickup_moves_to_wait_loaded_and_emits_load_cmd():
    m = Mission(_cfg())
    m.state = State.DOCK_PICKUP
    st, acts = m.step(Snapshot(dock_aligned=True))
    assert st == State.WAIT_LOADED
    assert _kinds(acts) == [ActionKind.MISSION_CMD]
    assert acts[0].payload['cmd'] == 'load'


def test_dock_lost_during_dock_state_faults():
    m = Mission(_cfg())
    m.state = State.DOCK_HALLWAY_CALL
    st, _ = m.step(Snapshot(dock_lost=True))
    assert st == State.FAULT


def test_press_call_button_action_fired_on_entry():
    m = Mission(_cfg())
    m.state = State.DOCK_HALLWAY_CALL
    st, acts = m.step(Snapshot(dock_aligned=True))
    assert st == State.PRESS_CALL_BUTTON
    assert _kinds(acts) == [ActionKind.PRESS_BUTTON]
    assert acts[0].payload['button'] == 'call_up'


def test_press_floor_button_uses_target_floor_in_payload():
    m = Mission(_cfg(target_floor=3))
    m.state = State.DOCK_IN_CAB_BUTTON
    st, acts = m.step(Snapshot(dock_aligned=True))
    assert st == State.PRESS_FLOOR_BUTTON
    assert acts[0].payload['button'] == 'floor_3'


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
    st, acts = m.step(Snapshot(press_done=True))
    assert st == State.WAIT_FOR_HALLWAY_DOOR_OPEN
    assert _kinds(acts) == [ActionKind.SET_SAFE_TARGET]
    assert acts[0].payload['floor'] == 1


def test_wait_for_floor_reached_emits_set_safe_target_for_destination():
    cfg = MissionConfig(target_floor=4, starting_floor=1,
                        pickup_tag_id=200, hallway_panel_tag_id=210,
                        in_cab_panel_tag_id=220, dropoff_tag_id=230)
    m = Mission(cfg)
    m.state = State.PRESS_FLOOR_BUTTON
    st, acts = m.step(Snapshot(press_done=True))
    assert st == State.WAIT_FOR_FLOOR_REACHED
    assert _kinds(acts) == [ActionKind.SET_SAFE_TARGET]
    assert acts[0].payload['floor'] == 4


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
    st, acts = m.step(Snapshot(nav_succeeded=True))
    assert st == State.RELOCALIZE_AT_FLOOR
    assert _kinds(acts) == [ActionKind.SWAP_MAP]
    assert acts[0].payload['floor'] == 4


def test_relocalize_blocks_until_map_floor_matches():
    m = Mission(_cfg(target_floor=4))
    m.state = State.RELOCALIZE_AT_FLOOR
    st, _ = m.step(Snapshot(current_map_floor=1))
    assert st == State.RELOCALIZE_AT_FLOOR
    st, _ = m.step(Snapshot(current_map_floor=4))
    assert st == State.NAV_TO_DROPOFF


def test_loaded_advance_emits_stow_then_nav_in_that_order():
    # Cart has just been loaded; leaving the loading station requires
    # retracting the lift+pusher FIRST, then driving. Order matters
    # because the Rio reads the latest /mission/cmd value — if NAV_GOAL
    # were dispatched first it'd be fine (separate path) but emitting
    # stow first matches the physical sequence.
    m = Mission(_cfg())
    m.state = State.WAIT_LOADED
    st, acts = m.step(Snapshot(loaded=True))
    assert st == State.NAV_TO_HALLWAY_CALL
    assert _kinds(acts) == [ActionKind.MISSION_CMD, ActionKind.NAV_GOAL]
    assert acts[0].payload['cmd'] == 'stow'


def test_wait_unloaded_emits_unload_cmd_on_entry():
    m = Mission(_cfg())
    m.state = State.DOCK_DROPOFF
    st, acts = m.step(Snapshot(dock_aligned=True))
    assert st == State.WAIT_UNLOADED
    assert _kinds(acts) == [ActionKind.MISSION_CMD]
    assert acts[0].payload['cmd'] == 'unload'


def test_full_happy_path_reaches_done():
    m = Mission(_cfg(target_floor=4))
    seq = [
        Snapshot(start_pressed=True),               # IDLE -> NAV_TO_PICKUP
        Snapshot(nav_succeeded=True),               # -> DOCK_PICKUP
        Snapshot(dock_aligned=True),                # -> WAIT_LOADED  (cmd: load)
        Snapshot(loaded=True),                      # -> NAV_TO_HALLWAY_CALL (cmd: stow + nav)
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
        Snapshot(dock_aligned=True),                # -> WAIT_UNLOADED (cmd: unload)
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


# ---- Hold-to-run / restart support (2026-05-20) ----

def test_reset_returns_state_to_idle_from_any_state():
    # Restart-button rising edge must drop us out of any state, including
    # FAULT and mid-mission states. The wrapper is responsible for also
    # clearing the Snapshot latches and cancelling Nav2 / dock targets.
    for start_state in [
        State.NAV_TO_PICKUP, State.DOCK_HALLWAY_CALL,
        State.WAIT_FOR_FLOOR_REACHED, State.DONE, State.FAULT,
    ]:
        m = Mission(_cfg())
        m.state = start_state
        m.reset()
        assert m.state == State.IDLE
        # After reset, an empty snapshot should NOT auto-advance — we
        # expect the operator to publish a fresh start.
        st, acts = m.step(Snapshot())
        assert st == State.IDLE
        assert acts == []


def test_entry_actions_for_current_state_is_idempotent_for_nav():
    # Used by the wrapper to re-issue Nav2 goals after a pause cancels
    # the in-flight one. Re-calling should give the same NAV_GOAL with
    # the same pose payload as the original step().
    m = Mission(_cfg())
    _, acts = m.step(Snapshot(start_pressed=True))
    assert _kinds(acts) == [ActionKind.NAV_GOAL]
    original_pose = acts[0].payload['pose']

    resume_acts = m.entry_actions_for_current_state()
    assert _kinds(resume_acts) == [ActionKind.NAV_GOAL]
    assert resume_acts[0].payload['pose'] == original_pose


def test_entry_actions_for_current_state_returns_empty_for_wait_states():
    # WAIT_LOADED's entry action is MISSION_CMD 'load', which is NOT
    # in the wrapper's resumable set — but the *pure* layer still emits
    # it. The wrapper is responsible for filtering. This test pins the
    # pure-layer contract: WAIT_FOR_HALLWAY_DOOR_OPEN emits SET_SAFE_TARGET
    # (resumable), while pure WAIT_LOADED emits MISSION_CMD (not resumable).
    m = Mission(_cfg())
    m.state = State.WAIT_FOR_HALLWAY_DOOR_OPEN
    acts = m.entry_actions_for_current_state()
    assert _kinds(acts) == [ActionKind.SET_SAFE_TARGET]

    m.state = State.WAIT_LOADED
    acts = m.entry_actions_for_current_state()
    assert _kinds(acts) == [ActionKind.MISSION_CMD]
    assert acts[0].payload['cmd'] == 'load'


def test_reset_then_start_runs_clean_mission():
    # Restart in the middle of a mission, then drive cleanly from IDLE.
    m = Mission(_cfg(target_floor=4))
    m.step(Snapshot(start_pressed=True))        # -> NAV_TO_PICKUP
    m.step(Snapshot(nav_succeeded=True))         # -> DOCK_PICKUP
    assert m.state == State.DOCK_PICKUP

    m.reset()
    assert m.state == State.IDLE

    # Fresh start_pressed should still trigger NAV_TO_PICKUP cleanly.
    st, acts = m.step(Snapshot(start_pressed=True))
    assert st == State.NAV_TO_PICKUP
    assert _kinds(acts) == [ActionKind.NAV_GOAL]
