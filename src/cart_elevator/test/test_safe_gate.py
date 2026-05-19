"""Unit tests for the safe_to_enter_gate decision logic.

Constants adjusted to match the current cart_elevator_msgs schemas:
DoorState uses OPEN/CLOSED (not STATE_OPEN/STATE_CLOSED).
"""

import pytest
import rclpy

from cart_elevator_msgs.msg import DoorState, ElevatorDirection, FloorEstimate
from cart_elevator.safe.safe_to_enter_gate import SafeToEnterGate


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_door(state, confidence, stamp_s):
    m = DoorState()
    m.state = state
    m.confidence = confidence
    m.header.stamp.sec = int(stamp_s)
    m.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
    return m


def _make_dir(direction, confidence, stamp_s):
    m = ElevatorDirection()
    m.direction = direction
    m.confidence = confidence
    m.header.stamp.sec = int(stamp_s)
    m.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
    return m


def _make_floor(floor, confidence, stamp_s):
    m = FloorEstimate()
    m.floor = floor
    m.confidence = confidence
    m.header.stamp.sec = int(stamp_s)
    m.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
    return m


def test_all_conditions_met_returns_safe():
    node = SafeToEnterGate()
    t = node._now_s()
    node.last_door = _make_door(DoorState.OPEN, 0.9, t)
    node.last_dir = _make_dir(ElevatorDirection.DIRECTION_IDLE, 0.8, t)
    node.last_floor = _make_floor(node.target_floor, 0.9, t)
    node.last_inside_clear = True
    node.last_inside_t = t
    safe, _ = node.evaluate()
    assert safe is True
    node.destroy_node()


def test_door_closed_fails_closed():
    node = SafeToEnterGate()
    t = node._now_s()
    node.last_door = _make_door(DoorState.CLOSED, 0.9, t)
    node.last_dir = _make_dir(ElevatorDirection.DIRECTION_IDLE, 0.8, t)
    node.last_floor = _make_floor(node.target_floor, 0.9, t)
    node.last_inside_clear = True
    node.last_inside_t = t
    safe, reason = node.evaluate()
    assert safe is False
    assert 'door' in reason
    node.destroy_node()


def test_direction_moving_fails_closed():
    node = SafeToEnterGate()
    t = node._now_s()
    node.last_door = _make_door(DoorState.OPEN, 0.9, t)
    node.last_dir = _make_dir(ElevatorDirection.DIRECTION_UP, 0.9, t)
    node.last_floor = _make_floor(node.target_floor, 0.9, t)
    node.last_inside_clear = True
    node.last_inside_t = t
    safe, reason = node.evaluate()
    assert safe is False
    assert 'direction' in reason
    node.destroy_node()


def test_wrong_floor_fails_closed():
    node = SafeToEnterGate()
    t = node._now_s()
    node.last_door = _make_door(DoorState.OPEN, 0.9, t)
    node.last_dir = _make_dir(ElevatorDirection.DIRECTION_IDLE, 0.8, t)
    node.last_floor = _make_floor(node.target_floor + 1, 0.9, t)
    node.last_inside_clear = True
    node.last_inside_t = t
    safe, reason = node.evaluate()
    assert safe is False
    assert 'floor' in reason
    node.destroy_node()


def test_stale_door_fails_closed():
    node = SafeToEnterGate()
    t = node._now_s()
    node.last_door = _make_door(DoorState.OPEN, 0.9, t - 100.0)
    node.last_dir = _make_dir(ElevatorDirection.DIRECTION_IDLE, 0.8, t)
    node.last_floor = _make_floor(node.target_floor, 0.9, t)
    node.last_inside_clear = True
    node.last_inside_t = t
    safe, reason = node.evaluate()
    assert safe is False
    assert 'stale' in reason
    node.destroy_node()


def test_inside_not_clear_fails_closed():
    node = SafeToEnterGate()
    t = node._now_s()
    node.last_door = _make_door(DoorState.OPEN, 0.9, t)
    node.last_dir = _make_dir(ElevatorDirection.DIRECTION_IDLE, 0.8, t)
    node.last_floor = _make_floor(node.target_floor, 0.9, t)
    node.last_inside_clear = False
    node.last_inside_t = t
    safe, reason = node.evaluate()
    assert safe is False
    assert 'inside' in reason
    node.destroy_node()


def test_missing_inside_uses_default_true():
    node = SafeToEnterGate()
    t = node._now_s()
    node.last_door = _make_door(DoorState.OPEN, 0.9, t)
    node.last_dir = _make_dir(ElevatorDirection.DIRECTION_IDLE, 0.8, t)
    node.last_floor = _make_floor(node.target_floor, 0.9, t)
    safe, _ = node.evaluate()
    assert safe is True
    node.destroy_node()
