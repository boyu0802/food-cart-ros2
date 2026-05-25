"""Tests for the reworked map_swap_node relocalize logic.

Like the floor tests: construct the node, stub its publishers, and drive
the callbacks directly — no spinning executor. Verifies that re-localize
keys off the floor node's FloorEstimate (not an AprilTag), snaps AMCL to
the configured cab-exit pose, falls back gracefully on timeout, and warns
(without deadlocking) when a floor has no cab-exit pose.
"""

import pytest
import rclpy
from std_msgs.msg import Int32
from cart_elevator_msgs.msg import FloorEstimate

from cart_elevator.maps.map_swap_node import MapSwapNode


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node(cab_exit=None):
    node = MapSwapNode()
    # Params default cab-exit poses to empty; inject for the test.
    node.cab_exit = cab_exit if cab_exit is not None else {4: (1.5, 0.3, 0.0)}
    node._initposes = []
    node._floors = []
    node.initpose_pub.publish = node._initposes.append
    node.current_floor_pub.publish = node._floors.append
    return node


def _floor_est(floor, confidence):
    e = FloorEstimate()
    e.floor = int(floor)
    e.confidence = float(confidence)
    return e


def test_confirmed_floor_snaps_to_cab_exit_pose():
    node = _make_node({4: (1.5, 0.3, 0.0)})
    # Floor node already says we're on floor 4 with high confidence.
    node._on_floor(_floor_est(4, 0.9))
    node._on_swap(Int32(data=4))

    assert len(node._initposes) == 1
    p = node._initposes[0].pose.pose.position
    assert (p.x, p.y) == (1.5, 0.3)
    assert [m.data for m in node._floors] == [4]
    assert node._pending_floor is None   # committed, no longer pending


def test_waits_then_commits_when_floor_node_confirms():
    node = _make_node({4: (2.0, -0.5, 0.0)})
    node._on_swap(Int32(data=4))
    # Not confirmed yet -> nothing published, still pending.
    assert node._initposes == [] and node._floors == []
    assert node._pending_floor == 4

    node._on_floor(_floor_est(4, 0.8))
    assert len(node._initposes) == 1
    assert [m.data for m in node._floors] == [4]


def test_low_confidence_does_not_confirm():
    node = _make_node({4: (2.0, 0.0, 0.0)})
    node._on_swap(Int32(data=4))
    node._on_floor(_floor_est(4, 0.2))   # below default 0.5 threshold
    assert node._initposes == [] and node._floors == []
    assert node._pending_floor == 4


def test_timeout_falls_back_and_still_reinitializes():
    node = _make_node({4: (1.0, 1.0, 0.0)})
    node._on_swap(Int32(data=4))
    # Force the deadline into the past and tick the watchdog.
    node._pending_deadline_s = node._now_s() - 1.0
    node._tick()
    # Re-inits to the cab-exit pose anyway (the key fix vs the old node).
    assert len(node._initposes) == 1
    assert [m.data for m in node._floors] == [4]
    assert node._pending_floor is None


def test_missing_cab_exit_pose_publishes_floor_but_no_initialpose():
    node = _make_node(cab_exit={})   # no pose for any floor
    node._on_floor(_floor_est(4, 0.9))
    node._on_swap(Int32(data=4))
    # No AMCL re-init possible, but current_floor still published so the
    # mission isn't deadlocked.
    assert node._initposes == []
    assert [m.data for m in node._floors] == [4]
