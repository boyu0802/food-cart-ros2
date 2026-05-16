"""Blind unit tests for the four floor detectors.

These tests directly call the node callbacks with crafted messages and
inspect internal state. They DO NOT rely on a running ROS network or
spinning executor — just rclpy.init() so node construction works.

For full end-to-end synthetic testing (with timers + topics actually
flowing), use:
    ros2 launch cart_elevator floor_blind_test.launch.py
"""

import time

import pytest
import rclpy
from sensor_msgs.msg import Imu
from cart_elevator_msgs.msg import TagDetection, FloorEstimate

from cart_elevator.floor.floor_v1_apriltag import FloorV1AprilTag
from cart_elevator.floor.floor_v2_time import FloorV2Time
from cart_elevator.floor.floor_v3_accel import FloorV3Accel
from cart_elevator.floor.floor_v4_fusion import FloorV4Fusion


GRAVITY = 9.81


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _capture_published(node, attr_name='pub'):
    """Replace node.pub.publish with a list-capturing stub."""
    captured = []
    real_publish = getattr(node, attr_name).publish
    def fake_publish(msg):
        captured.append(msg)
    getattr(node, attr_name).publish = fake_publish
    return captured


# ---------- v1: AprilTag ----------

def test_v1_emits_floor_for_valid_tag():
    node = FloorV1AprilTag()
    captured = _capture_published(node)

    tag = TagDetection()
    tag.tag_id = 103
    tag.area = 0.04
    node._on_tag(tag)

    assert len(captured) == 1
    est = captured[0]
    assert est.floor == 3
    assert est.source == FloorEstimate.SOURCE_APRILTAG
    assert 0.70 <= est.confidence <= 0.95
    node.destroy_node()


def test_v1_ignores_non_floor_tag():
    node = FloorV1AprilTag()
    captured = _capture_published(node)

    tag = TagDetection()
    tag.tag_id = 7      # below default offset of 100 -> not a floor tag
    tag.area = 0.04
    node._on_tag(tag)
    assert len(captured) == 0
    node.destroy_node()


def test_v1_ignores_tiny_tag():
    node = FloorV1AprilTag()
    captured = _capture_published(node)

    tag = TagDetection()
    tag.tag_id = 102
    tag.area = 0.001   # below min_area
    node._on_tag(tag)
    assert len(captured) == 0
    node.destroy_node()


# ---------- v2: time ----------

def _make_imu(accel_z: float) -> Imu:
    msg = Imu()
    msg.linear_acceleration.z = accel_z
    return msg


def test_v2_detects_motion_start_and_direction():
    node = FloorV2Time()
    # feed enough samples above the move threshold to trip 'start'
    for _ in range(node.start_samples + 5):
        node._on_imu(_make_imu(GRAVITY + 1.0))  # +1 m/s^2 = up
    assert node.is_moving
    assert node.direction == 1
    node.destroy_node()


def test_v2_detects_motion_stop_and_updates_floor():
    node = FloorV2Time()

    # Phase 1: accel up
    for _ in range(node.start_samples + 5):
        node._on_imu(_make_imu(GRAVITY + 1.0))
    # Phase 2: cruise — back to gravity (no |a|>thr)
    # Simulate ~3 seconds of cruise to traverse ~4.5m of floors
    # (1.5 m/s * 3 s = 4.5 m ≈ 1.3 floors at 3.5 m/floor -> rounds to 1).
    # But the node uses real wall clock, so we actually need to sleep.
    # We instead set the trip_start_t directly to a known offset to make
    # the time math deterministic.
    node.trip_start_t = node._now_s() - 3.0  # pretend trip started 3s ago
    # Phase 3: stop — feed many quiescent samples
    for _ in range(node.stop_samples + 5):
        node._on_imu(_make_imu(GRAVITY))
    assert not node.is_moving
    assert node.current_floor >= 2  # moved up at least 1 floor from start=1
    node.destroy_node()


# ---------- v3: accel ----------

def test_v3_bias_estimation_when_stationary():
    node = FloorV3Accel()
    # feed 200 stationary samples with a bias of +0.05
    for _ in range(200):
        node._on_imu(_make_imu(GRAVITY + 0.05))
    assert abs(node.bias_z - 0.05) < 0.01
    node.destroy_node()


def test_v3_integrates_and_snaps_to_floor():
    node = FloorV3Accel()
    # bias = 0, start at floor 1
    # Feed accel ramp to trigger motion start
    for _ in range(node.start_samples + 5):
        node._on_imu(_make_imu(GRAVITY + 1.0))
    assert node.is_moving
    # Force integrator to a value that should snap to ~+1 floor (3.5m)
    node.z = 3.4
    # Stop: feed quiescent samples
    for _ in range(node.stop_samples + 5):
        node._on_imu(_make_imu(GRAVITY))
    assert not node.is_moving
    assert node.current_floor == 2  # 1 + round(3.4/3.5) = 1 + 1
    node.destroy_node()


# ---------- v4: fusion ----------

def _make_estimate(floor: int, source: int, stamp_s: float,
                   confidence: float = 0.9) -> FloorEstimate:
    e = FloorEstimate()
    e.header.stamp.sec = int(stamp_s)
    e.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
    e.floor = floor
    e.confidence = confidence
    e.source = source
    return e


def test_v4_prefers_fresh_apriltag():
    node = FloorV4Fusion()
    captured = _capture_published(node)
    now = node._now_s()
    node.last_v1 = _make_estimate(5, FloorEstimate.SOURCE_APRILTAG, now)
    node.last_v2 = _make_estimate(3, FloorEstimate.SOURCE_TIME, now)
    node.last_v3 = _make_estimate(2, FloorEstimate.SOURCE_ACCEL, now)
    node._publish()
    assert len(captured) == 1
    assert captured[0].floor == 5
    assert captured[0].source == FloorEstimate.SOURCE_FUSION
    node.destroy_node()


def test_v4_falls_back_when_apriltag_stale():
    node = FloorV4Fusion()
    captured = _capture_published(node)
    now = node._now_s()
    node.last_v1 = _make_estimate(5, FloorEstimate.SOURCE_APRILTAG, now - 100.0)
    node.last_v2 = _make_estimate(3, FloorEstimate.SOURCE_TIME, now)
    node.last_v3 = _make_estimate(2, FloorEstimate.SOURCE_ACCEL, now)
    node._publish()
    assert len(captured) == 1
    assert captured[0].floor == 3
    node.destroy_node()


def test_v4_silent_when_no_sources():
    node = FloorV4Fusion()
    captured = _capture_published(node)
    node._publish()
    assert len(captured) == 0
    node.destroy_node()
