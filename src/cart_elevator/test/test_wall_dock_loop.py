"""Closed-loop test for the lidar wall-dock, with no ROS in the loop.

Runs the exact pipeline the node runs each scan — synthesize cab ranges
(cab_model) -> scan_to_points -> find_walls -> pose_from_walls ->
control() -> integrate the swerve base — and checks the cart converges
to the commanded standoff/centered/square pose. This is the math proof
behind the ROS sim (fake_cab_sim + wall_dock_controller).
"""

import math

import pytest

from cart_elevator.dock.dock_control import DockGains, control, aligned
from cart_elevator.dock import wall_fit
from cart_elevator.test_helpers.cab_model import cab_scan_ranges


def _run(start, bounds, standoff_x, side, side_target,
         steps=600, dt=0.05, noise=0.0):
    rx, ry, ryaw = start
    angle_min, n_beams = -math.pi, 360
    angle_inc = 2.0 * math.pi / n_beams
    gains = DockGains()
    import random
    rng = random.Random(0)

    last_err = None
    for _ in range(steps):
        ranges = cab_scan_ranges(rx, ry, ryaw, bounds, angle_min, angle_inc,
                                 n_beams, noise=noise, rng=rng)
        pts = wall_fit.scan_to_points(ranges, angle_min, angle_inc, 0.15, 4.0)
        walls = wall_fit.classify_walls(pts)
        front = walls['front']
        assert front is not None

        use_side = walls[side] if side in ('left', 'right') else None
        st = side_target if use_side is not None else None
        err = wall_fit.pose_from_walls(front, use_side, standoff_x, st)
        last_err = err

        vx, vy, omega = control(err, gains)
        c, s = math.cos(ryaw), math.sin(ryaw)
        rx += (c * vx - s * vy) * dt
        ry += (s * vx + c * vy) * dt
        ryaw = math.atan2(math.sin(ryaw + omega * dt), math.cos(ryaw + omega * dt))

    return (rx, ry, ryaw), last_err, gains


def test_converges_to_standoff_centered_square():
    # 1.4 m cab, panel on +x. standoff 0.30 -> x = 0.7-0.30 = 0.40.
    # left wall at +0.7, side_target 0.70 -> centered at y = 0.0. square.
    bounds = (-0.7, 0.7, -0.7, 0.7)
    pose, err, gains = _run(start=(-0.25, 0.18, 0.15), bounds=bounds,
                            standoff_x=0.30, side='left', side_target=0.70)
    rx, ry, ryaw = pose
    # Settles to within the controller deadband (dead_x/y = 2 cm) of the
    # target — comfortably inside the 2-5 cm pusher tolerance.
    assert rx == pytest.approx(0.40, abs=gains.dead_x + 0.01)
    assert ry == pytest.approx(0.0, abs=gains.dead_y + 0.01)
    assert abs(ryaw) < math.radians(2.5)
    assert aligned(err, gains)


def test_converges_from_the_other_side_and_yaw():
    # Start off to the right and yawed the other way.
    bounds = (-0.7, 0.7, -0.7, 0.7)
    pose, err, gains = _run(start=(-0.1, -0.22, -0.2), bounds=bounds,
                            standoff_x=0.35, side='left', side_target=0.70)
    rx, ry, ryaw = pose
    assert rx == pytest.approx(0.35, abs=gains.dead_x + 0.01)   # 0.7 - 0.35
    assert ry == pytest.approx(0.0, abs=gains.dead_y + 0.01)
    assert abs(ryaw) < math.radians(2.5)


def test_converges_with_lidar_noise():
    bounds = (-0.7, 0.7, -0.7, 0.7)
    pose, err, gains = _run(start=(-0.2, 0.15, 0.1), bounds=bounds,
                            standoff_x=0.30, side='left', side_target=0.70,
                            noise=0.01)
    rx, ry, ryaw = pose
    # Within the ~2-5 cm pusher tolerance even with 1 cm range noise.
    assert rx == pytest.approx(0.40, abs=0.03)
    assert ry == pytest.approx(0.0, abs=0.03)
    assert abs(ryaw) < math.radians(3.0)


def test_distance_and_squareness_hold_without_a_side_wall():
    # side='none' -> lateral uncorrected, but standoff + squareness must
    # still converge (the front wall alone gives those two).
    bounds = (-0.7, 0.7, -0.7, 0.7)
    pose, err, gains = _run(start=(-0.2, 0.10, 0.12), bounds=bounds,
                            standoff_x=0.30, side='none', side_target=0.0)
    rx, ry, ryaw = pose
    assert rx == pytest.approx(0.40, abs=gains.dead_x + 0.01)
    assert abs(ryaw) < math.radians(2.5)
    # ry is uncorrected (no side wall); it sidesteps a few cm while driving
    # forward yawed, then holds. This drift is exactly why the side wall
    # matters — it stays within ~5 cm here but eats into the pusher budget.
    assert ry == pytest.approx(0.10, abs=0.05)
