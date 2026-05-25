"""Unit tests for wall_fit — pure lidar wall-fitting for in-cab docking.

No ROS network and no rclpy needed: wall_fit has no ROS deps. We
synthesize cab walls as (x, y) point clouds and check that the fit
recovers the (rho, theta) we built in, that RANSAC ignores clutter,
and that pose_from_walls produces correctly-signed dock errors.
"""

import math
import random

import pytest

from cart_elevator.dock.wall_fit import (
    LineFit, scan_to_points, fit_line, ransac_line, find_walls,
    pose_from_walls,
)


def make_wall(rho, theta, n=40, extent=1.0, noise=0.0, seed=1):
    """Points along the line with normal `theta` at distance `rho`,
    spread `extent` meters along the wall. Optional Gaussian noise on
    the perpendicular (range) direction, like real lidar."""
    rng = random.Random(seed)
    nx, ny = math.cos(theta), math.sin(theta)        # normal
    tx, ty = -math.sin(theta), math.cos(theta)       # along wall
    pts = []
    for i in range(n):
        t = -extent / 2 + extent * i / (n - 1)
        r = rho + (rng.gauss(0.0, noise) if noise else 0.0)
        pts.append((r * nx + t * tx, r * ny + t * ty))
    return pts


# ---------- fit_line: recovers rho + theta ----------

def test_fit_front_wall_square():
    # Wall straight ahead at 0.34 m, perfectly square -> theta 0.
    fit = fit_line(make_wall(rho=0.34, theta=0.0))
    assert fit.rho == pytest.approx(0.34, abs=1e-6)
    assert fit.theta == pytest.approx(0.0, abs=1e-6)
    assert fit.rms == pytest.approx(0.0, abs=1e-9)


def test_fit_front_wall_yawed():
    # Cart crooked by ~6 deg: the front wall's normal tilts the same.
    fit = fit_line(make_wall(rho=0.40, theta=0.1))
    assert fit.rho == pytest.approx(0.40, abs=1e-6)
    assert fit.theta == pytest.approx(0.1, abs=1e-6)


def test_fit_left_wall():
    # A left wall: normal points +y -> theta +pi/2.
    fit = fit_line(make_wall(rho=0.30, theta=math.pi / 2))
    assert fit.rho == pytest.approx(0.30, abs=1e-6)
    assert fit.theta == pytest.approx(math.pi / 2, abs=1e-6)


def test_fit_too_few_points():
    assert fit_line([(1.0, 0.0)]) is None


def test_fit_noisy_wall_averages_down():
    # 1.5 cm per-point noise; the line should still land within a few mm.
    fit = fit_line(make_wall(rho=0.35, theta=0.0, n=80, noise=0.015))
    assert fit.rho == pytest.approx(0.35, abs=0.006)
    assert abs(fit.theta) < math.radians(2.0)


# ---------- RANSAC: robust to clutter ----------

def test_ransac_ignores_a_person_in_the_cab():
    rng = random.Random(0)
    wall = make_wall(rho=0.34, theta=0.0, n=60)
    # A blob of points ~ a person standing mid-cab, off any wall.
    blob = [(0.15 + rng.uniform(-0.03, 0.03),
             0.05 + rng.uniform(-0.03, 0.03)) for _ in range(15)]
    fit = ransac_line(wall + blob, threshold=0.02, rng=random.Random(0))
    assert fit is not None
    assert fit.rho == pytest.approx(0.34, abs=0.01)
    # The wall has way more support than the blob.
    assert fit.n_points >= 55


def test_ransac_returns_none_without_enough_support():
    assert ransac_line([(0.3, 0.0), (0.3, 0.1)], min_inliers=12) is None


# ---------- find_walls: classify front vs side ----------

def test_find_walls_separates_front_and_side():
    front_pts = make_wall(rho=0.34, theta=0.0, n=50)
    side_pts = make_wall(rho=0.30, theta=math.pi / 2, n=50, seed=2)
    front, side = find_walls(front_pts + side_pts, rng=random.Random(0))
    assert front is not None and side is not None
    # Front = normal closest to straight ahead.
    assert abs(front.theta) < math.radians(10)
    assert abs(abs(side.theta) - math.pi / 2) < math.radians(10)
    assert front.rho == pytest.approx(0.34, abs=0.02)
    assert side.rho == pytest.approx(0.30, abs=0.02)


def test_find_walls_front_only():
    front, side = find_walls(make_wall(rho=0.34, theta=0.0, n=50),
                             rng=random.Random(0))
    assert front is not None
    assert side is None


# ---------- pose_from_walls: signs feed control() correctly ----------

def test_pose_too_far_drives_forward():
    front = LineFit(rho=0.45, theta=0.0, n_points=40, rms=0.0)
    err = pose_from_walls(front, None, standoff_x=0.35)
    assert err.e_x == pytest.approx(0.10)   # > 0 -> vx forward, closes gap
    assert err.e_y == 0.0                    # no side wall -> hold lateral
    assert err.e_yaw == pytest.approx(0.0)


def test_pose_crooked_sets_yaw():
    front = LineFit(rho=0.35, theta=0.08, n_points=40, rms=0.0)
    err = pose_from_walls(front, None, standoff_x=0.35)
    assert err.e_yaw == pytest.approx(0.08)


def test_pose_lateral_left_wall_too_close_moves_right():
    front = LineFit(rho=0.35, theta=0.0, n_points=40, rms=0.0)
    left = LineFit(rho=0.25, theta=math.pi / 2, n_points=40, rms=0.0)
    err = pose_from_walls(front, left, standoff_x=0.35, side_target=0.30)
    # Too close to the LEFT wall (0.25 < 0.30) -> move right -> e_y < 0.
    assert err.e_y < 0.0
    assert err.e_y == pytest.approx(-0.05)


def test_pose_lateral_right_wall_too_close_moves_left():
    front = LineFit(rho=0.35, theta=0.0, n_points=40, rms=0.0)
    right = LineFit(rho=0.25, theta=-math.pi / 2, n_points=40, rms=0.0)
    err = pose_from_walls(front, right, standoff_x=0.35, side_target=0.30)
    # Too close to the RIGHT wall -> move left -> e_y > 0.
    assert err.e_y > 0.0
    assert err.e_y == pytest.approx(0.05)


# ---------- scan_to_points: filtering ----------

def test_scan_to_points_drops_invalid_ranges():
    ranges = [1.0, float('inf'), float('nan'), 0.05, 50.0, 2.0]
    pts = scan_to_points(ranges, angle_min=0.0, angle_increment=math.pi / 2,
                         range_min=0.1, range_max=10.0)
    # Keeps only 1.0 and 2.0 (inf/nan dropped, 0.05 < min, 50 > max).
    assert len(pts) == 2
