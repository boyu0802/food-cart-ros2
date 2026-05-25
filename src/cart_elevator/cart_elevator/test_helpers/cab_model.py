"""cab_model — pure geometry of a rectangular elevator cab for testing.

No ROS deps. Shared by fake_cab_sim (the ROS sim node) and the
closed-loop wall-dock test so the two can't drift apart.

A cab is an axis-aligned box; the cart sits inside it. Given the cart
pose we ray-cast each lidar beam to the nearest wall to synthesize a
LaserScan's `ranges`. Frame: cab world x forward / y left; cart heading
0 means facing +x (the panel wall).
"""

import math
import random
from typing import Optional, Sequence

# bounds = (x_back, x_front, y_right, y_left), cart somewhere inside.
Bounds = tuple


def raycast_box(px: float, py: float, dx: float, dy: float,
                bounds: Bounds) -> float:
    """Distance from interior point (px,py) along unit dir (dx,dy) to the
    first wall of the box. The box is convex, so the nearest wall plane
    crossing in the positive direction is the hit."""
    x_back, x_front, y_right, y_left = bounds
    eps = 1e-9
    t = math.inf
    if dx > eps:
        t = min(t, (x_front - px) / dx)
    elif dx < -eps:
        t = min(t, (x_back - px) / dx)
    if dy > eps:
        t = min(t, (y_left - py) / dy)
    elif dy < -eps:
        t = min(t, (y_right - py) / dy)
    return t


def _in_blind(body_angle: float, blind_pairs: Sequence[float]) -> bool:
    """blind_pairs is a flat [start_deg, end_deg, ...] list of body-frame
    sectors the lidar can't see (e.g. swerve modules)."""
    a = math.degrees(math.atan2(math.sin(body_angle), math.cos(body_angle)))
    for i in range(0, len(blind_pairs) - 1, 2):
        lo, hi = blind_pairs[i], blind_pairs[i + 1]
        if lo <= a <= hi:
            return True
    return False


def cab_scan_ranges(rx: float, ry: float, ryaw: float, bounds: Bounds,
                    angle_min: float, angle_increment: float, n_beams: int,
                    noise: float = 0.0, rng: Optional[random.Random] = None,
                    blind_pairs: Sequence[float] = ()) -> list:
    """Synthesize a LaserScan's `ranges` for a cart at (rx,ry,ryaw) in the
    cab. Beams in a blind sector come back as inf (no return)."""
    ranges = []
    for k in range(n_beams):
        a = angle_min + k * angle_increment
        if blind_pairs and _in_blind(a, blind_pairs):
            ranges.append(float('inf'))
            continue
        wa = ryaw + a
        r = raycast_box(rx, ry, math.cos(wa), math.sin(wa), bounds)
        if noise and rng is not None:
            r += rng.gauss(0.0, noise)
        ranges.append(r)
    return ranges
