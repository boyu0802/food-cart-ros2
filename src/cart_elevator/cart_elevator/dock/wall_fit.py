"""wall_fit — pure lidar wall-fitting for in-cab docking (no fiducial).

No ROS dependencies, same as `dock_control.py`, so it unit-tests in
isolation and reuses the same DockError the AprilTag dock controller
already feeds into `control()`.

The idea (see also the in-cab docking design):
  1. A /scan is a list of (angle, range) beams. Convert each to an
     (x, y) point in the robot frame.
  2. A flat wall is a row of those points lying ~on a straight line.
     Fit a line to them; in NORMAL FORM a line is two numbers:
        rho   = perpendicular distance from the robot to the wall
        theta = the wall's normal angle (which way it faces)
     -> rho is "how far am I from the wall", theta is "am I square".
  3. The FRONT wall (the one the panel is on, ~straight ahead) gives us
     distance + squareness: e_x from rho, e_yaw from theta.
  4. A perpendicular SIDE wall gives lateral position (e_y). If the
     blind lidar FOV hides it, we return e_y unknown and the caller
     holds lateral (this is exactly the repeatability question we're
     measuring inside the real cab).

Frame: ROS REP-103 robot body frame — x forward, y left, yaw CCW.
A wall straight ahead at distance d, perfectly square, has rho=d,
theta=0. A left wall at distance w has rho=w, theta=+pi/2; a right
wall theta=-pi/2.
"""

import math
import random
from dataclasses import dataclass
from typing import Optional, Sequence

from cart_elevator.dock.dock_control import DockError, wrap_angle


# A point cloud here is just a list of (x, y) tuples in the robot frame.
Point = tuple
Points = Sequence[Point]


@dataclass
class LineFit:
    rho: float            # perpendicular distance robot -> wall (m, >= 0)
    theta: float          # wall normal angle in robot frame (rad)
    n_points: int         # how many points supported this line
    rms: float            # RMS perpendicular residual of those points (m)

    def residual(self, x: float, y: float) -> float:
        """Signed perpendicular distance of (x, y) from this line."""
        return x * math.cos(self.theta) + y * math.sin(self.theta) - self.rho


def scan_to_points(ranges: Sequence[float], angle_min: float,
                   angle_increment: float, range_min: float,
                   range_max: float) -> list:
    """Convert a LaserScan's ranges into (x, y) points, dropping beams
    that are NaN/inf or outside [range_min, range_max]. Pure math — the
    node passes the raw arrays straight from the sensor_msgs/LaserScan."""
    pts = []
    a = angle_min
    for r in ranges:
        if (r is not None and math.isfinite(r)
                and range_min <= r <= range_max):
            pts.append((r * math.cos(a), r * math.sin(a)))
        a += angle_increment
    return pts


def _normalize(rho: float, theta: float) -> tuple:
    """Force rho >= 0 by flipping the normal 180 deg when needed, then
    wrap theta to (-pi, pi]. Keeps every line in one canonical form."""
    if rho < 0.0:
        rho = -rho
        theta = theta + math.pi
    return rho, wrap_angle(theta)


def fit_line(points: Points) -> Optional[LineFit]:
    """Total-least-squares line fit (minimizes PERPENDICULAR distance,
    which is what we care about for a wall). Returns the normal form
    (rho, theta). None if fewer than 2 points.

    Method: the best line passes through the centroid; its direction is
    the axis the points spread along most (the long axis of the cloud).
    The wall normal is perpendicular to that. We get the spread axis
    from the 2x2 scatter matrix in closed form — no iteration.
    """
    n = len(points)
    if n < 2:
        return None

    xm = sum(p[0] for p in points) / n
    ym = sum(p[1] for p in points) / n

    sxx = syy = sxy = 0.0
    for x, y in points:
        dx, dy = x - xm, y - ym
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy

    # Orientation of the long (max-spread) axis = the wall direction.
    phi = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    # Wall normal is perpendicular to its direction.
    theta = phi + math.pi / 2.0
    rho = xm * math.cos(theta) + ym * math.sin(theta)
    rho, theta = _normalize(rho, theta)

    ct, st = math.cos(theta), math.sin(theta)
    sq = 0.0
    for x, y in points:
        d = x * ct + y * st - rho
        sq += d * d
    rms = math.sqrt(sq / n)

    return LineFit(rho=rho, theta=theta, n_points=n, rms=rms)


def _line_through(p1: Point, p2: Point) -> Optional[tuple]:
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    if dx == 0.0 and dy == 0.0:
        return None
    theta = math.atan2(dy, dx) + math.pi / 2.0   # normal = direction + 90
    rho = p1[0] * math.cos(theta) + p1[1] * math.sin(theta)
    return _normalize(rho, theta)


def ransac_line(points: Points, threshold: float = 0.03,
                iterations: int = 60, min_inliers: int = 12,
                rng: Optional[random.Random] = None) -> Optional[LineFit]:
    """Robust single-wall fit. Repeatedly guess a line from 2 random
    points, count points within `threshold` of it ("inliers"), keep the
    best, then refit by least squares on those inliers. Clutter (a
    person standing in the cab) doesn't vote for any wall line, so it's
    rejected automatically. Returns None if no line clears min_inliers.
    """
    n = len(points)
    if n < min_inliers:
        return None
    rng = rng or random.Random(0)   # seeded -> deterministic for tests

    best_inliers: list = []
    for _ in range(iterations):
        i, j = rng.randrange(n), rng.randrange(n)
        if i == j:
            continue
        cand = _line_through(points[i], points[j])
        if cand is None:
            continue
        rho, theta = cand
        ct, st = math.cos(theta), math.sin(theta)
        inliers = [p for p in points
                   if abs(p[0] * ct + p[1] * st - rho) < threshold]
        if len(inliers) > len(best_inliers):
            best_inliers = inliers

    if len(best_inliers) < min_inliers:
        return None
    return fit_line(best_inliers)


def _sequential_fits(points: Points, threshold: float, min_inliers: int,
                     max_walls: int, rng: random.Random) -> list:
    """Fit up to `max_walls` walls: fit the dominant one, drop its
    inliers, fit the next on what's left, and so on."""
    fits = []
    remaining = list(points)
    for _ in range(max_walls):
        f = ransac_line(remaining, threshold, min_inliers=min_inliers, rng=rng)
        if f is None:
            break
        fits.append(f)
        ct, st = math.cos(f.theta), math.sin(f.theta)
        remaining = [p for p in remaining
                     if abs(p[0] * ct + p[1] * st - f.rho) >= threshold]
    return fits


# A cab wall's normal points one of four ways in the robot frame.
_WALL_REFS = {'front': 0.0, 'left': math.pi / 2,
              'right': -math.pi / 2, 'back': math.pi}


def classify_walls(points: Points, threshold: float = 0.03,
                   min_inliers: int = 12, max_walls: int = 4,
                   rng: Optional[random.Random] = None) -> dict:
    """Fit the cab walls and bucket each into front / left / right / back
    by which way its normal points (theta near 0 / +pi/2 / -pi/2 / pi).
    Returns a dict with those keys; each value is a LineFit or None. If
    two fits land in the same bucket, the one with more points wins.

    This is the robust labeling the controller needs: it can ask for the
    LEFT wall specifically and not get handed the (closer, point-richer)
    right wall by mistake.
    """
    rng = rng or random.Random(0)
    fits = _sequential_fits(points, threshold, min_inliers, max_walls, rng)
    buckets = {k: None for k in _WALL_REFS}
    for f in fits:
        key = min(_WALL_REFS, key=lambda k: abs(wrap_angle(f.theta - _WALL_REFS[k])))
        cur = buckets[key]
        if cur is None or f.n_points > cur.n_points:
            buckets[key] = f
    return buckets


def find_walls(points: Points, threshold: float = 0.03,
               min_inliers: int = 12, max_walls: int = 4,
               rng: Optional[random.Random] = None) -> tuple:
    """Convenience wrapper: (front, side) where side is whichever of the
    left/right walls has more support. For per-side control use
    classify_walls directly."""
    b = classify_walls(points, threshold, min_inliers, max_walls, rng)
    sides = [s for s in (b['left'], b['right']) if s is not None]
    side = max(sides, key=lambda s: s.n_points) if sides else None
    return b['front'], side


def pose_from_walls(front: LineFit, side: Optional[LineFit],
                    standoff_x: float, side_target: Optional[float] = None
                    ) -> DockError:
    """Turn fitted walls into the SAME DockError the AprilTag dock uses,
    so it flows straight into dock_control.control().

      e_x   = front.rho - standoff_x      (too far -> drive forward)
      e_yaw = front.theta                 (0 = square to the panel wall)
      e_y   = lateral correction from the side wall, signed so +e_y
              commands +y (left). Zero when there's no side wall or no
              side_target (caller holds lateral).

    side_target is the desired distance to the side wall. The side's
    own normal sign tells us if it's the left (+pi/2) or right (-pi/2)
    wall, so the correction pushes the right way regardless of which
    side we can see.
    """
    e_x = front.rho - standoff_x
    e_yaw = wrap_angle(front.theta)

    e_y = 0.0
    if side is not None and side_target is not None:
        # Left wall: moving +y (left) shrinks our distance to it, so to
        # grow that distance we go -y. Right wall is mirror-imaged. The
        # sign of sin(theta) (+1 left, -1 right) captures both.
        side_sign = 1.0 if math.sin(side.theta) >= 0.0 else -1.0
        e_y = -side_sign * (side_target - side.rho)

    return DockError(e_x=e_x, e_y=e_y, e_yaw=e_yaw)
