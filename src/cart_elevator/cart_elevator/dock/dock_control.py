"""dock_control — pure control law for AprilTag docking.

No ROS dependencies. Easy to unit-test and reuse from the sim.

Frame conventions (ROS REP-103, matches the rest of the stack):
  Robot body frame: x forward, y left, yaw CCW positive.
  Tag pose given in robot frame = where the tag sits relative to the robot.

A tag sitting head-on at the desired standoff has:
  position.x = standoff_x,  position.y = 0,  yaw = pi
The tag's "forward" axis points back toward the robot when it's facing us,
which is yaw = pi in the robot frame.
"""

import math
from dataclasses import dataclass


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class DockGains:
    kp_x: float = 0.8
    kp_y: float = 0.8
    kp_yaw: float = 1.2
    # Below these, treat as zero (avoids twitch at the goal).
    dead_x: float = 0.02      # m
    dead_y: float = 0.02      # m
    dead_yaw: float = 0.035   # rad (~2 deg)
    # Output caps.
    max_vx: float = 0.4       # m/s
    max_vy: float = 0.3       # m/s
    max_omega: float = 1.0    # rad/s


@dataclass
class DockTarget:
    standoff_x: float = 0.6   # m, desired tag.x in robot frame
    standoff_y: float = 0.0   # m, desired tag.y in robot frame
    facing_yaw: float = math.pi  # tag yaw in robot frame when head-on


@dataclass
class DockError:
    e_x: float
    e_y: float
    e_yaw: float

    def norm_inf(self, scale_yaw: float = 0.2) -> float:
        # Combined "how far off are we" metric (yaw scaled to meters-ish).
        return max(abs(self.e_x), abs(self.e_y), abs(self.e_yaw) * scale_yaw)


def pose_error(tag_x: float, tag_y: float, tag_yaw: float,
               target: DockTarget) -> DockError:
    return DockError(
        e_x=tag_x - target.standoff_x,
        e_y=tag_y - target.standoff_y,
        e_yaw=wrap_angle(tag_yaw - target.facing_yaw),
    )


def _clip_dead(value: float, dead: float, cap: float) -> float:
    if abs(value) < dead:
        return 0.0
    return max(-cap, min(cap, value))


def control(err: DockError, gains: DockGains) -> tuple[float, float, float]:
    """Return (vx, vy, omega) for a swerve base given the pose error."""
    vx = _clip_dead(gains.kp_x * err.e_x, gains.dead_x * gains.kp_x, gains.max_vx)
    vy = _clip_dead(gains.kp_y * err.e_y, gains.dead_y * gains.kp_y, gains.max_vy)
    omega = _clip_dead(gains.kp_yaw * err.e_yaw,
                       gains.dead_yaw * gains.kp_yaw, gains.max_omega)
    return vx, vy, omega


def aligned(err: DockError, gains: DockGains) -> bool:
    # Use <= so a value sitting exactly at the deadband (where the controller
    # has already clipped output to zero) counts as aligned instead of
    # leaving the robot stuck at the boundary forever.
    return (abs(err.e_x) <= gains.dead_x
            and abs(err.e_y) <= gains.dead_y
            and abs(err.e_yaw) <= gains.dead_yaw)
