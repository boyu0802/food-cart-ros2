"""Pure-function classifier for elevator door state from depth statistics.

Kept ROS-free so it can be unit-tested directly with crafted numbers, the
way `dock_control.py` is. The wrapper node (`door_state_detector.py`)
crops a depth image to the door ROI and feeds aggregates here.

Inputs per call:
  median_m   : float — median depth in the ROI right now.
  lateral_std_m : float — std-dev of column-wise medians in the ROI
                  (a closed door is flat across columns; a half-open one
                  has a depth step where the door panel ends).
  delta_lat_per_s : float — rate of change of lateral_std_m. Positive
                    while the door is moving (boundary travelling
                    sideways), near zero when stationary.

We do NOT keep recent depth frames here — that's the wrapper's job.

Decision logic (rule-based, intentionally simple):
  closed_depth_m : if median < this and lateral_std small → CLOSED
  open_depth_m   : if median > this and lateral_std small → OPEN
  moving_dlat_thr_per_s : |delta_lat_per_s| above this → OPENING or
                          CLOSING, sign discriminated by whether
                          lateral_std is rising (opening) or falling
                          (closing).
  otherwise UNKNOWN (transient between states or noisy frame).
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class DoorThresholds:
    closed_depth_m: float = 0.50         # door surface closer than this = closed
    open_depth_m: float = 1.50           # depth past this = into the cab
    lateral_std_flat_m: float = 0.05     # below = uniform depth column to column
    moving_dlat_thr_per_s: float = 0.10  # |d/dt lateral_std| above this = moving

    def __post_init__(self) -> None:
        if self.closed_depth_m >= self.open_depth_m:
            raise ValueError(
                'closed_depth_m must be < open_depth_m '
                f'(got {self.closed_depth_m} >= {self.open_depth_m})')


# Mirror the constants in DoorState.msg to keep the pure layer self-contained.
UNKNOWN = 0
CLOSED  = 1
OPENING = 2
OPEN    = 3
CLOSING = 4


@dataclass(frozen=True)
class Classification:
    state: int          # one of UNKNOWN/CLOSED/OPENING/OPEN/CLOSING
    confidence: float   # 0..1


def classify(median_m: float,
             lateral_std_m: float,
             delta_lat_per_s: float,
             thr: DoorThresholds = DoorThresholds()) -> Classification:
    # NaN / invalid sentinels surface as UNKNOWN rather than misclassifying.
    if not (median_m == median_m) or median_m <= 0.0:  # NaN check via x!=x
        return Classification(UNKNOWN, 0.0)

    flat = lateral_std_m < thr.lateral_std_flat_m
    moving = abs(delta_lat_per_s) > thr.moving_dlat_thr_per_s

    if moving:
        # Door panel sliding: lateral_std climbing = opening (a step is
        # being introduced as the panel retracts past the ROI edge),
        # falling = closing.
        if delta_lat_per_s > 0:
            return Classification(OPENING, _conf_from_motion(delta_lat_per_s, thr))
        else:
            return Classification(CLOSING, _conf_from_motion(-delta_lat_per_s, thr))

    if flat:
        if median_m < thr.closed_depth_m:
            return Classification(CLOSED,
                                  _conf_from_distance(thr.closed_depth_m - median_m,
                                                      thr.closed_depth_m))
        if median_m > thr.open_depth_m:
            return Classification(OPEN,
                                  _conf_from_distance(median_m - thr.open_depth_m,
                                                      thr.open_depth_m))

    # Flat but in the gap between closed_depth and open_depth, or jagged
    # lateral profile with no motion (e.g. someone standing in the way).
    return Classification(UNKNOWN, 0.3)


def _conf_from_distance(slack_m: float, ref_m: float) -> float:
    # Confidence saturates once median is 'one reference' past the boundary.
    return max(0.5, min(0.95, 0.5 + 0.45 * (slack_m / max(ref_m, 1e-3))))


def _conf_from_motion(speed: float, thr: DoorThresholds) -> float:
    # Cheap motion-confidence: how far above the moving threshold are we.
    excess = (speed - thr.moving_dlat_thr_per_s) / max(thr.moving_dlat_thr_per_s, 1e-3)
    return max(0.5, min(0.9, 0.5 + 0.4 * excess))
