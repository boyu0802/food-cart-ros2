"""Pure-function tests for the door state classifier."""

from cart_elevator.door.door_classifier import (
    DoorThresholds, classify, CLOSED, OPENING, OPEN, CLOSING, UNKNOWN,
)


def test_closed_when_shallow_and_flat():
    cls = classify(median_m=0.30, lateral_std_m=0.01,
                   delta_lat_per_s=0.0)
    assert cls.state == CLOSED
    assert cls.confidence > 0.5


def test_open_when_deep_and_flat():
    cls = classify(median_m=2.20, lateral_std_m=0.02,
                   delta_lat_per_s=0.0)
    assert cls.state == OPEN
    assert cls.confidence > 0.5


def test_opening_when_lateral_std_rising_fast():
    # Lateral std climbing -> door panel introducing a depth edge
    # into the ROI as it retracts.
    cls = classify(median_m=1.00, lateral_std_m=0.40,
                   delta_lat_per_s=+0.50)
    assert cls.state == OPENING


def test_closing_when_lateral_std_falling_fast():
    cls = classify(median_m=1.00, lateral_std_m=0.40,
                   delta_lat_per_s=-0.50)
    assert cls.state == CLOSING


def test_unknown_when_flat_but_mid_depth():
    # Flat lateral profile, depth between closed_depth and open_depth
    # — e.g. someone standing right in the doorway, blocking the view.
    cls = classify(median_m=0.90, lateral_std_m=0.01,
                   delta_lat_per_s=0.0)
    assert cls.state == UNKNOWN


def test_unknown_when_median_invalid():
    cls = classify(median_m=0.0, lateral_std_m=0.05,
                   delta_lat_per_s=0.0)
    assert cls.state == UNKNOWN


def test_threshold_validation():
    import pytest
    with pytest.raises(ValueError):
        DoorThresholds(closed_depth_m=2.0, open_depth_m=1.0)


def test_custom_thresholds_take_effect():
    # Tight elevator with shallow open depth.
    thr = DoorThresholds(closed_depth_m=0.30, open_depth_m=0.80,
                         lateral_std_flat_m=0.05,
                         moving_dlat_thr_per_s=0.10)
    cls = classify(median_m=0.90, lateral_std_m=0.02,
                   delta_lat_per_s=0.0, thr=thr)
    assert cls.state == OPEN
