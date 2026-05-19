# Finding: IMU-bias failure modes of the four floor detectors

**Date:** 2026-05-19
**Data:** `experiments/results/imu_bias.csv` (96 rows; 8 bias values × 3 trials × 4 detectors).
**Scenario:** `+3` trip (floor 1 → 4), 1.5 m/s cruise, 1.5 s accel/decel ramps, 3.5 m floor spacing, 25 s recording window. Tag stream silent for the entire run (`tag_scenario = 999:104:0.05` in `run_experiment.py`).
**Bias values swept on accel_z (m/s²):** 0.0, 0.05, 0.10, 0.20, 0.30, 0.50, 1.00, 2.00.

## Final-estimate floor by (bias, detector). Ground truth = 4.

| bias (m/s²) | v1_apriltag | v2_time   | v3_accel             | v4_fusion |
|------------:|:-----------:|:---------:|:--------------------:|:---------:|
| 0.00        | n/a         | 4         | 4                    | 4         |
| 0.05        | n/a         | 4         | 4                    | 4         |
| 0.10        | n/a         | 4         | 4                    | 4         |
| 0.20        | n/a         | 4         | 4                    | 4         |
| 0.30        | n/a         | **11** (+7) | 4                  | **11** (+7) |
| 0.50        | n/a         | **12** (+8) | **49–50** (+45–46) | **12** (+8) |
| 1.00        | n/a         | **12** (+8) | **94–96** (+90–92) | **12** (+8) |
| 2.00        | n/a         | **12** (+8) | **184–188** (+180–184) | **12** (+8) |

`n/a` for v1: the bias sweep deliberately suppresses tag visibility (the harness's
`NO_TAG_SCENARIO`). v1's CSV rows show `detector_msg_count = 0` and blank
`final_estimate`; this is the experiment design, not a v1 failure.

## Three failure patterns

### 1. v3_accel drifts linearly with bias above the bias-estimator's gate
Errors above the 0.3 m/s² break-point scale linearly: ≈ 46 floors at 0.5 m/s², ≈ 91 at 1.0,
≈ 182 at 2.0. The proportionality matches what a constant uncorrected bias `b`
does to a position integrator over a fixed trip time `T`: `Δz ≈ ½ b T²`, linear in `b`
because `T` is constant across the sweep.

**Mechanism:** at `floor_v3_accel.py:92`, the bias estimator only folds in samples
where `|signed_dev_raw| < move_thr` (default `move_threshold_mss = 0.30`). Once the
bias exceeds `move_thr`, every stationary sample looks like motion to the gate,
nothing is admitted to the estimator, `bias_z` stays at 0, and the bias never gets
subtracted from the integrand. The transition is exactly at `bias ≈ move_thr` as
predicted: v3 still works at 0.30 (right on the edge — bias barely triggers the
move detector but only intermittently), starts failing at 0.50.

### 2. v2_time steps at 0.30 m/s² and *plateaus* at +8 floors
v2 returns 11 (+7 from truth) at 0.30 and 12 (+8) for every bias from 0.50
upward. The error doesn't grow — it saturates. This isn't a tuning artifact; it's
algorithmic. v2 counts trip ends via a pulse-detection FSM with a `saw_opposite_pulse`
gate that requires a sign reversal in accel to re-arm. For a single +3 trip there's a
bounded number of times the gate can re-arm during the real ramps, regardless of how
big the bias is. So once bias is big enough to spuriously fire the pulse-counter at
all, the over-count immediately hits its algorithmic ceiling (+8) and stays there.

### 3. v4_fusion is byte-identical to v2_time in the broken regime
For every (bias, trial) cell at 0.30 m/s² and above, v4's `final_estimate` exactly
matches v2's. v4 is not actually fusing — `floor_v4_fusion.py:76-84` is a
freshness-ordered fallback chain (v1 → v2 → v3). v1 never publishes in this sweep,
v2 always publishes, so v3 is never even consulted regardless of bias. v4 inherits
v2's bounded-but-wrong failure mode unchanged. The "fusion" label is misleading for
the current implementation; it's a priority selector.

## What this means for the paper

The three failure shapes are mechanistically distinct and each tells a story:

- **v3 → linear drift** is the textbook IMU-only outcome that motivates *any* sensor fusion.
- **v2 → saturating bias-induced miscount** shows that a heuristic time-based detector
  fails *gracefully* (bounded) but still wrongly, and the cap is set by the FSM's
  structure, not by physical limits.
- **v4 → inherits v2's failure** shows that priority-fallback "fusion" is not actually
  fusion. A real fusion has to cross-check sources, not just pick the freshest one.

This is the "before" picture for the fixes that follow. The fixes (A1: variance-gated
bias estimator in v3; B3: v4 cross-checks sources, clamps to building bounds, and
uses v1 — when fresh and elevator stationary — to correct v3's position *and* bias)
are aimed at exactly these three mechanisms.

## Reproducing this table

```bash
cd ros2_ws
source install/setup.bash
python3 experiments/run_experiment.py --experiment imu_bias --trials 3 --duration-s 25
# Output: experiments/results/imu_bias.csv (96 rows)
```

Sweep values are hard-coded in `run_experiment.py:178` (`experiment_imu_bias`).
