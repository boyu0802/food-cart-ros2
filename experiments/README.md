# Science-fair experiment data

This directory contains the floor-detector ablation harness and the raw data feeding the science-fair paper. The paper draft lives separately (web Claude project); this folder is the data side.

## What's been run so far

**imu_bias sweep** (2026-05-19, `results/imu_bias.csv`, 96 rows)

- **Scenario:** elevator trip from floor 1 → 4 (a `+3` move), 1.5 m/s cruise, 1.5 s accel/decel ramps, 3.5 m floor spacing.
- **Variable:** constant IMU bias on accel_z, swept at 0.0 / 0.05 / 0.10 / 0.20 / 0.30 / 0.50 / 1.00 / 2.00 m/s².
- **Trials:** 3 per bias level (24 trials × 4 detectors = 96 rows).
- **Detectors compared:** v1 (AprilTag — not exercised in this sweep, no tag stream — see [`findings/imu_bias.md`](findings/imu_bias.md)), v2 (time × calibrated speed), v3 (double-integrated accel), v4 (fusion of v1-v3).
- **Headline result — three distinct failure modes found, full analysis in [`findings/imu_bias.md`](findings/imu_bias.md):**
  - **v3_accel** drifts linearly with bias once bias exceeds the bias-estimator's `move_thr` (0.30 m/s²). Errors ≈ 46 / 91 / 182 floors at bias = 0.5 / 1.0 / 2.0.
  - **v2_time** steps at 0.30 m/s² and *saturates* at +8 floors of error from 0.50 upward — algorithmic ceiling on the pulse-detection FSM, not a tuning issue.
  - **v4_fusion** is byte-identical to v2_time in the broken regime. The current implementation is a freshness-ordered fallback chain (v1 → v2 → v3), not a fusion; v3 is never consulted because v2 is always fresher.

This is the "before" data for the in-progress fix work (A1: variance-gated bias estimator in v3; B3: real cross-checking v4 that clamps to building bounds and uses v1 to correct v3 when the elevator is stationary).

## How to read `results/imu_bias.csv`

| column | meaning |
|---|---|
| `experiment` | sweep name (e.g. `imu_bias`) |
| `sweep_var`, `sweep_val` | which parameter was varied and at what value |
| `scenario` | trip target relative to the start floor (`+3` = up three floors) |
| `detector` | one of `v1_apriltag`, `v2_time`, `v3_accel`, `v4_fusion` |
| `final_estimate` | the floor the detector settled on (blank if it never published a stop) |
| `final_ground_truth` | the actual final floor from `fake_imu`'s parallel timeline |
| `abs_error` | `|final_estimate - final_ground_truth|`, blank if no estimate |
| `detector_msg_count`, `gt_msg_count` | how many messages each side published during the run |
| `trial` | 0-indexed trial number within this (sweep_var, sweep_val, scenario) cell |

## The detectors being ablated

All in `src/cart_elevator/cart_elevator/floor/`.

- **`floor_v1_apriltag.py`** — reads floor-labeled AprilTag detections from the Limelight bridge. Authoritative when a tag is visible; silent otherwise.
- **`floor_v2_time.py`** — detects trip start/stop from accel_z magnitude, then estimates floors as `(duration - ramp_time) × cruise_speed / floor_height`. Subtracting `ramp_time` corrects for the over-count you'd get if you naively assumed cruise speed during the accel/decel ramps.
- **`floor_v3_accel.py`** — double-integrates accel_z (with stationary-period bias estimation) to recover position. Stop condition is `|v_z| < v_threshold_ms` (default 0.20 m/s) — using velocity instead of raw accel bridges the cruise phase, which would otherwise look like a stop because `a_z ≈ 0` during cruise.
- **`floor_v4_fusion.py`** — confidence-weighted fusion of v1/v2/v3.

## How the sweep was generated

- **Runner:** `run_experiment.py` — for each (sweep_var, sweep_val, scenario, trial), launches `floor_blind_test.launch.py` with the swept parameter, records `/elevator/floor_v*` and `/ground_truth/floor` topics, writes one CSV row per detector.
- **Fake IMU:** `cart_elevator/test_helpers/fake_imu.py` synthesizes accel_z for the requested trip schedule, applies the bias/noise from launch args, and publishes the ground-truth floor in parallel so each trial is self-labeling.
- **Launch wiring:** `src/cart_elevator/launch/floor_blind_test.launch.py` exposes `imu_bias_mss`, `imu_noise_std_mss`, `starting_floor` as args.

## What's NOT in the repo yet

- `imu_noise` and `tag_dropout` sweeps — harness ready, not run.
- Digit-detector robustness sweep (`run_digit_robustness.py`) — harness ready, source videos in `experiments/degraded/` are gitignored (745 MB, regenerate locally with `degrade_video.py` against a source clip). Only `noise/` (σ = 0/5/10/20/40) has been generated so far; blur/brightness/jpeg/scale kinds and σ=80 still pending.
- Plotting / notebook — unstarted.

## For the full experimental plan

See `docs/SCIENCE_FAIR_EXPERIMENTS.md` in this repo — that's the Tier 1 / Tier 2 study design that this folder is implementing.

## Note for an LLM reading this to update a paper draft

The user (paper author) already has a draft with its own structure on a separate surface — this README does **not** dictate paper structure. Slot the imu_bias result into whatever Results section they've built, and describe the harness/detectors at whatever level of detail matches the rest of their Methods. The headline finding is now concrete: **the bias sweep up to 2.0 m/s² locates three mechanistically distinct failure modes** — v3 drifts linearly past `move_thr`, v2 saturates at +8 floors due to an FSM ceiling, and v4's "fusion" is actually a freshness-fallback chain that inherits v2's failure. See [`findings/imu_bias.md`](findings/imu_bias.md) for the per-cell table, the mathematical mechanism for each pattern (linear in bias for v3 from `Δz ≈ ½bT²`, etc.), and which lines of source each mechanism traces to.

When fixes for these (variance-gated bias estimator in v3; real cross-checking + tag-driven correction in v4) land and the sweep is re-run, the README will get an "after" comparison; the findings/ file is the durable record of the failure before the fix.
