# Science-fair experiment data

This directory contains the floor-detector ablation harness and the raw data feeding the science-fair paper. The paper draft lives separately (web Claude project); this folder is the data side.

## What's been run so far

**imu_bias sweep** (2026-05-19, `results/imu_bias.csv`)

- **Scenario:** elevator trip from floor 1 → 4 (a `+3` move), 1.5 m/s cruise, 1.5 s accel/decel ramps, 3.5 m floor spacing.
- **Variable:** constant IMU bias on accel_z, swept at 0.0 / 0.05 / 0.10 / 0.15 / 0.20 m/s².
- **Trials:** 3 per bias level.
- **Detectors compared:** v1 (AprilTag — not exercised in this sweep, no tag stream), v2 (time × calibrated speed), v3 (double-integrated accel), v4 (fusion of v1-v3).
- **Headline result:** every detector returns the correct final floor (4) at every bias level. `abs_error = 0` for all 60 rows.

**Known data caveat:** the last cell (`bias=0.2, trial=2`) was truncated by an SSH disconnect mid-run — `gt_msg_count=81` vs ~250 elsewhere. The detector still produced the right answer from the partial trip but the row is short on samples; treat it as an incomplete trial, not a failure.

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

The user (paper author) already has a draft with its own structure on a separate surface — this README does **not** dictate paper structure. Slot the imu_bias result into whatever Results section they've built, and describe the harness/detectors at whatever level of detail matches the rest of their Methods. The headline finding is narrow: **at constant IMU biases up to 0.20 m/s², all four floor detectors recover the correct floor on a +3 trip across 3 trials.** That's a "no-failure-yet" result — it bounds robustness from below but doesn't yet locate any detector's failure mode. The interesting science likely comes from the noise / tag-dropout sweeps that haven't been run yet, or from pushing bias further.
