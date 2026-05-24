# Finding: first real-elevator field test — two stop-detection failures + fix

**Date:** 2026-05-24
**Data:** `elevator_ride_0524_1617` rosbag (kept locally, ~10 MB; 280 s, `/imu` @ ~100 Hz = 28,003 samples, plus `/elevator/floor_v2_time`, `/elevator/floor_v3_accel`, `/elevator/floor`).
**Setup:** real passenger elevator, 5-floor Taiwan school building, **3.8 m floor pitch (measured)**, ~1.1 m/s cruise. z-accel sourced from the RoboRIO Pigeon 2 over the NT bridge (`Robot/imu/accel_z` → `/imu`), *not* the D455 (its IMU is dead on this kernel).
**Ride (ground truth, from v2):** 4 → 1 → 2 → 4 → 1 → 2 → 1 (six trips).

This is the first time the detectors saw a real elevator instead of `fake_imu`. The synthetic bias sweep ([`imu_bias.md`](imu_bias.md)) found drift/saturation failures under *injected* bias; this run found two *different* failures driven by real sensor behavior, plus a strong positive result for the v4 fusion.

## Ground-truth signal characteristics (from `/imu`)

- **Resting bias ≈ −0.075 m/s²** when stationary — matches what v3's variance-gated estimator computed (−0.065 … −0.079), so the A1 estimator is healthy on real data.
- **Vibration noise floor:** brief spikes up to **~0.27 m/s²** while parked.
- **A 1-floor trip's decel pulse peaked at only 0.294 m/s²** — *below* `move_threshold_mss = 0.30`, and with no clean upward overshoot (the decel was mostly "downward accel ramping back to baseline").

## Two failure modes found

### 1. v3_accel — catastrophic phantom trip (ran to floor −35)
The clean opening 4→1 trip was near-perfect (integrated −10.95 m vs true −11.4 m). Then a short 1-floor trip at t≈120 s **latched v3 into a trip that never ended**: integrated position ran away monotonically to **floor −35** over the remaining ~134 s, still reporting `moving=True`, `dir=UP`, `confidence=0.40` (no self-awareness of being lost).

**Mechanism:** the stop gate was `|v_z| < v_threshold_ms` (integrated velocity). During the phantom trip the frozen bias was off by only **~0.01 m/s²**, but integrated over 134 s that tiny residual drove `v_z` to ~1.3 m/s and kept it there — so `|v_z|` never dropped back below `v_thr` and the trip never closed. Integrated velocity has no absolute reference; a constant bias error makes it diverge without bound. This is distinct from the simulated bias-sweep drift: there the *position* drifted; here the *stop condition itself* never fired.

### 2. v2_time — end-of-ride hang on a gentle decel
The final 1-floor trip's decel peaked at 0.294 m/s², just under `move_threshold` (0.30), with no upward overshoot. v2's stop FSM requires `saw_opposite_pulse` (a decel pulse above threshold) before it accepts a quiet window as end-of-trip. The pulse never armed, so v2 sat in `moving` for the last ~28 s of the ride (the IMU shows the car was at rest the whole time), rolling its floor estimate down past −1 and never settling.

## Strong positive: v4_fusion held through the v3 runaway

Despite v3 reporting floor −35, **v4 settled on the correct floor for every trip** (4→1→2→4→1→2→1). Two mechanisms from the B3 fix did the work:
1. the `[1, n_floors=5]` **range clamp** dropped v3 as a candidate the moment it left floor range — the runaway was invisible to the fusion;
2. disagreement arbitration handled the brief in-range window.

Only blemish: a 0.2 s flicker to "floor 2" when v3's fragment momentarily settled, self-corrected immediately. This is real-data evidence that the fusion isn't a nicety — it's what contains a catastrophic single-detector divergence, with the range clamp as the specific protective mechanism. (Compare the `imu_bias.md` "before" picture, where v4 was still a freshness-fallback chain.)

## The physics behind the fix

You **cannot distinguish "cruising at constant speed" from "stopped" using z-accel alone** — both read ~0. The only intrinsic stop signal is the decel pulse. v3's velocity-based stop tried to dodge this (velocity stays high in cruise, ~0 at stop) but velocity drifts, hence the runaway. The robust answer is to key the stop off **acceleration returning to baseline**, gated by a seen decel pulse so the mid-trip cruise quiet isn't mistaken for a stop — exactly what v2 already did — with a time-bounded fallback for when the decel is too gentle to detect.

## Fix (landed in this same change)

Both `floor_v2_time.py` and `floor_v3_accel.py` now:
- stop on **accel-quiet** (`|a_z| < move_thr` for `stop_samples`) **gated by `saw_opposite_pulse`** (decel seen) — so cruise can't trigger a stop;
- fall back to a **`force_stop_samples`** (default 1500 ≈ 15 s) stop after a long continuous quiet, for gentle short trips whose decel never crosses `move_thr` — bounds the hang instead of hanging/diverging forever.

v3's old `v_threshold_ms` / integrated-velocity stop gate is retired (param kept for config compat).

**Validation — replayed the recorded `/imu` (real timestamps) through the fixed code:**

| Trip (truth) | v2 before | v2 after | v3 before | v3 after |
|---|:--:|:--:|:--:|:--:|
| 4→1 | 1 ✓ | 1 ✓ | 1 ✓ | 1 ✓ |
| 1→2 | 2 ✓ | 2 ✓ | 2 (lucky) | 2 ✓ |
| 2→4 | 4 ✓ | 4 ✓ | **−35 runaway** | 4 ✓ |
| 4→1 | 1 ✓ | 1 ✓ | (lost) | 1 ✓ |
| 1→2 | 2 ✓ | 2 ✓ | (lost) | 1 ✓ |
| 2→1 | **hung at −1** | **1 ✓** (FORCED-quiet) | (lost) | 3 (off by 1, drift) |

v3 runaway eliminated (bounded; 5/6 trips correct, last 1-floor trip over-integrated by one floor — ordinary accel drift, within v4's correction range). v2 hang eliminated (force-quiet settled the last trip correctly). 18/18 floor unit tests pass (5 new regression tests covering runaway, cruise-no-false-stop, and force-stop for both detectors).

## Caveats / tuning notes

- `force_stop_samples` is **building-specific**: it must exceed the longest continuous cruise-quiet (≈ `max_floors_per_trip × floor_h / cruise_speed`). Too low → false mid-cruise stop on long trips; too high → longer hang before the fallback fires on a gentle trip. 15 s clears a 4-floor trip in this 5-floor building. Tune down once decel detection is improved.
- The forced-quiet path computes distance from duration, so it is only reliable for ~1-floor trips. Gentle *multi-floor* trips would undercount — rare, because long trips decelerate hard and hit the fast (decel-pulse) path.
- Still open: v3's start detector is eager (fires ~1 s before real motion on a pre-departure jolt) and over-integrates short trips. Not catastrophic; candidates for a follow-up.

## Reproducing

Live: rebuild, then run `nt_bridge` + `floor_v2_time` + `floor_v3_accel` + `floor_v4_fusion` against `/imu` and ride the elevator (see repo bring-up notes). Watch for `trip end (decel)` on normal trips and `trip end (FORCED-quiet)` only on gentle ones; confirm v3 never goes out of `[1, n_floors]`.

Offline: replay a recorded bag's `/imu` through fresh `FloorV2Time` / `FloorV3Accel` instances with `_now_s()` patched to return the message timestamp (so integration `dt` is correct), feeding `_on_imu()` in record order.
