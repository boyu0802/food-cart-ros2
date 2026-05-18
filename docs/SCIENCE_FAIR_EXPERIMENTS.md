# Science Fair — Experiment Plan

Brainstormed 2026-05-18. Goal: extract quantitative, poster-worthy results from the blind-test rigs without needing the physical robot.

Framing question for the whole project (one sentence that ties every experiment together):

> **How does sensor redundancy enable autonomy in environments where any single sensor is unreliable?**

Every experiment below answers a piece of that one question.

---

## The opportunity

We already built **four floor detectors that solve the same problem in different ways** (v1 AprilTag, v2 time-integration, v3 accel-double-integration, v4 fusion), plus a digit-tracking direction detector. That structure is a built-in ablation study — exactly what science-fair judges reward, because it lets us make *measured* claims instead of doing a demo. Multiple algorithms behind one interface = controlled experiment.

The blind-test rigs (`fake_elevator_imu`, `fake_tag_publisher`, `fake_direction_image`, `video_replay`) provide **ground-truth scenarios with no hardware**. That means every experiment below is reproducible from a Python harness.

---

## Tier 1 — Do these first (highest impact, lowest effort)

### Experiment 1: Floor-detector ablation across degraded conditions

**Question:** Does multi-modal fusion (v4) beat any single sensor (v1, v2, v3) when individual sensors degrade?

**Method:** sweep one variable at a time, hold others fixed, measure final floor-estimate error on `/elevator/floor`.

| Sweep | Range | Expected outcome |
|---|---|---|
| IMU bias (`fake_elevator_imu.bias_mss`) | 0 → 0.20 m/s² in 5 steps | v3 (double-integration) blows up; v2 robust; fusion stays accurate |
| Tag dropout duration (gap in `fake_tag_publisher.scenario`) | 0 → 30 s | v1 fails during dropout; fusion degrades gracefully |
| IMU noise (`fake_elevator_imu.noise_std_mss`) | 0 → 0.10 m/s² | All single sensors degrade differently; fusion best |
| Scenario complexity | "+1", "+3", "+5,p3,-2", "+2,p5,-2,p5,+1" | Fusion advantage grows with complexity |

Each (sweep value, scenario) cell = run blind-test for ~30 s, record final estimate vs ground-truth floor, compute error.

**Poster output:** A 2×2 grid of line plots. X-axis = sweep value. Y-axis = floor-estimation error. Four lines per plot (v1, v2, v3, v4). The story tells itself: each subplot shows one single-sensor curve exploding while fusion stays flat.

**Headline claim:** "Multi-modal fusion reduced floor-estimation error by X× compared to the best single sensor under N% IMU noise."

### Experiment 2: Digit-detector robustness curve

**Question:** How much can we degrade the camera feed before digit-tracking fails?

**Method:** take `test/data/elevator_1_to_4_up.mp4`, generate degraded copies, re-run `direction_v1_digit` on each. Measure per-frame digit accuracy and per-trip direction accuracy.

| Degradation | Severity sweep |
|---|---|
| Gaussian noise | σ = 0, 5, 10, 20, 40, 80 |
| Motion blur | kernel = 0, 3, 7, 15, 25 px |
| Brightness shift | offset = -80, -40, 0, +40, +80 |
| JPEG re-encoding | quality = 95, 75, 50, 25, 10 |
| Resolution downsampling | scale = 1.0, 0.75, 0.5, 0.25 |

**Poster output:** Five robustness curves (accuracy vs degradation severity). Plus a confusion matrix showing which digit pairs are most often mistaken (probably 3↔8-shaped failures, but we have 1/2/3/4 so the actual failure modes will be interesting and unique to our display).

**Headline claim:** "Detector tolerates up to X dB of additive noise / Y-pixel motion blur / Z% JPEG quality before accuracy drops below 95%."

---

## Tier 2 — Do these if Tier 1 finishes early

### Experiment 3: Latency × accuracy Pareto frontier

**Question:** What's the trade-off between how quickly we report a floor change and how often we get it right?

**Method:** sweep `history_window_s` (direction detector) and `*_max_age_s` (fusion) over a range. For each setting, measure both:
- **Latency:** time between ground-truth floor change and `/elevator/floor` reflecting it.
- **Accuracy:** % of timesteps where `/elevator/floor` matches ground truth.

**Poster output:** A single scatter plot — x-axis latency, y-axis accuracy, one point per parameter setting. The Pareto frontier (upper-left envelope) shows the operating points that aren't dominated by any other.

**Why it stands out:** almost no high-schooler does this. Sweeping hyperparameters and showing a Pareto frontier is graduate-level methodology and immediately reads as rigorous.

### Experiment 4: ML baseline comparison

**Question:** Does our hand-rolled IoU template-match approach beat off-the-shelf OCR / a small CNN on this task?

**Method:**
- Run **EasyOCR** (or **Tesseract**) on the same recorded mp4, frame by frame, restricted to digit output.
- Optionally train a tiny CNN (~5k params, 4-class softmax over digits 1–4) on crops extracted by our pipeline.
- Compare to `direction_v1_digit` on: per-frame accuracy, mean per-frame compute time, model size.

**Expected outcome:** template-match probably wins on latency by 10–100×, ties on accuracy (4 classes is easy), wins on model size (4 PNGs vs an OCR model). Even if OCR wins on accuracy, we have a clean conclusion either way:
- IoU wins → "domain-specific simple methods beat general ML on narrow tasks"
- OCR wins → "we accepted a Y× compute cost for an X% accuracy gain"

**Poster output:** A three-column comparison table (accuracy / latency / model size) and a side-by-side example frame.

### Experiment 5: Cross-modal consistency check

**Question:** When the IMU-based floor stack disagrees with the vision-based direction detector, which one is right?

**Method:** run both stacks in parallel against scripted scenarios that include sensor failures (e.g. `fake_elevator_imu` with high bias + correct video). Log every frame where `/elevator/floor` direction-of-change disagrees with `/elevator/direction`. Categorise each disagreement.

**Why it matters:** redundant sensing is a safety-critical-perception framing. "Our system detects its own sensor failures by cross-checking modalities" is a strong claim for any engineering-leaning fair.

**Poster output:** Timeline plot showing one scenario where the IMU drifts but vision catches the inconsistency.

---

## What we'd need to build to run these

### The experiment-runner harness

A single Python script that:
1. Takes a scenario specification (YAML or CLI) with parameter sweeps.
2. For each (parameter × scenario) cell:
   - Launches the appropriate blind-test launch file as a subprocess.
   - Records `/elevator/floor`, `/elevator/direction`, and the ground-truth scenario topic (we may need to add a `/ground_truth/floor` publisher to the fake_imu node) into a rosbag or directly to CSV.
   - Computes error metrics.
3. Writes one big CSV: `experiment, sweep_var, sweep_val, scenario, detector, error, latency_s, …`.
4. A separate plot script reads the CSV and emits PNGs for the poster.

**Effort estimate:** ~1 day for a working harness. The fake helpers already publish on the right topics; the new work is the orchestration + metric computation.

### Small code additions needed

- `fake_elevator_imu` should publish its scripted ground-truth floor on `/ground_truth/floor` so the harness has a clean reference. (~30 min.)
- A script to generate the degraded mp4 variants for Experiment 2. Pure ffmpeg + opencv; no ROS. (~1 hour.)
- The plot script (`matplotlib`) — straightforward once the CSV exists. (~half day for nice-looking poster plots.)

---

## Recommended priority for a 3-day deadline

1. **Day 1:** Build the experiment-runner harness + add ground-truth publishing to the fake nodes. Run Experiment 1 end-to-end and look at the CSV — make sure the story is what we expect before sinking more time in.
2. **Day 2:** Generate degraded videos, run Experiment 2. By end of day we should have all the data for the headline plots.
3. **Day 3:** Plot polishing, poster layout, and (if there's time) one of the Tier 2 experiments — most likely #3 (Pareto) since it reuses Experiment 1's infrastructure.

If we slip, Tier 1 alone is a complete, defensible project. Tier 2 items are independent bonus points, not load-bearing.

---

## What this looks like on a poster

Suggested layout (3-panel):

| Left | Center | Right |
|---|---|---|
| **Problem & approach** — what an autonomous elevator-riding cart needs to know, our four-algorithm + fusion architecture diagram, the "redundancy" framing question. | **Results** — the 2×2 grid from Experiment 1 (fusion-vs-singles under degraded conditions), plus the robustness curves from Experiment 2. | **Discussion** — why fusion works (each sensor fails differently), failure modes we found, next steps (real-robot validation, ML comparison). |

Headline: something like *"Sensor Redundancy for Autonomous Elevator Navigation: A Quantitative Comparison of Single-Sensor and Fused Floor Estimation Under Degraded Conditions."*

---

## Open questions to resolve before starting

- [ ] Confirm we can run all of Tier 1 from the existing fake helpers, or do we need to add scripted-failure modes (currently `fake_elevator_imu` supports noise + bias but not e.g. axis dropouts)?
- [ ] Do we have time / inclination to also do Experiment 4 (ML baseline)? It's the most "wow" experiment for ML-leaning judges but eats half a day on OCR plumbing.
- [ ] Poster physical format / size requirements from the competition — affects how much we can fit.
- [ ] Is there a written-report component in addition to the poster? Most national fairs want a 5–20 page paper. Same experiments power both.
