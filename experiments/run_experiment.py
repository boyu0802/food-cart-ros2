#!/usr/bin/env python3
"""Experiment runner for the cart_elevator floor-detection ablation study.

Drives the floor_blind_test.launch.py with parameter sweeps and records
per-detector estimates against the ground-truth floor published by
fake_elevator_imu. Outputs one CSV per experiment under experiments/results/.

Usage:
    cd ~/ros2_ws
    source install/setup.bash
    python3 experiments/run_experiment.py --experiment imu_bias
    python3 experiments/run_experiment.py --experiment imu_noise
    python3 experiments/run_experiment.py --experiment tag_dropout
    python3 experiments/run_experiment.py --experiment all
"""

from __future__ import annotations

import argparse
import csv
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.node import Node
from cart_elevator_msgs.msg import FloorEstimate


DETECTOR_TOPICS = {
    'v1_apriltag': '/elevator/floor_v1_apriltag',
    'v2_time':     '/elevator/floor_v2_time',
    'v3_accel':    '/elevator/floor_v3_accel',
    'v4_fusion':   '/elevator/floor',
}
GROUND_TRUTH_TOPIC = '/ground_truth/floor'


@dataclass
class RunResult:
    detector_floor: dict
    detector_count: dict
    ground_truth_floor: int | None
    ground_truth_count: int


class Recorder(Node):
    def __init__(self) -> None:
        super().__init__('experiment_recorder')
        self._last_floor: dict[str, int] = {}
        self._counts: dict[str, int] = {n: 0 for n in DETECTOR_TOPICS}
        self._counts['ground_truth'] = 0
        for name, topic in DETECTOR_TOPICS.items():
            self.create_subscription(
                FloorEstimate, topic,
                lambda msg, n=name: self._on(n, msg), 50)
        self.create_subscription(
            FloorEstimate, GROUND_TRUTH_TOPIC,
            lambda msg: self._on('ground_truth', msg), 50)

    def _on(self, name: str, msg: FloorEstimate) -> None:
        self._last_floor[name] = int(msg.floor)
        self._counts[name] += 1

    def snapshot(self) -> RunResult:
        return RunResult(
            detector_floor={n: self._last_floor.get(n) for n in DETECTOR_TOPICS},
            detector_count={n: self._counts.get(n, 0) for n in DETECTOR_TOPICS},
            ground_truth_floor=self._last_floor.get('ground_truth'),
            ground_truth_count=self._counts.get('ground_truth', 0),
        )


def run_one_cell(launch_args: dict, duration_s: float,
                 pre_roll_s: float = 2.0, verbose: bool = False) -> RunResult | None:
    """Spawn the blind-test launch, record for duration_s, return last values."""
    cmd = ['ros2', 'launch', 'cart_elevator', 'floor_blind_test.launch.py']
    for k, v in launch_args.items():
        cmd.append(f'{k}:={v}')
    if verbose:
        print(f'  launching: {" ".join(cmd)}')

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )

    try:
        rclpy.init()
        recorder = Recorder()

        deadline = time.time() + pre_roll_s
        while time.time() < deadline:
            rclpy.spin_once(recorder, timeout_sec=0.05)

        deadline = time.time() + duration_s
        while time.time() < deadline:
            rclpy.spin_once(recorder, timeout_sec=0.05)

        result = recorder.snapshot()
        recorder.destroy_node()
        rclpy.shutdown()
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=3)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait()
            except ProcessLookupError:
                pass
        # Brief settle to let DDS clean up.
        time.sleep(0.5)

    if result.ground_truth_count == 0:
        if verbose:
            print('  WARN: no ground-truth messages received — discarding')
        return None
    return result


def _abs_err(estimate: int | None, truth: int | None) -> int | None:
    if estimate is None or truth is None:
        return None
    return abs(estimate - truth)


def _row(experiment: str, sweep_var: str, sweep_val, scenario: str,
         detector: str, result: RunResult, trial: int) -> dict:
    est = result.detector_floor.get(detector)
    truth = result.ground_truth_floor
    return {
        'experiment': experiment,
        'sweep_var': sweep_var,
        'sweep_val': sweep_val,
        'scenario': scenario,
        'detector': detector,
        'final_estimate': est if est is not None else '',
        'final_ground_truth': truth if truth is not None else '',
        'abs_error': _abs_err(est, truth) if _abs_err(est, truth) is not None else '',
        'detector_msg_count': result.detector_count.get(detector, 0),
        'gt_msg_count': result.ground_truth_count,
        'trial': trial,
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        print(f'  no rows for {path}, skipping write')
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f'wrote {len(rows)} rows to {path}')


# Reference IMU scenario for the IMU-degradation experiments. "+3" gives ~10s
# of motion ending at floor 4 from start floor 1 — long enough that
# floor_v2_time's start/stop detection works at noise=0, so the bias/noise
# sweep cleanly shows the algorithm degrading vs. masked by edge cases.
REFERENCE_SCENARIO = '+3'
REFERENCE_FINAL_FLOOR = 4
NO_TAG_SCENARIO = '999:104:0.05'  # tag never visible during a normal recording


def experiment_imu_bias(out_csv: Path, duration_s: float, trials: int) -> None:
    """How does floor estimate degrade as IMU bias grows? No tag visible —
    isolates pure IMU performance."""
    sweep = [0.0, 0.05, 0.10, 0.20, 0.30, 0.50, 1.00, 2.00]
    rows = []
    total = len(sweep) * trials
    i = 0
    for bias in sweep:
        for trial in range(trials):
            i += 1
            print(f'[imu_bias {i}/{total}] bias={bias} trial={trial}')
            args = {
                'imu_scenario': REFERENCE_SCENARIO,
                'imu_bias_mss': bias,
                'imu_noise_std_mss': 0.02,
                'starting_floor': 1,
                'tag_scenario': NO_TAG_SCENARIO,
            }
            r = run_one_cell(args, duration_s)
            if r is None:
                continue
            for detector in DETECTOR_TOPICS:
                rows.append(_row('imu_bias', 'imu_bias_mss', bias,
                                 REFERENCE_SCENARIO, detector, r, trial))
    write_csv(out_csv, rows)


def experiment_imu_noise(out_csv: Path, duration_s: float, trials: int) -> None:
    """How does floor estimate degrade as IMU noise grows? No tag visible."""
    sweep = [0.0, 0.02, 0.05, 0.10, 0.20]
    rows = []
    total = len(sweep) * trials
    i = 0
    for noise in sweep:
        for trial in range(trials):
            i += 1
            print(f'[imu_noise {i}/{total}] noise={noise} trial={trial}')
            args = {
                'imu_scenario': REFERENCE_SCENARIO,
                'imu_bias_mss': 0.0,
                'imu_noise_std_mss': noise,
                'starting_floor': 1,
                'tag_scenario': NO_TAG_SCENARIO,
            }
            r = run_one_cell(args, duration_s)
            if r is None:
                continue
            for detector in DETECTOR_TOPICS:
                rows.append(_row('imu_noise', 'imu_noise_std_mss', noise,
                                 REFERENCE_SCENARIO, detector, r, trial))
    write_csv(out_csv, rows)


def experiment_tag_dropout(out_csv: Path, duration_s: float, trials: int) -> None:
    """Vary the moment the correct AprilTag first becomes visible.
    Late tag = long 'dropout'. IMU is clean — isolates the question
    'how well does fusion bridge the dropout using IMU as fallback?'"""
    sweep = [0.0, 2.0, 5.0, 10.0, 20.0]
    rows = []
    total = len(sweep) * trials
    i = 0
    for tag_first_s in sweep:
        # Tag 104 == floor 4 (the correct final floor for the +3 scenario).
        # When tag_first_s == 20, the tag appears near/after the end of the
        # recording window → effectively "tag never seen during this trip".
        t_str = max(0.1, tag_first_s)
        tag_scenario = f'{t_str}:104:0.05'
        for trial in range(trials):
            i += 1
            print(f'[tag_dropout {i}/{total}] tag_first={tag_first_s}s trial={trial}')
            args = {
                'imu_scenario': REFERENCE_SCENARIO,
                'imu_bias_mss': 0.0,
                'imu_noise_std_mss': 0.02,
                'starting_floor': 1,
                'tag_scenario': tag_scenario,
            }
            r = run_one_cell(args, duration_s)
            if r is None:
                continue
            for detector in DETECTOR_TOPICS:
                rows.append(_row('tag_dropout', 'tag_first_visible_s', tag_first_s,
                                 REFERENCE_SCENARIO, detector, r, trial))
    write_csv(out_csv, rows)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('--experiment', required=True,
                   choices=['imu_bias', 'imu_noise', 'tag_dropout', 'all'])
    p.add_argument('--out-dir', default='experiments/results')
    p.add_argument('--duration-s', type=float, default=25.0,
                   help='Per-cell recording window (excludes pre-roll).')
    p.add_argument('--trials', type=int, default=3,
                   help='Trials per (scenario, sweep_value) cell.')
    args = p.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    todo = {
        'imu_bias':    lambda: experiment_imu_bias(out_dir / 'imu_bias.csv',
                                                    args.duration_s, args.trials),
        'imu_noise':   lambda: experiment_imu_noise(out_dir / 'imu_noise.csv',
                                                     args.duration_s, args.trials),
        'tag_dropout': lambda: experiment_tag_dropout(out_dir / 'tag_dropout.csv',
                                                      args.duration_s, args.trials),
    }
    if args.experiment == 'all':
        for fn in todo.values():
            fn()
    else:
        todo[args.experiment]()
    return 0


if __name__ == '__main__':
    sys.exit(main())
