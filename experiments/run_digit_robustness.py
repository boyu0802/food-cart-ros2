#!/usr/bin/env python3
"""Digit-detector robustness sweep for the cart_elevator direction stack.

For each degraded mp4 produced by experiments/degrade_video.py, runs the
direction_video_test launch (video_replay + direction_v1_digit) and records
every ElevatorDirection message it publishes. One CSV row per message, with
the cell's degradation kind / severity attached.

Compare each row's direction against --ground-truth (the known-true
direction for the source clip; default UP for elevator_1_to_4_up.mp4).

Usage:
    cd ~/ros2_ws
    source install/setup.bash
    python3 experiments/run_digit_robustness.py
    python3 experiments/run_digit_robustness.py --kind noise --kind blur
"""

from __future__ import annotations

import argparse
import csv
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from cart_elevator_msgs.msg import ElevatorDirection


DETECTOR_TOPIC = '/elevator/direction_v1_digit'

DIRECTION_LABEL = {
    ElevatorDirection.DIRECTION_UNKNOWN: 'UNKNOWN',
    ElevatorDirection.DIRECTION_UP: 'UP',
    ElevatorDirection.DIRECTION_DOWN: 'DOWN',
    ElevatorDirection.DIRECTION_IDLE: 'IDLE',
}


class FrameRecorder(Node):
    def __init__(self) -> None:
        super().__init__('digit_robustness_recorder')
        self.frames: list[dict] = []
        self._t0 = time.time()
        self.create_subscription(
            ElevatorDirection, DETECTOR_TOPIC, self._on, 50)

    def _on(self, msg: ElevatorDirection) -> None:
        self.frames.append({
            't_rel': time.time() - self._t0,
            'direction': int(msg.direction),
            'confidence': float(msg.confidence),
        })


def parse_severity(sev_str: str):
    # Strip a one-letter (or 'sigma') prefix then parse the remaining digits/sign
    # as a number. Examples: 'sigma020' -> 20, 'k15' -> 15, 'b+040' -> 40,
    # 'b-080' -> -80, 'q50' -> 50, 's075' -> 75. Falls back to the raw string.
    i = 0
    while i < len(sev_str) and not (sev_str[i].isdigit() or sev_str[i] in '+-'):
        i += 1
    numeric = sev_str[i:]
    try:
        return float(numeric) if '.' in numeric else int(numeric)
    except ValueError:
        return sev_str


def parse_cell(mp4_path: Path) -> tuple[str, object]:
    name = mp4_path.stem
    if '_' not in name:
        return (name, '')
    kind, sev_str = name.split('_', 1)
    return (kind, parse_severity(sev_str))


def run_one_video(mp4_path: Path, duration_s: float,
                  pre_roll_s: float = 1.0, verbose: bool = False) -> list[dict]:
    cmd = [
        'ros2', 'launch', 'cart_elevator', 'direction_video_test.launch.py',
        f'video_path:={mp4_path}',
        'show:=false',
    ]
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
        rec = FrameRecorder()

        # Let the launch settle / camera replay start.
        deadline = time.time() + pre_roll_s
        while time.time() < deadline:
            rclpy.spin_once(rec, timeout_sec=0.05)

        # Reset clock so t_rel is relative to "video should be playing".
        rec._t0 = time.time()
        rec.frames.clear()

        deadline = time.time() + duration_s
        while time.time() < deadline:
            rclpy.spin_once(rec, timeout_sec=0.05)

        frames = list(rec.frames)
        rec.destroy_node()
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
        time.sleep(0.5)

    return frames


def discover_videos(input_dir: Path, kinds: list[str] | None) -> list[Path]:
    if kinds:
        out = []
        for k in kinds:
            out.extend(sorted((input_dir / k).glob('*.mp4')))
        return out
    return sorted(input_dir.rglob('*.mp4'))


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('--input-dir', default='experiments/degraded',
                   help='Directory holding {kind}/*.mp4 produced by degrade_video.py')
    p.add_argument('--out-csv', default='experiments/results/digit_robustness.csv')
    p.add_argument('--ground-truth', default='UP',
                   choices=['UP', 'DOWN', 'IDLE', 'UNKNOWN'],
                   help='Known-true direction for the source clip')
    p.add_argument('--duration-s', type=float, default=20.0,
                   help='Per-cell recording window after pre-roll (source clip is ~17s)')
    p.add_argument('--kind', action='append', default=None,
                   help='Restrict to one degradation kind; repeatable. '
                        'Default: all kinds under --input-dir.')
    p.add_argument('--verbose', action='store_true')
    args = p.parse_args()

    input_dir = Path(args.input_dir)
    if not input_dir.exists():
        print(f'input dir {input_dir} not found', file=sys.stderr)
        return 1

    videos = discover_videos(input_dir, args.kind)
    if not videos:
        print(f'no mp4s found under {input_dir}'
              + (f' for kinds={args.kind}' if args.kind else ''),
              file=sys.stderr)
        return 1
    print(f'found {len(videos)} videos')

    rows: list[dict] = []
    for i, mp4 in enumerate(videos, 1):
        kind, severity = parse_cell(mp4)
        print(f'[{i}/{len(videos)}] kind={kind} severity={severity} ({mp4.name})')
        frames = run_one_video(mp4, args.duration_s, verbose=args.verbose)
        if not frames:
            print('  WARN: no messages received')
            continue
        for fidx, f in enumerate(frames):
            label = DIRECTION_LABEL.get(f['direction'], 'UNKNOWN')
            rows.append({
                'kind': kind,
                'severity': severity,
                'video': mp4.name,
                'msg_idx': fidx,
                't_rel': round(f['t_rel'], 3),
                'direction': label,
                'confidence': round(f['confidence'], 3),
                'ground_truth': args.ground_truth,
                'correct': int(label == args.ground_truth),
            })

    out_path = Path(args.out_csv)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        print('no rows collected, nothing written', file=sys.stderr)
        return 2
    with open(out_path, 'w', newline='') as fp:
        w = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f'wrote {len(rows)} rows to {out_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
