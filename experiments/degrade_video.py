#!/usr/bin/env python3
"""Generate degraded variants of a video for the digit-detector robustness study.

Reads one input mp4 and writes N output mp4s under an output dir, each
applying one type of degradation at one severity level. Pure OpenCV — no ROS.

Usage:
    cd ~/ros2_ws
    python3 experiments/degrade_video.py \\
        --input src/cart_elevator/test/data/elevator_1_to_4_up.mp4 \\
        --out-dir experiments/degraded
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


def open_video(path: Path):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f'cannot open {path}')
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return cap, fps, w, h


def open_writer(path: Path, fps: float, w: int, h: int):
    path.parent.mkdir(parents=True, exist_ok=True)
    # mp4v works on this ARM build of OpenCV; H264 would be smaller but needs ffmpeg.
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    return cv2.VideoWriter(str(path), fourcc, fps, (w, h))


def degrade_noise(frame: np.ndarray, sigma: float) -> np.ndarray:
    if sigma <= 0:
        return frame
    noise = np.random.normal(0.0, sigma, frame.shape).astype(np.float32)
    out = frame.astype(np.float32) + noise
    return np.clip(out, 0, 255).astype(np.uint8)


def degrade_blur(frame: np.ndarray, ksize: int) -> np.ndarray:
    if ksize <= 1:
        return frame
    k = ksize if ksize % 2 == 1 else ksize + 1
    return cv2.GaussianBlur(frame, (k, k), 0)


def degrade_brightness(frame: np.ndarray, offset: int) -> np.ndarray:
    if offset == 0:
        return frame
    out = frame.astype(np.int16) + offset
    return np.clip(out, 0, 255).astype(np.uint8)


def degrade_jpeg(frame: np.ndarray, quality: int) -> np.ndarray:
    if quality >= 100:
        return frame
    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        return frame
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def degrade_scale(frame: np.ndarray, scale: float) -> np.ndarray:
    if scale >= 1.0:
        return frame
    h, w = frame.shape[:2]
    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
    small = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)
    return cv2.resize(small, (w, h), interpolation=cv2.INTER_LINEAR)


# (kind, severity_label, severity_value, transform_callable)
DEGRADATIONS = []
for s in (0, 5, 10, 20, 40, 80):
    DEGRADATIONS.append(('noise', f'sigma{s:03d}', s, lambda f, s=s: degrade_noise(f, s)))
for k in (0, 3, 7, 15, 25):
    DEGRADATIONS.append(('blur', f'k{k:02d}', k, lambda f, k=k: degrade_blur(f, k)))
for b in (-80, -40, 0, 40, 80):
    DEGRADATIONS.append(('brightness', f'b{b:+04d}', b, lambda f, b=b: degrade_brightness(f, b)))
for q in (95, 75, 50, 25, 10):
    DEGRADATIONS.append(('jpeg', f'q{q:02d}', q, lambda f, q=q: degrade_jpeg(f, q)))
for s in (1.0, 0.75, 0.50, 0.25):
    DEGRADATIONS.append(('scale', f's{int(s*100):03d}', s, lambda f, s=s: degrade_scale(f, s)))


def process(input_path: Path, out_dir: Path) -> None:
    cap, fps, w, h = open_video(input_path)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f'{input_path}: {total} frames {w}x{h} @ {fps:.1f} fps')

    # Read frames once, apply each degradation in pass.
    frames: list[np.ndarray] = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frames.append(frame)
    cap.release()
    print(f'  loaded {len(frames)} frames into memory')

    for kind, label, value, fn in DEGRADATIONS:
        out_path = out_dir / kind / f'{kind}_{label}.mp4'
        writer = open_writer(out_path, fps, w, h)
        for frame in frames:
            writer.write(fn(frame))
        writer.release()
        print(f'  wrote {out_path.relative_to(out_dir.parent)}'
              f' ({kind}={value})')


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('--input', required=True,
                   help='Input mp4 (e.g. test/data/elevator_1_to_4_up.mp4)')
    p.add_argument('--out-dir', required=True,
                   help='Output dir; per-kind subdirs are created underneath')
    args = p.parse_args()

    input_path = Path(args.input)
    out_dir = Path(args.out_dir)
    process(input_path, out_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
