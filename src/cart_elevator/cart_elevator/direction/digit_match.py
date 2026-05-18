#!/usr/bin/env python3
"""Image-processing helpers shared by direction_v1_digit and any future
tools that need to read the elevator hall display.

Pipeline (per frame):
  bgr -> find_housing      -> bbox of the dark display panel
      -> warm_mask         -> binary mask of lit amber/red LEDs
      -> extract_digit     -> tight, scale-normalized binary crop of the
                              digit alone (arrow region is excluded by
                              picking the lower-half lit component)
      -> classify_digit    -> (digit, score)
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

# Template canvas — digits are normalized to this size before matching.
TPL_W, TPL_H = 32, 48

# Housing detection
HOUSING_MIN_AREA_FRAC = 0.01
HOUSING_MAX_AREA_FRAC = 0.25
HOUSING_MIN_ASPECT = 1.2
HOUSING_MAX_ASPECT = 4.0
HOUSING_DARK_THRESHOLD = 55
HOUSING_CLOSE_KERNEL = 25

# Warm-bright mask (covers red wall buttons AND amber digit LEDs).
HSV_LO_1, HSV_HI_1 = (0, 80, 80), (30, 255, 255)
HSV_LO_2, HSV_HI_2 = (160, 80, 80), (180, 255, 255)

# Minimum lit-pixel area to consider a digit candidate.
MIN_DIGIT_AREA = 30


def find_housing(bgr: np.ndarray) -> tuple[int, int, int, int] | None:
    """Locate the dark display housing on the elevator panel.

    Returns (x, y, w, h) or None if no plausible housing was found.
    """
    h_img, w_img = bgr.shape[:2]
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    _, dark = cv2.threshold(gray, HOUSING_DARK_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((HOUSING_CLOSE_KERNEL, HOUSING_CLOSE_KERNEL), np.uint8)
    closed = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    area_img = h_img * w_img
    best = None
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        a = w * h
        if a < HOUSING_MIN_AREA_FRAC * area_img:
            continue
        if a > HOUSING_MAX_AREA_FRAC * area_img:
            continue
        aspect = h / w if w else 0
        if not (HOUSING_MIN_ASPECT <= aspect <= HOUSING_MAX_ASPECT):
            continue
        # Housing must actually contain warm-bright LEDs.
        roi = bgr[y:y + h, x:x + w]
        if warm_mask(roi).sum() < 50 * 255:
            continue
        cx = x + w / 2
        center_pen = abs(cx - w_img / 2) / (w_img / 2)
        score = a * (1.0 - 0.3 * center_pen)
        if best is None or score > best[0]:
            best = (score, (x, y, w, h))
    return best[1] if best else None


def warm_mask(bgr: np.ndarray) -> np.ndarray:
    """Binary mask of lit amber/red LEDs."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return cv2.bitwise_or(cv2.inRange(hsv, HSV_LO_1, HSV_HI_1),
                          cv2.inRange(hsv, HSV_LO_2, HSV_HI_2))


def extract_digit(mask: np.ndarray, housing: tuple[int, int, int, int]
                  ) -> np.ndarray | None:
    """Isolate the digit and return it normalized to TPL_W x TPL_H.

    Picks the largest connected lit component whose centroid is in the
    lower half of the housing -- that's the digit, never the arrow.
    """
    x, y, w, h = housing
    sub = mask[y:y + h, x:x + w]
    fused = cv2.dilate(sub, np.ones((3, 3), np.uint8), iterations=2)
    num, _, stats, centroids = cv2.connectedComponentsWithStats(fused, 8)

    half_y = h / 2
    best = None
    for lbl in range(1, num):
        area = stats[lbl, cv2.CC_STAT_AREA]
        if area < MIN_DIGIT_AREA:
            continue
        if centroids[lbl][1] < half_y:
            continue  # arrow region
        if best is None or area > best[0]:
            best = (area, lbl)
    if best is None:
        return None

    lbl = best[1]
    cx0, cy0, cw, ch, _ = stats[lbl]
    # Use the un-dilated mask for the actual template content.
    digit = sub[cy0:cy0 + ch, cx0:cx0 + cw]
    if digit.sum() < 20 * 255:
        return None

    dh, dw = digit.shape
    scale = min(TPL_W / dw, TPL_H / dh)
    nw, nh = max(1, int(dw * scale)), max(1, int(dh * scale))
    resized = cv2.resize(digit, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((TPL_H, TPL_W), dtype=np.uint8)
    ox, oy = (TPL_W - nw) // 2, (TPL_H - nh) // 2
    canvas[oy:oy + nh, ox:ox + nw] = resized
    return canvas


def load_templates(directory: str | Path) -> dict[int, np.ndarray]:
    """Load digit_<N>.png templates from a directory."""
    out: dict[int, np.ndarray] = {}
    for p in Path(directory).glob("digit_*.png"):
        try:
            n = int(p.stem.split("_")[1])
        except (IndexError, ValueError):
            continue
        img = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        if img.shape != (TPL_H, TPL_W):
            img = cv2.resize(img, (TPL_W, TPL_H), interpolation=cv2.INTER_AREA)
        out[n] = (img > 127).astype(np.uint8)
    return out


def classify_digit(crop: np.ndarray, templates: dict[int, np.ndarray]
                   ) -> tuple[int, float, float] | None:
    """Pick the best-matching template by IoU.

    Returns (digit, top_score, margin) or None if no templates loaded.
    `margin` = top_score - second_score; small margins mean low confidence.
    """
    if not templates:
        return None
    crop_b = (crop > 127).astype(np.uint8)
    scores: list[tuple[float, int]] = []
    for digit, tpl in templates.items():
        inter = int(np.bitwise_and(crop_b, tpl).sum())
        union = int(np.bitwise_or(crop_b, tpl).sum())
        iou = inter / union if union else 0.0
        scores.append((iou, digit))
    scores.sort(reverse=True)
    top_score, top_digit = scores[0]
    margin = top_score - scores[1][0] if len(scores) > 1 else top_score
    return top_digit, top_score, margin
