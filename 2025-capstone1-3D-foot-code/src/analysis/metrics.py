"""Metric computations from extracted landmarks."""

from __future__ import annotations

import math

from .common_types import FootFrame, FootLandmarks, FootMetrics


def compute_metrics(landmarks: FootLandmarks, frame: FootFrame) -> FootMetrics:
    """Compute derived metrics using the canonical Y axis as length."""
    if frame.unit != "mm":
        raise ValueError("Foot frame unit must be millimetres.")

    # print("[pipeline]  - metrics computation started")
    heel_y = landmarks.heel_tip.xy[1]
    toe1_y = landmarks.toe1_tip.xy[1]
    toe2_y = landmarks.toe2_tip.xy[1]
    toe3_y = landmarks.toe3_tip.xy[1]
    mtp1_y = landmarks.mtp1_head.xy[1]

    len1 = toe1_y - heel_y
    len2 = toe2_y - heel_y
    len3 = toe3_y - heel_y

    delta21 = len2 - len1
    delta31 = len3 - len1
    truncated_length = mtp1_y - heel_y if math.isfinite(mtp1_y) else float("nan")

    quality = {
        "toe_confidence": min(
            landmarks.toe1_tip.confidence,
            landmarks.toe2_tip.confidence,
            landmarks.toe3_tip.confidence,
        ),
        "instep_available": 1.0 if landmarks.instep_height_mm > 0 else 0.0,
        "mtp_available": 1.0 if math.isfinite(mtp1_y) else 0.0,
    }

    metrics = FootMetrics(
        delta21=float(delta21),
        delta31=float(delta31),
        instep_height=float(landmarks.instep_height_mm),
        truncated_length=float(truncated_length),
        heel_tip_xy=landmarks.heel_tip.xy,
        quality=quality,
    )
    # print("[pipeline]  - metrics computation completed")
    return metrics
