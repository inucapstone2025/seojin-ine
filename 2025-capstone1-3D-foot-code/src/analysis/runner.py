"""Pipeline runner for the left foot analysis demo."""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np

from .common_types import FootFrame, FootLandmarks, FootMetrics, Footprint2D
from .foot_information_extraction import extract_foot_information
from .foot_point_extraction import extract_landmarks
from .metrics import compute_metrics

try:
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover - optional dependency
    plt = None


def _show_footprint(footprint: Footprint2D) -> None:
    if plt is None:
        print("[visualize] matplotlib is not available; skipping footprint plot.")
        return
    fig, ax = plt.subplots()
    pts = footprint.points_xy
    ax.scatter(pts[:, 0], pts[:, 1], s=6, alpha=0.6, label="Footprint")
    ax.scatter(
        [footprint.heel_x],
        [footprint.heel_y],
        c="red",
        s=40,
        label="Heel",
    )
    ax.set_title("Footprint Projection (mm)")
    ax.set_xlabel("+X medial [mm]")
    ax.set_ylabel("+Y length [mm]")
    ax.axis("equal")
    ax.legend(loc="upper left")
    fig.tight_layout()
    plt.show()


def _show_landmarks(footprint: Footprint2D, landmarks: FootLandmarks) -> None:
    if plt is None:
        print("[visualize] matplotlib is not available; skipping landmark plot.")
        return
    fig, ax = plt.subplots()
    pts = footprint.points_xy
    ax.scatter(pts[:, 0], pts[:, 1], s=6, alpha=0.4, label="Footprint")
    band_pts = getattr(landmarks, "toe_band_points", None)
    if band_pts is not None and len(band_pts):
        ax.scatter(
            band_pts[:, 0],
            band_pts[:, 1],
            s=8,
            c="gold",
            alpha=0.8,
            label="Forefoot band",
        )
    points = {
        "Heel": landmarks.heel_tip.xy,
        "Toe1": landmarks.toe1_tip.xy,
        "Toe2": landmarks.toe2_tip.xy,
        "Toe3": landmarks.toe3_tip.xy,
        "Instep": landmarks.instep_point_xy,
    }
    for name, (x, y) in points.items():
        ax.scatter([x], [y], s=50, label=name)
        ax.text(x, y, f" {name}", fontsize=9)
    ax.set_title("Footprint Landmarks (mm)")
    ax.set_xlabel("+X medial [mm]")
    ax.set_ylabel("+Y length [mm]")
    ax.axis("equal")
    ax.legend(loc="upper left")
    fig.tight_layout()
    plt.show()


def _show_metrics(metrics: FootMetrics) -> None:
    if plt is None:
        # print("[visualize] matplotlib is not available; metrics summary:")
        print(metrics)
        return
    fig, ax = plt.subplots()
    ax.axis("off")
    ax.set_title("Computed Metrics (mm)")
    lines = [
        f"delta21: {metrics.delta21:.2f}",
        f"delta31: {metrics.delta31:.2f}",
        f"truncated length: {metrics.truncated_length:.2f}",
        f"instep height: {metrics.instep_height:.2f}",
        f"heel tip: {metrics.heel_tip_xy}",
    ]
    for idx, text in enumerate(lines):
        ax.text(0.05, 0.95 - idx * 0.1, text, transform=ax.transAxes, fontsize=11, va="top")
    fig.tight_layout()
    plt.show()


def process(
    points3d: np.ndarray,
    side: str = "L",
    visualize_steps: bool = False,
) -> Tuple[FootMetrics, FootLandmarks, FootFrame, Footprint2D]:
    """Execute the analysis pipeline on a point cloud."""
    # print("[pipeline] Starting extract_foot_information")
    frame, footprint = extract_foot_information(points3d, side=side)
    # if visualize_steps:
    #     _show_footprint(footprint)
    # print("[pipeline] extract_foot_information completed")

    # print("[pipeline] Starting extract_landmarks")
    landmarks = extract_landmarks(footprint, points3d, frame)
    if visualize_steps:
        _show_landmarks(footprint, landmarks)
    # print("[pipeline] extract_landmarks completed")

    # print("[pipeline] Starting compute_metrics")
    metrics = compute_metrics(landmarks, frame)
    if visualize_steps:
        _show_metrics(metrics)
    # print("[pipeline] compute_metrics completed")

    return metrics, landmarks, frame, footprint


# def _synthetic_left_foot(num_points: int = 1500) -> np.ndarray:
#     """Generate a capsule-like synthetic left foot point cloud in metres."""
#     length = 0.25
#     heel_radius = 0.035
#     fore_radius = 0.045
#     y = np.linspace(0.0, length, num_points // 3)

#     theta = np.linspace(math.pi / 2, 3 * math.pi / 2, num_points // 3)
#     heel_x = heel_radius * np.cos(theta)
#     heel_y = heel_radius * np.sin(theta)

#     width_profile = np.linspace(heel_radius, fore_radius, y.size)
#     mid_x = []
#     mid_y = []
#     for yi, wi in zip(y, width_profile):
#         xs = np.linspace(-wi * 0.6, wi, 10)
#         mid_x.append(xs)
#         mid_y.append(np.full_like(xs, yi))
#     mid_x = np.concatenate(mid_x)
#     mid_y = np.concatenate(mid_y)

#     phi = np.linspace(-math.pi / 2, math.pi / 2, num_points // 3)
#     fore_x = fore_radius * np.cos(phi)
#     fore_y = fore_radius * np.sin(phi) + length

#     x = np.concatenate([heel_x, mid_x, fore_x])
#     y = np.concatenate([heel_y, mid_y, fore_y])

#     x += 0.01 * np.exp(-((y - length) ** 2) / 0.002)

#     rng = np.random.default_rng(42)
#     x += rng.normal(0.0, 0.001, size=x.shape)
#     y += rng.normal(0.0, 0.001, size=y.shape)

#     z = np.zeros_like(x)
#     instep_mask = (y > length * 0.4) & (y < length * 0.65)
#     z[instep_mask] = 0.01 * np.exp(-((x[instep_mask]) ** 2) / 0.0008)
#     z += rng.normal(0.0, 0.0005, size=z.shape)

#     points = np.column_stack([x, y, z])
#     return points


# if __name__ == "__main__":
#     demo_points = _synthetic_left_foot()
#     metrics, landmarks, frame, footprint = process(demo_points, side="L", visualize_steps=True)

#     print("=== Foot Metrics (mm) ===")
#     print(f"len delta toe2-toe1: {metrics.delta21:.2f}")
#     print(f"len delta toe3-toe1: {metrics.delta31:.2f}")
#     print(f"truncated length:    {metrics.truncated_length:.2f}")
#     print(f"instep height:       {metrics.instep_height:.2f}")
#     print("\n=== Key Landmarks (mm) ===")
#     print(f"Heel tip: {metrics.heel_tip_xy}")
#     print(f"MTP1 head: {metrics.mtp1_head_xy}")
#     print(f"Instep height: {metrics.instep_height:.2f} at {metrics.instep_cross_xy}")
