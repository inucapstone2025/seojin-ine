import math
import sys
from pathlib import Path
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from src.analysis.geometry_utils import compute_foot_axes, align_points_to_axes
from src.analysis.toe_detection import ToeDetectionParams, detect_toe_tips
from src.analysis.toe_measurement import compute_toe_measurements


def synthetic_foot_point_cloud(num_points: int = 2000) -> np.ndarray:
    rng = np.random.default_rng(42)

    heel = np.zeros((400, 3))
    heel[:, 0] = rng.uniform(-0.01, 0.01, size=heel.shape[0])
    heel[:, 1] = rng.uniform(-0.02, 0.02, size=heel.shape[0])
    heel[:, 2] = rng.uniform(-0.005, 0.005, size=heel.shape[0])

    body = []
    for x in np.linspace(0.0, 0.22, 50):
        width = 0.03 * (1 - math.cos(math.pi * x / 0.22))
        section = np.stack([
            np.full(40, x) + rng.normal(0.0, 0.001, 40),
            rng.uniform(-width / 2, width / 2, 40),
            rng.normal(0.0, 0.002, 40),
        ], axis=1)
        body.append(section)
    body = np.concatenate(body, axis=0)

    big_toe = np.stack([
        rng.normal(0.245, 0.002, 180),
        rng.normal(-0.018, 0.0015, 180),
        rng.normal(0.01, 0.001, 180),
    ], axis=1)

    second_toe = np.stack([
        rng.normal(0.242, 0.002, 160),
        rng.normal(-0.005, 0.0015, 160),
        rng.normal(0.012, 0.001, 160),
    ], axis=1)

    cloud = np.concatenate([heel, body, big_toe, second_toe], axis=0)
    return cloud[:num_points]


class ToeMeasurementTest(unittest.TestCase):
    def setUp(self) -> None:
        self.points = synthetic_foot_point_cloud()

    def test_toe_detection_and_measurement(self) -> None:
        axes = compute_foot_axes(self.points)
        params = ToeDetectionParams(
            forefoot_slice_depth=0.03,
            dbscan_eps=0.008,
            dbscan_min_samples=8,
            min_cluster_size=60,
        )
        detection = detect_toe_tips(self.points, axes, params)
        self.assertIsNotNone(detection, "Toe detection should find two clusters")

        measurements = compute_toe_measurements(axes, detection)
        self.assertGreater(measurements.heel_to_big_toe_mm, 200.0)
        self.assertGreater(measurements.heel_to_second_toe_mm, 200.0)
        self.assertLess(
            abs(measurements.second_minus_big_mm),
            15.0,
            "The toes synthetic difference should be within 15mm",
        )

    def test_aligned_points_origin_at_heel(self) -> None:
        axes = compute_foot_axes(self.points)
        aligned = align_points_to_axes(self.points, axes)
        self.assertTrue(np.allclose(aligned.min(axis=0)[0], 0.0, atol=5e-3))


if __name__ == "__main__":
    unittest.main()
