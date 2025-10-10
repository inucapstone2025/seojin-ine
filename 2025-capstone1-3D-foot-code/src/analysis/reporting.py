"""Reporting helpers for foot measurements."""

from __future__ import annotations

import csv
from dataclasses import asdict
from pathlib import Path
from typing import Dict, Iterable, Tuple

from .toe_measurement import ToeMeasurements


def measurement_to_row(name: str, meas: ToeMeasurements) -> Dict[str, float]:
    row = asdict(meas)
    row["foot"] = name
    row["second_minus_big_mm"] = meas.second_minus_big_mm
    return row


def save_measurements_csv(
    path: Path,
    entries: Iterable[Tuple[str, ToeMeasurements]],
) -> None:
    rows = [measurement_to_row(name, meas) for name, meas in entries]
    if not rows:
        return

    fieldnames = list(rows[0].keys())
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def format_measurement_summary(name: str, meas: ToeMeasurements) -> str:
    return (
        f"{name}: heel→big {meas.heel_to_big_toe_mm:.1f} mm, "
        f"heel→second {meas.heel_to_second_toe_mm:.1f} mm, "
        f"foot length {meas.foot_length_mm:.1f} mm"
    )
