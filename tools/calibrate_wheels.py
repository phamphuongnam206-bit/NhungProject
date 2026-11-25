"""Estimate encoder counts per centimetre from measured straight runs.

This host-side tool does not configure or move the robot. It deliberately has
no default geometry or encoder scale: every input row must be a real measure.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Run:
    name: str
    distance_cm: float
    ticks_left: int
    ticks_right: int


@dataclass(frozen=True)
class Estimate:
    counts_per_cm_left: float
    counts_per_cm_right: float
    max_deviation_percent_left: float
    max_deviation_percent_right: float
    runs: int


def load_runs(path: Path) -> list[Run]:
    required = ["run", "distance_cm", "ticks_left", "ticks_right"]
    with path.open(newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        if reader.fieldnames != required:
            raise ValueError("CSV requires exactly: run,distance_cm,ticks_left,ticks_right")
        runs = []
        for number, row in enumerate(reader, start=2):
            if None in row:
                raise ValueError(f"row {number}: too many columns")
            try:
                runs.append(Run(
                    name=row["run"].strip(),
                    distance_cm=float(row["distance_cm"]),
                    ticks_left=int(row["ticks_left"]),
                    ticks_right=int(row["ticks_right"]),
                ))
            except (TypeError, ValueError, AttributeError) as exc:
                raise ValueError(f"row {number}: invalid or missing number") from exc
    return runs


def estimate(runs: list[Run]) -> Estimate:
    if not runs:
        raise ValueError("at least one measured run is required")
    names: set[str] = set()
    signs_left: set[bool] = set()
    signs_right: set[bool] = set()
    for run in runs:
        if not run.name or run.name in names:
            raise ValueError("run names must be nonempty and unique")
        names.add(run.name)
        if not math.isfinite(run.distance_cm) or run.distance_cm <= 0:
            raise ValueError(f"{run.name}: distance_cm must be finite and positive")
        if run.ticks_left == 0 or run.ticks_right == 0:
            raise ValueError(f"{run.name}: both wheels must have nonzero ticks")
        signs_left.add(run.ticks_left > 0)
        signs_right.add(run.ticks_right > 0)
    if len(signs_left) != 1 or len(signs_right) != 1:
        raise ValueError("tick direction changed between runs; check wiring and motion")

    distance = sum(run.distance_cm for run in runs)
    if not math.isfinite(distance):
        raise ValueError("total distance is not finite")
    left = sum(abs(run.ticks_left) for run in runs) / distance
    right = sum(abs(run.ticks_right) for run in runs) / distance

    def deviation(side: str, pooled: float) -> float:
        return max(
            abs(abs(getattr(run, side)) / run.distance_cm - pooled) / pooled * 100
            for run in runs
        )

    return Estimate(left, right,
                    deviation("ticks_left", left),
                    deviation("ticks_right", right), len(runs))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="CSV of measured straight runs")
    args = parser.parse_args()
    try:
        result = estimate(load_runs(args.csv))
    except (OSError, ValueError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 2
    print(f"Runs: {result.runs}")
    print(f"Left:  {result.counts_per_cm_left:.3f} counts/cm "
          f"(max run deviation {result.max_deviation_percent_left:.1f}%)")
    print(f"Right: {result.counts_per_cm_right:.3f} counts/cm "
          f"(max run deviation {result.max_deviation_percent_right:.1f}%)")
    print("STATUS: CANDIDATE_ONLY; confirm repeatability and distance on the vehicle")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
