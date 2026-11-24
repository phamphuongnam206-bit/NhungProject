"""Host tests for measured wheel-scale calculations."""

import tempfile
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from calibrate_wheels import Run, estimate, load_runs  # noqa: E402


class WheelCalibrationTests(unittest.TestCase):
    def test_weighted_distance_and_encoder_signs(self):
        result = estimate([
            Run("a", 20, -400, 420),
            Run("b", 30, -600, 630),
            Run("c", 50, -1000, 1050),
        ])
        self.assertEqual(result.counts_per_cm_left, 20)
        self.assertEqual(result.counts_per_cm_right, 21)
        self.assertEqual(result.max_deviation_percent_left, 0)
        self.assertEqual(result.max_deviation_percent_right, 0)

    def test_unequal_distances_use_total_counts_over_total_distance(self):
        result = estimate([
            Run("a", 10, -100, 110),
            Run("b", 90, -1800, 1800),
        ])
        self.assertEqual(result.counts_per_cm_left, 19)
        self.assertEqual(result.counts_per_cm_right, 19.1)
        self.assertGreater(result.max_deviation_percent_left, 40)

    def test_rejects_invalid_measurements_and_changed_direction(self):
        bad_sets = [
            [],
            [Run("a", 0, 1, 1)],
            [Run("a", float("nan"), 1, 1)],
            [Run("a", 10, 0, 1)],
            [Run("a", 10, 1, 1), Run("b", 10, -1, 1)],
            [Run("a", 10, 1, 1), Run("a", 10, 1, 1)],
        ]
        for runs in bad_sets:
            with self.subTest(runs=runs), self.assertRaises(ValueError):
                estimate(runs)

    def test_csv_input_rejects_missing_and_noninteger_ticks(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "measure.csv"
            path.write_text("run,distance_cm,ticks_left,ticks_right\n"
                            "first,50,-1000,1001\n", encoding="utf-8")
            self.assertEqual(load_runs(path), [Run("first", 50, -1000, 1001)])
            path.write_text("run,distance_cm,ticks_left,ticks_right\n"
                            "first,50,-1000,\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                load_runs(path)
            path.write_text("run,distance_cm,ticks_left,ticks_right\n"
                            "first,50,-1000,1001,extra\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                load_runs(path)


if __name__ == "__main__":
    unittest.main()
