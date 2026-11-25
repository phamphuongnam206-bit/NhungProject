"""Nominal straight gate sweeps; these do not authorize physical motion."""
import json
import math
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
COURSE = json.loads((ROOT / "tests/fixtures/simulation_map.json")
                    .read_text(encoding="utf-8"))
SOURCE = (ROOT / "src/hybrid_2laps.cpp").read_text(encoding="utf-8")


def firmware_float(name):
    match = re.search(rf"\b{name}\s*=\s*([0-9.]+)f", SOURCE)
    if match is None:
        raise ValueError(f"missing firmware constant {name}")
    return float(match.group(1))


HALF_LENGTH_CM = firmware_float("HALF_LENGTH")
HALF_WIDTH_CM = firmware_float("HALF_WIDTH")
OBSTACLE_MARGIN_CM = firmware_float("OBSTACLE_MARGIN_CM")


def straight_gate_clear(x_cm, y0_cm, y1_cm, gate, blocks):
    """Exact swept AABB for a body held parallel to Y on a straight segment."""
    if not all(math.isfinite(v) for v in (x_cm, y0_cm, y1_cm)):
        return False
    xmin, xmax, line_y = gate
    left, right = x_cm - HALF_WIDTH_CM, x_cm + HALF_WIDTH_CM
    if left <= xmin or right >= xmax:
        return False
    bottom = min(y0_cm, y1_cm) - HALF_LENGTH_CM
    top = max(y0_cm, y1_cm) + HALF_LENGTH_CM
    if y1_cm > y0_cm:  # START: whole body behind, then past the line
        if y0_cm + HALF_LENGTH_CM > line_y or y1_cm - HALF_LENGTH_CM <= line_y:
            return False
    elif y1_cm < y0_cm:  # GOAL: whole body inside, then beyond the line
        if y0_cm - HALF_LENGTH_CM < line_y or y1_cm + HALF_LENGTH_CM >= line_y:
            return False
    else:
        return False
    for bx, by, width, height in blocks:
        if (right >= bx - OBSTACLE_MARGIN_CM and
                left <= bx + width + OBSTACLE_MARGIN_CM and
                top >= by - OBSTACLE_MARGIN_CM and
                bottom <= by + height + OBSTACLE_MARGIN_CM):
            return False
    return True


class GateGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.static_blocks = [COURSE["forbidden_rect_cm"]] + [
            obstacle["rect_cm"] for obstacle in COURSE["obstacles"]]

    def test_nominal_start_and_goal_corridors(self):
        self.assertTrue(straight_gate_clear(42.5, -8, 52.5,
                                           COURSE["start_gate_cm"], self.static_blocks))
        self.assertTrue(straight_gate_clear(162.5, 12.5, -8,
                                           COURSE["goal_gate_cm"], self.static_blocks))
        self.assertAlmostEqual((15 - 2 * HALF_WIDTH_CM) / 2 -
                               OBSTACLE_MARGIN_CM, 1.0)

    def test_shift_or_random_obstacle_closes_narrow_gate(self):
        self.assertFalse(straight_gate_clear(43.6, -8, 52.5,
                                            COURSE["start_gate_cm"], self.static_blocks))
        self.assertFalse(straight_gate_clear(163.6, 12.5, -8,
                                            COURSE["goal_gate_cm"], self.static_blocks))
        dynamic = self.static_blocks + [[40, 20, 5, 5]]
        self.assertFalse(straight_gate_clear(42.5, -8, 52.5,
                                            COURSE["start_gate_cm"], dynamic))

    def test_full_body_must_cross_line(self):
        self.assertFalse(straight_gate_clear(42.5, -2, 52.5,
                                            COURSE["start_gate_cm"], self.static_blocks))
        self.assertFalse(straight_gate_clear(162.5, 12.5, -2,
                                            COURSE["goal_gate_cm"], self.static_blocks))


if __name__ == "__main__":
    unittest.main()
