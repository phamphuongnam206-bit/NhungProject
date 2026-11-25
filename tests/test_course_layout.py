import json, re, unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / "tests" / "fixtures" / "simulation_map.json").read_text(encoding="utf-8"))
SOURCE = (ROOT / "src" / "hybrid_2laps.cpp").read_text()

class CourseLayoutTest(unittest.TestCase):
    def test_all_fixed_obstacles_and_forbidden_zone(self):
        block = SOURCE.split("constexpr Box MAP_BOXES[] = {", 1)[1].split("};", 1)[0]
        entries = re.findall(r"\{([0-9., ]+)\},\s*//\s*(T\d+|forbidden)", block)
        self.assertEqual(len(entries), 18)
        boxes = {}
        for raw, name in entries:
            x, y, hx, hy = map(float, raw.split(","))
            boxes[name] = [x-hx, y-hy, 2*hx, 2*hy]
        self.assertEqual(set(boxes), {"forbidden"} | {f"T{i}" for i in range(1,18)})
        self.assertEqual(boxes["forbidden"], CONFIG["forbidden_rect_cm"])
        self.assertEqual(boxes["T7"], [30.0, 85.0, 10.0, 10.0])
        for obstacle in CONFIG["obstacles"]:
            self.assertEqual(boxes[obstacle["id"]], obstacle["rect_cm"])
        for rect in boxes.values():
            self.assertTrue(all(value % 5 == 0 for value in rect))
        self.assertIn("MapGrid_SetRectangle(&baseMap", SOURCE)
        self.assertIn("for (const Box &box : MAP_BOXES)", SOURCE)
        self.assertIn("#define HYBRID_PLANNER_INPUTS_VERIFIED 0", SOURCE)

    def test_tape_and_gate_classification(self):
        self.assertEqual(CONFIG["grid_cm"], 5)
        self.assertEqual(CONFIG["start_gate_cm"], [0, 60, 0])
        self.assertEqual(CONFIG["goal_gate_cm"], [120, 180, 0])
        self.assertIsNone(CONFIG["start_cm"])
        self.assertIsNone(CONFIG["goal_cm"])
        self.assertFalse(CONFIG["arena_edges_reflect"])
        self.assertFalse(CONFIG["forbidden_reflects"])
        self.assertIn("scan.first_reflector_index = 1U", SOURCE)
        self.assertIn("scan.arena_edges_reflect = 0U", SOURCE)

if __name__ == "__main__": unittest.main()
