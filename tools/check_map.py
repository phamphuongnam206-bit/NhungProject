"""Fail when the source-of-truth map JSON and compiled C geometry drift apart."""
import json
import re
from pathlib import Path

root = Path(__file__).resolve().parents[1]
model = json.loads((root / "data/course_map.json").read_text(encoding="utf-8"))
source = (root / "src/static_map.c").read_text(encoding="utf-8")
boxes_text = source.split("const MapRect robot_obstacles[18] = {")[1].split("};")[0]
boxes = [[int(x) for x in row.split(",")] for row in re.findall(r"\{([^{}]+)\}", boxes_text)]
assert boxes == [model["forbidden"], *model["blocks"]]
route_text = source.split("const RobotPoint robot_route[] = {")[1].split("};")[0]
route = [[int(x) for x in row.split(",")] for row in re.findall(r"\{([^{}]+)\}", route_text)]
assert route == model["route"]
assert len(model["blocks"]) == 17
print("map JSON and compiled geometry PASS")
