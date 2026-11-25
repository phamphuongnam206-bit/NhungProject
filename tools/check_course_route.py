"""Run the real C11 planner on the nominal 17-block course; no hardware IO."""
import argparse
import json
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--core", type=Path, required=True)
parser.add_argument("--zig", type=Path, required=True)
args = parser.parse_args()
course = json.loads((ROOT / "tests/fixtures/simulation_map.json").read_text(encoding="utf-8"))
rects = [course["forbidden_rect_cm"]] + [o["rect_cm"] for o in course["obstacles"]]
assert len(rects) == 18
assert course["grid_cm"] == 5
assert all(all(n % 5 == 0 for n in rect) for rect in rects)
rows = ",\n".join("  {%d,%d,%d,%d}" % tuple(rect) for rect in rects)
source = r'''
#include "planner_replan.h"
#include <assert.h>
#include <stdio.h>

static const unsigned short rects[][4] = {
REPLACE_RECTS
};
static MapGrid grid;
static PlannerWorkspace workspace;
static MapCellPosition route[MAP_GRID_CELL_COUNT];

int main(void) {
  uint16_t count = 0, cost = 0;
  MapCellPosition outside;
  assert(MapGrid_WorldToCell(30.0f, -1.0f, &outside) == MAP_GRID_OUT_OF_BOUNDS);
  assert(MapGrid_InitBase(&grid) == MAP_GRID_OK);
  for (unsigned i = 0; i < 18; ++i) {
    const unsigned short *r = rects[i];
    assert(MapGrid_SetRectangle(&grid, r[1]/5, r[0]/5,
        r[3]/5, r[2]/5, i == 0 ? MAP_CELL_FORBIDDEN : MAP_CELL_OBSTACLE)
        == MAP_GRID_OK);
  }
  const MapCellPosition near_start = {2, 6};
  const MapCellPosition goal = {2, 32};
  PlannerPolicy topology = {{4,2}, 0, {0.0f,0.0f}};
  PlannerStatus status = Planner_ReplanWithPolicy(&grid, 0, near_start, goal,
      &topology, &workspace, route, MAP_GRID_CELL_COUNT, &count, &cost);
  if (status != PLANNER_OK) {
    fprintf(stderr, "FAIL C11 A* topology status=%d\n", (int)status);
    return 1;
  }
  assert(count >= 2);
  for (unsigned i = 0; i < count; ++i) {
    MapCellPosition p = route[i];
    assert(grid.cells[p.row][p.col] == MAP_CELL_FREE);
    assert(p.row != 0); /* no A* path outside the positive-Y grid */
  }
  printf("PASS C11 A* nominal topology: %u waypoints, cost %u\n", count, cost);
  PlannerPolicy safety = {{4,2}, 1, {6.8f/5.0f,5.8f/5.0f}};
  status = Planner_ReplanWithPolicy(&grid, 1, near_start, goal,
      &safety, &workspace, route, MAP_GRID_CELL_COUNT, &count, &cost);
  printf("Near-line plan status=%d; START_ENTRY needs separate proof\n", (int)status);
  const MapCellPosition interior_start = {10, 8}; /* 42.5,52.5 cm test fixture */
  status = Planner_ReplanWithPolicy(&grid, 1, interior_start, goal,
      &safety, &workspace, route, MAP_GRID_CELL_COUNT, &count, &cost);
  if (status != PLANNER_OK) {
    fprintf(stderr, "FAIL interior full-footprint route status=%d\n", (int)status);
    return 2;
  }
  assert(count >= 2);
  assert(route[count-1].row == goal.row && route[count-1].col == goal.col);
  assert(route[count-2].row > goal.row); /* final approach heads toward -Y */
  for (unsigned i=0;i<count;++i) {
    MapCellPosition p = route[i];
    assert(grid.cells[p.row][p.col] == MAP_CELL_FREE);
  }
  printf("PASS interior C11 route with footprint: %u waypoints, cost %u\n",
      count, cost);
  return 0;
}
'''.replace("REPLACE_RECTS", rows)

with tempfile.TemporaryDirectory() as folder:
    work = Path(folder)
    runner = work / "course_route.c"
    runner.write_text(source, encoding="utf-8")
    exe = work / "course_route.exe"
    command = [str(args.zig), "cc", "-std=c11", "-I" + str(args.core / "Core/Inc"),
               *(str(args.core / "Core/Src" / name) for name in
                 ("map_grid.c", "astar.c", "path_smooth.c", "planner_replan.c")),
               str(runner), "-lm", "-o", str(exe)]
    subprocess.run(command, check=True)
    raise SystemExit(subprocess.run([str(exe)], check=False).returncode)
