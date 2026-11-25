"""Build and run pinned C11 core algorithm unit tests on the host only."""

import argparse
import json
from pathlib import Path
import subprocess
import tempfile

TESTS = (
    "map_grid", "astar", "path_smooth", "planner_replan",
    "path_follower", "wheel_targets", "navigation_safety",
    "navigation_replan", "navigation_controller", "sensor_map_update",
    "one_run_mission", "nav_bootstrap_qualification",
)
PINNED_CORE_COMMIT = "2454ca8be28c8fedff0204a62e339cdd162d5427"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--core", type=Path, required=True)
    parser.add_argument("--zig", type=Path, required=True)
    args = parser.parse_args()
    core = args.core.resolve()
    revision = subprocess.check_output(
        ["git", "-C", str(core), "rev-parse", "HEAD"], text=True
    ).strip()
    if revision != PINNED_CORE_COMMIT:
        parser.error(f"core HEAD {revision} differs from pinned {PINNED_CORE_COMMIT}")
    if subprocess.check_output(
        ["git", "-C", str(core), "status", "--porcelain"], text=True
    ).strip():
        parser.error("core working tree is dirty; test the pinned source unchanged")
    sources = json.loads((core / "library.json").read_text(encoding="utf-8"))[
        "build"]["srcFilter"]
    source_names = [item[2:-1] for item in sources if item.startswith("+<")]
    include = "-I" + str(core / "Core/Inc")
    with tempfile.TemporaryDirectory(prefix="nhung_core_algorithms_") as folder:
        work = Path(folder)
        objects = []
        for name in source_names:
            obj = work / (name + ".o")
            subprocess.run([str(args.zig), "cc", "-std=c11", "-Wall",
                            "-Wextra", "-Werror", include, "-c",
                            str(core / "Core/Src" / name), "-o", str(obj)],
                           check=True)
            objects.append(str(obj))
        for name in TESTS:
            test_source = core / "tests" / (name + "_test.c")
            exe = work / (name + ".exe")
            subprocess.run([str(args.zig), "cc", "-std=c11", "-Wall",
                            "-Wextra", "-Werror", include, str(test_source),
                            *objects, "-lm", "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
            print(f"PASS pinned C11 core unit: {name}")


if __name__ == "__main__":
    main()
