"""Host test with real C core, no hardware; provide matching core checkout."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
p = argparse.ArgumentParser()
p.add_argument('--core', type=Path, required=True)
p.add_argument('--zig', required=True)
a = p.parse_args()
root = Path(__file__).resolve().parents[1]
course = json.loads((root / 'tests/fixtures/simulation_map.json').read_text(encoding='utf-8'))
course_rects = [course['forbidden_rect_cm']] + [o['rect_cm'] for o in course['obstacles']]
assert len(course_rects) == 18 and course['grid_cm'] == 5
assert all(all(isinstance(n, int) and n >= 0 and n % 5 == 0 for n in r)
           for r in course_rects)
assert course['robot_length_cm'] == 13 and course['robot_width_cm'] == 10
sources = json.loads((a.core / 'library.json').read_text())['build']['srcFilter']
with tempfile.TemporaryDirectory(prefix='nhung_runtime_') as folder:
    fixture = Path(folder) / 'course_fixture.h'
    fixture.write_text('static const unsigned short course_rects[][4] = {\n' +
                       ',\n'.join('  {%d,%d,%d,%d}' % tuple(r) for r in course_rects) +
                       '\n};\n#define COURSE_RECT_COUNT 18U\n'
                       '#define COURSE_HALF_LENGTH_CM 6.5f\n'
                       '#define COURSE_HALF_WIDTH_CM 5.0f\n', encoding='utf-8')
    objects = []
    for entry in sources:
        if not entry.startswith('+<'):
            continue
        name = entry[2:-1]
        obj = Path(folder) / (name + '.o')
        subprocess.run([a.zig, 'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-I' + str(a.core / 'Core/Inc'), '-c',
                        str(a.core / 'Core/Src' / name), '-o', str(obj)], check=True)
        objects.append(str(obj))
    exe = Path(folder) / 'runtime.exe'
    subprocess.run([a.zig, 'cc', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                    '-DHYBRID_MOTOR_LOCK=1', '-I' + str(root/'include'),
                    '-I' + str(a.core/'Core/Inc'), str(root/'tests/planner_runtime_test.cpp'),
                    str(root/'src/one_run_link_probe.cpp'), str(root/'src/planner_route_view.cpp'),
                    *objects, '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
    course_runtime = Path(folder) / 'course_runtime.exe'
    subprocess.run([a.zig, 'cc', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                    '-DHYBRID_MOTOR_LOCK=1', '-I' + str(root/'include'),
                    '-I' + folder, '-I' + str(a.core/'Core/Inc'),
                    str(root/'tests/planner_course_runtime_test.cpp'),
                    str(root/'src/planner_snapshot.cpp'),
                    str(root/'src/one_run_link_probe.cpp'),
                    str(root/'src/planner_route_view.cpp'), *objects,
                    '-o', str(course_runtime)], check=True)
    subprocess.run([str(course_runtime)], check=True)
    snapshot = Path(folder) / 'snapshot.exe'
    subprocess.run([a.zig, 'cc', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(root/'include'), '-I' + str(a.core/'Core/Inc'),
                    str(root/'tests/planner_snapshot_test.cpp'),
                    str(root/'src/planner_snapshot.cpp'), *objects,
                    '-o', str(snapshot)], check=True)
    subprocess.run([str(snapshot)], check=True)
    four = Path(folder) / 'four_sensor_runtime.exe'
    subprocess.run([a.zig, 'cc', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                    '-DHYBRID_MOTOR_LOCK=1', '-I' + str(root/'include'),
                    '-I' + str(a.core/'Core/Inc'),
                    str(root/'tests/planner_four_sensor_runtime_test.cpp'),
                    str(root/'src/planner_snapshot.cpp'),
                    str(root/'src/one_run_link_probe.cpp'), *objects,
                    '-o', str(four)], check=True)
    subprocess.run([str(four)], check=True)
    start = Path(folder) / 'start_localization.exe'
    subprocess.run([a.zig, 'cc', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(root/'include'), '-I' + str(a.core/'Core/Inc'),
                    str(root/'tests/start_localization_test.cpp'),
                    str(root/'src/start_localization.cpp'), *objects,
                    '-o', str(start)], check=True)
    subprocess.run([str(start)], check=True)
    pose_window = Path(folder) / 'pose_window.exe'
    subprocess.run([a.zig, 'cc', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(root/'include'),
                    str(root/'tests/planner_pose_window_test.cpp'),
                    '-o', str(pose_window)], check=True)
    subprocess.run([str(pose_window)], check=True)
    for name in ('planner_motion_health', 'planner_goal_gate',
                  'planner_start_entry', 'planner_goal_exit',
                  'planner_goal_exit_motion', 'planner_start_entry_motion',
                 'planner_kinematics_gate'):
        test = Path(folder) / (name + '.exe')
        subprocess.run([a.zig, 'cc', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                        '-I' + str(root/'include'),
                        str(root/'tests'/(name + '_test.cpp')),
                        '-o', str(test)], check=True)
        subprocess.run([str(test)], check=True)
