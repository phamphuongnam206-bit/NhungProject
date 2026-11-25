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
sources = json.loads((a.core / 'library.json').read_text())['build']['srcFilter']
with tempfile.TemporaryDirectory(prefix='nhung_runtime_') as folder:
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
