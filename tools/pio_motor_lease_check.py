"""PlatformIO post-link gate for the legacy motor-capable environment."""
from pathlib import Path
import sys

Import('env')
sys.path.insert(0, str(Path(env.subst('$PROJECT_DIR')) / 'tools'))
from check_motor_lease_elf import check


def verify_motor_lease(source, target, env):
    package = env.PioPlatform().get_package_dir('toolchain-gccarmnoneeabi')
    check(Path(str(target[0])), Path(package) / 'bin')


env.AddPostAction('$BUILD_DIR/${PROGNAME}.elf',
                  env.Action(verify_motor_lease, 'Checking motor lease SysTick link'))
