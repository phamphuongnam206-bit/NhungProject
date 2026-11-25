"""Attach planner retention and SRAM budget checks to the diagnostic ELF link."""
from pathlib import Path
import sys

Import("env")
# Only project sources carry LTO; framework/libraries keep their normal build.
# Do not silently accept cross-translation-unit type disagreements.
env.Append(LINKFLAGS=["-flto", "-Werror=odr", "-Werror=lto-type-mismatch"])
sys.path.insert(0, str(Path(env.subst("$PROJECT_DIR")) / "tools"))
from check_planner_elf import check


def verify_planner(source, target, env):
    package = env.PioPlatform().get_package_dir("toolchain-gccarmnoneeabi")
    check(Path(str(target[0])), Path(package) / "bin")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf",
                  env.Action(verify_planner, "Checking planner symbols and memory budget"))
