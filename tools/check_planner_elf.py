"""Check real ARM ELF symbols and static memory; never uploads or opens serial."""
import argparse
import os
from pathlib import Path
import re
import subprocess

FLASH_LIMIT = 64512
RAM_LIMIT = 20480
EXTRA_RAM_MARGIN = 4096  # Beyond linker reservations; not measured stack usage.
REQUIRED = {
    "OneRunAdapter_Poll", "OneRunMission_Init", "OneRunMission_Step",
    "Planner_ReplanWithPolicy", "AStar_FindPathWithPolicy", "PathFollower_Compute",
    "NavigationReplan_Evaluate", "SensorMap_Update", "OneRunFirmwareLinkProbe",
    "one_run_link_storage", "PlannerRoute_Sample",
    "PlannerSnapshot_Build", "PlannerRuntime_Begin", "PlannerRuntime_Poll",
    "PlannerRuntime_Invalidate",
}


def check(elf, toolchain_bin, flash_limit=FLASH_LIMIT, ram_limit=RAM_LIMIT,
          require_start_scan=False):
    if flash_limit not in (64512, 130048) or ram_limit != 20480:
        raise RuntimeError("Unexpected board memory budget")
    def tool(name):
        suffix = ".exe" if os.name == "nt" else ""
        return str(Path(toolchain_bin) / ("arm-none-eabi-" + name + suffix))

    # Include vector table, constructor tables, .data load image and alignment
    # gaps. PlatformIO's headline size omits some of these sections.
    sections = subprocess.check_output([tool("objdump"), "-h", str(elf)], text=True).splitlines()
    flash_end, ram_end, static_end = 0x08000000, 0x20000000, 0x20000000
    for index, line in enumerate(sections):
        match = re.match(r"\s*\d+\s+(\S+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s", line)
        if not match:
            continue
        name = match[1]
        size, vma, lma = (int(match[i], 16) for i in (2, 3, 4))
        flags = sections[index + 1] if index + 1 < len(sections) else ""
        if not size:
            continue
        if "LOAD" in flags and 0x08000000 <= lma < 0x09000000:
            flash_end = max(flash_end, lma + size)
        if "ALLOC" in flags and 0x20000000 <= vma < 0x20100000:
            ram_end = max(ram_end, vma + size)
            if name != "._user_heap_stack":
                static_end = max(static_end, vma + size)
    flash, ram = flash_end - 0x08000000, ram_end - 0x20000000
    static_ram = static_end - 0x20000000
    symbols = subprocess.check_output([tool("nm"), "-S", "--defined-only", str(elf)], text=True)
    names = {line.split()[-1] for line in symbols.splitlines() if line.split()}
    missing = REQUIRED - names
    if require_start_scan:
        missing.update({"NavBoot_Estimate", "StartLocalization_Estimate"} - names)
    if missing:
        raise RuntimeError("Planner was not fully linked; missing: " + ", ".join(sorted(missing)))
    if require_start_scan:
        disassembly = subprocess.check_output([tool("objdump"), "-d", str(elf)], text=True)
        for called in ("StartLocalization_Estimate", "PlannerRuntime_Poll"):
            if not re.search(r"\bbl\b[^\n]*<" + called + r">", disassembly):
                raise RuntimeError("Qualification path lacks a live call to " + called)
    storage_line = next(line for line in symbols.splitlines()
                        if line.split()[-1] == "one_run_link_storage")
    storage_size = int(storage_line.split()[1], 16)
    if storage_size < 11544:
        raise RuntimeError("Mission workspace was not retained in static storage")
    if not flash or not ram or flash > flash_limit or ram > ram_limit - EXTRA_RAM_MARGIN:
        raise RuntimeError(f"Memory budget exceeded/invalid: Flash={flash}, allocated RAM={ram}")
    print(f"PASS planner ELF: Flash image span {flash}/{flash_limit}; static RAM {static_ram}; "
          f"RAM including linker reserve {ram}/{ram_limit}; spare {ram_limit - ram}; "
          f"planner storage {storage_size}")
    print("Extra RAM margin >=4096 bytes; runtime stack and hardware NOT verified.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("elf", type=Path)
    parser.add_argument("--toolchain-bin", type=Path, required=True)
    args = parser.parse_args()
    check(args.elf, args.toolchain_bin)
