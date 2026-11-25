"""Offline inspection of a 128 KiB STM32 readback; never accesses hardware."""

import argparse
import hashlib
import json
from pathlib import Path

FLASH_BYTES = 128 * 1024
HALF_BYTES = 64 * 1024
PAGE_BYTES = 1024
FLASH_BASE = 0x08000000


def inspect_backup(data: bytes) -> dict:
    if len(data) != FLASH_BYTES:
        raise ValueError(f"Expected {FLASH_BYTES} bytes, got {len(data)}")
    low, high = data[:HALF_BYTES], data[HALF_BYTES:]
    pages = []
    for index in range(HALF_BYTES // PAGE_BYTES):
        page = high[index * PAGE_BYTES:(index + 1) * PAGE_BYTES]
        pages.append({
            "address": f"0x{FLASH_BASE + HALF_BYTES + index * PAGE_BYTES:08X}",
            "non_ff_bytes": PAGE_BYTES - page.count(0xFF),
            "sha256": hashlib.sha256(page).hexdigest(),
        })
    return {
        "size_bytes": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
        "lower_upper_identical": low == high,
        "upper_pages": pages,
        "interpretation": "Readback only. An all-FF page is not proof it is unused; "
                          "different halves do not prove write retention.",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("backup", type=Path, help="131072-byte readback file")
    args = parser.parse_args()
    report = inspect_backup(args.backup.read_bytes())
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
