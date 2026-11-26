"""Verify the motor-capable image links the actual SysTick lease hook."""
import argparse
import os
from pathlib import Path
import re
import subprocess


def check(elf: Path, toolchain_bin: Path) -> None:
    suffix = '.exe' if os.name == 'nt' else ''
    prefix = str(toolchain_bin / 'arm-none-eabi-')
    symbols = subprocess.check_output([prefix + 'nm' + suffix, '-C', str(elf)], text=True)
    if not re.search(r'^\S+ T SysTick_Handler$', symbols, re.M):
        raise RuntimeError('No strong SysTick_Handler in motor-capable image')
    if not re.search(r'^\S+ T osSystickHandler$', symbols, re.M):
        raise RuntimeError('Motor lease hook is absent or weak')
    if not re.search(r'^\S+ T armMotorLease\(unsigned char\)$', symbols, re.M):
        raise RuntimeError('Motor lease arming path is absent')
    assembly = subprocess.check_output([prefix + 'objdump' + suffix, '-d', str(elf)], text=True)
    handler = re.search(r'^\S+ <SysTick_Handler>:(.*?)(?=^\S+ <|\Z)',
                        assembly, re.M | re.S)
    if not handler or not re.search(r'\bbl\b[^\n]*<osSystickHandler>', handler[1]):
        raise RuntimeError('SysTick does not call the motor lease hook')
    print('PASS motor lease ELF: strong SysTick -> osSystickHandler; arming path linked')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('elf', type=Path)
    parser.add_argument('--toolchain-bin', type=Path, required=True)
    args = parser.parse_args()
    check(args.elf, args.toolchain_bin)
