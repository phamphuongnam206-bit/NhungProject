# 128 KiB Flash acceptance, physical gate

OpenOCD reported size register `0x80` and could read `0x08010000` and `0x0801FFF0`. That is **read evidence only**. The current 128 KiB image has a 74,924-byte load span, so its bytes extend above 64 KiB. Do not upload it before this gate.

## 1. Backup without Flash writes

Disconnect the motor battery, keep PB0/STBY LOW, use a stable logic supply and ST-Link. Make a new empty directory outside the repository. Run `tools/backup_flash_128k.ps1 -OpenOcd <full path to openocd.exe> -OutputDirectory <existing empty directory>` manually. The script reads 131,072 bytes twice, checks both lengths and SHA-256 equality, and does not issue an erase/program command. Copy both files and the hash to separate storage. If either read fails or differs, stop; do not attempt the write test.

Compare the two 64 KiB halves of the backup. An identical upper half is a possible address alias, not proof by itself. Check the firmware's actual occupied image, option bytes/read protection and any application data before choosing a test page. The reserved final 1 KiB (`0x0801FC00..0x0801FFFF`) is outside the current linker image, but may contain user data. **Do not assume it is free from the linker setting alone.**

## 2. Controlled destructive test, manual only

After reviewing the full backup and proving a page has no needed data, record the page address, its original 1,024 bytes and their SHA-256. If no page can be proved safe, stop and retain the 64 KiB fallback. With motor power disconnected, manually erase only the selected upper page, program a nontrivial pattern, read it back, and compare every byte. Read the corresponding lower address before and after; any change is an alias failure. Do not mass erase.

Power the board fully off, then back on; read the upper test page again. PASS only if the pattern is identical, the lower page is unchanged, and no read/protection error occurred. Restore the original page bytes from the backup, read back and compare SHA-256. Then verify the original firmware region and any application data against the pretest backup, and check that the old firmware still boots with motor power disconnected. Any mismatch, unexpected reset, alias or restoration problem is FAIL. Do not upload the 128 KiB image in that case.

The write/erase/power-cycle steps deliberately have no automatic command in this repository. They require the operator to choose the page after inspecting the specific board and backup. Record OpenOCD log, page address, both hashes, power-cycle result and restored hash before declaring Flash `VERIFIED`.
