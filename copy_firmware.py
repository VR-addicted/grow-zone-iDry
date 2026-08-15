import os
import shutil
from pathlib import Path
from SCons.Script import Import

Import("env")

GREEN = "\033[1;32m"
CYAN = "\033[1;36m"
YELLOW = "\033[1;33m"
BOLD = "\033[1m"
RESET = "\033[0m"

def copy_firmware_files(source, target, env):
    build_dir = Path(env.subst("$BUILD_DIR"))
    firmware_dir = Path("FIRMWARE")
    firmware_dir.mkdir(exist_ok=True)

    build_num = "64"
    build_num_file = Path(".build_number")
    if build_num_file.exists():
        build_num = build_num_file.read_text().strip()

    print(f"\n{GREEN}========================================================================{RESET}")
    print(f"{GREEN}   🚀 IDRY-26 AUTOMATED FIRMWARE BUNDLE SYNCHRONIZATION (v{build_num}){RESET}")
    print(f"{GREEN}========================================================================{RESET}")

    files_to_copy = ["firmware.bin", "bootloader.bin", "partitions.bin"]

    for file_name in files_to_copy:
        src_path = build_dir / file_name
        if src_path.exists():
            dst_path = firmware_dir / file_name
            shutil.copy(src_path, dst_path)
            print(f" {CYAN}[copy_firmware]{RESET} Copied {GREEN}{file_name}{RESET} -> {YELLOW}{dst_path}{RESET}")
        else:
            print(f" {CYAN}[copy_firmware]{RESET} Note: {file_name} not found in build directory.")

    # Write version.txt into FIRMWARE
    version_txt = firmware_dir / "version.txt"
    version_txt.write_text(build_num + "\n")
    print(f" {CYAN}[copy_firmware]{RESET} Written {GREEN}version.txt{RESET} -> {YELLOW}{version_txt}{RESET} ({GREEN}v{build_num}{RESET})")

    print(f"{GREEN}========================================================================{RESET}")
    print(f"{GREEN}✔ Firmware bundle (v{build_num}) successfully synchronized into FIRMWARE/{RESET}\n")

# Register post-action after binary build finishes
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", copy_firmware_files)
