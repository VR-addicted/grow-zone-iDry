import os
from SCons.Script import Import

Import("env")

GREEN = "\033[1;32m"
CYAN = "\033[1;36m"
RESET = "\033[0m"

build_file = ".build_number"
build_number = 61

if os.path.exists(build_file):
    try:
        with open(build_file, "r") as f:
            build_number = int(f.read().strip()) + 1
    except Exception:
        build_number = 64
elif os.path.exists("FIRMWARE/version.txt"):
    try:
        with open("FIRMWARE/version.txt", "r") as f:
            build_number = int(f.read().strip()) + 1
    except Exception:
        build_number = 64
else:
    build_number = 64

# Save new build number to .build_number
with open(build_file, "w") as f:
    f.write(str(build_number))

# Save new build number to FIRMWARE/version.txt
os.makedirs("FIRMWARE", exist_ok=True)
with open("FIRMWARE/version.txt", "w") as f:
    f.write(str(build_number))

print(f"{CYAN}[Auto-Version]{RESET} {GREEN}★ Incrementing build number -> v{build_number}{RESET}")
env.Append(CPPDEFINES=[("BUILD_NUMBER", build_number)])
