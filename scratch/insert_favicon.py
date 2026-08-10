with open(r'C:\Users\DOM\.gemini\antigravity\brain\05cca191-6fa2-4945-8499-3fc4c6ed21a5\scratch\favicon_cpp.txt', 'r') as f:
    favicon_code = f.read()

with open(r'D:\Developement\Workspace-Anti\ESP32-S3-eINK-IDRY-2026\src\main.cpp', 'r') as f:
    main_code = f.read()

target = "// Forward declarations\nextern const uint8_t favicon_png[4191];"
replacement = favicon_code + "\n\n// Forward declarations"

if target in main_code:
    main_code = main_code.replace(target, replacement)
    with open(r'D:\Developement\Workspace-Anti\ESP32-S3-eINK-IDRY-2026\src\main.cpp', 'w') as f:
        f.write(main_code)
    print("SUCCESS")
else:
    print("TARGET NOT FOUND")
