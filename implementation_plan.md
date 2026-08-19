# Implementation Plan - Advanced Log Engine, RF Streaming & Telemetry (v100 - v121)

This document presents the completed technical implementation plan for the **Dual-Console Live Log Engine**, **ESP-NOW RF T-Pipe Streaming**, **3-Level Filter System**, **1000-Line Browser Memory Buffer**, **1-Click Floppy Disk TXT Export**, **Automatic VPD AUTO Flash Persistence**, **HTTP Socket Leak Elimination**, **Remote Linked Device Reboot Notification Modal**, **CDN Cache Bypassing**, **Home Assistant MQTT Firmware Update Auto-Discovery**, **GPIO 0 Hardware Factory Reset Button**, and **Web UI Optional Password Protection with Public Telemetry**.

---

## Completed Milestones (Builds v100 - v121)

### 1. ESP-NOW RF T-Pipe Log Streaming (Builds v100 - v101)
- Implemented `EspNowLogMessage` (type 3, 180-byte payload) for live RF log transmission from Master to Slave.
- Enables remote diagnostic monitoring of Master output from Slave's Web UI even during web server load stalls.

### 2. Role-Based Diagonal Console Color Matching (Builds v102 - v103)
- **Master UI:** Local Console = BLUE (`#38bdf8`), Remote Console (Slave) = RED (`#f87171`).
- **Slave UI:** Local Console = RED (`#f87171`), Remote Console (Master) = BLUE (`#38bdf8`).

### 3. 3-Level Logging System & 1000-Line Memory Buffer (Builds v104 - v106 & v112)
- Implemented `addAppLogEx(uint8_t level, const char* format, ...)`.
  - **Level 1 (`STAT` / `ALARM`):** Status heartbeats, buzzer test chimes, low-humidity alarms, bypass alerts, settings saves (`[Config]`), pairing events (`[Pairing]`), and OTA updates (`[OTA]`). Always displayed!
  - **Level 2 (`WARN`):** Warning chimes, sensor resets, link loss events.
  - **Level 3 (`DBG `):** Talkative debug output (BME280/SHT3x/TSL2561 readings, VPD AUTO matrix lookups, Servo angle ramping, ESP-NOW pings, MQTT publishes).
- Expanded browser RAM log history array from 300 to **1000 lines** per console with zero ESP32 memory impact.

### 4. Independent Per-Console Filter Controls (Build v105)
- Embedded independent `( ) L1  ( ) L2  (•) L3` filter radios in Local and Remote console headers.
- Client-side filtering operates dynamically on cached lines in browser RAM without page reloads or missing history.

### 5. Automatic VPD AUTO Flash Persistence on Midnight Rollover (Build v107)
- Automatically saves updated `sysConfig.vpd_auto_day` and start timestamp to `/config.json` via LittleFS when crossing 00:00 midnight or 24h uptime boundary.
- Day progression survives reboots and firmware updates without resetting.

### 6. Struct Zeroing & Mode=32 Cleanup (Builds v108 - v109)
- Enforced `memset(&msg, 0, sizeof(EspNowMessage))` across all ESP-NOW struct initializations, resolving uninitialized stack memory values (`Mode=32`).

### 7. 100% Comprehensive Level 1 System & Settings Logging (Build v111)
- Promoted all configuration saves, pairing actions, reset triggers, and GitHub/OTA update events to Level 1 (`L1`) priority.

### 8. 1-Click Floppy Disk TXT Log Export (`💾`) (Build v112)
- Added a compact Floppy Disk icon button `💾` between title and filter in each console header.
- Clicking `💾` invokes `downloadLogHistory()` to generate a clean `.txt` file from client RAM and prompt the browser's native Save File dialog.

### 9. HTTP Socket Leak Elimination (Build v113)
- Enforced `server.sendHeader("Connection", "close")` on `/api/data` and `/api/history` responses.
- Eliminates LWIP TCP socket starvation (`CLOSE_WAIT` exhaustion) caused by continuous 1-second browser AJAX polling.

### 10. Remote Linked Device Reboot & Dismissible Notification Modal (v114 - v116)
- Added `Reboot linked Device` button in Settings UI with an SVG Link Established icon (`🔗`), visible only when an ESP-NOW peer MAC is active.
- Shortened triggering device's response to an instant 1-second redirect back to `/settings` without any blocking splash screen.
- Added a clean, dismissible modal dialog (`#remote-reboot-modal`) on the receiving device's Web UI.

### 11. Instant GitHub CDN Cache Bypassing & Conservative 60-Min Polling (v117 - v119)
- Appended `?nocache=` + `String(millis())` to `raw.githubusercontent.com` fetch requests, bypassing Fastly/GitHub 5-minute CDN caching.
- Adjusted automatic background update check schedule to **60 minutes** (3,600,000 ms = 1 Hour) post-boot, while retaining instant live check when opening `/firmware`.

### 12. Home Assistant MQTT Firmware Update Auto-Discovery (v119)
- Registered HA Auto-Discovery entities for `update_available` (binary update sensor) and `fw_version` (sensor) in `registerHomeAssistantDevices()`.
- Telemetry payload publishes `update_available` (bool), `fw_version` (string), and `online_version` (int) continuously via MQTT.

### 13. Hardware Factory Reset Button (GPIO 0) & Captive Portal Modal UX (v120)
- Configured **GPIO 0** (`INPUT_PULLUP`) for 3-second press-and-hold hardware factory reset.
- Plays descending alert melody, wipes `/config.json`, resets `sysConfig` to defaults with `dry_strategy = 0` (60/60 Mode standard for safe blind operation without Wi-Fi), and restarts into Captive Portal mode.
- Added an amber, persistent Web UI notification modal (`⚙️ Werkseinstellungen geladen`) guiding users to connect to `iDRY26-Setup` at `http://192.168.4.1`.
- Automatically executes `window.location.reload()` in the browser when telemetry signals return after re-configuration.

### 14. Web UI Optional Password Protection with Public Telemetry (v121)
- Added `web_password` (up to 32 chars) to `Config` struct, saved in `/config.json` with CRC32 protection.
- **Optional Activation:**
  - Empty field = Web UI protection **DISABLED** (open access).
  - Masked input `"passwort eintragen"` in Settings under WLAN/Security section. When filled, protection is **ENABLED**.
- **"Kostenlose" Public Telemetry (No Login Required):**
  - Dry Strategy Header & Active Mode Indicator (60/60, VPD, VPD AUTO).
  - Poti cards (Poti A, B, C).
  - Rotor position opening (%).
  - Sensor cards (BME280/SHT3x Temp, Hum, Dewpoint, Barometer, Light).
- **Protected Areas (Login Card Required):**
  - Terminal Consoles (Local Console & Remote Console).
  - Settings Page (`/settings`) & Firmware OTA Page (`/firmware`).
  - Mode switching & POST API endpoints.
  - Unauthenticated `/api/data` requests automatically omit system log arrays.
- **Session Retention:** Logged-in session stored in browser `sessionStorage` for seamless, persistent navigation without re-typing.

---

## Verification & Build Status

- **Firmware Version:** `v121`
- **PlatformIO Build:** `SUCCESS` (Code 0)
- **Synchronized Bundle Files:** `firmware.bin` (v121), `bootloader.bin`, `partitions.bin`, `version.txt` (v121) in `FIRMWARE/`.
