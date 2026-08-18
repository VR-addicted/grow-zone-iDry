# Implementation Plan - Advanced Log Engine, RF Streaming & Telemetry (v100 - v118)

This document presents the completed technical implementation plan for the **Dual-Console Live Log Engine**, **ESP-NOW RF T-Pipe Streaming**, **3-Level Filter System**, **1000-Line Browser Memory Buffer**, **1-Click Floppy Disk TXT Export**, **Automatic VPD AUTO Flash Persistence**, **HTTP Socket Leak Elimination**, **Remote Linked Device Reboot Notification Modal**, and **Instant GitHub Update Detection via CDN Cache Bypassing**.

---

## Completed Milestones (Builds v100 - v118)

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
- Added a clean, dismissible modal dialog (`#remote-reboot-modal`) on the receiving device's Web UI:
  `⚡ Remote Reboot ausgelöst! Dieses Gerät wurde aus der Ferne von deinem gekoppelten Partner-Gerät per ESP-NOW neugestartet.`

### 11. Instant GitHub Update Detection via CDN Cache Bypassing & 10s Boot Schedule (v117 - v118)
- Appended `?nocache=` + `String(millis())` to `raw.githubusercontent.com` fetch requests, bypassing Fastly/GitHub 5-minute CDN caching.
- Enforced a 10-second post-connection buffer (`connectedSince + 10000ms`) for the initial update check after boot, followed by regular 10-minute checks or instant checks upon entering `/firmware`.

---

## Verification & Build Status

- **Firmware Version:** `v118`
- **PlatformIO Build:** `SUCCESS` (Code 0)
- **Synchronized Bundle Files:** `firmware.bin` (v118), `bootloader.bin`, `partitions.bin`, `version.txt` (v118) in `FIRMWARE/`.
