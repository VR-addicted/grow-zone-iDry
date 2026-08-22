# Project Workspace Customization Rules

This workspace contains customized configuration rules for the YD-ESP32-S3, Waveshare 3.52" e-Paper display, ILI9341 TFT display, I2C sensor interfaces, analog potentiometers, LEDC servo motor control, ESP-NOW dual-mesh communication, Home Assistant MQTT auto-discovery, live OTA terminal, dual RAM ring buffers, gapless canvas sparklines, spike detection caps, dynamic moon favicon, and ESP32 header validation.

---

## Core Purpose & System Philosophy (Hermetic Drying Valve)
* **Hermetic Sealing vs. Over-Drying:** In a harvest drying tent or curing box, standard internal fans only circulate air but cannot block external climate exchange. Once target humidity/climate is reached, iDry-26 completely closes the mechanical servo shutter valve—hermetically sealing intake and exhaust vents like a sealed storage container or a bucket with a lid (Eimer mit Deckel). This prevents critical over-drying (Übertrocknung).
* **Master-Slave Hermetic Combo:** Two iDry-26 units (Master on intake, Slave on exhaust) communicate via ESP-NOW to seal the drying tent synchronously on both sides. (Works standalone with a single unit as well).
* **1-Knob Simple Operation:** Controlled primarily via one main potentiometer (Poti A) to set target humidity (48–72%), featuring explicit "Rigoros ZU" ($\le 49\%$) and "Rigoros AUF" ($\ge 71\%$) boundary modes.

---

## Custom Driver & Display Autodetect Rules
* **Hardware Profiles Supported:** Waveshare 3.52" e-Paper (B) 360x240 pixels (Red/Black/White), ILI9341 3.2" TFT Display 320x240 pixels, or Headless Mode (no display connected).
* **Driver Class:** For 3.52" e-Paper, use `GxEPD2_213_Z19c` initialized for 360x240 resolution. All driver parameters are handled automatically in source code without requiring manual file edits.
* **Display Class Wrapper:** Always use `GxEPD2_3C` for three-colour rendering (Black, Red, White).
* **Do NOT use 4-colour driver (`GxEPD2_4C` or `GxEPD2_350c_GDEM035F51`)** because the panel physically lacks yellow/orange pigments and will produce layout corruption.
* **3-State Display Autodetector:**
  1. Pin `EPD_BUSY` (GPIO 8) initialized with `INPUT_PULLUP`. If read `LOW` -> **TFT Mode** (`isTFTMode = true`, transistor load on backlight line).
  2. If read `HIGH`, send a reset pulse on `EPD_RST` (GPIO 14). If `EPD_BUSY` state toggles -> **e-Paper Mode** (`isTFTMode = false`).
  3. If no state change -> **Headless Mode** (`isHeadless = true`). Completely bypass display initializations and rendering calls to conserve power and CPU cycles.

---

## PIN Configuration Constraints
Do not change SPI, I2C, ADC, or actuator pin assignments:
* **SPI Bus (Shared Display Cable):** SCK -> GPIO 12, MOSI -> GPIO 11, MISO -> GPIO 13 (unused), CS -> GPIO 10, DC -> GPIO 9, RST -> GPIO 14, BUSY/LED -> GPIO 8
* **I2C Bus:** SDA -> GPIO 15, SCL -> GPIO 16
* **Potentiometers (ADC):** Poti A (Target Humidity) -> GPIO 4, Poti B (Gain) -> GPIO 5, Poti C (Calibration Offset) -> GPIO 1
* **Buzzer:** GPIO 17 (Passive Buzzer, NPN transistor driven)
* **Servo Motor:** GPIO 18 (PWM controlled via LEDC Channel 2)

---

## RF Brownout Prevention & Network Stability
* **RF Brownout Protection:** Always execute `display.powerOff();` and insert `delay(500);` prior to activating SoftAP / Wi-Fi on e-Paper modules to drop current draw and prevent 3.3V rail voltage dips.
* **Wi-Fi STA Stability:** Call `WiFi.disconnect(true);` and `WiFi.setSleep(false);` during initialization to clear socket state and prevent packet loss or router sleep dropouts.
* **Non-Blocking Network Calls:** Enforce a 500ms timeout on socket connections to prevent freezing the main loop during network disruptions.

---

## High-Performance Web UI & Telemetry Architecture
* **Dual RAM Ring-Buffer Layout:**
  - `history120mBuffer[120]`: 120 samples x 1-min resolution (2 hours history for cards & system status).
  - `history24hBuffer[288]`: 288 samples x 5-min resolution (24 hours history for zoom modal).
  - Total RAM footprint: ~21 KB out of 320 KB (~28.4% total RAM used).
* **Gapless Sparkline Geometry (Nahtlose Balken):** Bar charts calculated continuously from $x_1$ to $x_2$ with zero gap spacing, producing smooth connected curves.
* **Spike Detection (Yellow Top Segment):** Temperature and Humidity sparklines feature a light blue base candle body ($0 \to Min$) plus a vibrant yellow cap ($Min \to Max$), measuring positive delta fluctuations within each bucket.
* **RSSI Multi-Color Gradient:** Canvas linear vertical gradient (Red -> Orange -> Yellow -> Green) for signal strength bars.
* **Double-Width 2h System Status Preview Card:** Displays 120 candles across full-width container with 30-min tick marks.
* **Interactive 24h Zoom Modal & Floating Badge:** Auto-scrolls to rightmost live edge on open. Pointer/Touch handler calculates sample index and segment, showing floating popup badge 15px above candle top.
* **Dynamic Moon Favicon:** 32x32 offscreen HTML5 canvas dynamically renders live shutter opening phase into browser tab icon (0 Byte ESP32 RAM).
* **Dynamic Tab Title & Bookmarks:** Server emits `<title>IDRY-26 Master</title>` / `<title>IDRY-26 Slave</title>` for instant clean bookmarking.
* **Pulsing Red OTA Update Border:** Background check (at boot, every 10 min, and `/firmware`) toggles a 1-second pulsing red border animation with glowing halo around the `Firmware & OTA Update` button on Settings page when an update is available.
* **Compact VPD AUTO & Dropdown Selector:** 3rd Dry Strategy Mode (`VPD AU`). Displays compact 42px high 14-candle strip with continuous glowing pulse animation (`@keyframes vpd-candle-pulse`) on active day. Includes a 14-option dropdown selector (`Tag 1 (0.70 kPa)` .. `Tag 14 (1.10 kPa)`) linked to 2-second hold confirmation modal. Features dual NTP (00:00 midnight sync) and MCU microtime (uptime fallback) 24h day rollover.

---

## Active Connection Watchdogs & Web UI Timeouts
* **Gateway Watchdog:** Perform TCP client connection checks to `WiFi.gatewayIP()` on port 80 every 2 seconds. If a timeout (> 400ms) occurs, call `WiFi.disconnect(true)` immediately and initiate reconnect cycle.
* **Embedded CRC32 Config Integrity Protection & Auto-Self-Healing:** `saveConfiguration()` embeds a 32-bit CRC (`"crc": 0xXXXXXXXX`) inside `/config.json`. On boot, `loadConfiguration()` computes the CRC over payload fields. If a CRC mismatch or JSON parse error occurs (e.g. power outage abort), the corrupted file is automatically purged (`LittleFS.remove("/config.json")`) and the device boots cleanly into Captive Portal Setup Mode.
* **Web UI AJAX Timeout:** Fetch `/api/data` in Web UI using a 1000ms `AbortController` timeout to transition UI immediately into offline status when link drops.
* **WLAN Connection Watchdog Alarm (`wlan_time_trap`):** Configurable slider (0–330s, default 120s). When connection drops, play double beep buzzer sequence and repeat at configured interval.
* **Weekly Reboot Watchdog:** Uptime monitored via `millis()`. When uptime exceeds 1 week (7 days / 604,800,000ms), check NTP clock and reboot at 03:00 AM local time.

---

## ESP-NOW Master/Slave Mesh & Fail-Safe Protection
* **Protocol Versioning (V3):** Increment `localProtocolVersion` (currently V3) whenever `EspNowMessage` struct or command payload changes. `EspNowMessage` includes `uint8_t dry_strategy` to continuously sync strategy (0 = 60/60, 1 = VPD, 2 = VPD AUTO) every 1 second. Web UI alerts user on version mismatch.
* **Fast-Track Channel Pairing:** Master broadcasts pairing beacons on Wi-Fi channel; Slave hops channels 1–13 every 1.2s to establish peer MAC address binding and protocol version verification (`peerInfo.encrypt = false` to guarantee 0% packet loss during Wi-Fi channel hopping). Case-insensitive MAC comparison (`strcasecmp`).
* **Aggressive Reconnection (>20s):** On Slave devices, if no packet is received for >20 seconds, re-initialize ESP-NOW stack (`initEspNow()`) every 15 seconds without MCU reboot.
* **2-Stage Fail-Safe Mode (>60s Connection Loss):**
  - `espnow_failsafe_mode = 0` (Default) or no local sensor: Force rotor position to 50% (Safety Open).
  - `espnow_failsafe_mode = 1` with active local sensor: Calculate rotor position locally using Slave's Poti A and local humidity sensor.

---

## VPD Strategy Engine & Hygro-Limit Mold Protection
* **3-Mode Strategy Selection:** Configurable via Web UI or HTTP POST `/api/settings/dry_strategy?mode=X&limit=Y` (`sysConfig.dry_strategy`: 0 = 60/60 Mode, 1 = VPD Mode, 2 = VPD AUTO Mode; `sysConfig.hygro_limit`: 70, 75, or 80%).
* **VPD AUTO 14-Day Progression & 21x14 Temp Matrix:** 14-day automated Curing schedule backed by a 21x14 scientific temperature matrix ($15\text{ to }35^\circ\text{C}$). Dynamically looks up target VPD for the active room temperature and lands on **0.85 kPa (~62% RH Goldstandard Curing Landing Zone)** on Days 11–14+. Features an interactive **2D Heatmap Canvas Widget** with a cyan/yellow laser crosshair, glowing white dot, floating badge tooltip, and right-hand color scale legend ($0.50 \text{ to } 1.40\text{ kPa}$).
* **Yellow Dotted Baseline Line (`#facc15`):** Both VPD charts (`VPD Innen` and `VPD Außen`) and the 24h Zoom Modal draw a bright yellow dashed target line tracking the active day's target VPD in `VPD AUTO` mode or manual target VPD in `VPD` mode. Hidden in `60/60` mode.
* **Poti A Re-Mapping (VPD Mode):** 0% to 100% knob position maps to **0.60 kPa to 1.40 kPa**, with **1.00 kPa at 50% midpoint knob position**. (In `VPD AUTO` mode, Poti A is overridden by current day target VPD).
* **Hygro-Limit Mold Protection Cap:** Target RH derived from VPD is clamped: $RH_{\text{effective}} = \min(RH_{\text{calculated}}, \text{HygroLimit})$.
* **RAW Telemetry & Dynamic Notice:** Server transmits `raw_calculated_rh` alongside `effective_target_rh`. UI displays `RH calculated soll: XX.X %` with a dedicated red warning line `(limited to XX%)` when raw RH exceeds the Hygro Limit.
* **Slave [remote] Indicator & Adaptive Use-Case UI:** On Slave devices (`espnow_role === 2`), Rotor & Servo card explicitly displays `Rotor Stellung: [remote] X %` in bold soft red (`#f87171`). When no active temperature/humidity sensor is connected, Dry Strategy controls and Hygro-Limit boxes are automatically hidden to keep the UI clean. On Slaves without sensors, interactive strategy buttons are replaced by a non-clickable double-width badge (`REMOTE 60/60`, `REMOTE VPD`, `REMOTE VPD AU`, or `NOTFALL 50% OPEN` / `NOTFALL 60/60` / `NOTFALL VPD`).
* **Unfiltered Realtime Telemetry Benchmark:** `avgEspNowIntervalMs` outputs raw, unfiltered millisecond delta between 1s sync packets (`msg.command == 2`) without low-pass smoothing. Offline threshold triggers after 5.0s (5 missed heartbeats).

---

## Potentiometer Signal Conditioning & Discrete Zones
* **EMA Low-Pass Filter:** Analog inputs filtered using Exponential Moving Average (EMA). Poti B uses heavy low-pass filtering ($\alpha = 0.05$).
* **Poti A Discrete Zones:**
  - $\le 49\%$: Rigorously Closed ($0\%$ opening, displays `"Rigoros ZU"`).
  - $\ge 71\%$: Rigorously Open ($100\%$ opening, displays `"Rigoros AUF"`).
  - $50 - 70\%$: Closed-loop proportional humidity regulation.

---

## Servo Motion Profiling & Powerdown Management
* **Sine Easing:** Smooth acceleration and deceleration using sine curves (`0.5f * (1.0f - cos(t * PI))`).
* **Idle Powerdown:** Shut off PWM signal (`ledcWrite(SERVO_LEDC_CHANNEL, 0)`) after 1 second of inactivity once target angle is reached.
* **Update Rate Limiting:** Closed-loop servo updates throttled to user-configured interval (`sysConfig.servo_update_interval`, 1–30s, default 5s).

---

## Thermodynamic Feuchteschutz & Acoustic Alerts
* **Thermodynamic Bypass (Saug-Sperre):** If outside humidity is higher than inside humidity or $>2\%$ above target, rotor forces fully closed ($0\%$).
* **Acoustic Signalization:** Passive buzzer handles boot melody (C5 -> G6), boundary chimes, drying progress alerts, and connection loss watchdog alarms.

---

## Live ESP-NOW RF Log Streaming, 3-Level Filtering, Protocol V5 & Remote Linked Reboot Rules
* **Protocol Versioning (V5):** `localProtocolVersion` is updated to **V5**.
  - `EspNowLogMessage` (type 3, 180-byte string payload) streams all `addAppLog(...)` calls from Master to Slave over ESP-NOW.
  - `EspNowMessage` command `99` (Remote Reboot Request) allows triggering a clean remote restart of the linked peer device over ESP-NOW.
* **T-Pipe Logging Architecture (`addAppLogEx(level, format, ...)`):**
  - **Level 1 (`STAT` / `ALARM`):** Essential telemetry heartbeats, buzzer test chimes, low-humidity alarms, thermodynamic bypass alerts, settings saves (`[Config]`), pairing events (`[Pairing]`), remote reboots (`[System]`), and OTA updates (`[OTA]`). **Always displayed!**
  - **Level 2 (`WARN`):** Warning chimes, sensor reset events, link loss events.
  - **Level 3 (`DBG `):** Rich, talkative debug output (BME280/SHT3x/TSL2561 readings, VPD AUTO matrix calculations, Servo ramping steps, ESP-NOW pings, MQTT publishes).
* **Independent Client-Side Per-Console Filters & 1-Click Floppy Disk TXT Export:**
  - `Local Terminal Console`: Headers contain independent `( ) L1  ( ) L2  (•) L3` filter radios and a Floppy Disk button `💾` between title and filter.
  - `Remote Peer Terminal Console [ESP-NOW]`: Headers contain independent `( ) L1  ( ) L2  (•) L3` filter radios and a Floppy Disk button `💾` between title and filter.
  - Clicking `💾` invokes `downloadLogHistory()` to generate and prompt a native browser `.txt` file download of the full 1000-line history array accumulated in client RAM (`webLogHistoryLocal` or `webLogHistoryRemote`).
  - History buffer holds up to 1000 lines in browser RAM (0 Bytes ESP32 RAM used).
* **HTTP Socket Teardown Hardening (`Connection: close`):**
  - All REST endpoints (`/api/data`, `/api/history`) enforce `server.sendHeader("Connection", "close")` prior to sending HTTP responses. Ensures sockets are torn down immediately after each 1-second AJAX poll, eliminating LWIP TCP socket starvation.
* **Role-Based Dynamic Console Styling (Diagonal Symmetry):**
  - **Master Terminal Box:** Header `#38bdf8`, Border `1px solid rgba(56, 189, 248, 0.5)`, Background `#090d16`, Monospace Text `#38bdf8`.
  - **Slave Terminal Box:** Header `#f87171`, Border `1px solid rgba(248, 113, 113, 0.5)`, Background `#160909`, Monospace Text `#fca5a5`.
  - Master Screen: Local = Blue Box, Remote (Slave) = Red Box.
  - Slave Screen: Local = Red Box, Remote (Master) = Blue Box.
* **Automatic VPD AUTO Flash Persistence on Midnight Rollover:**
  - `getVpdAutoCurrentDay()` detects `daysPassed > 0` (00:00 midnight sync or 24h uptime boundary), updates `sysConfig.vpd_auto_day`, resets start timestamp, and invokes `saveConfiguration()` to LittleFS Flash immediately. Ensures day progression survives reboots and firmware updates.

---

## Stoßlüftungs-Timer (Intervall-Purge) & 3D Drum Selection Wheels
* **Stoßlüftungs-Timer Engine:**
  - Configurable periodic ventilation pulse (`sysConfig.purge_interval_min`, `sysConfig.purge_duration_sec`).
  - Animated SVG Sanduhr Widget (`#hourglass-svg`) with live sand trickling stream (`@keyframes sand-pour`), dynamic upper/lower sand level geometry, and live badge countdown (`IN mm:ss` in blue during waiting phase, `OFFEN mm:ss` in red during active open purge).
  - High priority override: Forces rotor to 100% open during active pulse.
  - Strict Slave Mode Isolation: On Slave devices (`espnow_role == 2`), the purge section is hidden from UI and firmware engine explicitly disables purge timer (`isPurgeActive = false`) so the Slave strictly mirrors the Master's remote instructions.
* **3D Drum Selection Wheels (Walzen-Drehwähler):**
  - Interactive 3-row CSS drum pickers (`#wheel-interval`, `#wheel-duration`).
  - Authentic 2D Y-axis compression (`scale(0.92, 0.65)`) on upper and lower neighbor rows for true drum curvature depth.
  - Active center cell framed by glowing cyan glass borders (`#38bdf8`, `box-shadow: 0 0 24px rgba(56, 189, 248, 0.42)`, `text-shadow: 0 0 10px rgba(56, 189, 248, 0.7)`).
  - PC Mouse Drag-to-Scroll Engine: Event handlers (`mousedown`, `mousemove`, `mouseup`) provide fluid 1:1 mouse dragging with grabbing hand cursor (`cursor: grabbing`) and smooth snapping (`28px` item boundaries) on desktop browsers.
* **4-Hour RSSI Signal Sparkline Graph:**
  - Double-width RSSI card renders 4-hour telemetry window (24 historical samples) with multi-color gradient (Red -> Orange -> Yellow -> Green).

---

## Smart Live-Advisor & Heuristic Ticker Engine
* **Full-Width Header-Widget & Non-Stop Scroller:** Placed directly beneath the main dashboard title with animated pulsing brain icon (`🧠`), history navigation controls (`◀ 1 / X ▶`), dedicated info button (`Index 20`), and continuous seamless looping text scroller.
* **100% Solid Dark Speech Bubble Modal (`.advisor-popup-bubble`):** Pure opaque `#090d16` with `z-index: 9999` preventing any bleed-through of underlying canvas/moon elements. Clicking the ticker opens the full-text popup with integrated `◀ Älter` / `Neuer ▶` history buttons and a `✕` close button.
* **History Navigation Boundaries & Dynamic Visibility:** Hard stop at message index 0 (`Neuer ▶` button auto-hides) and oldest message index (`◀ Älter` button auto-hides) to eliminate accidental wrap-around.
* **20-Message Ringbuffer & Anti-Spam Deduplication:** Holds up to 20 historical thermodynamic grower tips with exact timestamps `[HH:MM:SS]` and individual color-coded badges (🟢 `OPTIMAL`, 🟡 `DIY TIPP`, 🔴 `WARNUNG`, 🔵 `WETTER`, 🟣 `SYSTEM`). Evaluates every 10 seconds and rejects duplicate messages.
* **Integer-Quantized Environmental Heuristics:** Quantizes sensor values to integer rounded steps (`Math.round(...)`) to eliminate floating-point noise and prevent false-positive ringbuffer spamming.
* **Zero-RAM Chunked HTTP Streaming Architecture:** Splits dashboard HTML into PROGMEM chunks streamed via `server.setContentLength(CONTENT_LENGTH_UNKNOWN)` and `server.sendContent(...)` using 0 Bytes of dynamic RAM heap, preventing string truncation and memory crashes.


