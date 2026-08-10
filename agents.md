# Project Workspace Customization Rules

This workspace contains customized configuration rules for the YD-ESP32-S3, Waveshare 3.52" e-Paper display, ILI9341 TFT display, I2C sensor interfaces, analog potentiometers, LEDC servo motor control, ESP-NOW dual-mesh communication, and Home Assistant MQTT auto-discovery.

---

## Custom Driver & Display Autodetect Rules
* **Hardware Profiles Supported:** Waveshare 3.52" e-Paper (B) 360x240 pixels (Red/Black/White), ILI9341 3.2" TFT Display 320x240 pixels, or Headless Mode (no display connected).
* **Mismatched Driver Class:** For 3.52" e-Paper, always use `GxEPD2_213_Z19c` as driver class spoofed to 360x240 resolution (`WIDTH = 240`, `HEIGHT = 360` in `GxEPD2_213_Z19c.h`).
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

## Active Connection Watchdogs & Web UI Timeouts
* **Gateway Watchdog:** Perform TCP client connection checks to `WiFi.gatewayIP()` on port 80 every 2 seconds. If a timeout (> 400ms) occurs, call `WiFi.disconnect(true)` immediately and initiate reconnect cycle.
* **Web UI AJAX Timeout:** Fetch `/api/data` in Web UI using a 1000ms `AbortController` timeout to transition UI immediately into offline status when link drops.
* **WLAN Connection Watchdog Alarm (`wlan_time_trap`):** Configurable slider (0–330s, default 120s). When connection drops, immediately play a double beep buzzer sequence (two 250ms tones at 500Hz with 100ms pause) and repeat at configured interval. Disabled when set to 0.
* **RSSI Bargraph Indicator:** CSS-based 50px horizontal bar next to RSSI dBm values in Web UI. Dynamically scaled 0–100% (-100 to -30 dBm) with HSL/RGB color transitions (Red -> Orange -> Yellow -> Green -> Light Green).
* **Weekly Reboot Watchdog:** Uptime monitored via `millis()`. When uptime exceeds 1 week (7 days / 604,800,000ms), check NTP clock. If NTP synced, delay reboot until 03:00 AM local time. If no NTP, reboot immediately (`ESP.restart()`). Countdown exposed in `/api/data`, Web UI, and MQTT.

---

## ESP-NOW Master/Slave Mesh & Fail-Safe Protection
* **Protocol Versioning:** Increment `localProtocolVersion` whenever `EspNowMessage` struct or command payload changes. Web UI alerts user on version mismatch.
* **Fast-Track Channel Pairing:** Master broadcasts pairing beacons on Wi-Fi channel; Slave hops channels 1–13 every 1.2s to establish peer MAC and 128-bit CCMP LMK hardware encryption.
* **Aggressive Reconnection (>20s):** On Slave devices, if no packet is received for >20 seconds, re-initialize ESP-NOW stack (`initEspNow()`) every 15 seconds without rebooting the MCU (preventing unwanted servo movements).
* **2-Stage Fail-Safe Mode (>60s Connection Loss):**
  - `espnow_failsafe_mode = 0` (Default) or no local sensor: Force rotor position to 50% (Safety Open) so ventilation is never choked. UI displays 3-line warning hint when no sensor is detected.
  - `espnow_failsafe_mode = 1` with active local sensor: Calculate rotor position locally using Slave's Poti A and local humidity sensor.
  - Automatically resumes mirroring Master when connection is restored.

---

## Potentiometer Signal Conditioning & Discrete Zones
* **EMA Low-Pass Filter:** Analog inputs filtered using Exponential Moving Average (EMA). Poti B (Gain Factor) uses heavy low-pass filtering ($\alpha = 0.05$) to eliminate ADC noise and prevent jitter.
* **Integer Percentage Formatting:** Poti B Gain Factor is formatted and displayed as a clean whole integer percentage ($0 - 400\%$) across Web UI, JSON API, and MQTT.
* **Poti A Discrete Zones & Hysteresis:** Poti A mapped to target humidity ($48 - 72\%$).
  - $\le 49\%$: Rigorously Closed ($0\%$ opening, displays `"Rigoros ZU"`).
  - $\ge 71\%$: Rigorously Open ($100\%$ opening, displays `"Rigoros AUF"`).
  - $50 - 70\%$: Closed-loop proportional humidity regulation.

---

## Servo Motion Profiling & Powerdown Management
* **Sine Easing (Sinusoidal Ramping):** Smooth acceleration and deceleration using sine curves (`0.5f * (1.0f - cos(t * PI))`). Short movements throttled to $<50\%$ max speed.
* **Idle Powerdown:** Shut off PWM signal (`ledcWrite(SERVO_LEDC_CHANNEL, 0)`) after 1 second of inactivity once target angle is reached, eliminating idle servo buzzing and current draw.
* **Update Rate Limiting:** Closed-loop servo updates throttled to user-configured interval (`sysConfig.servo_update_interval`, 1–30s, default 5s). Bypassed immediately when Poti A enters boundary zones or thermodynamic bypass triggers.

---

## Thermodynamic Feuchteschutz & Acoustic Alerts
* **Thermodynamic Bypass (Saug-Sperre):** If outside humidity is higher than inside humidity or $>2\%$ above target, rotor forces fully closed ($0\%$) to prevent moisture influx. Accompanied by acoustic warning chime.
* **Acoustic Signalization:** Passive buzzer handles boot melody (C5 -> G6), boundary chimes (descending/ascending arpeggios), drying progress alerts, and connection loss watchdog alarms.
