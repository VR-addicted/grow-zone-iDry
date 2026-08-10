# Project Workspace Customization Rules

This workspace contains customized configuration rules for the YD-ESP32-S3, the Waveshare 3.52" e-Paper display, I2C sensor interfaces, analog potentiometers, LEDC Servo control, and the passive boot Buzzer.

## Custom Driver & Display Rules
* **Hardware Profile:** Waveshare 3.52" e-Paper (B), Red/Black/White, 360x240 pixels.
* **Mismatched Driver Class:** Always use `GxEPD2_213_Z19c` as the driver class spoofed to 360x240 resolution.
* **Display Class Wrapper:** Always use `GxEPD2_3C` for three-colour rendering (Black, Red, White).
* **Do NOT attempt to use 4-colour driver (`GxEPD2_4C` or `GxEPD2_350c_GDEM035F51`)** because the panel physically lacks yellow/orange pigments and will produce a corrupted image layout.

## PIN Configuration Constraints
Do not change the display SPI, I2C, potentiometer, or buzzer pins from their configured assignments:
* SCK: 12
* MOSI: 11
* MISO: 13 (unused)
* CS: 10
* DC: 9
* RST: 14
* BUSY: 8
* **I2C Bus:** SDA -> GPIO 15, SCL -> GPIO 16
* **Buzzer Pin:** GPIO 17
* **Servo Pin:** GPIO 18 (LEDC PWM, Channel 2)
* **Potentiometers:** Poti A -> GPIO 4, Poti B -> GPIO 5, Poti C -> GPIO 1

## Software Autodetect & RF Brownout Prevention
* **Display Autodetect:** 3-state hardware detection:
  1. Pin `EPD_BUSY` (GPIO 8) set as `INPUT_PULLUP`. If read `LOW` -> TFT Mode (backlight transistor load).
  2. If read `HIGH`, toggle `EPD_RST` (GPIO 14). If state on GPIO 8 changes -> e-Paper Mode. Else -> Headless Mode (`isHeadless = true`).
  3. When `isHeadless` is true, completely bypass display rendering in the loop and initializations to conserve CPU cycles/power.
* **RF Brownout Protection:** Always call `display.powerOff();` and insert a `delay(500);` before initiating SoftAP / WLAN on e-Paper modules to drop current load and stabilize the shared 3.3V rail.
* **WiFi STA Stability:** Always use `WiFi.disconnect(true);` and `WiFi.setSleep(false);` during initialization to clear socket states and prevent packet drops.

## Active Connection Watchdogs & Web UI Timeouts
* **Non-Blocking Network Execution:** 
  * Always set socket timeout to 500ms (`espClient.setTimeout(500)` and `client.setTimeout(500)` in gateway checks).
  * Rate-limit MQTT connection attempts (`mqttClient.connect()`) to at most once every 10 seconds to prevent blocking the servo control loop.
* **Gateway Watchdog:** Perform TCP client connection checks to `WiFi.gatewayIP()` on port 80 every 2 seconds. Use a 500ms timeout. If it fails, assume a zombie link, call `WiFi.disconnect(true)` immediately, and trigger the reconnect cycle.
* **WLAN Event Handlers:** Log event changes without triggering blocking recursion (avoid `WiFi.begin` inside event callback interrupts).
* **Web UI AJAX Timeout:** Always fetch `/api/data` in the browser client using a timeout wrapper (1000ms via `AbortController`) to immediately transition the UI into "try to reconnect to: [SSID]" state when offline.
* **MQTT Interval:** Throttle Home Assistant status payload publish updates to every 30 seconds to minimize network bandwidth consumption.

## Servo & Ramping Constraints
* **PWM Setup:** LEDC Setup is configured at 50 Hz, 14-bit resolution on Channel 2, GPIO 18.
* **Ramp Updates (Sine Easing):** Servo motion ramping uses a sine wave curve (`0.5f * (1.0f - cos(t * PI))`).
* **Poti & Logic Updates (20Hz / 1Hz):** Run `updateServoRamping()` continuously in `loop()`, but rate-limit analog potentiometer evaluations to **50 ms (20 Hz)**. Trigger a closed-loop target recalculation directly after the **1-second** sensor read loop to ensure an immediate (max. 1 second) emergency shut-off reaction.
* **Calibrated Limits:** Physical sweep is capped at **$121^\circ$**. The Poti C offset is mapped up to **$59^\circ$** to guarantee the servo never exceeds its physical $180^\circ$ limit.
* **Poti A Steps (24 Steps):** Mapped to 48..72. Values <=49 are closed, values >=71 are open. Dazwischen is 50..70. In the Web UI, hide the raw numeric values when in boundary zones and display only "Rigoros ZU" and "Rigoros AUF".

## Thermodynamic Shutter Logic
* **Sensor Roles:** `tempSensors[0]` is Inside/Master; `tempSensors[1]` is Outside.
* **Priority Sorting:** In `scanI2C()`, if a BME280 (which has a built-in barometer/pressure sensor) is detected on position 1 while position 0 is not a BME280, they must be swapped to promote BME280 to the Inside/Master slot.
* **Bypass Lock:** If `hum_outside > hum_inside` OR `hum_outside > (potiAVal + 2.0f)`, the rotor position is overridden to `0%` (fully closed) to protect the grow space from drawing in wet outside air.
* **Dryness Multiplier:** If the outside air is extremely dry compared to the inside target (`potiAVal - hum_outside > 10%`), scale the maximum allowed opening linearly down to a maximum cap of `70%` (reached at 30% difference) to prevent rapid drying shocks.
* **Self-Healing I2C Sensor Recovery:** If a BME280 or SHT3X sensor fails (returns `NAN` or exactly `0.0` for temperature and humidity due to line noise/EMI), discard the values (store `NAN` to trigger `--` in UI) and execute a silent `begin()` reset on the sensor object every 2 seconds to recover the link.


## Buzzer Melodies
* The passive buzzer is connected to **GPIO 17**.
* Use the Arduino `tone(pin, freq, duration)` function to play short melodies. 
* Avoid long delays in the main loop; sound sequences should only block during the boot `setup()` phase.
* **Grenz-Chimes (Poti A):** Play a short double-beep feedback exactly once when Poti A crosses into boundary modes (<=49% -> descending beep, >=71% -> ascending beep). Keep track of boundary zones to prevent repeated triggering. Keep chimes short (e.g. 40ms/60ms with 50ms delay) to minimize main loop jitter.
* **Low Humidity Alarm:** Check every 300000ms (5 minutes) if inside humidity (from tempSensors[0] or fallback to tempSensors[1]) is below Poti A target. If so, play a pleasant descending arpeggio of 3 tones (C5 -> A4 -> F4, 500ms duration per tone, no pauses, blocking delays allowed). Do not send this alarm status to MQTT.
* **Bypass Active Alarm:** If the thermodynamic bypass (notschließen) is active, play a warning chime every 300000ms (5 minutes) starting with a **5-second initial offset** (at 5000ms) to prevent collision. The chime plays 3 short tones of 500Hz (80ms on, 80ms off), a 1-second long pause, and then repeats the 3 short tones.

