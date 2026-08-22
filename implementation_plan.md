# Implementation Plan - Display Backlight Auto-Dimmer, Servo Odometer & Settings Reorganization (Build v163)

Intelligent light-sensor-based display power saving, a persistent wear-leveled Servo Odometer engine with live calibration controls, and an optimized `/settings` panel layout hierarchy.

---

## 🛠️ Architecture & Delivered Specifications

### 1. Display Backlight Auto-Dimmer (TFT Light-Sensor Control)
- **Sensor Presence Detection:**
  - If **NO light sensor** (TSL2561) is connected/active: Backlight follows user-configured brightness (0–100%) in Settings.
  - If **AT LEAST ONE light sensor** is active:
    - If ANY active sensor reads **$> 200\text{ Lux}$**: Display Backlight turns **ON** at configured brightness (`sysConfig.display_brightness`).
    - If ALL active sensors read **$\le 200\text{ Lux}$** continuously for **$> 3\text{ seconds}$** (3s debounce filter against temporary shadows): Backlight turns **COMPLETELY OFF ($0\%$)** to prevent room light pollution and conserve power.
    - Threshold of $200\text{ Lux}$ ensures no recursive feedback loop between TFT backlight and ambient sensor.

### 2. Servo Odometer Engine & Wear-Leveling Persistence
- **Kinematic Calculation ($r = 27\text{ mm}$ gear radius):**
  - Arc length per degree: $\Delta d = \frac{\pi \cdot 27\text{ mm}}{180^\circ} \approx 0.00047124\text{ m/deg}$.
  - Every angular step adds to `servoTotalMeters`.
  - Nominal 100% lifetime baseline: **$50,000\text{ m}$ ($50\text{ km}$)**.
  - Lifetime percentage: $\text{Lifetime \%} = \min\left(100.00, \frac{\text{servoTotalMeters}}{50000} \times 100\right)$ (displayed with 2 decimal places, e.g. `1.42 %`).
- **Dual-Storage Persistence (LittleFS + NVS Mirroring):**
  - **RAM Caching:** Accumulated in RAM and only committed to Flash once per hour (and only if actual movement occurred).
  - **Dual Mirroring:**
    1. Saved in LittleFS (`/config.json`).
    2. Mirrored to ESP32 NVS partition (`Preferences.h`, namespace `idry_odo`) with Magic Word `0x49445259` (`IDRY`).
  - **Boot Recovery:** Reads both LittleFS and NVS. If a discrepancy exists (e.g. after a partial flash wipe), the higher valid number wins and re-synchronizes both storage layers.

### 3. Settings UI Panel (`/settings`) & Layout Hierarchy
- **Optimized Natural Flow:**
  1. **Wi-Fi Verbindung**
  2. **MQTT Konfiguration**
  3. **ESP-NOW Funknetzwerk**
  4. **Buzzer Test**
  5. **Servo Laufleistung & Odometer** *(placed directly below Buzzer Test with `ℹ`-Info Button Index 21)*
  6. **System Status** *(placed directly above System & Anzeige)*
  7. **System & Anzeige**
  8. **Save & Back Buttons** *(placed at the bottom of the configuration cards, right above Geräte-Management)*
  9. **Geräte-Management** (Firmware & OTA, Reboot, Reset)
- **Live Editable Input Field & Instant Bar Update:**
  - Typing in the input field calculates and updates the dual-tone blue progress bar (`#0284c7` to `#38bdf8`) and percentage label immediately.
  - AJAX polling on that input temporarily pauses while editing to prevent input overwriting.
  - `Ändern` button submits `POST /api/settings/odometer?meters=...`. Setting `0` resets the odometer; entering an existing value restores previous mileage.

### 4. Boot Safety & Network Decoupling
- ESP-NOW packet streaming (`sendEspNowLogLine`) is guarded by `isEspNowInitialized` to prevent dereferencing uninitialized driver structures during early boot.
- NVS initialization opens in read-write mode to prevent `nvs_open` errors on fresh boards.

---

## 🧪 Verification & Build Status
- PlatformIO Build: `SUCCESS` (Code 0, RAM: 29.7%, Flash: 21.9%).
- Firmware bundle updated in `FIRMWARE/` (v163).
