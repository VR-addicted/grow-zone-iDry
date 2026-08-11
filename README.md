# 🌿 iDry-26: Smart Climate & Servo Shutter Controller (ESP32-S3)

<p align="center">
  <img src="https://img.shields.io/badge/PlatformIO-Core-orange.svg" alt="PlatformIO">
  <img src="https://img.shields.io/badge/Board-YD--ESP32--S3-blue.svg" alt="ESP32-S3">
  <img src="https://img.shields.io/badge/ESP--NOW-Encrypted%20Mesh-red.svg" alt="ESP-NOW">
  <img src="https://img.shields.io/badge/Home%20Assistant-MQTT%20Auto%20Discovery-03a9f4.svg" alt="Home Assistant">
  <img src="https://img.shields.io/badge/OTA-1--Click%20GitHub%20Update-green.svg" alt="OTA Update">
  <img src="https://img.shields.io/badge/Display-e--Paper%20%2F%20TFT%20%2F%20Headless-green.svg" alt="Display">
</p>

**iDry-26** ist eine hochentwickelte, ausfallsichere IoT-Klima- und Lüftungsklappensteuerung auf Basis des **YD-ESP32-S3**. Das System regelt Lüftungsrohre über Servo-Blendenverschlüsse vollautomatisch, schützt Erntegut und Kräuter vor Übertrocknung sowie Feuchtigkeitsinjektion (Thermodynamischer Feuchteschutz), kommuniziert verschlüsselt über **ESP-NOW Master/Slave-Kopplung**, bietet nahtlose **1-Klick Home Assistant Auto-Discovery** und unterstützt flexible Hardware-Ausbaustufen von *Headless* bis *Vollausbau*.

---

## 🎯 Der tiefere Sinn: Hermetischer Verschluss gegen Übertrocknung

Im Gegensatz zu gewöhnlichen Umluftventilatoren im Trockenzelt (die die Luft nur intern umwälzen, aber keinen Verschluss zur Außenwelt darstellen) ist **iDry-26 ein echtes mechanisches Ventil**:

* **Wie ein Eimer mit Deckel (Storage Container):** Sobald der eingestellte Feuchte-Wunschwert im Zelt erreicht ist, schließt iDry-26 die Zu- und Abluftlöcher **komplett hermetisch**. 
* **Schutz vor Übertrocknung:** Das Zelt verhält sich bei geschlossener Klappe wie ein geschlossener Curing-Behälter. Wertvolle Terpene und die ideale Restfeuchte bleiben perfekt erhalten, anstatt durch kontinuierlichen Luftzug auszutrocknen.
* **Master-Slave Kombination:** Zwei Einheiten (Master an der Abluft, Slave an der Zuluft) synchronisieren sich in Echtzeit über ESP-NOW. Bei Zielerreichung wird das Trockenzelt an **beiden Enden gleichzeitig** hermetisch abgedichtet. *(Funktioniert natürlich auch hervorragend als Einzelgerät!)*
* **Super einfache 1-Knob Bedienung:** Die tägliche Steuerung erfolgt kinderleicht über **einen einzigen Hauptdrehknopf (Poti A)** für den Feuchte-Sollwert (48–72%), inklusive automatischer *"Rigoros ZU"* ($\le 49\%$) und *"Rigoros AUF"* ($\ge 71\%$) Stellungen.

---

## 📷 Galerie & UI-Vorschau

<div align="center">

### ⚙️ Hardware & Mechanischer Aufbau

| Frontansicht mit Blende | Mechanik & Servosteuerscheibe |
| :---: | :---: |
| <img src="PICTURES/iDry24-Front.jpg" width="420" alt="Frontansicht Blende"> | <img src="PICTURES/iDry24-Front_Mechanics.jpg" width="420" alt="Mechanik Servolenkung"> |

| Rückseite & Verkabelung | Gehäuse & Seitenansicht |
| :---: | :---: |
| <img src="PICTURES/iDry24-Back.jpg" width="420" alt="Rückseite Platine"> | <img src="PICTURES/iDry24-Back-angle.jpg" width="420" alt="Winkelansicht Gehäuse"> |

<br>

### 🖥️ Web-Interface & Monitoring Dashboard

| Dual Live Monitor (Master & Slave synchronisiert) | Einstellungen: WLAN, MQTT & ESP-NOW Pairing |
| :---: | :---: |
| <img src="PICTURES/iDry24-Web-UI-Master-Slave-MODE.jpg" width="420" alt="Dual Dashboard Live Monitor"> | <img src="PICTURES/iDry-Web-UI-Settings-1.jpg" width="420" alt="Web UI Network Settings"> |

| Einstellungen: Watchdogs & Signalstärke-Balken | Einstellungen: Fail-Safe Schutz & Geräte-Management |
| :---: | :---: |
| <img src="PICTURES/iDry-Web-UI-Settings-2.jpg" width="420" alt="Web UI Watchdogs Settings"> | <img src="PICTURES/iDry-Web-UI-Settings-3.jpg" width="420" alt="Web UI Fail Safe Settings"> |

</div>

---

## 🏗️ Hardware-Ausbaustufen (Skalierbarkeit)

Das System erkennt alle Komponenten beim Systemstart vollautomatisch und passt sein Verhalten dynamisch an:

1. **Stufe 1: Minimalist (Headless Mode)**
   * Nur das YD-ESP32-S3 Board ohne angeschlossenes Display oder Sensoren.
   * **Vorteil:** Minimale Leistungsaufnahme, ideal als reiner Netzwerk-Aktor oder ESP-NOW Empfänger. Volle Steuerung über das Web-UI und MQTT.
2. **Stufe 2: Standard Single-Display**
   * Automatische Erkennung am gemeinsamen JST-Kabelbaum:
     * **Waveshare 3.52" e-Paper (B)** (360x240, Rot/Schwarz/Weiß, extrem stromsparend).
     * **ILI9341 3.2" TFT-Touchdisplay** (320x240, hohe Bildwiederholrate).
3. **Stufe 3: Vollausbau (Sensorik & Analogregler)**
   * **Dual I2C Feuchtesensoren:** BME280 (Innen/Barometer) + SHT31 (Außen).
   * **Dual I2C Helligkeitssensoren:** TSL2561 (Breitband + Infrarot).
   * **3x Analog-Potentiometer:** Poti A (Sollwert Feuchte 48-72%), Poti B (Gain 0-400%), Poti C (Servo-Nullpunkt Offset 0-59°).
   * **Aktorik & Akustik:** PWM-Servo (LEDC Sinus-Ramping) + Passiver Buzzer für Arpeggio-Melodien und Alarme.
4. **Stufe 4: Verschlüsseltes ESP-NOW Master/Slave Mesh**
   * Drahtlose Synchronisation zweier Einheiten über CCMP LMK 128-Bit Hardware-Verschlüsselung.
   * Der Master spiegelt seine Klappenstellung in Echtzeit auf den Slave.
   * **Notfall-Fail-Safe Schutz:** Bricht die Funkverbindung >60s ab, fährt der Slave automatisch auf 50% Sicherheitsöffnung oder übernimmt autonom über lokale Sensoren.
5. **Stufe 5: Nahtlose Home Assistant Integration (1-Klick Auto-Discovery)**
   * Vollautomatische Erkennung aller Entitäten in Home Assistant (Rotor Position, Servo-Winkel, Temperatur, Feuchte, Taupunkt, VPD, RSSI, Potis A/B/C, Einzel-Sensoren).

---

## 🔌 Hardware-Verkabelung (Pinout)

Die Displays teilen sich denselben physischen SPI-Kabelbaum (JST-Stecker am YD-ESP32-S3):

| Display-Pin (E-Ink / TFT) | Kabelfarbe | YD-ESP32-S3 Pin | Funktion |
| :--- | :--- | :--- | :--- |
| **VCC** | Rot | `3.3V` | Stromversorgung |
| **GND** | Schwarz | `GND` | Masse |
| **DIN / MOSI** | Blau | `GPIO 11` | SPI-Datenleitung (MOSI) |
| **CLK / SCK** | Gelb | `GPIO 12` | SPI-Taktleitung (SCK) |
| **CS** | Orange | `GPIO 10` | Chip Select |
| **DC / RS** | Grün | `GPIO 9` | Data / Command Control |
| **RST** | Weiß | `GPIO 14` | Panel Reset |
| **BUSY / LED** | Grau | `GPIO 8` | E-Ink: Busy-Line / TFT: Backlight-PWM |

### Sensorik, Potentiometer, Buzzer & Servo Pinout
* **I2C Bus:** `SDA` -> `GPIO 15` | `SCL` -> `GPIO 16`
* **Passiver Buzzer:** `GPIO 17` (NPN-Transistor Ansteuerung)
* **Servo Motor (PWM):** `GPIO 18` (LEDC Kanal 2)
* **Poti A (Sollwert Feuchte):** `GPIO 4` (ADC)
* **Poti B (Regel-Gain):** `GPIO 5` (ADC, heavily EMA low-pass filtered)
* **Poti C (Servo-Nullpunkt Offset):** `GPIO 1` (ADC)

---

## 🚀 Key Features & Algorithmen

### 1. Simple 1-Knob Steuerung & Diskrete Zonen
* Die Steuerung erfolgt im Alltag über **einen einzigen Drehknopf (Poti A)**:
  * $\le 49\%$: **Rigoros ZU** ($0\%$ Öffnung, Klappe schließt vollständig).
  * $\ge 71\%$: **Rigoros AUF** ($100\%$ Öffnung, maximale Entlüftung).
  * $50 - 70\%$: Stufenlose proportionale Feuchteregelung.

### 2. Nahtlose Home Assistant Integration (1-Klick Auto-Discovery)
* **Zero-Configuration:** Home Assistant erkennt iDry-26 unter *Einstellungen ➔ Geräte & Dienste ➔ MQTT* **vollautomatisch**.
* Überträgt live über 20+ Sensorwerte: Rotor-Position (%), Servo-Winkel (°), Temperatur (°C), Luftfeuchtigkeit (%), Taupunkt (°C), VPD (kPa), Signalstärke (dBm/LQI), Potis A/B/C und alle angeschlossenen I2C-Sensoren.
* Spezieller 2048-Byte MQTT-Puffer verhindert das Abschneiden großer JSON-Entitäten.

### 3. 1-Klick GitHub OTA Update mit Live-Terminal & Header-Schutz
* **Online 1-Klick Update:** Prüft die aktuellste Version auf GitHub. Ein Klick auf **"🚀 Automatisch Online Updaten"** startet das Update.
* **Interaktives Live-Terminal:** Zeigt im Web-Browser in Echtzeit die Download-Schritte, Dateigröße, Header-Prüfung, Flash-Fortschritt in % und den Neustart an.
* **ESP32 Header-Schutz (Magic Byte `0xE9`):** Prüft vor dem Schreiben in die Flash-Bank das ESP32 Bootloader Magic Byte `0xE9`. Verhindert das versehentliche Flashen von fehlerhaften Dateien oder 404-Seiten zuverlässig.

### 4. Closed-Loop Servo Ramping & Powerdown
* **Sinus-Ramping (Sine Easing):** Klappenbewegungen erfolgen stufenlos über eine Sinuskurve (`0.5f * (1.0f - cos(t * PI))`) für vibrationsfreies Anfahren.
* **Stromsparmodus & Summschutz:** Nach 1 Sekunde Stillstand an der Zielposition schaltet der ESP32 das PWM-Signal des Servos ab (`ledcWrite(18, 0)`). Das verhindert kontinuierliches Mikrozucken und Brummen des Servos im Leerlauf.

### 5. Thermodynamischer Feuchteschutz (Saug-Sperre)
* **Saug-Sperre:** Ist die Außenluft feuchter als die Innenluft oder liegt sie $>2\%$ über dem Sollwert, schließt die Klappe sofort auf **0%**, um das Einsaugen feuchter Luft zu verhindern (inkl. akustischem Warn-Chime).

### 6. ESP-NOW Master/Slave Reconnection & 2-Stufen Fail-Safe
* **Fast-Track Pairing:** Automatisches Channel-Hopping (Kanal 1–13) und Pairing per 128-Bit CCMP LMK Verschlüsselung mit case-insensitivem MAC-Vergleich (`strcasecmp`).
* **Fail-Safe Schutz (>60s):** Fällt die Funkverbindung aus, schaltet der Slave auf **50% Notfall-Öffnung** oder übernimmt autonom über eigene Sensoren.

---

## 🛠️ Treiber-Anpassung für 3.52" e-Paper (GxEPD2 Spoofing)

Um das 3.52" e-Paper Display mit der `GxEPD2`-Bibliothek anzusteuern, muss die Datei `.pio/libdeps/esp32-s3-devkitc-1/GxEPD2/src/epd3c/GxEPD2_213_Z19c.h` angepasst werden:

```cpp
// Zeilen 24-26 auf 240x360 auflösen:
static const uint16_t WIDTH = 240;
static const uint16_t WIDTH_VISIBLE = WIDTH;
static const uint16_t HEIGHT = 360;
```

---

## 💻 Bauen & Flashen via PlatformIO

```bash
# Firmware bauen und flashen
pio run -t upload

# Seriellen Monitor öffnen
pio device monitor
```

## 📝 Lizenz

Dieses Projekt ist unter der **MIT Lizenz** veröffentlicht.
