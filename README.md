# 🌿 iDry-26: Smart Climate & Servo Shutter Controller (ESP32-S3)

<p align="center">
  <img src="https://img.shields.io/badge/PlatformIO-Core-orange.svg" alt="PlatformIO">
  <img src="https://img.shields.io/badge/Board-YD--ESP32--S3-blue.svg" alt="ESP32-S3">
  <img src="https://img.shields.io/badge/ESP--NOW-Fast--Track%20Mesh-red.svg" alt="ESP-NOW">
  <img src="https://img.shields.io/badge/Home%20Assistant-MQTT%20Auto%20Discovery-03a9f4.svg" alt="Home Assistant">
  <img src="https://img.shields.io/badge/OTA-1--Click%20GitHub%20Update-green.svg" alt="OTA Update">
  <img src="https://img.shields.io/badge/Display-e--Paper%20%2F%20TFT%20%2F%20Headless-green.svg" alt="Display">
  <img src="https://img.shields.io/badge/Telemetry-Gapless%20Canvas%20Sparklines-blueviolet.svg" alt="Sparklines">
</p>

**iDry-26** *(kurz für **intelligent Dry 2026**)* ist eine hochentwickelte, ausfallsichere IoT-Klima- und Lüftungsklappensteuerung auf Basis des **YD-ESP32-S3**. Das System regelt Lüftungsrohre über Servo-Blendenverschlüsse vollautomatisch, schützt Erntegut und Kräuter vor Übertrocknung sowie Feuchtigkeitsinjektion (Thermodynamischer Feuchteschutz), kommuniziert ausfallsicher über **ESP-NOW Master/Slave-Kopplung** (gezielte MAC- & Protokoll-Verifikation), bietet nahtlose **1-Klick Home Assistant Auto-Discovery** und unterstützt flexible Hardware-Ausbaustufen von *Headless* bis *Vollausbau*.

---

## 🔥 **HIGHLIGHT FEATURE:** Scientific VPD Strategy, Automated 14-Day Curve & Hygro-Limit Mold Protection

**iDry-26** unterstützt drei wählbare Trocknungs-Strategien, die stufenlos per Umschalter im Web-Dashboard gewählt werden können:

1. **60/60 Mode (Klassische Feuchteregelung):**  
   Poti A bestimmt direkt die Wunsch-Luftfeuchtigkeit (z. B. 60% RH).
2. **VPD Target Mode (Manuelle Sättigungsdefizit-Regelung):**  
   - **Präzise VPD-Skalierung auf Poti A:** Der Drehknopf regelt stufenlos den **Wunsch-VPD von 0.60 kPa bis 1.40 kPa**, mit **1.00 kPa exakt in der physikalischen Mittelstellung (50%)**.
   - **Automatische Temperatur-Kompensation:** Aus der aktuellen Innentemperatur berechnet das System in Echtzeit die exakt benötigten Feuchteprozente ($RH_{\text{calculated}}$), um deinen Wunsch-VPD perfekt einzuregeln.
3. **VPD AUTO Mode (`VPD AU` – Wissenschaftlicher 14-Tage-Trocknungsplan):**  
   - **Vollautomatischer 14-Tage-Kurvenverlauf:** Regelt das Sättigungsdefizit nach einem wissenschaftlichen Curing-Profil stufenweise von **0.70 kPa an Tag 1 (~70% RH)** sanft ansteigend bis **1.10 kPa an Tag 14 (~55% RH)**.
   - **Interaktives 14-Tage Dropdown & Kerzenleiste:** Das Web-UI bietet ein 14-zeiliges Dropdown zur manuelle Tag-Auswahl inklusive farbiger Kerzenleiste mit leuchtend pulsierender Animation (`@keyframes vpd-candle-pulse`) für den aktiven Tag.
   - **Automatischer Tageswechsel (Midnight Sync & MCU Microtime):** Tägliche Weiterschaltung punkt 00:00 Uhr Mitternacht (via SNTP/NTP) oder nach jeweils 24 Stunden Laufzeit.
   - **Nach Tag 14 (Haltemodus):** Auch ab Tag 15, 20 oder 30 bleibt der Ziel-VPD stabil auf dem optimalen Wert von Tag 14 (1.10 kPa), ohne abzubrechen.
   - **Dynamische gelbe Ideallinie (`#facc15`):** Auf den beiden VPD-Sättigungsdefizit-Graphen sowie im 24h Zoom Modal zeichnet das System eine leuchtend gelbe, gestrichelte Sollwert-Linie, die im `VPD AU` Modus exakt auf der tagesaktuellen Zielhöhe wandert.

- **Hygro-Limit Schimmelschutz-Garantie (70% / 75% / 80%):** Per Radio-Button wählbarer Maximalwert für die relative Luftfeuchte. Das System berechnet die Zielfeuchte, kappt sie jedoch **unwiderruflich bei deinem eingestellten Hygro-Limit** ($RH_{\text{effective}} = \min(RH_{\text{calculated}}, \text{Limit})$). Somit wird Schimmelbildung selbst bei extrem schwankenden Temperaturen zu 100% ausgeschlossen!
- **Transparente RAW-Telemetrie:** Im Dashboard wird live der un-gekapte mathematische Sollwert angezeigt (`RH calculated soll: 73.4 %`), inklusive dynamischem rotem Hinweis `(limited to 70%)`, wenn die Schimmelbremse aktiv greift.
- **Persistent & Ausfallsicher:** Die gewählte Strategie, der aktive Tag und das Hygro-Limit werden dauerhaft in LittleFS gespeichert und überleben Stromausfälle sowie Neustarts ohne Datenverlust.

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

## 📊 High-Performance Telemetrie & Dynamic Web UI Features

### 1. Doppel-Ringpuffer Telemetrie-Architektur (RAM)
* **120-Minuten Ringpuffer (`history120mBuffer[120]`):** Speichert 2 Stunden Verlauf in minütlicher Auflösung.
* **24-Stunden Ringpuffer (`history24hBuffer[288]`):** Speichert 24 Stunden Verlauf in 5-Minuten Auflösung.
* **Proben-Umfang:** Erfasst Min/Max-Werte für Temp 0/1, Hum 0/1, Lux 0/1, Rotor-Stellung, ESP-NOW Link-Loss (0–60s), MQTT Link-Loss (0–60s) und RSSI dBm.
* **Effizienter Speicherbedarf:** Nur ~21 KB RAM out of 320 KB (~28.4% Gesamt-RAM des ESP32-S3).

### 2. Nahtlose Sparkline-Balkendiagramme & Spike Detection (Gelbe Kappen)
* **Nahtlose Balken (Gapless Curves):** Exakte Pixel-Geometrie ($x_1 \to x_2$) ohne Lücken zwischen Kerzen erzeugt geschmeidige, gefüllte Flächenverläufe.
* **Spike Detection (Gelbe Delta-Segment-Kappe):** Bei Temperatur und Luftfeuchte reicht der hellblaue Kerzenkörper von 0 bis zum **Minimum ($Min$)**. Das **gelbe Dach ($Min \to Max$)** misst die positive Schwankungsbreite $|Max - Min|$ der Minute und wandert stets nach oben!

### 3. Mehrfarbiger RSSI-Verlauf & 2h System Status Karte
* **Bunter RSSI-Gradient:** Vertikaler 4-Farben Canvas-Verlauf (Rot $\to$ Orange $\to$ Gelb $\to$ Grün) färbt RSSI-Balken entsprechend der Empfangsstärke.
* **2-Stunden System Status Vorschau:** Das doppelt breite System Status Panel zeigt 120 Kerzen (2 Stunden minütlicher Verlauf) mit 30-Minuten Skalierungs-Ticks.

### 4. Interaktives 24h Zoom Modal mit Touch/Hover Floating Badge
* **Auto-Scroll zum Live-Moment:** Das Zoom-Fenster scrollt beim Öffnen automatisch an das rechte Live-Ende ($X = \text{max}$).
* **Touch & Pointer Badge:** Beim Tippen am Smartphone oder Hovern am PC schlägt 15px oberhalb der Kerze ein schwebender Tooltip-Badge auf. Zeigt exakte Werte, Min/Max, Spike-Delta, Zeitversatz (`-3h 45m` / `JETZT`) und farbcodierten Rahmen.

### 5. Dynamisches Mond-Favicon im Browser-Tab
* **Live-Mondphase im Tab-Icon:** Ein clientseitiger Offscreen-Canvas (32x32 Pixel) rendert die echte Blendenöffnung live in das Browser-Tab-Icon (0 Byte ESP32 RAM-Verbrauch!).
* **Theme-Awareness:** Dunkelblaues Abzeichen (`#171a33`) für Master, Dunkelrotes Abzeichen (`#3f0e0e`) für Slave.

### 6. Automatische Lesezeichen-Namen & Pulsierendes Update-Red-Glow
* **Auto-Bookmark Titel:** Der HTML-Header liefert dynamisch `<title>IDRY-26 Master</title>` bzw. `<title>IDRY-26 Slave</title>` aus.
* **1s Rot-Pulsierender Update-Rahmen:** Ein schonender Hintergrund-Check (beim Boot, alle 10 min und beim Aufruf von `/firmware`) prüft GitHub-Releases. Liegt ein Update vor, pulsiert der Rahmen des `Firmware & OTA Update` Buttons im 1-Sekunden-Takt rot mit einem leuchtenden Halo.

### 8. Live ESP-NOW RF Log Streaming & Dual Terminal Consoles (v100-v103)
* **T-Pipe RF Pipeline:** Master streamt alle `addAppLog(...)` Systemnachrichten live über 182-Byte ESP-NOW Funkpakete (`EspNowLogMessage`, Typ 3) an den Slave.
* **Fern-Diagnose bei Server-Hängern:** Selbst wenn der Webserver des Masters unter hoher Last stehen bleibt, können alle Log-Meldungen des Masters live über die Web-Oberfläche des Slaves überwacht werden!
* **Rollenbasierte Farb-Themes (Diagonale Farbsymmetrie):**
  * **Master Terminal Box:** Cyan-Blauer Header (`#38bdf8`), blauer Rahmen & blaue Schrift (passend zum blauen Master-UI).
  * **Slave Terminal Box:** Soft-Roter Header (`#f87171`), roter Rahmen & rote Schrift (passend zum roten Slave-UI).

### 9. 3-Stufiges Log-Filter-System, 1.000-Zeilen Historie & 1-Klick Disketten-Export (`💾`) (v104-v112)
* **3-Stufige Nachrichten-Klassifizierung:**
  * **Level 1 (`STAT` / `ALARM`):** Telemetrie, Buzzer-Test-Chimes, Schimmelschutz-Alarme, Einstellungs-Änderungen (`[Config]`), Pairing-Events (`[Pairing]`) & OTA-Update Status (`[OTA]`). **Immer sichtbar!**
  * **Level 2 (`WARN`):** Warnmeldungen, Sensor-Resets & Verbindungsabbrüche.
  * **Level 3 (`DBG `):** Gesprächiger Verbose-Debug-Modus mit kontinuierlicher Ausgabe von Sensor-Messwerten (BME280/SHT3x/TSL2561), VPD-Matrix-Berechnungen, Servo-Winkeln & ESP-NOW Pings.
* **Unabhängige Konsolen-Filter:** Jede Konsole (`Local` & `Remote`) besitzt in ihrer Überschrift eigene Filter-Radiobuttons `( ) L1  ( ) L2  (•) L3` zur clientseitigen Echtzeit-Gliederung.
* **1.000-Zeilen Browser-Arbeitsspeicher:** Speichert im Browser-RAM bis zu 1.000 historische Log-Zeilen pro Terminal mit geschmeidigem Scrollbalken (0 Byte ESP32 RAM).
* **1-Klick Disketten-Export (`💾`):** Ein Klick auf den Disketten-Button `💾` im Header generiert augenblicklich eine saubere `.txt`-Datei der jeweiligen Log-Historie und öffnet den nativen Download-Dialog des Browsers!

### 10. Automatische Flash-Persistenz bei Mitternachts-Tageswechsel (v107)
* **Mitternachts-Flash-Sync:** Beim automatischen Tageswechsel von `VPD AUTO` Punkt 00:00 Uhr Mitternacht (oder nach 24 Stunden) wird der neue Tag (z. B. Tag 8) sofort dauerhaft in `/config.json` via LittleFS Flash gespeichert.
* **Neustart- & Update-Sicher:** Nach Firmware-Updates oder Stromausfällen startet das Gerät garantiert auf dem aktuellsten Tagesstand neu, ohne auf frühere Tage zurückzusprengen!

### 11. Remote Linked Device Reboot & ESP-NOW Protokoll V5 (v114 Milestone)
* **Fern-Neustart gekoppelter Partner (`Reboot linked Device`):** Ein neuer Button in den Einstellungen unter *Geräte-Management* mit stylischem *Link Established* Icon (`🔗`) ermöglicht das gezielte Neustarten des per ESP-NOW verbundenen Partner-Geräts (Master -> Slave oder Slave -> Master).
* **Protokoll-Version V5:** Erweiterung des ESP-NOW Nachrichtenprotokolls um Command `99` (Remote Reboot Request). Das Zielgerät quittiert den Befehl mit einem Level-1 Log-Eintrag und führt nach 300ms einen sauberen Neustart aus.

### 12. HTTP Socket-Teardown & Stabilitäts-Härtung (v113)
* **Socket Leak Prevention (`Connection: close`):** Alle REST-API-Endpunkte (`/api/data`, `/api/history`) schließen TCP-Sockets unmittelbar nach der Datenübertragung. Verhindert das Überlaufen des LWIP TCP-Socket-Pools bei sekündlichem AJAX-Polling nachhaltig.

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
4. **Stufe 4: Ausfallsicheres ESP-NOW Master/Slave Mesh**
   * Drahtlose Echtzeit-Synchronisation zweier Einheiten mit gezielter MAC-Adressen-Kopplung & Protokollversions-Verifikation (unverschlüsseltes Peering für 100% verlässliche Übertragung ohne Paketverluste bei WLAN-Kanalwechseln).
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

### 1. Simple 1-Knob Steuerung, 3 Trocknungsstrategien & Diskrete Zonen
* **1-Knob Steuerung (Poti A):**
  * $\le 49\%$: **Rigoros ZU** ($0\%$ Öffnung, Klappe schließt vollständig).
  * $\ge 71\%$: **Rigoros AUF** ($100\%$ Öffnung, maximale Entlüftung).
  * $50 - 70\%$: Stufenlose proportionale Feuchteregelung.
* **3 Trocknungsstrategien:**
  * **60/60 Mode:** Klassische manuelle Ziel-Luftfeuchte über Poti A.
  * **VPD Mode:** Manuelle Ziel-VPD Einstellung ($0.60$ bis $1.40\text{ kPa}$) über Poti A.
  * **VPD AUTO Mode:** Automatischer 14-Tage Reifungs- & Trocknungsplan. Nutzt eine wissenschaftliche **21x14 Temperatur-Matrix ($15\text{ bis }35\text{ °C}$)**, die das Sättigungsdefizit dynamisch an die echte Raumtemperatur anpasst und an Tag 11–14+ exakt im **Goldstandard-Curing-Zielwert von $0.85\text{ kPa}$ (~$62\%\text{ RH}$)** landet. Visualisiert über ein interaktives **2D-Heatmap Canvas mit Fadenkreuz & Laser-Dot** im Web-UI!

### 2. Nahtlose Home Assistant Integration (1-Klick Auto-Discovery)
* **Zero-Configuration:** Home Assistant erkennt iDry-26 unter *Einstellungen ➔ Geräte & Dienste ➔ MQTT* **vollautomatisch**.
* Überträgt live über 20+ Sensorwerte: Rotor-Position (%), Servo-Winkel (°), Temperatur (°C), Luftfeuchtigkeit (%), Taupunkt (°C), VPD (kPa), Signalstärke (dBm/LQI), Potis A/B/C, Dry Strategy (0=60/60, 1=VPD, 2=VPD AU), VPD Auto Tag (1-14 bzw. -1) und alle angeschlossenen I2C-Sensoren.
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
* **Fast-Track Pairing:** Automatisches Channel-Hopping (Kanal 1–13) und unverschlüsseltes Peering (`peerInfo.encrypt = false` für 0% Paketverlust bei Kanalwechseln) mit gezieltem MAC-Vergleich (`strcasecmp`) und Protokollversions-Verifikation (V5).
* **Fail-Safe Schutz (>60s):** Fällt die Funkverbindung aus, schaltet der Slave auf **50% Notfall-Öffnung** oder übernimmt autonom über eigene Sensoren.

### 7. Stoßlüftungs-Timer (Intervall-Purge) & 3D Walzen-Drehwähler
* **Intervall-Stoßlüftung:** Ermöglicht periodisches Zwangs-Öffnen der Klappe (Intervall 5–60 min, Dauer 10–600 sec) zum schnellen Gasaustausch im Trockenzelt.
* **Animierte SVG-Sanduhr:** Visualisiert live das herabrieselnde Sandkorn (`@keyframes sand-pour`), dynamische Füllstände oben/unten, asynchron pulsierenden Sandstrahl-Glow (310ms / 190ms Primzahl-Farbflimmern) und die verbleibende Restzeit (`IN mm:ss` in Blau / `OFFEN mm:ss` in Rot).
* **3D Walzen-Drehwähler (Drum Pickers):** Touch- & PC-Maus-optimierte Walzen mit 2D Y-Stauchung (`scale(0.92, 0.65)`), leuchtendem Cyan-Glow (`#38bdf8`) und nativer Drag-to-Scroll PC-Maus-Steuerung (`cursor: grabbing`).
* **Strikte Slave-Isolation:** Im Slave-Modus ist der Timer ausgeblendet und in der Firmware deaktiviert, sodass der Slave 100% synchron zum Master läuft.

### 8. Interaktives Sprechblasen-Info-System (Integrierte Bedienhilfe)
* **Reines Text-Icon `ℹ`:** Dezente 15px Kreisschaltflächen rechts in allen Panel- & Einstellungs-Titeln.
* **100% Blickdichte Sprechblasen (`#090d16` & `z-index: 9999`):** Stark abgerundete Tiefschwarz-Boxen mit CSS-Dreiecksspitze zum Icon, Desktop-Hover, Mobile-Touch-Toggle und absolut blickdichter Deckkraft (kein Durchscheinen von Mond-Animationen oder Text).
* **HTML-Formatierung:** Unterstützt formatierten Text mit Tags (`<b>`, `<br>`, Listen) für 50–1000 Zeichen pro Panel.
* **Dictionary-Indizierung:** Strukturierte Zuordnung `PANEL_INFOS` (Index 0..13 auf dem Dashboard, 13..19 in den Einstellungen, Index 20 für den Grow Advisor).

### 9. Smart Live-Advisor & Heuristik-Ticker (Grower 1×1 Engine)
* **Full-Width Header-Widget & Non-Stop-Scroller:** Direkt unter dem Haupttitel mit animiertem 🧠-Icon, Wischgesten-Steuerung, `◀ 1 / X ▶` History-Zähler und kontinuierlichem Endlos-Laufband (Text läuft links heraus und rechts nahtlos wieder hinein).
* **Interaktive Volltext-Sprechblase (`.advisor-popup-bubble`):** Durch Antippen/Klicken auf das Laufband öffnet sich eine 100% blickdichte Sprechblase (`#090d16`) mit vollständigem Bericht, Farb-Badge, Zeitstempel, eigenem `✕`-Schließen-Button sowie integrierten `◀ Älter` / `Neuer ▶` Blätter-Buttons für alle 20 Ringpuffer-Einträge.
* **Feste Endanschläge & Dynamische Button-Ausblendung:**
  - Bei der neuesten Meldung (1) wird der `Neuer ▶` Button automatisch ausgeblendet.
  - Bei der ältesten Meldung im Puffer wird der `◀ Älter` Button automatisch ausgeblendet.
  - Kein versehentliches Überspringen oder verwirrendes Durchmischen von Alt-Meldungen.
* **10-Sekunden KI-Heuristik & Ganzzahlige Anti-Spam-Deduplizierung:**
  - Analysiert in Echtzeit alle 10s das Feuchteverhältnis zur gewählten Trockenstrategie (60/60, VPD, VPD AUTO mit 14-Tage-Stufenplan & Stängel-Knicktest).
  - Sensorwerte werden auf ganzzahlige bzw. quantisierte Stufen gerundet (z. B. `Math.round(rF)`), sodass normales Sensorrauschen keinen Spam im 20er-Ringpuffer erzeugt.
* **Authentischer Grow-Bro Disclaimer (Info-Button `ℹ`):**
  > *„Dies sind unverbindliche Tipps & Denkanstöße – nimm sie bitte nicht zu bierernst! Die Automatik regelt so gut es geht, aber kein Algorithmus kann dein gärtnerisches Feingefühl ersetzen. Jeder Grow, jedes Zelt und jedes Raumklima ist anders. Sieh die Tipps nicht als Panik-Alarm, sondern als Anregung zum Mitdenken und selber Recherchieren. Keine Gewähr auf dynamische Tipps – Happy Growing! 🌿✌️“*

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

