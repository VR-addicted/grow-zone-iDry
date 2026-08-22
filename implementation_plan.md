# Implementation Plan - Smart Live-Advisor, Heuristic Engine & Layout Symmetries (Build v155)

Comprehensive architecture and implementation plan for the iDRY-26 Dashboard.

---

## 🛠️ Architecture & Delivered Components

### 1. Zero-RAM Chunked PROGMEM HTTP Streaming
- **Heap RAM Protection:** Eliminates dynamic string allocations (`String html`) for the 50KB dashboard HTML.
- **PROGMEM Chunking:** HTML is partitioned into `DASHBOARD_HTML_PART1`, `DASHBOARD_HTML_PART2`, and `DASHBOARD_HTML_PART3`.
- **Chunked HTTP Response:** Handled via `server.setContentLength(CONTENT_LENGTH_UNKNOWN)` and `server.sendContent(...)` using 0 Bytes of dynamic RAM heap to guarantee immunity against truncation or memory crashes.

### 2. Full-Width Header Widget & Non-Stop Seamless Scroller
- **Placement:** Positioned prominently directly beneath `#device-title`.
- **Continuous Infinite Looping:** Active report scrolls continuously from right to left (`requestAnimationFrame` at 38px/s). When the text completely leaves the viewport on the left, it wraps seamlessly to the right edge and continues looping non-stop.
- **Gesture Drag & Swipe:** Pointer events distinguish between horizontal swipe gestures ($> 35\text{px}$) and static clicks/taps ($\le 6\text{px}$).

### 3. Interactive 100% Solid Full-Text Speech Bubble (`#advisor-popup-bubble`)
- **100% Opaque Pitch-Black Background (`#090d16` & `z-index: 9999`):** Completely eliminates background bleed-through of underlying animated rotor moons, sparklines, or icons.
- **Full Text Rendering:** Displays the entire diagnostic report, color-coded badge, and timestamp `[HH:MM:SS]` without truncation.
- **Embedded History Controls:** Contains `◀ Älter` and `Neuer ▶` buttons with a live counter (`X / Total`) and an explicit `✕` close button.

### 4. Navigation Boundary Stops & Dynamic Visibility
- **No Wrap-Around:** Stays fixed at message index 0 when navigating newer, and stays fixed at the oldest message when navigating older.
- **Dynamic Button Hiding:**
  - `Neuer ▶` automatically hides (`visibility: hidden`) when viewing message 1.
  - `◀ Älter` automatically hides (`visibility: hidden`) when viewing the oldest message.

### 5. 10-Second Discretized Climate Heuristics & Anti-Spam Deduplication
- **10s Evaluation Cycle:** Evaluates live sensor telemetry every 10 seconds.
- **Integer Quantization (Anti-Jitter):** Rounds raw sensor floats to whole integer numbers (`Math.round(rF)`, `Math.round(temp)`, `Math.round(rotorPos)`) and $0.1\text{ kPa}$ steps for VPD.
- **Anti-Spam Ringbuffer Deduplication:** Checks if the report text matches `advisorRingBuffer[0].rawText`. Only genuine climate changes and state transitions generate new ringbuffer entries with fresh timestamps.

### 6. Interactive Info Help System & Grow-Bro Disclaimer (`PANEL_INFOS`)
- Structured as a fast, sparse dictionary object `{ 0: ..., 20: ... }`.
- **Grow Advisor Disclaimer (Index 20):**
  > *„Dies sind unverbindliche Tipps & Denkanstöße – nimm sie bitte nicht zu bierernst! Die Automatik regelt so gut es geht, aber kein Algorithmus kann dein gärtnerisches Feingefühl ersetzen. Jeder Grow, jedes Zelt und jedes Raumklima ist anders. Sieh die Tipps nicht als Panik-Alarm, sondern als Anregung zum Mitdenken und selber Recherchieren. Keine Gewähr auf dynamische Tipps – Happy Growing! 🌿✌️“*

### 7. Restored MQTT Card Display in Slave Mode (Build v155)
- Displays MQTT Card symmetrically alongside ESPNOW whenever `data.mqtt_enabled`, `data.espnow_role > 0`, or an ESP-NOW peer MAC is stored in settings.
- Restores Broker, Status (`connected` / `try to connect` / `disconnected`), and Topic visibility without layout gaps.

---

## 🧪 Verification & Release Status

- **PlatformIO Compilation:** `SUCCESS` (Code 0, RAM: 29.6%, Flash: 21.7%).
- **Firmware Bundle:** `FIRMWARE/firmware.bin` (v155), `bootloader.bin`, `partitions.bin`, `version.txt` (v155).
- **Documentation:** `README.md`, `agents.md`, `TODO.md`, `walkthrough.md`, and `implementation_plan.md` fully synchronized.
