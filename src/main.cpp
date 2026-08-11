#include <Arduino.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <GxEPD2_3C.h>
#include <SPI.h>
#include <Wire.h>
#define LGFX_USE_V1
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <LovyanGFX.hpp>
#include <PubSubClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// OTA Firmware Update Libraries
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Update.h>
#include <WiFiClientSecure.h>

// Hardcoded Firmware Version (incremented on each release)
const int localFirmwareVersion = 31;

// Sensor Libraries
#include <Adafruit_BME280.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

// --- PIN DEFINITIONS ---
#define EPD_SCK 12
#define EPD_MOSI 11
#define EPD_MISO 13
#define EPD_CS 10
#define EPD_DC 9
#define EPD_RST 14
#define EPD_BUSY 8

// Poti Analog Pins
#define POTI_A_PIN 4 // Target Humidity
#define POTI_B_PIN 5 // Gain
#define POTI_C_PIN 1 // Servo Calibration Offset

// Servo Configuration
#define SERVO_PIN 18
#define SERVO_LEDC_CHANNEL 2

// Buzzer Configuration
#define BUZZER_PIN 17

// =====================================================================
// SELECT DRIVER CLASS (3-Colour driver class hacked to 240x360)
// =====================================================================
#define DRIVER_CLASS GxEPD2_213_Z19c

// Instantiate the working 3-colour display wrapper
GxEPD2_3C<DRIVER_CLASS, DRIVER_CLASS::HEIGHT>
    display(DRIVER_CLASS(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

// =====================================================================
// LOVYANGFX ILI9341 PANEL CONFIGURATION
// =====================================================================
class LGFX : public lgfx::LGFX_Device {
  lgfx::Bus_SPI _bus_instance;
  lgfx::Panel_ILI9341 _panel_instance;
  lgfx::Light_PWM _light_instance;

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read = 16000000;
      cfg.pin_sclk = EPD_SCK;
      cfg.pin_mosi = EPD_MOSI;
      cfg.pin_miso = EPD_MISO;
      cfg.pin_dc = EPD_DC;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs = EPD_CS;
      cfg.pin_rst = EPD_RST;
      cfg.pin_busy = -1;
      cfg.panel_width = 240;
      cfg.panel_height = 320;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = false; // Write-only module
      cfg.invert = false;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;
      _panel_instance.config(cfg);
    }
    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = EPD_BUSY;
      cfg.freq = 12000;
      cfg.pwm_channel = 1;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }
    setPanel(&_panel_instance);
  }
};

LGFX tft;
bool isTFTMode = false;
bool isHeadless = false;

// Global MQTT Client definitions (declared early for scope access)
WiFiClient espClient;
PubSubClient mqttClient(espClient);
String baseTopic = "";
String stateTopic = "";

// =====================================================================
// SYSTEM CONFIGURATION STRUCT & LITTLEFS
// =====================================================================
struct Config {
  char wifi_ssid[33] = "";
  char wifi_pass[65] = "";
  char mqtt_server[65] = "";
  int mqtt_port = 1883;
  char mqtt_user[65] = "";
  char mqtt_pass[65] = "";
  char mqtt_device_name[33] = "";
  int mqtt_report_interval = 5; // Publish interval in minutes (1 to 60)
  int display_brightness = 80;  // Display brightness percentage (0 to 100%)
  int wifi_tx_power =
      52; // WiFi TX Power limit (default 13dBm, WIFI_POWER_13dBm = 52)
  int espnow_role = 0;    // 0 = Disabled, 1 = Master, 2 = Slave
  int espnow_channel = 1; // Manual channel for Slave (1 to 13)
  char espnow_peer_mac[18] =
      ""; // Target controller MAC address (XX:XX:XX:XX:XX:XX)
  char espnow_lmk[33] =
      ""; // Local Master Key for hardware encryption (hex string)
  int servo_update_interval =
      5; // Servo update rate limit interval in seconds (1 to 30)
  int wlan_time_trap = 120; // WLAN connection watchdog timeout in seconds (0 =
                            // disabled, 1 to 330)
  int espnow_failsafe_mode = 0; // Slave fail-safe mode on connection loss: 0 =
                                // 50% Safety Open, 1 = Local Control
};

Config sysConfig;
bool isConfigLoaded = false;

// Potentiometer States
float potiAVal = 0.0;          // Target Humidity: 0 - 100%
float potiBVal = 0.0;          // Gain: 0 - 400%
float potiCVal = 0.0;          // Calibration Offset: 0 to 120 deg
float rotorPosition = 0.0;     // Logical Rotor opening: 0 - 100%
bool bypassModeActive = false; // Thermodynamic bypass (notschließen) is active

// Servo Motion Profiling (Ease-In-Ease-Out Softstart/Stop Ramping)
float targetServoAngle = 0.0f;
float startServoAngle = 0.0f;
float currentServoAngle = 0.0f;
unsigned long servoMoveStartTime = 0;
float servoMoveDuration = 0.0f; // in milliseconds
bool servoMoving = false;
bool servoFinishedPending = false;
unsigned long servoFinishedTime = 0;

// =====================================================================
// ESP-NOW & PAIRING STATE MACHINE & WI-FI CHANNEL HOPS
// =====================================================================
struct __attribute__((packed)) EspNowMessage {
  uint8_t pv;      // Protocol version (currently 1)
  uint8_t type;    // 0 = Pairing Beacon, 1 = Pairing Response, 2 = Command/Data
  char key[33];    // LMK hex string exchanged during pairing
  uint8_t command; // 0 = None, 1 = Play Winner Melody, 2 = Ping-Request, 3 =
                   // Ping-Reply
  float value;     // Numeric payload value
};

const uint8_t localProtocolVersion = 2;
uint8_t remoteProtocolVersion = 1;
bool protocolVersionMismatch = false;
uint32_t avgEspNowIntervalMs = 1000;

unsigned long lastEspNowRxTime = 0;
bool isPairingActive = false;
unsigned long pairingStartTime = 0;
int currentPairingChannel = 1;
int originalWifiChannel = 1;
unsigned long lastPairingBeaconTime = 0;
unsigned long lastChannelHopTime = 0;
#include <time.h>

char proposedLmk[33] = "";

struct HistorySample {
  float temp_0_min;
  float temp_0_max;
  float hum_0_min;
  float hum_0_max;
  float temp_1_min;
  float temp_1_max;
  float hum_1_min;
  float hum_1_max;
  float lux_0_max;
  float lux_1_max;
  float rotor_max;
  uint16_t espnow_loss_sec;
  uint16_t mqtt_loss_sec;
  int8_t rssi_min;
};

// 120-Minute (1-min resolution) and 24-Hour (5-min resolution) RAM Ring Buffers
const int HIST_120M_SIZE =
    120; // 120 samples x 1 minute = 120 minutes (2 hours)
HistorySample history120mBuffer[HIST_120M_SIZE];
int history120mCount = 0;
int history120mHead = 0;

const int HIST_24H_SIZE = 288; // 288 samples x 5 minutes = 24 hours
HistorySample history24hBuffer[HIST_24H_SIZE];
int history24hCount = 0;
int history24hHead = 0;

// 1-minute bucket accumulators
static float b1m_temp_0_min = NAN;
static float b1m_temp_0_max = NAN;
static float b1m_hum_0_min = NAN;
static float b1m_hum_0_max = NAN;
static float b1m_temp_1_min = NAN;
static float b1m_temp_1_max = NAN;
static float b1m_hum_1_min = NAN;
static float b1m_hum_1_max = NAN;
static float b1m_lux_0_max = 0.0f;
static float b1m_lux_1_max = 0.0f;
static float b1m_rotor_max = 0.0f;
static uint16_t b1m_espnow_loss_sec = 0;
static uint16_t b1m_mqtt_loss_sec = 0;
static int8_t b1m_rssi_min = 0;
static unsigned long last1mBucketTime = 0;

// 5-minute bucket accumulators
static float b5m_temp_0_min = NAN;
static float b5m_temp_0_max = NAN;
static float b5m_hum_0_min = NAN;
static float b5m_hum_0_max = NAN;
static float b5m_temp_1_min = NAN;
static float b5m_temp_1_max = NAN;
static float b5m_hum_1_min = NAN;
static float b5m_hum_1_max = NAN;
static float b5m_lux_0_max = 0.0f;
static float b5m_lux_1_max = 0.0f;
static float b5m_rotor_max = 0.0f;
static uint16_t b5m_espnow_loss_sec = 0;
static uint16_t b5m_mqtt_loss_sec = 0;
static int8_t b5m_rssi_min = 0;
static unsigned long last5mBucketTime = 0;

// Main Loop Benchmark Counter
static unsigned long loopCounter = 0;
static uint32_t loopsPerSecond = 0;
static unsigned long lastLoopBenchTime = 0;

void updateHistoryAccumulators1s();

// NTP & Weekly Watchdog Reset Helpers
static bool ntpInitialized = false;

String getWatchdogResetCountdown() {
  const unsigned long ONE_WEEK_MS = 604800000UL; // 7 days in ms
  unsigned long nowMs = millis();

  if (nowMs < ONE_WEEK_MS) {
    unsigned long msRemaining = ONE_WEEK_MS - nowMs;
    unsigned long totalSecs = msRemaining / 1000UL;
    unsigned long days = totalSecs / 86400UL;
    unsigned long hours = (totalSecs % 86400UL) / 3600UL;
    unsigned long mins = (totalSecs % 3600UL) / 60UL;
    unsigned long secs = totalSecs % 60UL;

    char buf[32];
    snprintf(buf, sizeof(buf), "%02luD - %02lu:%02lu:%02lu", days, hours, mins,
             secs);
    return String(buf);
  } else {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 10) && (timeinfo.tm_year >= 120)) {
      int curH = timeinfo.tm_hour;
      int curM = timeinfo.tm_min;
      int curS = timeinfo.tm_sec;
      int curSecOfDay = curH * 3600 + curM * 60 + curS;
      int targetSecOfDay = 3 * 3600; // 03:00:00 AM

      int diffSecs = (curSecOfDay < targetSecOfDay)
                         ? (targetSecOfDay - curSecOfDay)
                         : (24 * 3600 - curSecOfDay + targetSecOfDay);
      unsigned long hours = diffSecs / 3600;
      unsigned long mins = (diffSecs % 3600) / 60;
      unsigned long secs = diffSecs % 60;

      char buf[32];
      snprintf(buf, sizeof(buf), "00D - %02lu:%02lu:%02lu", hours, mins, secs);
      return String(buf);
    } else {
      return String("00D - 00:00:00");
    }
  }
}

void checkWeeklyWatchdogReset() {
  const unsigned long ONE_WEEK_MS = 604800000UL;
  if (millis() >= ONE_WEEK_MS) {
    struct tm timeinfo;
    bool hasNtp = getLocalTime(&timeinfo, 10) && (timeinfo.tm_year >= 120);

    if (!hasNtp) {
      Serial.println("[Watchdog] 1 week uptime reached without NTP. Triggering "
                     "weekly reset...");
      delay(500);
      ESP.restart();
    } else if (timeinfo.tm_hour == 3) {
      Serial.println("[Watchdog] 1 week uptime reached and 03:00 AM local time "
                     "reached. Triggering weekly reset...");
      delay(500);
      ESP.restart();
    }
  }
}

void playWinnerMelody() {
  Serial.println("[Buzzer] Playing winner melody...");
  int notes[] = {523, 587,  659,  698,  784,  880,  988,  1047, 1047,
                 988, 880,  784,  698,  659,  587,  523,  523,  659,
                 784, 1047, 1319, 1568, 2093, 2093, 1568, 1319, 1047,
                 784, 659,  523,  523,  587,  659,  698,  784,  880,
                 988, 1047, 1319, 1568, 2093, 2093};
  int durations[] = {60,  60, 60, 60, 60,  60, 60, 120, 60, 60, 60,
                     60,  60, 60, 60, 120, 60, 60, 60,  60, 60, 60,
                     150, 60, 60, 60, 60,  60, 60, 150, 50, 50, 50,
                     50,  50, 50, 50, 50,  50, 50, 100, 300};
  int length = sizeof(notes) / sizeof(notes[0]);
  for (int i = 0; i < length; i++) {
    tone(BUZZER_PIN, notes[i], durations[i]);
    delay(durations[i] + 15);
  }
  noTone(BUZZER_PIN);
}

// Forward declarations
bool saveConfiguration();
void initEspNow();

void onEspNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1],
          mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.printf("[ESP-NOW] Message sent to %s, status: %s\n", macStr,
                (status == ESP_NOW_SEND_SUCCESS) ? "SUCCESS" : "FAIL");
}

void onEspNowDataRecv(const uint8_t *mac_addr, const uint8_t *data,
                      int data_len) {
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1],
          mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  if (data_len < (int)sizeof(EspNowMessage)) {
    Serial.printf("[ESP-NOW] Packet too small from %s: %d bytes\n", macStr,
                  data_len);
    return;
  }

  EspNowMessage msg;
  memcpy(&msg, data, sizeof(EspNowMessage));

  // Track remote protocol version and update rx timestamp from paired peer or
  // during active pairing
  bool isFromPeer = (strlen(sysConfig.espnow_peer_mac) > 0 &&
                     strcasecmp(macStr, sysConfig.espnow_peer_mac) == 0);
  if (isFromPeer || isPairingActive) {
    if (lastEspNowRxTime != 0) {
      unsigned long diff = millis() - lastEspNowRxTime;
      if (diff < 5000) {
        avgEspNowIntervalMs = (avgEspNowIntervalMs * 3 + diff) / 4;
      }
    }
    lastEspNowRxTime =
        millis(); // Refresh RX timestamp for every received packet

    if (sysConfig.espnow_role == 2 && isFromPeer) {
      uint8_t curChan = 1;
      wifi_second_chan_t secondChan;
      esp_wifi_get_channel(&curChan, &secondChan);
      if (curChan > 0 && curChan != sysConfig.espnow_channel) {
        sysConfig.espnow_channel = curChan;
        saveConfiguration();
        Serial.printf(
            "[ESP-NOW] Slave locked and saved active Master channel %d!\n",
            curChan);
      }
    }

    remoteProtocolVersion = msg.pv;
    if (msg.pv != localProtocolVersion) {
      if (!protocolVersionMismatch) {
        Serial.printf(
            "[ESP-NOW] Protocol version mismatch! Local: %d, Remote: %d\n",
            localProtocolVersion, msg.pv);
        protocolVersionMismatch = true;
      }
    } else {
      protocolVersionMismatch = false;
    }
  }

  // 1. Pairing Beacon (Type 0) -> Received by Slave
  if (msg.type == 0 && sysConfig.espnow_role == 2 &&
      (isPairingActive || strlen(sysConfig.espnow_peer_mac) == 0)) {
    Serial.printf("[Pairing] Received Master beacon from %s on channel %d!\n",
                  macStr, currentPairingChannel);
    // Lock channel and peer details
    sysConfig.espnow_channel = currentPairingChannel;
    strlcpy(sysConfig.espnow_peer_mac, macStr,
            sizeof(sysConfig.espnow_peer_mac));
    strlcpy(sysConfig.espnow_lmk, msg.key, sizeof(sysConfig.espnow_lmk));

    // Send response back
    EspNowMessage response;
    response.pv = localProtocolVersion;
    response.type = 1; // Response
    memset(response.key, 0, sizeof(response.key));
    response.command = 0;
    response.value = 0;

    // Register temporary master peer info to reply
    esp_now_peer_info_t tempPeer;
    memset(&tempPeer, 0, sizeof(tempPeer));
    memcpy(tempPeer.peer_addr, mac_addr, 6);
    tempPeer.channel = currentPairingChannel;
    tempPeer.encrypt = false;
    if (esp_now_is_peer_exist(mac_addr)) {
      esp_now_del_peer(mac_addr);
    }
    esp_now_add_peer(&tempPeer);

    esp_now_send(mac_addr, (uint8_t *)&response, sizeof(EspNowMessage));
    delay(200);

    saveConfiguration();
    isPairingActive = false;
    lastEspNowRxTime = millis();

    // Play happy arpeggio
    tone(BUZZER_PIN, 523, 100);
    delay(120);
    tone(BUZZER_PIN, 659, 100);
    delay(120);
    tone(BUZZER_PIN, 784, 100);
    delay(120);
    tone(BUZZER_PIN, 1047, 300);

    initEspNow(); // Re-initialize peer
    Serial.println("[Pairing] Slave paired successfully!");
  }

  // 2. Pairing Response (Type 1) -> Received by Master
  else if (msg.type == 1 && sysConfig.espnow_role == 1 && isPairingActive) {
    Serial.printf("[Pairing] Received response from Slave %s!\n", macStr);
    strlcpy(sysConfig.espnow_peer_mac, macStr,
            sizeof(sysConfig.espnow_peer_mac));
    strlcpy(sysConfig.espnow_lmk, proposedLmk, sizeof(sysConfig.espnow_lmk));

    saveConfiguration();
    isPairingActive = false;
    lastEspNowRxTime = millis();

    // Play happy arpeggio
    tone(BUZZER_PIN, 523, 100);
    delay(120);
    tone(BUZZER_PIN, 659, 100);
    delay(120);
    tone(BUZZER_PIN, 784, 100);
    delay(120);
    tone(BUZZER_PIN, 1047, 300);

    initEspNow(); // Re-initialize peer
    Serial.println("[Pairing] Master paired successfully!");
  }

  // 3. Command/Data (Type 2)
  else if (msg.type == 2) {
    // Only accept commands from paired peer (case insensitive MAC check)
    if (strcasecmp(macStr, sysConfig.espnow_peer_mac) == 0) {
      lastEspNowRxTime = millis();
      Serial.printf("[ESP-NOW] Received command %d from peer %s\n", msg.command,
                    macStr);
      if (msg.command == 1) {
        playWinnerMelody();
      } else if (msg.command == 2) {
        if (sysConfig.espnow_role == 2) {
          rotorPosition = msg.value;
        }
        // Reply with Ping-Response (command 3)
        EspNowMessage response;
        response.pv = localProtocolVersion;
        response.type = 2;
        strlcpy(response.key, sysConfig.espnow_lmk, sizeof(response.key));
        response.command = 3; // Ping-Reply
        response.value = 0;
        esp_now_send(mac_addr, (uint8_t *)&response, sizeof(EspNowMessage));
      }
    } else {
      Serial.printf("[ESP-NOW] Blocked command from unpaired peer %s\n",
                    macStr);
    }
  }
}

static const uint8_t IDRY_PMK[16] = {0x69, 0x44, 0x72, 0x79, 0x32, 0x36,
                                     0x5F, 0x50, 0x4D, 0x4B, 0x5F, 0x53,
                                     0x45, 0x43, 0x52, 0x45};

void initEspNow() {
  if (sysConfig.espnow_role == 0) {
    esp_now_deinit();
    return;
  }

  // Safely reset driver before initializing
  esp_now_deinit();

  Serial.println("[ESP-NOW] Initializing ESP-NOW...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Initialization failed!");
    return;
  }

  // Set 16-byte Primary Master Key (PMK)
  esp_now_set_pmk(IDRY_PMK);

  esp_now_register_send_cb(onEspNowDataSent);
  esp_now_register_recv_cb(onEspNowDataRecv);

  // Lock Wi-Fi channel for Slave when connected
  if (sysConfig.espnow_role == 2 && strlen(sysConfig.espnow_peer_mac) > 0) {
    esp_wifi_set_channel(sysConfig.espnow_channel, WIFI_SECOND_CHAN_NONE);
  }

  // Register paired peer if stored
  if (strlen(sysConfig.espnow_peer_mac) > 0) {
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));

    int mac[6];
    sscanf(sysConfig.espnow_peer_mac, "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1],
           &mac[2], &mac[3], &mac[4], &mac[5]);
    for (int i = 0; i < 6; i++) {
      peerInfo.peer_addr[i] = (uint8_t)mac[i];
    }

    uint8_t activeChannel = 1;
    if (sysConfig.espnow_role == 1) {
      activeChannel = (WiFi.status() == WL_CONNECTED) ? WiFi.channel() : 1;
    } else {
      activeChannel = sysConfig.espnow_channel;
    }

    peerInfo.channel = activeChannel;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt =
        false; // Always use unencrypted ESP-NOW for 100% reliable connection

    if (esp_now_is_peer_exist(peerInfo.peer_addr)) {
      esp_now_del_peer(peerInfo.peer_addr);
    }
    esp_now_add_peer(&peerInfo);

    Serial.printf("[ESP-NOW] Registered peer %s on channel %d\n",
                  sysConfig.espnow_peer_mac, peerInfo.channel);

    // Immediate ping from Master to establish active rx state on Slave
    if (sysConfig.espnow_role == 1 && !isPairingActive) {
      EspNowMessage pingMsg;
      pingMsg.pv = localProtocolVersion;
      pingMsg.type = 2; // Data/Command
      strlcpy(pingMsg.key, sysConfig.espnow_lmk, sizeof(pingMsg.key));
      pingMsg.command = 2; // Ping-Request
      pingMsg.value = rotorPosition;
      esp_now_send(peerInfo.peer_addr, (uint8_t *)&pingMsg,
                   sizeof(EspNowMessage));
    }
  }
}

bool loadConfiguration() {
  if (!LittleFS.begin(true)) {
    Serial.println("[LittleFS] Mount Failed, formatting filesystem...");
    return false;
  }
  if (!LittleFS.exists("/config.json")) {
    Serial.println("[LittleFS] Configuration file not found.");
    return false;
  }
  File file = LittleFS.open("/config.json", "r");
  if (!file) {
    Serial.println("[LittleFS] Failed to open config file.");
    return false;
  }
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  if (error) {
    Serial.println("[LittleFS] Failed to parse config JSON.");
    return false;
  }
  strlcpy(sysConfig.wifi_ssid, doc["wifi_ssid"] | "",
          sizeof(sysConfig.wifi_ssid));
  strlcpy(sysConfig.wifi_pass, doc["wifi_pass"] | "",
          sizeof(sysConfig.wifi_pass));
  strlcpy(sysConfig.mqtt_server, doc["mqtt_server"] | "",
          sizeof(sysConfig.mqtt_server));
  sysConfig.mqtt_port = doc["mqtt_port"] | 1883;
  strlcpy(sysConfig.mqtt_user, doc["mqtt_user"] | "",
          sizeof(sysConfig.mqtt_user));
  strlcpy(sysConfig.mqtt_pass, doc["mqtt_pass"] | "",
          sizeof(sysConfig.mqtt_pass));
  strlcpy(sysConfig.mqtt_device_name, doc["mqtt_device_name"] | "",
          sizeof(sysConfig.mqtt_device_name));
  sysConfig.mqtt_report_interval = doc["mqtt_report_interval"] | 5;
  sysConfig.display_brightness = doc["display_brightness"] | 80;
  sysConfig.wifi_tx_power = doc["wifi_tx_power"] | 52;
  sysConfig.espnow_role = doc["espnow_role"] | 0;
  sysConfig.espnow_channel = doc["espnow_channel"] | 1;
  strlcpy(sysConfig.espnow_peer_mac, doc["espnow_peer_mac"] | "",
          sizeof(sysConfig.espnow_peer_mac));
  strlcpy(sysConfig.espnow_lmk, doc["espnow_lmk"] | "",
          sizeof(sysConfig.espnow_lmk));
  sysConfig.servo_update_interval = doc["servo_update_interval"] | 5;
  sysConfig.wlan_time_trap = doc["wlan_time_trap"] | 120;
  sysConfig.espnow_failsafe_mode = doc["espnow_failsafe_mode"] | 0;

  Serial.println("[LittleFS] Configuration successfully loaded.");
  return true;
}

bool saveConfiguration() {
  File file = LittleFS.open("/config.json", "w");
  if (!file) {
    Serial.println("[LittleFS] Failed to open config file for writing.");
    return false;
  }
  JsonDocument doc;
  doc["wifi_ssid"] = sysConfig.wifi_ssid;
  doc["wifi_pass"] = sysConfig.wifi_pass;
  doc["mqtt_server"] = sysConfig.mqtt_server;
  doc["mqtt_port"] = sysConfig.mqtt_port;
  doc["mqtt_user"] = sysConfig.mqtt_user;
  doc["mqtt_pass"] = sysConfig.mqtt_pass;
  doc["mqtt_device_name"] = sysConfig.mqtt_device_name;
  doc["mqtt_report_interval"] = sysConfig.mqtt_report_interval;
  doc["display_brightness"] = sysConfig.display_brightness;
  doc["wifi_tx_power"] = sysConfig.wifi_tx_power;
  doc["espnow_role"] = sysConfig.espnow_role;
  doc["espnow_channel"] = sysConfig.espnow_channel;
  doc["espnow_peer_mac"] = sysConfig.espnow_peer_mac;
  doc["espnow_lmk"] = sysConfig.espnow_lmk;
  doc["servo_update_interval"] = sysConfig.servo_update_interval;
  doc["wlan_time_trap"] = sysConfig.wlan_time_trap;
  doc["espnow_failsafe_mode"] = sysConfig.espnow_failsafe_mode;

  if (serializeJson(doc, file) == 0) {
    Serial.println("[LittleFS] Failed to serialize configuration JSON.");
    file.close();
    return false;
  }
  file.close();
  Serial.println("[LittleFS] Configuration successfully saved.");
  return true;
}

// Favicon PNG (32x32) - 4191 bytes
const uint8_t favicon_png[4191] PROGMEM = {
    0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x00, 0x00, 0x00, 0x0D,
    0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x20,
    0x08, 0x06, 0x00, 0x00, 0x00, 0x73, 0x7A, 0x7A, 0xF4, 0x00, 0x00, 0x00,
    0x09, 0x70, 0x48, 0x59, 0x73, 0x00, 0x00, 0x0B, 0x13, 0x00, 0x00, 0x0B,
    0x13, 0x01, 0x00, 0x9A, 0x9C, 0x18, 0x00, 0x00, 0x05, 0xC8, 0x69, 0x54,
    0x58, 0x74, 0x58, 0x4D, 0x4C, 0x3A, 0x63, 0x6F, 0x6D, 0x2E, 0x61, 0x64,
    0x6F, 0x62, 0x65, 0x2E, 0x78, 0x6D, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3C, 0x3F, 0x78, 0x70, 0x61, 0x63, 0x6B, 0x65, 0x74, 0x20, 0x62, 0x65,
    0x67, 0x69, 0x6E, 0x3D, 0x22, 0xEF, 0xBB, 0xBF, 0x22, 0x20, 0x69, 0x64,
    0x3D, 0x22, 0x57, 0x35, 0x4D, 0x30, 0x4D, 0x70, 0x43, 0x65, 0x68, 0x69,
    0x48, 0x7A, 0x72, 0x65, 0x53, 0x7A, 0x4E, 0x54, 0x63, 0x7A, 0x6B, 0x63,
    0x39, 0x64, 0x22, 0x3F, 0x3E, 0x20, 0x3C, 0x78, 0x3A, 0x78, 0x6D, 0x70,
    0x6D, 0x65, 0x74, 0x61, 0x20, 0x78, 0x6D, 0x6C, 0x6E, 0x73, 0x3A, 0x78,
    0x3D, 0x22, 0x61, 0x64, 0x6F, 0x62, 0x65, 0x3A, 0x6E, 0x73, 0x3A, 0x6D,
    0x65, 0x74, 0x61, 0x2F, 0x22, 0x20, 0x78, 0x3A, 0x78, 0x6D, 0x70, 0x74,
    0x6B, 0x3D, 0x22, 0x41, 0x64, 0x6F, 0x62, 0x65, 0x20, 0x58, 0x4D, 0x50,
    0x20, 0x43, 0x6F, 0x72, 0x65, 0x20, 0x36, 0x2E, 0x30, 0x2D, 0x63, 0x30,
    0x30, 0x36, 0x20, 0x37, 0x39, 0x2E, 0x31, 0x36, 0x34, 0x36, 0x34, 0x38,
    0x2C, 0x20, 0x32, 0x30, 0x32, 0x31, 0x2F, 0x30, 0x31, 0x2F, 0x31, 0x32,
    0x2D, 0x31, 0x35, 0x3A, 0x35, 0x32, 0x3A, 0x32, 0x39, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x20, 0x20, 0x22, 0x3E, 0x20, 0x3C, 0x72, 0x64, 0x66,
    0x3A, 0x52, 0x44, 0x46, 0x20, 0x78, 0x6D, 0x6C, 0x6E, 0x73, 0x3A, 0x72,
    0x64, 0x66, 0x3D, 0x22, 0x68, 0x74, 0x74, 0x70, 0x3A, 0x2F, 0x2F, 0x77,
    0x77, 0x77, 0x2E, 0x77, 0x33, 0x2E, 0x6F, 0x72, 0x67, 0x2F, 0x31, 0x39,
    0x39, 0x39, 0x2F, 0x30, 0x32, 0x2F, 0x32, 0x32, 0x2D, 0x72, 0x64, 0x66,
    0x2D, 0x73, 0x79, 0x6E, 0x74, 0x61, 0x78, 0x2D, 0x6E, 0x73, 0x23, 0x22,
    0x3E, 0x20, 0x3C, 0x72, 0x64, 0x66, 0x3A, 0x44, 0x65, 0x73, 0x63, 0x72,
    0x69, 0x70, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x72, 0x64, 0x66, 0x3A, 0x61,
    0x62, 0x6F, 0x75, 0x74, 0x3D, 0x22, 0x22, 0x20, 0x78, 0x6D, 0x6C, 0x6E,
    0x73, 0x3A, 0x78, 0x6D, 0x70, 0x3D, 0x22, 0x68, 0x74, 0x74, 0x70, 0x3A,
    0x2F, 0x2F, 0x6E, 0x73, 0x2E, 0x61, 0x64, 0x6F, 0x62, 0x65, 0x2E, 0x63,
    0x6F, 0x6D, 0x2F, 0x78, 0x61, 0x70, 0x2F, 0x31, 0x2E, 0x30, 0x2F, 0x22,
    0x20, 0x78, 0x6D, 0x6C, 0x6E, 0x73, 0x3A, 0x78, 0x6D, 0x70, 0x4D, 0x4D,
    0x3D, 0x22, 0x68, 0x74, 0x74, 0x70, 0x3A, 0x2F, 0x2F, 0x6E, 0x73, 0x2E,
    0x61, 0x64, 0x6F, 0x62, 0x65, 0x2E, 0x63, 0x6F, 0x6D, 0x2F, 0x78, 0x61,
    0x70, 0x2F, 0x31, 0x2E, 0x30, 0x2F, 0x6D, 0x6D, 0x2F, 0x22, 0x20, 0x78,
    0x6D, 0x6C, 0x6E, 0x73, 0x3A, 0x73, 0x74, 0x45, 0x76, 0x74, 0x3D, 0x22,
    0x68, 0x74, 0x74, 0x70, 0x3A, 0x2F, 0x2F, 0x6E, 0x73, 0x2E, 0x61, 0x64,
    0x6F, 0x62, 0x65, 0x2E, 0x63, 0x6F, 0x6D, 0x2F, 0x78, 0x61, 0x70, 0x2F,
    0x31, 0x2E, 0x30, 0x2F, 0x73, 0x54, 0x79, 0x70, 0x65, 0x2F, 0x52, 0x65,
    0x73, 0x6F, 0x75, 0x72, 0x63, 0x65, 0x45, 0x76, 0x65, 0x6E, 0x74, 0x23,
    0x22, 0x20, 0x78, 0x6D, 0x6C, 0x6E, 0x73, 0x3A, 0x64, 0x63, 0x3D, 0x22,
    0x68, 0x74, 0x74, 0x70, 0x3A, 0x2F, 0x2F, 0x70, 0x75, 0x72, 0x6C, 0x2E,
    0x6F, 0x72, 0x67, 0x2F, 0x64, 0x63, 0x2F, 0x65, 0x6C, 0x65, 0x6D, 0x65,
    0x6E, 0x74, 0x73, 0x2F, 0x31, 0x2E, 0x31, 0x2F, 0x22, 0x20, 0x78, 0x6D,
    0x6C, 0x6E, 0x73, 0x3A, 0x70, 0x68, 0x6F, 0x74, 0x6F, 0x73, 0x68, 0x6F,
    0x70, 0x3D, 0x22, 0x68, 0x74, 0x74, 0x70, 0x3A, 0x2F, 0x2F, 0x6E, 0x73,
    0x2E, 0x61, 0x64, 0x6F, 0x62, 0x65, 0x2E, 0x63, 0x6F, 0x6D, 0x2F, 0x70,
    0x68, 0x6F, 0x74, 0x6F, 0x73, 0x68, 0x6F, 0x70, 0x2F, 0x31, 0x2E, 0x30,
    0x2F, 0x22, 0x20, 0x78, 0x6D, 0x70, 0x3A, 0x43, 0x72, 0x65, 0x61, 0x74,
    0x6F, 0x72, 0x54, 0x6F, 0x6F, 0x6C, 0x3D, 0x22, 0x41, 0x64, 0x6F, 0x62,
    0x65, 0x20, 0x50, 0x68, 0x6F, 0x74, 0x6F, 0x73, 0x68, 0x6F, 0x70, 0x20,
    0x32, 0x32, 0x2E, 0x32, 0x20, 0x28, 0x57, 0x69, 0x6E, 0x64, 0x6F, 0x77,
    0x73, 0x29, 0x22, 0x20, 0x78, 0x6D, 0x70, 0x3A, 0x43, 0x72, 0x65, 0x61,
    0x74, 0x65, 0x44, 0x61, 0x74, 0x65, 0x3D, 0x22, 0x32, 0x30, 0x32, 0x36,
    0x2D, 0x30, 0x38, 0x2D, 0x31, 0x30, 0x54, 0x31, 0x39, 0x3A, 0x34, 0x37,
    0x3A, 0x34, 0x38, 0x2B, 0x30, 0x32, 0x3A, 0x30, 0x30, 0x22, 0x20, 0x78,
    0x6D, 0x70, 0x3A, 0x4D, 0x65, 0x74, 0x61, 0x64, 0x61, 0x74, 0x61, 0x44,
    0x61, 0x74, 0x65, 0x3D, 0x22, 0x32, 0x30, 0x32, 0x36, 0x2D, 0x30, 0x38,
    0x2D, 0x31, 0x30, 0x54, 0x31, 0x39, 0x3A, 0x34, 0x37, 0x3A, 0x34, 0x38,
    0x2B, 0x30, 0x32, 0x3A, 0x30, 0x30, 0x22, 0x20, 0x78, 0x6D, 0x70, 0x3A,
    0x4D, 0x6F, 0x64, 0x69, 0x66, 0x79, 0x44, 0x61, 0x74, 0x65, 0x3D, 0x22,
    0x32, 0x30, 0x32, 0x36, 0x2D, 0x30, 0x38, 0x2D, 0x31, 0x30, 0x54, 0x31,
    0x39, 0x3A, 0x34, 0x37, 0x3A, 0x34, 0x38, 0x2B, 0x30, 0x32, 0x3A, 0x30,
    0x30, 0x22, 0x20, 0x78, 0x6D, 0x70, 0x4D, 0x4D, 0x3A, 0x49, 0x6E, 0x73,
    0x74, 0x61, 0x6E, 0x63, 0x65, 0x49, 0x44, 0x3D, 0x22, 0x78, 0x6D, 0x70,
    0x2E, 0x69, 0x69, 0x64, 0x3A, 0x34, 0x64, 0x62, 0x33, 0x64, 0x33, 0x31,
    0x35, 0x2D, 0x64, 0x31, 0x33, 0x62, 0x2D, 0x64, 0x64, 0x34, 0x39, 0x2D,
    0x38, 0x39, 0x31, 0x64, 0x2D, 0x32, 0x61, 0x65, 0x62, 0x33, 0x64, 0x62,
    0x30, 0x36, 0x36, 0x34, 0x30, 0x22, 0x20, 0x78, 0x6D, 0x70, 0x4D, 0x4D,
    0x3A, 0x44, 0x6F, 0x63, 0x75, 0x6D, 0x65, 0x6E, 0x74, 0x49, 0x44, 0x3D,
    0x22, 0x61, 0x64, 0x6F, 0x62, 0x65, 0x3A, 0x64, 0x6F, 0x63, 0x69, 0x64,
    0x3A, 0x70, 0x68, 0x6F, 0x74, 0x6F, 0x73, 0x68, 0x6F, 0x70, 0x3A, 0x35,
    0x64, 0x33, 0x37, 0x61, 0x35, 0x35, 0x34, 0x2D, 0x31, 0x61, 0x38, 0x38,
    0x2D, 0x35, 0x35, 0x34, 0x37, 0x2D, 0x39, 0x63, 0x39, 0x37, 0x2D, 0x36,
    0x66, 0x62, 0x33, 0x38, 0x33, 0x38, 0x30, 0x33, 0x65, 0x33, 0x33, 0x22,
    0x20, 0x78, 0x6D, 0x70, 0x4D, 0x4D, 0x3A, 0x4F, 0x72, 0x69, 0x67, 0x69,
    0x6E, 0x61, 0x6C, 0x44, 0x6F, 0x63, 0x75, 0x6D, 0x65, 0x6E, 0x74, 0x49,
    0x44, 0x3D, 0x22, 0x78, 0x6D, 0x70, 0x2E, 0x64, 0x69, 0x64, 0x3A, 0x63,
    0x31, 0x34, 0x62, 0x32, 0x64, 0x39, 0x39, 0x2D, 0x36, 0x62, 0x66, 0x62,
    0x2D, 0x35, 0x64, 0x34, 0x33, 0x2D, 0x61, 0x31, 0x36, 0x32, 0x2D, 0x36,
    0x36, 0x31, 0x62, 0x36, 0x32, 0x64, 0x33, 0x33, 0x35, 0x64, 0x31, 0x22,
    0x20, 0x64, 0x63, 0x3A, 0x66, 0x6F, 0x72, 0x6D, 0x61, 0x74, 0x3D, 0x22,
    0x69, 0x6D, 0x61, 0x67, 0x65, 0x2F, 0x70, 0x6E, 0x67, 0x22, 0x20, 0x70,
    0x68, 0x6F, 0x74, 0x6F, 0x73, 0x68, 0x6F, 0x70, 0x3A, 0x43, 0x6F, 0x6C,
    0x6F, 0x72, 0x4D, 0x6F, 0x64, 0x65, 0x3D, 0x22, 0x33, 0x22, 0x3E, 0x20,
    0x3C, 0x78, 0x6D, 0x70, 0x4D, 0x4D, 0x3A, 0x48, 0x69, 0x73, 0x74, 0x6F,
    0x72, 0x79, 0x3E, 0x20, 0x3C, 0x72, 0x64, 0x66, 0x3A, 0x53, 0x65, 0x71,
    0x3E, 0x20, 0x3C, 0x72, 0x64, 0x66, 0x3A, 0x6C, 0x69, 0x20, 0x73, 0x74,
    0x45, 0x76, 0x74, 0x3A, 0x61, 0x63, 0x74, 0x69, 0x6F, 0x6E, 0x3D, 0x22,
    0x63, 0x72, 0x65, 0x61, 0x74, 0x65, 0x64, 0x22, 0x20, 0x73, 0x74, 0x45,
    0x76, 0x74, 0x3A, 0x69, 0x6E, 0x73, 0x74, 0x61, 0x6E, 0x63, 0x65, 0x49,
    0x44, 0x3D, 0x22, 0x78, 0x6D, 0x70, 0x2E, 0x69, 0x69, 0x64, 0x3A, 0x63,
    0x31, 0x34, 0x62, 0x32, 0x64, 0x39, 0x39, 0x2D, 0x36, 0x62, 0x66, 0x62,
    0x2D, 0x35, 0x64, 0x34, 0x33, 0x2D, 0x61, 0x31, 0x36, 0x32, 0x2D, 0x36,
    0x36, 0x31, 0x62, 0x36, 0x32, 0x64, 0x33, 0x33, 0x35, 0x64, 0x31, 0x22,
    0x20, 0x73, 0x74, 0x45, 0x76, 0x74, 0x3A, 0x77, 0x68, 0x65, 0x6E, 0x3D,
    0x22, 0x32, 0x30, 0x32, 0x36, 0x2D, 0x30, 0x38, 0x2D, 0x31, 0x30, 0x54,
    0x31, 0x39, 0x3A, 0x34, 0x37, 0x3A, 0x34, 0x38, 0x2B, 0x30, 0x32, 0x3A,
    0x30, 0x30, 0x22, 0x20, 0x73, 0x74, 0x45, 0x76, 0x74, 0x3A, 0x73, 0x6F,
    0x66, 0x74, 0x77, 0x61, 0x72, 0x65, 0x41, 0x67, 0x65, 0x6E, 0x74, 0x3D,
    0x22, 0x41, 0x64, 0x6F, 0x62, 0x65, 0x20, 0x50, 0x68, 0x6F, 0x74, 0x6F,
    0x73, 0x68, 0x6F, 0x70, 0x20, 0x32, 0x32, 0x2E, 0x32, 0x20, 0x28, 0x57,
    0x69, 0x6E, 0x64, 0x6F, 0x77, 0x73, 0x29, 0x22, 0x2F, 0x3E, 0x20, 0x3C,
    0x72, 0x64, 0x66, 0x3A, 0x6C, 0x69, 0x20, 0x73, 0x74, 0x45, 0x76, 0x74,
    0x3A, 0x61, 0x63, 0x74, 0x69, 0x6F, 0x6E, 0x3D, 0x22, 0x73, 0x61, 0x76,
    0x65, 0x64, 0x22, 0x20, 0x73, 0x74, 0x45, 0x76, 0x74, 0x3A, 0x69, 0x6E,
    0x73, 0x74, 0x61, 0x6E, 0x63, 0x65, 0x49, 0x44, 0x3D, 0x22, 0x78, 0x6D,
    0x70, 0x2E, 0x69, 0x69, 0x64, 0x3A, 0x34, 0x64, 0x62, 0x33, 0x64, 0x33,
    0x31, 0x35, 0x2D, 0x64, 0x31, 0x33, 0x62, 0x2D, 0x64, 0x64, 0x34, 0x39,
    0x2D, 0x38, 0x39, 0x31, 0x64, 0x2D, 0x32, 0x61, 0x65, 0x62, 0x33, 0x64,
    0x62, 0x30, 0x36, 0x36, 0x34, 0x30, 0x22, 0x20, 0x73, 0x74, 0x45, 0x76,
    0x74, 0x3A, 0x77, 0x68, 0x65, 0x6E, 0x3D, 0x22, 0x32, 0x30, 0x32, 0x36,
    0x2D, 0x30, 0x38, 0x2D, 0x31, 0x30, 0x54, 0x31, 0x39, 0x3A, 0x34, 0x37,
    0x3A, 0x34, 0x38, 0x2B, 0x30, 0x32, 0x3A, 0x30, 0x30, 0x22, 0x20, 0x73,
    0x74, 0x45, 0x76, 0x74, 0x3A, 0x73, 0x6F, 0x66, 0x74, 0x77, 0x61, 0x72,
    0x65, 0x41, 0x67, 0x65, 0x6E, 0x74, 0x3D, 0x22, 0x41, 0x64, 0x6F, 0x62,
    0x65, 0x20, 0x50, 0x68, 0x6F, 0x74, 0x6F, 0x73, 0x68, 0x6F, 0x70, 0x20,
    0x32, 0x32, 0x2E, 0x32, 0x20, 0x28, 0x57, 0x69, 0x6E, 0x64, 0x6F, 0x77,
    0x73, 0x29, 0x22, 0x20, 0x73, 0x74, 0x45, 0x76, 0x74, 0x3A, 0x63, 0x68,
    0x61, 0x6E, 0x67, 0x65, 0x64, 0x3D, 0x22, 0x2F, 0x22, 0x2F, 0x3E, 0x20,
    0x3C, 0x2F, 0x72, 0x64, 0x66, 0x3A, 0x53, 0x65, 0x71, 0x3E, 0x20, 0x3C,
    0x2F, 0x78, 0x6D, 0x70, 0x4D, 0x4D, 0x3A, 0x48, 0x69, 0x73, 0x74, 0x6F,
    0x72, 0x79, 0x3E, 0x20, 0x3C, 0x2F, 0x72, 0x64, 0x66, 0x3A, 0x44, 0x65,
    0x73, 0x63, 0x72, 0x69, 0x70, 0x74, 0x69, 0x6F, 0x6E, 0x3E, 0x20, 0x3C,
    0x2F, 0x72, 0x64, 0x66, 0x3A, 0x52, 0x44, 0x46, 0x3E, 0x20, 0x3C, 0x2F,
    0x78, 0x3A, 0x78, 0x6D, 0x70, 0x6D, 0x65, 0x74, 0x61, 0x3E, 0x20, 0x3C,
    0x3F, 0x78, 0x70, 0x61, 0x63, 0x6B, 0x65, 0x74, 0x20, 0x65, 0x6E, 0x64,
    0x3D, 0x22, 0x72, 0x22, 0x3F, 0x3E, 0x4C, 0xBF, 0xE0, 0xF2, 0x00, 0x00,
    0x0A, 0x3D, 0x49, 0x44, 0x41, 0x54, 0x58, 0x85, 0x8D, 0x97, 0x6B, 0x8C,
    0x5D, 0xD7, 0x55, 0xC7, 0x7F, 0x6B, 0xEF, 0x73, 0xEE, 0xB9, 0xEF, 0x99,
    0xB9, 0xF3, 0x76, 0xEC, 0xD8, 0x63, 0xBB, 0xC6, 0x8F, 0x49, 0x4D, 0xEC,
    0x38, 0x36, 0x6D, 0x9C, 0x30, 0x69, 0x4A, 0x1A, 0x15, 0x5A, 0x89, 0xD2,
    0x96, 0x48, 0xAD, 0x44, 0x0B, 0x42, 0xAA, 0x4A, 0x5A, 0x89, 0x52, 0xA9,
    0xA0, 0x42, 0x89, 0xA8, 0xA8, 0x04, 0x08, 0x91, 0x8A, 0xF0, 0x0D, 0x09,
    0x51, 0xD4, 0x5A, 0x0D, 0x0A, 0x51, 0x54, 0x22, 0x8A, 0x1A, 0x88, 0xA3,
    0x3A, 0x25, 0x89, 0xE3, 0xC4, 0xF1, 0xF8, 0x39, 0x8E, 0x3D, 0xF6, 0x8C,
    0x67, 0xC6, 0xF3, 0xBE, 0x33, 0xF7, 0x71, 0xEE, 0x3D, 0x67, 0xEF, 0xC5,
    0x07, 0x3F, 0xE2, 0xB1, 0x1D, 0xC4, 0xD2, 0xF9, 0x70, 0xB4, 0xB7, 0xB4,
    0xFE, 0xFF, 0xFD, 0xDF, 0xEB, 0xB5, 0xE5, 0xE9, 0xA7, 0x9F, 0xEE, 0x07,
    0xAE, 0x02, 0x45, 0xEE, 0x6E, 0x06, 0xD5, 0xC4, 0xE5, 0x0B, 0x4D, 0x0D,
    0x02, 0xA2, 0xC5, 0x85, 0xED, 0xD1, 0xDC, 0xCC, 0xE7, 0x32, 0x4B, 0x4B,
    0x0F, 0xDB, 0x7A, 0x6D, 0xC8, 0xB4, 0xE3, 0x2E, 0xF1, 0x1A, 0x68, 0x18,
    0xAE, 0xBA, 0x6C, 0xFE, 0x6A, 0xDA, 0xD1, 0xF1, 0x4E, 0xAB, 0xA7, 0xF7,
    0xA5, 0xB8, 0x77, 0xE0, 0x85, 0x34, 0xCA, 0xA4, 0x41, 0xBD, 0x16, 0x4A,
    0xEA, 0x72, 0x88, 0xF8, 0xDB, 0xFC, 0xD6, 0x80, 0xFE, 0x00, 0xA8, 0xDF,
    0xB2, 0xB0, 0xD6, 0x54, 0xD1, 0x20, 0x24, 0x2D, 0x14, 0xC9, 0xCE, 0x5F,
    0xED, 0x2F, 0x9C, 0x3F, 0xF7, 0xDD, 0xEC, 0xF4, 0x95, 0x2F, 0x85, 0xF5,
    0x55, 0x6B, 0x1D, 0x88, 0x09, 0x31, 0x26, 0xC4, 0x88, 0xA0, 0xBE, 0x51,
    0xF2, 0x7E, 0x6E, 0x5D, 0x8A, 0xBB, 0xDF, 0x67, 0xA2, 0x2F, 0xB5, 0x7B,
    0x7A, 0x4F, 0x36, 0x36, 0x0E, 0xFD, 0x69, 0x6D, 0x68, 0xCB, 0xBF, 0xB9,
    0x0C, 0x49, 0xD0, 0xA8, 0x83, 0xC8, 0xED, 0x08, 0xF5, 0xE0, 0x03, 0x4E,
    0x8D, 0xA8, 0xC7, 0x65, 0xF3, 0x68, 0x10, 0xD2, 0x79, 0xF2, 0x9D, 0xEF,
    0x15, 0x4F, 0x1E, 0xFF, 0x96, 0x6D, 0x34, 0x28, 0xD8, 0x22, 0xA9, 0x2D,
    0x31, 0x99, 0x0B, 0x69, 0x67, 0x63, 0x24, 0x74, 0x18, 0x9B, 0x27, 0x48,
    0x1D, 0xC5, 0x38, 0x43, 0x47, 0xAC, 0x44, 0x89, 0xD2, 0x9C, 0x9A, 0xDE,
    0x95, 0x99, 0x9E, 0x7C, 0x3E, 0x3F, 0x31, 0x7E, 0x71, 0x69, 0xEF, 0xFE,
    0x27, 0xE2, 0x72, 0xE7, 0xD9, 0x70, 0xA5, 0x7A, 0x07, 0x89, 0xBB, 0x13,
    0x50, 0x4F, 0x9A, 0xCB, 0x83, 0x18, 0xE9, 0xFE, 0xC5, 0x2B, 0x87, 0x73,
    0xE7, 0xCF, 0x1E, 0xCC, 0x06, 0x05, 0xB2, 0x51, 0x17, 0xEF, 0x56, 0x0C,
    0xA7, 0x07, 0x33, 0xEC, 0xE8, 0x5A, 0x62, 0x63, 0xA1, 0x3C, 0x7E, 0xD8,
    0xF4, 0x5F, 0x7D, 0x2B, 0x9D, 0xD9, 0xDF, 0xE9, 0x43, 0x72, 0x6D, 0xA5,
    0xAB, 0x0E, 0x1B, 0xE7, 0x3D, 0x3B, 0x67, 0x8A, 0x94, 0x63, 0xA5, 0x36,
    0x31, 0x31, 0xD4, 0xB7, 0xB8, 0x70, 0x66, 0xF1, 0xA1, 0x91, 0xCF, 0xD4,
    0x06, 0xD6, 0x3D, 0x1F, 0x56, 0x97, 0xD7, 0x90, 0xB0, 0x23, 0x23, 0x23,
    0x19, 0xA0, 0x7D, 0xAB, 0xEC, 0x3E, 0x13, 0x41, 0x10, 0xD2, 0xF3, 0xDA,
    0xE1, 0x57, 0x73, 0x17, 0xCE, 0x1D, 0x2C, 0x86, 0x1D, 0xD4, 0x8B, 0x19,
    0x7E, 0xB2, 0x23, 0xE4, 0xF0, 0x8E, 0x88, 0x91, 0xC1, 0x65, 0x3E, 0x9A,
    0x0D, 0x57, 0xE2, 0xCC, 0xAF, 0x6F, 0x3D, 0x97, 0xC4, 0x9B, 0x17, 0x92,
    0x89, 0x8F, 0x64, 0x4C, 0x44, 0x3B, 0x23, 0xCC, 0x74, 0x1A, 0x8E, 0xF5,
    0x07, 0x9C, 0xA8, 0x58, 0x24, 0x51, 0x36, 0x35, 0x23, 0x6C, 0xAB, 0x49,
    0x66, 0xE2, 0xE2, 0xE7, 0xD3, 0xBE, 0x81, 0x37, 0xE3, 0x4A, 0xF7, 0x98,
    0x6D, 0xC5, 0x37, 0x48, 0x64, 0xCC, 0x9D, 0xDA, 0x0B, 0x3E, 0x5F, 0xA0,
    0xE3, 0xF8, 0x5B, 0xFF, 0x98, 0x7D, 0xEF, 0xDC, 0xC1, 0xB2, 0xED, 0x64,
    0xBA, 0x3B, 0xE4, 0x9F, 0x3F, 0x92, 0xE1, 0xD5, 0xA1, 0x90, 0xCF, 0x68,
    0x9D, 0x4F, 0xD5, 0x6A, 0x9C, 0x77, 0x8F, 0x3E, 0xFE, 0x7A, 0x2B, 0x4C,
    0xC7, 0xE3, 0xD1, 0x6F, 0x94, 0x88, 0x08, 0x9C, 0x12, 0x26, 0x4A, 0x50,
    0x83, 0xBD, 0x8D, 0x94, 0xE1, 0xAE, 0x94, 0x93, 0x07, 0x0C, 0x2F, 0x6C,
    0x0F, 0xA8, 0x87, 0x65, 0x72, 0x2D, 0x47, 0xE5, 0xC8, 0x2B, 0x2F, 0x46,
    0xB5, 0x95, 0xED, 0x69, 0x2E, 0x0F, 0xAA, 0x77, 0xB9, 0x02, 0x55, 0xD2,
    0x52, 0x99, 0xFC, 0xE5, 0xF1, 0xDF, 0x2C, 0x9C, 0x19, 0xFD, 0x72, 0xC9,
    0x96, 0xB9, 0xD2, 0x1B, 0xF2, 0xC3, 0x03, 0x21, 0xAB, 0x06, 0x76, 0xAC,
    0xA4, 0xEC, 0x0E, 0x9B, 0xCC, 0x11, 0xE0, 0x28, 0x9F, 0xAF, 0xBA, 0xA9,
    0x4F, 0xC6, 0x1A, 0x53, 0x96, 0x0E, 0x14, 0xA5, 0x8A, 0x61, 0x9F, 0x69,
    0xF3, 0xDB, 0x41, 0x8D, 0x7C, 0xAB, 0xC6, 0xAA, 0xB5, 0x1C, 0x1D, 0x0E,
    0x18, 0xB5, 0x39, 0xEE, 0x3B, 0xD9, 0x41, 0x6E, 0x69, 0xDE, 0x76, 0x1D,
    0x3F, 0xF6, 0xEC, 0xEC, 0x47, 0x1F, 0xF9, 0x98, 0xB6, 0x0C, 0xA2, 0xCA,
    0x1A, 0x05, 0xD4, 0x5A, 0x4C, 0xBB, 0x4D, 0xF1, 0xCC, 0xE8, 0x9F, 0x17,
    0x9A, 0x9E, 0x5A, 0x39, 0xE2, 0xC5, 0x3D, 0x21, 0x2D, 0xA3, 0xF4, 0xD5,
    0x3C, 0x55, 0x31, 0x4C, 0x6A, 0x86, 0x6E, 0x9A, 0x74, 0xBB, 0xD3, 0x5F,
    0xB7, 0x26, 0x93, 0x8A, 0x04, 0x28, 0x4A, 0x0A, 0x64, 0x51, 0x1E, 0xB2,
    0x31, 0x59, 0xAD, 0x71, 0xD6, 0xEE, 0x78, 0x71, 0xDC, 0x7E, 0x76, 0xCB,
    0xDE, 0xE4, 0xB1, 0x83, 0xDB, 0xEF, 0xBB, 0xE7, 0xBF, 0xCF, 0x0F, 0xA5,
    0x10, 0x94, 0x88, 0x26, 0xC6, 0x1F, 0x2D, 0x5E, 0x1E, 0xFF, 0xAC, 0xCB,
    0xE5, 0x00, 0xD6, 0x12, 0xF0, 0x51, 0x96, 0x68, 0x7E, 0xF6, 0x60, 0xB4,
    0xB0, 0x70, 0x9F, 0x8D, 0x4A, 0xFC, 0x64, 0x9B, 0x67, 0xA2, 0xD0, 0xA6,
    0xB7, 0x2E, 0x18, 0x03, 0x06, 0xE5, 0xC7, 0xAE, 0xC0, 0x31, 0xED, 0x62,
    0xD8, 0xBD, 0xFE, 0xED, 0xED, 0xCC, 0x1D, 0xC0, 0x0C, 0x5E, 0x70, 0x9A,
    0x90, 0x60, 0xE8, 0x44, 0x29, 0xF9, 0x25, 0x16, 0xCD, 0xA6, 0xC3, 0x53,
    0x99, 0x4F, 0x7D, 0xFA, 0xB8, 0xE6, 0x16, 0x0E, 0xB5, 0xF3, 0x1C, 0x6B,
    0xF4, 0x8E, 0xBD, 0xB5, 0xCD, 0x32, 0x53, 0xB1, 0x14, 0x6B, 0x29, 0xD9,
    0xA9, 0xC9, 0x2F, 0x8A, 0x18, 0x10, 0x71, 0x77, 0x66, 0x81, 0x6A, 0x26,
    0x6A, 0xC1, 0x64, 0x9F, 0x23, 0xBF, 0x71, 0xFD, 0xA9, 0xF5, 0xCD, 0x7A,
    0xAE, 0xCE, 0xEA, 0x50, 0x96, 0x88, 0x12, 0xCA, 0x22, 0x86, 0x7F, 0x70,
    0x9D, 0x34, 0xB4, 0xC5, 0x88, 0x39, 0xF5, 0xF9, 0xA3, 0x66, 0x9D, 0x19,
    0xF3, 0x82, 0x00, 0x25, 0x49, 0xE9, 0x33, 0x11, 0xA3, 0xF6, 0xC3, 0x7F,
    0x7D, 0xA4, 0xF9, 0xC6, 0xF7, 0xE7, 0xDA, 0x6F, 0x3F, 0xE5, 0x01, 0x6D,
    0x86, 0xD4, 0x3B, 0x72, 0x9C, 0xD9, 0x28, 0x6C, 0x9E, 0x32, 0x80, 0xE4,
    0xC5, 0x7B, 0x14, 0xF4, 0xB6, 0x20, 0x54, 0xD4, 0x18, 0xDA, 0xDE, 0xE2,
    0xFB, 0x6B, 0x7C, 0xA2, 0xD0, 0xFF, 0xAF, 0xC3, 0xC1, 0xFE, 0x3F, 0xA8,
    0x6B, 0x9D, 0x96, 0xB6, 0xF0, 0x08, 0x15, 0x3C, 0x06, 0xE5, 0x59, 0x3F,
    0xC8, 0xE9, 0x34, 0xDE, 0xF4, 0xB8, 0x4C, 0x6E, 0x50, 0x22, 0xEA, 0x6A,
    0xF8, 0x25, 0xA9, 0x23, 0xC1, 0x86, 0xE3, 0x2F, 0xB6, 0x67, 0x9E, 0xBA,
    0xD4, 0x3E, 0xFA, 0x54, 0xD6, 0xE4, 0x29, 0x9A, 0x22, 0x25, 0x13, 0xD1,
    0x19, 0x2B, 0xEF, 0xF5, 0x18, 0xE6, 0x4A, 0x42, 0xE4, 0x04, 0xAE, 0x7D,
    0xDC, 0x46, 0x40, 0xB0, 0x4E, 0x6D, 0x35, 0x6B, 0x68, 0x77, 0x1A, 0x82,
    0xF8, 0xC2, 0x13, 0xEB, 0xC2, 0xA1, 0x97, 0x1E, 0xC8, 0x3D, 0xF2, 0xD5,
    0x04, 0xCB, 0x82, 0xB6, 0x98, 0x57, 0x87, 0xA8, 0x92, 0xC3, 0xF3, 0x03,
    0x57, 0x96, 0x0B, 0x5E, 0x4D, 0x49, 0x3C, 0xEB, 0x24, 0xE5, 0x7E, 0x23,
    0xFC, 0x28, 0x71, 0x5B, 0x2E, 0xA6, 0x53, 0x23, 0x3D, 0xA6, 0x8C, 0xC5,
    0xA0, 0x80, 0x47, 0xC8, 0xA6, 0xB0, 0x92, 0x17, 0xA6, 0x3A, 0x0D, 0xD9,
    0x44, 0xAF, 0x1F, 0xF7, 0xB6, 0x2C, 0x10, 0x04, 0xD2, 0xA4, 0xB2, 0x92,
    0x53, 0x66, 0x73, 0x25, 0xF6, 0xB5, 0x2E, 0xEE, 0x5B, 0x92, 0x9F, 0x7F,
    0x6D, 0x67, 0xB8, 0xE7, 0xFB, 0x83, 0xA6, 0x74, 0x7C, 0x21, 0xBD, 0xFC,
    0x87, 0x89, 0x5F, 0xF8, 0xB8, 0xD3, 0x5A, 0x69, 0x49, 0x53, 0xA6, 0x35,
    0xE4, 0x67, 0xBE, 0x40, 0x88, 0xD2, 0x23, 0x9E, 0xFF, 0xF0, 0x79, 0x8E,
    0xFA, 0xB4, 0xD8, 0x27, 0x4A, 0x82, 0x61, 0x15, 0x83, 0x01, 0x02, 0x14,
    0xAF, 0x90, 0x1A, 0xA5, 0x5A, 0x10, 0x7C, 0xE2, 0x8A, 0x5E, 0x95, 0x00,
    0xDC, 0x6D, 0x04, 0x40, 0x7D, 0x5A, 0x91, 0x8C, 0xE7, 0xA4, 0xCD, 0xF3,
    0x88, 0x8B, 0x18, 0xD6, 0x57, 0x9F, 0x99, 0x4E, 0x4F, 0x7D, 0x77, 0x8F,
    0x64, 0xCF, 0x94, 0x25, 0x0D, 0x97, 0x4C, 0x2A, 0xB3, 0x9A, 0xF2, 0x96,
    0xB7, 0xCC, 0x6A, 0x40, 0x88, 0xF8, 0x48, 0x8C, 0xCE, 0xA9, 0xDA, 0x33,
    0x3E, 0xC2, 0x21, 0xA4, 0xA2, 0x74, 0xE0, 0xD9, 0x23, 0x31, 0xBB, 0xA4,
    0x45, 0x45, 0x94, 0xD7, 0x34, 0xCF, 0x7F, 0x69, 0x44, 0x2B, 0x34, 0xB4,
    0x5D, 0x5A, 0xF1, 0xEA, 0x10, 0x24, 0xB9, 0x23, 0x08, 0x05, 0x24, 0x2B,
    0xCA, 0xA2, 0x0A, 0xCF, 0xFB, 0x4E, 0x9E, 0xA4, 0x4A, 0xC9, 0x2F, 0x95,
    0xC6, 0xB0, 0xFB, 0x4E, 0x69, 0x8E, 0x71, 0x8D, 0xA8, 0x92, 0xA7, 0x8D,
    0x41, 0x51, 0x22, 0x31, 0x2A, 0x18, 0xDF, 0x50, 0xB1, 0x5B, 0x4D, 0xC2,
    0x83, 0xD2, 0xA2, 0x83, 0x94, 0x01, 0x49, 0xE8, 0xA7, 0x85, 0xE2, 0x08,
    0x35, 0x61, 0xB3, 0x94, 0xB8, 0x24, 0x7D, 0x2C, 0x5D, 0xBB, 0x96, 0x9B,
    0xB8, 0x77, 0x66, 0x81, 0x98, 0x66, 0x26, 0x85, 0x5E, 0xEF, 0x18, 0x95,
    0x90, 0x67, 0x7D, 0x85, 0x92, 0x38, 0x26, 0x35, 0x60, 0x15, 0x43, 0x1E,
    0xC5, 0xA0, 0x84, 0x28, 0x03, 0x28, 0xCB, 0x9A, 0xDA, 0x86, 0x8A, 0x4D,
    0xC4, 0xE2, 0xD5, 0x91, 0x8A, 0x22, 0x26, 0xEF, 0xA6, 0xA5, 0xF3, 0xF8,
    0x15, 0x3B, 0xF8, 0x5C, 0xC6, 0xF4, 0xFD, 0x18, 0x7F, 0xE9, 0xF7, 0x86,
    0x93, 0x37, 0xFF, 0x78, 0x9F, 0x59, 0xE5, 0x92, 0x13, 0xAC, 0x9A, 0xAA,
    0x11, 0x83, 0x82, 0x5D, 0x43, 0x40, 0x51, 0x08, 0xC2, 0xE9, 0x72, 0x2C,
    0xE4, 0xDB, 0x4A, 0xBE, 0xE4, 0x59, 0x6E, 0x1B, 0x16, 0xD4, 0x90, 0xC3,
    0xD3, 0x8F, 0x23, 0xC5, 0xB1, 0x48, 0xC6, 0xFF, 0x8E, 0x6D, 0xCA, 0xB2,
    0x4F, 0xE5, 0x39, 0x5F, 0x66, 0x9F, 0x89, 0x59, 0x55, 0xCF, 0x45, 0x32,
    0x8C, 0xFB, 0x10, 0xBC, 0xB5, 0xA1, 0xB8, 0x2D, 0x39, 0x37, 0xFB, 0x85,
    0xCD, 0x61, 0x69, 0x7C, 0x6B, 0xF4, 0xC4, 0x9F, 0x4C, 0xE1, 0xC3, 0x1E,
    0x3D, 0xFA, 0x47, 0xCD, 0xB8, 0x84, 0x95, 0x60, 0xD9, 0x88, 0x01, 0x34,
    0x5C, 0x5B, 0x09, 0x01, 0x6F, 0x6D, 0xBB, 0xD4, 0x54, 0x7A, 0x57, 0x3D,
    0xCD, 0x50, 0x28, 0xE0, 0x29, 0xE1, 0x09, 0x11, 0x1C, 0x8E, 0x79, 0xD5,
    0x78, 0x24, 0x28, 0x9D, 0x7B, 0x40, 0x5A, 0xBC, 0x4B, 0x86, 0x54, 0x84,
    0x01, 0x71, 0x7C, 0xCE, 0x56, 0x11, 0x85, 0x0C, 0xD0, 0x43, 0x42, 0x51,
    0xEB, 0x1D, 0x89, 0x9B, 0xD9, 0xF5, 0x76, 0xE3, 0xA7, 0x3F, 0x1A, 0x6B,
    0xBD, 0xF1, 0x15, 0x13, 0x6D, 0xFF, 0x7B, 0x9B, 0x16, 0xE8, 0x5A, 0x49,
    0x48, 0x43, 0xE3, 0x40, 0xD0, 0x3B, 0xD3, 0x10, 0x3C, 0x8A, 0xF7, 0xCA,
    0xBD, 0x0B, 0x4A, 0x20, 0x86, 0x96, 0xA6, 0xD4, 0x7D, 0x83, 0xAA, 0x5F,
    0xA1, 0xEA, 0x57, 0xD9, 0x19, 0xED, 0xFE, 0xBB, 0x4F, 0x07, 0xF6, 0xC8,
    0x98, 0xAF, 0xC9, 0xA4, 0x46, 0xF4, 0xD3, 0xE6, 0x75, 0xED, 0x4C, 0xBB,
    0x24, 0x5A, 0xFD, 0x9A, 0x9D, 0xA1, 0xAD, 0x96, 0x45, 0x2C, 0x16, 0x21,
    0x2F, 0x39, 0x3A, 0x6C, 0x27, 0xE7, 0xE2, 0x77, 0xBE, 0xF7, 0x9A, 0xBD,
    0xF4, 0xD5, 0xC5, 0xA5, 0x8E, 0x95, 0xC1, 0x45, 0x47, 0x2B, 0x54, 0x40,
    0xD7, 0xD6, 0x01, 0x11, 0xC1, 0x3B, 0x87, 0x4B, 0x53, 0x56, 0x23, 0x61,
    0xEB, 0x74, 0xCA, 0xBD, 0x0B, 0x2D, 0x5A, 0xA5, 0xEE, 0xF3, 0xFD, 0xC1,
    0xB6, 0x43, 0x1B, 0xC3, 0xA1, 0x97, 0xF7, 0xE6, 0x7F, 0x63, 0xDF, 0xC3,
    0xE1, 0xE0, 0x33, 0xA5, 0xF4, 0xF4, 0xEF, 0x5E, 0xD2, 0x02, 0x09, 0x90,
    0x05, 0x96, 0x7D, 0x2B, 0xF8, 0xA9, 0x6C, 0xF9, 0x9F, 0xFD, 0xD6, 0x5E,
    0xF9, 0xBA, 0x9D, 0xA1, 0x8C, 0x67, 0x19, 0x83, 0xA0, 0x84, 0x84, 0x58,
    0xE3, 0x3A, 0x5E, 0x8A, 0xCF, 0x7E, 0xD3, 0x8F, 0xBB, 0x72, 0xB7, 0x82,
    0x33, 0xD7, 0xE5, 0xBE, 0x95, 0x40, 0x9A, 0xA6, 0xE4, 0x4B, 0xA5, 0x9D,
    0x9B, 0xEE, 0x59, 0xF7, 0x8C, 0xB7, 0x8A, 0x89, 0x1D, 0xC3, 0x67, 0xDB,
    0x8C, 0x04, 0xAE, 0xF5, 0x64, 0x69, 0xD3, 0x37, 0x7F, 0x25, 0xF7, 0xE8,
    0x63, 0xC3, 0x41, 0xCF, 0xD1, 0xEE, 0xF6, 0xCB, 0x87, 0x9C, 0x5F, 0xA5,
    0x2A, 0x59, 0x54, 0xAF, 0x9D, 0x23, 0xAB, 0x2B, 0x9C, 0xD7, 0xE2, 0xE9,
    0x37, 0xED, 0xFE, 0xBF, 0xD9, 0x2D, 0x55, 0xBE, 0x6C, 0x97, 0x29, 0xE2,
    0x69, 0x22, 0x88, 0x2A, 0x2B, 0xA5, 0x90, 0xDD, 0xD3, 0xC2, 0xAE, 0xE9,
    0x84, 0x95, 0x9C, 0x41, 0xF4, 0x7D, 0xC5, 0x03, 0x20, 0x08, 0x82, 0xE0,
    0x5E, 0x63, 0x24, 0x9F, 0xCF, 0x66, 0xFE, 0x36, 0x9F, 0x86, 0xBB, 0x5A,
    0x62, 0x59, 0xCA, 0x79, 0xB6, 0x4D, 0x07, 0x5C, 0x39, 0x71, 0x75, 0xD7,
    0xEC, 0x2F, 0xFF, 0xFB, 0xF8, 0x87, 0xDB, 0xC5, 0xE9, 0x4C, 0x7B, 0xA5,
    0x52, 0x64, 0x25, 0x3F, 0x2D, 0x15, 0xC6, 0x5C, 0x48, 0xE1, 0xBA, 0xA7,
    0x40, 0x02, 0x52, 0x3F, 0x7B, 0xE0, 0x8A, 0x7D, 0xF8, 0x1B, 0x59, 0x37,
    0xF6, 0x95, 0x9D, 0xFE, 0xF2, 0xB6, 0xDD, 0x92, 0xE3, 0x15, 0x97, 0xA3,
    0x56, 0xF6, 0x0C, 0x54, 0x3D, 0x8F, 0x8F, 0xA6, 0xA4, 0x16, 0x12, 0x55,
    0xA2, 0x28, 0x2A, 0x1A, 0x6B, 0x51, 0x55, 0x17, 0x00, 0x1B, 0xE2, 0xB8,
    0x71, 0xD8, 0x39, 0xD7, 0xC5, 0x4A, 0x95, 0x66, 0x3B, 0xC6, 0x98, 0x6B,
    0xBD, 0xDA, 0xE5, 0x3D, 0x3D, 0x67, 0x33, 0x9C, 0x30, 0x91, 0x3D, 0xB9,
    0x63, 0x75, 0xFD, 0x26, 0x3C, 0xA5, 0xB8, 0x9B, 0xD7, 0x34, 0xC7, 0x3C,
    0x01, 0x65, 0x1C, 0x20, 0x24, 0x9A, 0x52, 0xC0, 0x5E, 0x15, 0xD3, 0x99,
    0x22, 0xB9, 0x95, 0xB6, 0x3A, 0x66, 0x4D, 0xC0, 0x62, 0x49, 0xD8, 0xB9,
    0xAC, 0xFC, 0xD6, 0x1B, 0x6D, 0xF2, 0xB1, 0xB2, 0x54, 0x14, 0xEC, 0x8A,
    0x12, 0x04, 0x81, 0x35, 0x46, 0x48, 0x15, 0x0D, 0x54, 0x75, 0xCA, 0x39,
    0x7D, 0x4C, 0x95, 0x9E, 0xEA, 0xCA, 0xEA, 0x83, 0x0D, 0xCB, 0xB7, 0x32,
    0x41, 0x50, 0x10, 0xE7, 0x88, 0x8D, 0x12, 0xA4, 0x4D, 0xF6, 0xBD, 0x6D,
    0x79, 0xBD, 0xD1, 0xC9, 0xF3, 0xDB, 0x03, 0x66, 0x8A, 0x42, 0xA9, 0xE5,
    0x19, 0x4C, 0x1C, 0x28, 0x78, 0x1C, 0xA1, 0x64, 0x18, 0x8C, 0xF6, 0x7E,
    0x7B, 0x93, 0xBF, 0xB0, 0x6D, 0x93, 0xBC, 0xF7, 0xC0, 0xA1, 0x42, 0x3F,
    0x6F, 0xF8, 0x0C, 0xBF, 0x7A, 0x39, 0xE1, 0xB1, 0xD1, 0x84, 0x5C, 0x4B,
    0x59, 0x2A, 0x08, 0xE6, 0xFA, 0x60, 0xAE, 0x7A, 0x7D, 0x1C, 0x12, 0x24,
    0x10, 0x91, 0x56, 0x14, 0x65, 0x8F, 0x01, 0x60, 0xCD, 0x7F, 0x9A, 0x8C,
    0xFD, 0xA4, 0x09, 0xCC, 0x81, 0xF6, 0x62, 0x8B, 0xDE, 0x9E, 0x0A, 0x5D,
    0xF7, 0x0F, 0xD2, 0x5C, 0x6A, 0xF1, 0xD0, 0xE8, 0x1C, 0xC3, 0xB3, 0x96,
    0x13, 0x5B, 0xB2, 0xBC, 0xD7, 0x67, 0x58, 0xCC, 0x0B, 0x6D, 0x2B, 0xC4,
    0xEA, 0xE8, 0x30, 0x7D, 0xE7, 0x36, 0x67, 0x27, 0x3F, 0xAE, 0xED, 0x5F,
    0xFC, 0xC5, 0x73, 0xED, 0x22, 0x63, 0x53, 0x65, 0xBE, 0x38, 0x11, 0x73,
    0xDF, 0xB4, 0x23, 0x0E, 0xB8, 0x06, 0x7E, 0xFD, 0xDE, 0xE5, 0x46, 0x1F,
    0xD2, 0xF7, 0x63, 0xA0, 0xD7, 0x18, 0x9E, 0x05, 0x36, 0x54, 0x2A, 0x5D,
    0x1F, 0x2A, 0x26, 0xF1, 0xFA, 0x6A, 0xB5, 0x4E, 0xDF, 0x40, 0x17, 0x67,
    0x2B, 0x59, 0x46, 0x67, 0x27, 0xD9, 0xD2, 0x5F, 0x66, 0xD7, 0x83, 0x03,
    0xE4, 0x4E, 0xCC, 0x31, 0x72, 0x22, 0xE6, 0xFE, 0x92, 0x65, 0xA6, 0xD3,
    0xB0, 0x50, 0x10, 0x1A, 0xA1, 0x60, 0xCC, 0xF2, 0xD6, 0x38, 0x1D, 0xFF,
    0xAB, 0x0B, 0xCD, 0xD0, 0xE4, 0xAB, 0x45, 0x9E, 0xAC, 0x36, 0xC8, 0x7A,
    0x65, 0x29, 0x27, 0x78, 0xC3, 0x4D, 0xF0, 0x1B, 0xB5, 0x86, 0xEB, 0x7D,
    0x0F, 0x20, 0x50, 0xD5, 0x7C, 0xAB, 0xD5, 0xFE, 0x35, 0xAF, 0x7E, 0xC0,
    0x37, 0x1B, 0x24, 0xED, 0x98, 0x8E, 0x4C, 0x96, 0xCB, 0x95, 0x1C, 0x63,
    0xE9, 0x19, 0x1E, 0x18, 0x80, 0xF9, 0x78, 0x8A, 0xC5, 0xF5, 0x1F, 0x62,
    0xBD, 0x1B, 0x64, 0xFA, 0xEC, 0x04, 0xF9, 0x44, 0xD8, 0x32, 0xE5, 0xD9,
    0xEE, 0xAF, 0x4D, 0x14, 0x0E, 0x35, 0x0D, 0x9F, 0xC3, 0x02, 0x36, 0x6C,
    0xD2, 0x8C, 0x0C, 0x0D, 0x23, 0x88, 0xB2, 0x26, 0xE2, 0x6F, 0xA6, 0x3C,
    0x72, 0x23, 0xFB, 0x34, 0x10, 0x91, 0x4B, 0x61, 0x10, 0x0E, 0x22, 0xC2,
    0x72, 0x75, 0xE5, 0xE9, 0xF9, 0xEA, 0xD2, 0x9F, 0xAD, 0x8B, 0x0A, 0x54,
    0xFA, 0x62, 0x9E, 0xDA, 0xB7, 0x9B, 0xCD, 0xDD, 0xFD, 0xFC, 0xFC, 0xDD,
    0x19, 0xA2, 0xBE, 0x12, 0xB5, 0x85, 0x45, 0x7C, 0x4B, 0x49, 0x0A, 0x42,
    0x12, 0x80, 0xDC, 0xE2, 0x12, 0x20, 0xBD, 0xE5, 0xFF, 0x6E, 0xC0, 0x37,
    0xEA, 0x4D, 0x3B, 0x69, 0x27, 0xDE, 0x79, 0x84, 0x1B, 0x4C, 0x44, 0x10,
    0x11, 0x72, 0xB9, 0xE2, 0x77, 0x62, 0xE7, 0xBF, 0x33, 0x37, 0x7D, 0x95,
    0x2D, 0xC5, 0x3C, 0x99, 0xBD, 0xBF, 0xCF, 0xCF, 0x96, 0xF7, 0xE1, 0x87,
    0x9F, 0xA4, 0x7B, 0xF3, 0x27, 0xA8, 0x5F, 0x9E, 0xC5, 0x44, 0x77, 0x77,
    0xBC, 0xD6, 0x3E, 0x00, 0xFD, 0x06, 0x81, 0xB8, 0x55, 0xF7, 0xDE, 0x23,
    0x22, 0xEF, 0x37, 0x23, 0x55, 0x25, 0xB4, 0x06, 0x53, 0x28, 0x1E, 0x91,
    0x30, 0x65, 0xFC, 0xC8, 0x2A, 0xE5, 0x8E, 0x23, 0x78, 0x3B, 0xC9, 0xCE,
    0xF2, 0x26, 0x4E, 0x1D, 0x9A, 0x60, 0x75, 0x26, 0x25, 0xEA, 0xCC, 0xDD,
    0xC4, 0x50, 0xB9, 0x55, 0x85, 0xFF, 0x87, 0x89, 0x20, 0x69, 0x8A, 0x81,
    0xAA, 0x5A, 0x03, 0x20, 0x6B, 0x07, 0x92, 0x56, 0x8B, 0x56, 0x57, 0xE5,
    0xE5, 0xE2, 0x86, 0x7B, 0x8E, 0xC5, 0xA3, 0x67, 0xF6, 0xB4, 0xFF, 0x65,
    0x9E, 0xCA, 0x50, 0x2F, 0x87, 0x0F, 0xBD, 0x43, 0xB2, 0x1A, 0x93, 0xEB,
    0xEA, 0x80, 0xD4, 0xDD, 0x7C, 0x54, 0xDC, 0x40, 0xFF, 0x60, 0x12, 0xB7,
    0xEC, 0x88, 0x20, 0xAD, 0x18, 0x23, 0xD0, 0x58, 0xBF, 0xE1, 0x07, 0xEA,
    0x15, 0x54, 0x6D, 0x00, 0x14, 0xB8, 0xF6, 0x32, 0x2E, 0x8A, 0x4B, 0x8D,
    0x0B, 0xC3, 0x95, 0xC5, 0xBD, 0x07, 0xBE, 0x50, 0x0E, 0xA3, 0x7F, 0xB2,
    0x4B, 0x4B, 0x3B, 0xAE, 0xCC, 0xA5, 0x89, 0xE4, 0x3A, 0x9C, 0xED, 0xEA,
    0x26, 0x11, 0xEE, 0x54, 0xF7, 0xFF, 0x54, 0xE1, 0xE6, 0x8E, 0x88, 0xF7,
    0x19, 0x0A, 0x45, 0xB7, 0xBC, 0x6D, 0xFB, 0x5F, 0x36, 0xEE, 0xD9, 0xF0,
    0x42, 0xD0, 0xA8, 0x03, 0x14, 0xFE, 0x17, 0xB6, 0x1A, 0xE6, 0xEF, 0xC7,
    0xEC, 0x8D, 0x55, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4E, 0x44, 0xAE,
    0x42, 0x60, 0x82};

// Forward declarations
void handleFavicon();
void handleFirmwarePage();
void handleAutoUpdate();
void handleAutoUpdateApi();
void handleUploadProgress();
void handleUploadFinish();
void publishMqttState();

// =====================================================================
// SENSOR CONFIGURATIONS
// =====================================================================
struct TempSensor {
  enum Type { TYPE_NONE, TYPE_BME280, TYPE_SHT3X } type = TYPE_NONE;
  uint8_t address = 0;
  float temperature = NAN;
  float humidity = NAN;
  float pressure = NAN; // BME280 only
  bool active = false;

  Adafruit_BME280 *bme = nullptr;
  Adafruit_SHT31 *sht = nullptr;
};

TempSensor tempSensors[2];
int detectedTempSensors = 0;

// Light Sensor State
struct LightSensor {
  uint8_t address = 0;
  float lux = NAN;
  uint16_t broadband = 0;
  uint16_t ir = 0;
  bool active = false;
  Adafruit_TSL2561_Unified *tsl = nullptr;
};

LightSensor lightSensors[2];
int detectedLightSensors = 0;

void updateHistoryAccumulators1s() {
  if (tempSensors[0].active && !isnan(tempSensors[0].temperature)) {
    if (isnan(b1m_temp_0_min) || tempSensors[0].temperature < b1m_temp_0_min)
      b1m_temp_0_min = tempSensors[0].temperature;
    if (isnan(b1m_temp_0_max) || tempSensors[0].temperature > b1m_temp_0_max)
      b1m_temp_0_max = tempSensors[0].temperature;

    if (isnan(b1m_hum_0_min) || tempSensors[0].humidity < b1m_hum_0_min)
      b1m_hum_0_min = tempSensors[0].humidity;
    if (isnan(b1m_hum_0_max) || tempSensors[0].humidity > b1m_hum_0_max)
      b1m_hum_0_max = tempSensors[0].humidity;

    if (isnan(b5m_temp_0_min) || tempSensors[0].temperature < b5m_temp_0_min)
      b5m_temp_0_min = tempSensors[0].temperature;
    if (isnan(b5m_temp_0_max) || tempSensors[0].temperature > b5m_temp_0_max)
      b5m_temp_0_max = tempSensors[0].temperature;

    if (isnan(b5m_hum_0_min) || tempSensors[0].humidity < b5m_hum_0_min)
      b5m_hum_0_min = tempSensors[0].humidity;
    if (isnan(b5m_hum_0_max) || tempSensors[0].humidity > b5m_hum_0_max)
      b5m_hum_0_max = tempSensors[0].humidity;
  }
  if (tempSensors[1].active && !isnan(tempSensors[1].temperature)) {
    if (isnan(b1m_temp_1_min) || tempSensors[1].temperature < b1m_temp_1_min)
      b1m_temp_1_min = tempSensors[1].temperature;
    if (isnan(b1m_temp_1_max) || tempSensors[1].temperature > b1m_temp_1_max)
      b1m_temp_1_max = tempSensors[1].temperature;

    if (isnan(b1m_hum_1_min) || tempSensors[1].humidity < b1m_hum_1_min)
      b1m_hum_1_min = tempSensors[1].humidity;
    if (isnan(b1m_hum_1_max) || tempSensors[1].humidity > b1m_hum_1_max)
      b1m_hum_1_max = tempSensors[1].humidity;

    if (isnan(b5m_temp_1_min) || tempSensors[1].temperature < b5m_temp_1_min)
      b5m_temp_1_min = tempSensors[1].temperature;
    if (isnan(b5m_temp_1_max) || tempSensors[1].temperature > b5m_temp_1_max)
      b5m_temp_1_max = tempSensors[1].temperature;

    if (isnan(b5m_hum_1_min) || tempSensors[1].humidity < b5m_hum_1_min)
      b5m_hum_1_min = tempSensors[1].humidity;
    if (isnan(b5m_hum_1_max) || tempSensors[1].humidity > b5m_hum_1_max)
      b5m_hum_1_max = tempSensors[1].humidity;
  }
  if (lightSensors[0].active && !isnan(lightSensors[0].lux)) {
    if (lightSensors[0].lux > b1m_lux_0_max)
      b1m_lux_0_max = lightSensors[0].lux;
    if (lightSensors[0].lux > b5m_lux_0_max)
      b5m_lux_0_max = lightSensors[0].lux;
  }
  if (lightSensors[1].active && !isnan(lightSensors[1].lux)) {
    if (lightSensors[1].lux > b1m_lux_1_max)
      b1m_lux_1_max = lightSensors[1].lux;
    if (lightSensors[1].lux > b5m_lux_1_max)
      b5m_lux_1_max = lightSensors[1].lux;
  }
  if (rotorPosition > b1m_rotor_max)
    b1m_rotor_max = rotorPosition;
  if (rotorPosition > b5m_rotor_max)
    b5m_rotor_max = rotorPosition;

  if (sysConfig.espnow_role > 0 &&
      (lastEspNowRxTime == 0 || (millis() - lastEspNowRxTime > 3000))) {
    b1m_espnow_loss_sec++;
    b5m_espnow_loss_sec++;
  }
  if (strlen(sysConfig.mqtt_server) > 0 && !mqttClient.connected()) {
    b1m_mqtt_loss_sec++;
    b5m_mqtt_loss_sec++;
  }
  int currentRssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -100;
  if (b1m_rssi_min == 0 || currentRssi < b1m_rssi_min)
    b1m_rssi_min = (int8_t)currentRssi;
  if (b5m_rssi_min == 0 || currentRssi < b5m_rssi_min)
    b5m_rssi_min = (int8_t)currentRssi;

  // 1-minute bucket commit for 60m history
  if (last1mBucketTime == 0) {
    last1mBucketTime = millis();
  } else if (millis() - last1mBucketTime >= 60000UL) {
    last1mBucketTime = millis();
    HistorySample s;
    s.temp_0_min = b1m_temp_0_min;
    s.temp_0_max = b1m_temp_0_max;
    s.hum_0_min = b1m_hum_0_min;
    s.hum_0_max = b1m_hum_0_max;
    s.temp_1_min = b1m_temp_1_min;
    s.temp_1_max = b1m_temp_1_max;
    s.hum_1_min = b1m_hum_1_min;
    s.hum_1_max = b1m_hum_1_max;
    s.lux_0_max = b1m_lux_0_max;
    s.lux_1_max = b1m_lux_1_max;
    s.rotor_max = b1m_rotor_max;
    s.espnow_loss_sec = b1m_espnow_loss_sec;
    s.mqtt_loss_sec = b1m_mqtt_loss_sec;
    s.rssi_min = b1m_rssi_min;

    history120mBuffer[history120mHead] = s;
    history120mHead = (history120mHead + 1) % HIST_120M_SIZE;
    if (history120mCount < HIST_120M_SIZE)
      history120mCount++;

    b1m_temp_0_min = NAN;
    b1m_temp_0_max = NAN;
    b1m_hum_0_min = NAN;
    b1m_hum_0_max = NAN;
    b1m_temp_1_min = NAN;
    b1m_temp_1_max = NAN;
    b1m_hum_1_min = NAN;
    b1m_hum_1_max = NAN;
    b1m_lux_0_max = 0.0f;
    b1m_lux_1_max = 0.0f;
    b1m_rotor_max = 0.0f;
    b1m_espnow_loss_sec = 0;
    b1m_mqtt_loss_sec = 0;
    b1m_rssi_min = (int8_t)currentRssi;
  }

  // 5-minute bucket commit for 24h history
  if (last5mBucketTime == 0) {
    last5mBucketTime = millis();
  } else if (millis() - last5mBucketTime >= 300000UL) {
    last5mBucketTime = millis();
    HistorySample s;
    s.temp_0_min = b5m_temp_0_min;
    s.temp_0_max = b5m_temp_0_max;
    s.hum_0_min = b5m_hum_0_min;
    s.hum_0_max = b5m_hum_0_max;
    s.temp_1_min = b5m_temp_1_min;
    s.temp_1_max = b5m_temp_1_max;
    s.hum_1_min = b5m_hum_1_min;
    s.hum_1_max = b5m_hum_1_max;
    s.lux_0_max = b5m_lux_0_max;
    s.lux_1_max = b5m_lux_1_max;
    s.rotor_max = b5m_rotor_max;
    s.espnow_loss_sec = b5m_espnow_loss_sec;
    s.mqtt_loss_sec = b5m_mqtt_loss_sec;
    s.rssi_min = b5m_rssi_min;

    history24hBuffer[history24hHead] = s;
    history24hHead = (history24hHead + 1) % HIST_24H_SIZE;
    if (history24hCount < HIST_24H_SIZE)
      history24hCount++;

    b5m_temp_0_min = NAN;
    b5m_temp_0_max = NAN;
    b5m_hum_0_min = NAN;
    b5m_hum_0_max = NAN;
    b5m_temp_1_min = NAN;
    b5m_temp_1_max = NAN;
    b5m_hum_1_min = NAN;
    b5m_hum_1_max = NAN;
    b5m_lux_0_max = 0.0f;
    b5m_lux_1_max = 0.0f;
    b5m_rotor_max = 0.0f;
    b5m_espnow_loss_sec = 0;
    b5m_mqtt_loss_sec = 0;
    b5m_rssi_min = (int8_t)currentRssi;
  }
}

// =====================================================================
// HELPER CALCULATIONS
// =====================================================================
float calculateDewPoint(float temp, float hum) {
  if (isnan(temp) || isnan(hum))
    return NAN;
  const float b = 17.67f;
  const float c = 243.5f;
  float gamma = (b * temp) / (c + temp) + log(hum / 100.0f);
  return (c * gamma) / (b - gamma);
}

float calculateVPD(float temp, float hum) {
  if (isnan(temp) || isnan(hum))
    return NAN;
  float svp = 0.61078f * exp((17.27f * temp) / (temp + 237.3f));
  float avp = svp * (hum / 100.0f);
  return svp - avp;
}

void scanI2C() {
  Wire.begin(15, 16);
  Serial.println("[I2C] Scanning bus on SDA=15, SCL=16...");

  // 1. Scan for temperature/humidity sensors (BME280: 0x76, 0x77 | SHT3x: 0x44,
  // 0x45)
  uint8_t tempAddresses[] = {0x76, 0x77, 0x44, 0x45};
  for (uint8_t addr : tempAddresses) {
    if (detectedTempSensors >= 2)
      break; // Max 2 sensors

    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (addr == 0x76 || addr == 0x77) {
        Adafruit_BME280 *bme = new Adafruit_BME280();
        if (bme->begin(addr, &Wire)) {
          Serial.printf("[I2C] BME280 initialized at address 0x%02X\n", addr);
          tempSensors[detectedTempSensors].type = TempSensor::TYPE_BME280;
          tempSensors[detectedTempSensors].address = addr;
          tempSensors[detectedTempSensors].bme = bme;
          tempSensors[detectedTempSensors].active = true;
          detectedTempSensors++;
        } else {
          delete bme;
        }
      } else if (addr == 0x44 || addr == 0x45) {
        Adafruit_SHT31 *sht = new Adafruit_SHT31();
        if (sht->begin(addr)) {
          Serial.printf("[I2C] SHT3x initialized at address 0x%02X\n", addr);
          tempSensors[detectedTempSensors].type = TempSensor::TYPE_SHT3X;
          tempSensors[detectedTempSensors].address = addr;
          tempSensors[detectedTempSensors].sht = sht;
          tempSensors[detectedTempSensors].active = true;
          detectedTempSensors++;
        } else {
          delete sht;
        }
      }
    }
  }

  // Swap sensors if necessary to ensure BME280 is always tempSensors[0] (Inside
  // / Master)
  if (tempSensors[1].active && tempSensors[1].type == TempSensor::TYPE_BME280 &&
      tempSensors[0].active && tempSensors[0].type != TempSensor::TYPE_BME280) {
    TempSensor temp = tempSensors[0];
    tempSensors[0] = tempSensors[1];
    tempSensors[1] = temp;
    Serial.println("[I2C] Swapped sensors: BME280 promoted to Inside (Master) "
                   "sensor tempSensors[0]");
  }

  // 2. Scan for TSL2561 (Light Sensors: addresses 0x29, 0x39, 0x49)
  uint8_t tslAddresses[] = {0x29, 0x39, 0x49};
  detectedLightSensors = 0;
  for (uint8_t addr : tslAddresses) {
    if (detectedLightSensors >= 2)
      break; // Max 2 light sensors

    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Adafruit_TSL2561_Unified *tsl =
          new Adafruit_TSL2561_Unified(addr, 12345 + detectedLightSensors);
      if (tsl->begin(&Wire)) {
        Serial.printf("[I2C] TSL2561 initialized at address 0x%02X\n", addr);
        tsl->enableAutoRange(true);
        tsl->setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);
        lightSensors[detectedLightSensors].address = addr;
        lightSensors[detectedLightSensors].tsl = tsl;
        lightSensors[detectedLightSensors].active = true;
        detectedLightSensors++;
      } else {
        delete tsl;
      }
    }
  }
}

void readSensors() {
  // Read Temperature & Humidity sensors
  for (int i = 0; i < 2; i++) {
    if (tempSensors[i].active) {
      if (tempSensors[i].type == TempSensor::TYPE_BME280 &&
          tempSensors[i].bme) {
        float t = tempSensors[i].bme->readTemperature();
        float h = tempSensors[i].bme->readHumidity();
        float p = tempSensors[i].bme->readPressure() / 100.0F;

        if (isnan(t) || isnan(h) || (t == 0.0f && h == 0.0f)) {
          tempSensors[i].temperature = NAN;
          tempSensors[i].humidity = NAN;
          tempSensors[i].pressure = NAN;

          static unsigned long lastBmeResetTime[2] = {0, 0};
          if (millis() - lastBmeResetTime[i] > 2000) {
            lastBmeResetTime[i] = millis();
            Serial.printf("[Sensor] BME280 at 0x%02X failed to read. "
                          "Re-initializing...\n",
                          tempSensors[i].address);
            tempSensors[i].bme->begin(tempSensors[i].address, &Wire);
          }
        } else {
          tempSensors[i].temperature = t;
          tempSensors[i].humidity = h;
          tempSensors[i].pressure = p;
        }
      } else if (tempSensors[i].type == TempSensor::TYPE_SHT3X &&
                 tempSensors[i].sht) {
        float t = tempSensors[i].sht->readTemperature();
        float h = tempSensors[i].sht->readHumidity();

        if (isnan(t) || isnan(h) || (t == 0.0f && h == 0.0f)) {
          tempSensors[i].temperature = NAN;
          tempSensors[i].humidity = NAN;
          tempSensors[i].pressure = NAN;

          static unsigned long lastShtResetTime[2] = {0, 0};
          if (millis() - lastShtResetTime[i] > 2000) {
            lastShtResetTime[i] = millis();
            Serial.printf(
                "[Sensor] SHT3x at 0x%02X failed to read. Re-initializing...\n",
                tempSensors[i].address);
            tempSensors[i].sht->begin(tempSensors[i].address);
          }
        } else {
          tempSensors[i].temperature = t;
          tempSensors[i].humidity = h;
          tempSensors[i].pressure = NAN;
        }
      }
    }
  }

  // Read Light Sensors
  for (int i = 0; i < 2; i++) {
    if (lightSensors[i].active && lightSensors[i].tsl) {
      sensors_event_t event;
      lightSensors[i].tsl->getEvent(&event);
      if (event.light) {
        lightSensors[i].lux = event.light;
      } else {
        lightSensors[i].lux = NAN;
      }
      // Read raw broadband and ir values
      uint16_t broadband = 0;
      uint16_t ir = 0;
      lightSensors[i].tsl->getLuminosity(&broadband, &ir);
      lightSensors[i].broadband = broadband;
      lightSensors[i].ir = ir;
    }
  }
}

void updateServoRamping(bool updateTarget = false) {
  static bool pendingTargetUpdate = false;
  if (updateTarget) {
    pendingTargetUpdate = true;
  }

  static unsigned long lastPotiReadTime = 0;
  if (millis() - lastPotiReadTime < 50) {
    return; // Rate limit ADC reading and target evaluations to 20Hz (50ms) to
            // ensure stable change detection
  }
  lastPotiReadTime = millis();

  bool runUpdate = pendingTargetUpdate;
  pendingTargetUpdate = false;

  // Read Potentiometers
  int rawA = analogRead(POTI_A_PIN);
  int rawB = analogRead(POTI_B_PIN);
  int rawC = analogRead(POTI_C_PIN);

  // Exponential Moving Average (EMA) noise filter (alpha = 0.15f)
  static float smoothedA = -1.0f;
  static float smoothedB = -1.0f;
  static float smoothedC = -1.0f;
  if (smoothedA < 0.0f) {
    smoothedA = rawA;
    smoothedB = rawB;
    smoothedC = rawC;
  } else {
    smoothedA = 0.15f * rawA + 0.85f * smoothedA;
    smoothedB =
        0.05f * rawB + 0.95f * smoothedB; // Heavy low-pass filter for Poti B to
                                          // eliminate ADC noise
    smoothedC = 0.15f * rawC + 0.85f * smoothedC;
  }

  potiAVal = map((int)round(smoothedA), 0, 4095, 48,
                 72); // 48 to 72 (24 intervals / 25 discrete steps)
  potiBVal = (float)round((smoothedB / 4095.0F) *
                          400.0F); // Whole integer gain percentage (0 - 400%)
  potiCVal =
      (smoothedC / 4095.0F) * 59.0F; // 0 - 59 degrees virtual 0-point offset
                                     // (180 - 121 = 59 max offset)

  static float lastPotiAVal = -1.0f;
  static float lastPotiBVal = -1.0f;
  static float lastPotiCVal = -1.0f;

  // Use thresholds to detect real user turns and filter ADC noise (now stable
  // because of 50ms rate limit)
  bool potiAChanged =
      (lastPotiAVal >= 0.0f) && (fabs(potiAVal - lastPotiAVal) > 1.0f);
  bool potiBChanged =
      (lastPotiBVal >= 0.0f) && (fabs(potiBVal - lastPotiBVal) >= 1.0f);
  bool potiCChanged =
      (lastPotiCVal >= 0.0f) && (fabs(potiCVal - lastPotiCVal) > 2.0f);

  if (runUpdate || potiAChanged || potiBChanged || potiCChanged ||
      lastPotiCVal < 0.0f) {
    if (potiAChanged || lastPotiAVal < 0.0f)
      lastPotiAVal = potiAVal;
    if (potiBChanged || lastPotiBVal < 0.0f)
      lastPotiBVal = potiBVal;
    if (potiCChanged || lastPotiCVal < 0.0f)
      lastPotiCVal = potiCVal;

    static int currentPotiAZone = 0; // 0: Proportional, 1: Closed, 2: Open
    static int lastPotiAZone = -1;
    int nextZone = currentPotiAZone;

    if (currentPotiAZone == 1) { // Currently closed (48 or 49)
      if (potiAVal >= 50.0f) {
        nextZone = 0; // Exit closed zone
      }
    } else if (currentPotiAZone == 2) { // Currently open (71 or 72)
      if (potiAVal <= 70.0f) {
        nextZone = 0; // Exit open zone
      }
    } else { // Currently proportional (50 to 70)
      if (potiAVal <= 49.0f) {
        nextZone = 1; // Enter closed zone
      } else if (potiAVal >= 71.0f) {
        nextZone = 2; // Enter open zone
      }
    }

    if (lastPotiAZone >= 0 && nextZone != lastPotiAZone) {
      if (nextZone == 1) {
        // Melodious descending arpeggio (6 notes, ~1.2 seconds): Rigorously
        // closed
        tone(BUZZER_PIN, 1568, 150); // G6
        delay(170);
        tone(BUZZER_PIN, 1319, 150); // E6
        delay(170);
        tone(BUZZER_PIN, 1047, 150); // C6
        delay(170);
        tone(BUZZER_PIN, 784, 150); // G5
        delay(170);
        tone(BUZZER_PIN, 659, 150); // E5
        delay(170);
        tone(BUZZER_PIN, 523, 300); // C5
        delay(350);
        noTone(BUZZER_PIN);
      } else if (nextZone == 2) {
        // Melodious ascending arpeggio (6 notes, ~1.2 seconds): Rigorously open
        tone(BUZZER_PIN, 523, 150); // C5
        delay(170);
        tone(BUZZER_PIN, 659, 150); // E5
        delay(170);
        tone(BUZZER_PIN, 784, 150); // G5
        delay(170);
        tone(BUZZER_PIN, 1047, 150); // C6
        delay(170);
        tone(BUZZER_PIN, 1319, 150); // E6
        delay(170);
        tone(BUZZER_PIN, 1568, 300); // G6
        delay(350);
        noTone(BUZZER_PIN);
      }
    }
    currentPotiAZone = nextZone;
    lastPotiAZone = currentPotiAZone;

    bool isSlaveMode =
        (sysConfig.espnow_role == 2 && strlen(sysConfig.espnow_peer_mac) > 0);
    bool isSlaveConnected = isSlaveMode && (lastEspNowRxTime != 0) &&
                            (millis() - lastEspNowRxTime <= 60000);
    bool isSlaveFailSafe = isSlaveMode && !isSlaveConnected;

    if (isSlaveConnected) {
      // Active Slave connection: rotorPosition is mirrored from Master via
      // ESP-NOW.
    } else if (isSlaveFailSafe && sysConfig.espnow_failsafe_mode == 0) {
      // Slave Fail-Safe Mode 0 (Default Safety Open): Force 50% Rotor position
      rotorPosition = 50.0f;
      bypassModeActive = false;
    } else {
      // Master Mode OR Slave Fail-Safe Mode 1 (Local Control via Slave's Poti A
      // & Sensor)
      if (potiAVal <= 49.0f) {
        // Virtual switch at bottom end: Rigorously closed (0% opening)
        rotorPosition = 0.0f;
        bypassModeActive = false;
      } else if (potiAVal >= 71.0f) {
        // Virtual switch at top end: Rigorously open (100% opening)
        rotorPosition = 100.0f;
        bypassModeActive = false;
      } else {
        // Normal closed-loop sensor-servo control algorithm (50% to 70%)
        float hum_inside = NAN;
        float hum_outside = NAN;

        // Find inside sensor (first active sensor in array)
        if (tempSensors[0].active && !isnan(tempSensors[0].humidity)) {
          hum_inside = tempSensors[0].humidity;
        }
        // Find outside sensor (second active sensor in array)
        if (tempSensors[1].active && !isnan(tempSensors[1].humidity)) {
          hum_outside = tempSensors[1].humidity;
        }

        // If we don't have an inside sensor but the second one is active, treat
        // the second one as inside
        if (isnan(hum_inside) && !isnan(hum_outside)) {
          hum_inside = hum_outside;
          hum_outside = NAN; // No outside sensor available
        }

        if (!isnan(hum_inside)) {
          // Thermodynamic bypass check: If outside humidity is higher than
          // inside humidity OR outside humidity is more than 2% above the
          // target humidity, keep the rotor closed!
          if (!isnan(hum_outside) &&
              (hum_outside > hum_inside || hum_outside > (potiAVal + 2.0f))) {
            rotorPosition =
                0.0f; // Moisture loading threat! Keep shutter fully closed.
            if (!bypassModeActive) {
              bypassModeActive = true;
              // Suppress warning chime if we are already dry (below target
              // humidity)
              if (isnan(hum_inside) || hum_inside >= potiAVal) {
                Serial.println("[Alarm] Thermodynamic bypass triggered "
                               "immediately. Playing warning chime.");
                for (int repeat = 0; repeat < 2; repeat++) {
                  for (int note = 0; note < 3; note++) {
                    tone(BUZZER_PIN, 500, 80); // 500 Hz, 80ms duration
                    delay(160);                // 80ms sound + 80ms pause
                  }
                  if (repeat == 0) {
                    delay(840); // 1000ms total pause between sequences (1000 -
                                // 160 = 840ms extra delay)
                  }
                }
                noTone(BUZZER_PIN);
              }
            }
          } else {
            bypassModeActive = false;
            // If current inside humidity is higher than Target (Poti A), we
            // open the rotor to dry the system
            float error = hum_inside - potiAVal;
            if (error < 0.0f)
              error = 0.0f;

            // Scale error by Poti B (Gain). Proportional control: rotor
            // position = error * (gain * 10.0)
            float gain = potiBVal / 100.0f;
            float target_pos = error * gain * 10.0f;

            // Flow-limiter based on drying potential (dryness multiplier):
            // If the outside air is extremely dry compared to our inside
            // target, we scale down the maximum opening to prevent drying shock
            // (incoming air too dry leads to rapid humidity drop).
            if (!isnan(hum_outside)) {
              float diff = potiAVal - hum_outside;
              if (diff > 10.0f) {
                float factor =
                    (diff - 10.0f) /
                    20.0f; // 0.0 to 1.0 (between 10% and 30% difference)
                if (factor > 1.0f)
                  factor = 1.0f;
                float multiplier =
                    1.0f - factor * 0.3f; // scales from 1.0 down to 0.7
                target_pos *= multiplier;
              }
            }

            if (target_pos > 100.0f)
              target_pos = 100.0f;
            if (target_pos < 0.0f)
              target_pos = 0.0f;
            rotorPosition = target_pos;
          }
        } else {
          // If no active temp/humidity sensor connected:
          // In Slave Fail-Safe mode, default to 50% safety open; otherwise 0%
          // (closed)
          rotorPosition = isSlaveFailSafe ? 50.0f : 0.0f;
          bypassModeActive = false;
        }
      }
    }

    // Calculate Target Servo angle: virtual 0-point offset + 121 degrees sweep
    // (perfected scale)
    float newTargetAngle = potiCVal + (rotorPosition / 100.0f) * 121.0f;
    if (newTargetAngle > 180.0f)
      newTargetAngle = 180.0f;
    if (newTargetAngle < 0.0f)
      newTargetAngle = 0.0f;

    static bool firstRun = true;
    bool significantChange = (fabs(newTargetAngle - targetServoAngle) > 1.5f);
    if (significantChange || firstRun) {
      firstRun = false;
      targetServoAngle = newTargetAngle;
      startServoAngle = currentServoAngle;
      servoMoveStartTime = millis();
      float diff = fabs(targetServoAngle - startServoAngle);
      // Duration = 0.5s + (diff / 121.0) * 4.5s
      servoMoveDuration = (0.5f + (diff / 121.0f) * 4.5f) * 1000.0f;
      servoMoving = true;
      servoFinishedPending = false; // Reset shutdown timer
    }
  }
}

// =====================================================================
// WIFI CONFIG AP, CAPTIVE PORTAL & REALTIME WEB MONITOR
// =====================================================================
WebServer server(80);
DNSServer dnsServer;
String apSSID = "";
const char *apPassword = "growblox";
bool portalActive = false;

// Generate unique SSID from MAC Address
void generateUniqueSSID() {
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  String last6 = mac.substring(mac.length() - 6);
  last6.toUpperCase();
  apSSID = "IDRY26-" + last6;
}

// REST API for Real-time monitor updates
void handleGetData() {
  JsonDocument doc;
  doc["device_name"] = sysConfig.mqtt_device_name;
  if (WiFi.status() == WL_CONNECTED) {
    doc["ip_address"] = WiFi.localIP().toString();
  } else {
    doc["ip_address"] =
        "try to reconnect to: [" + String(sysConfig.wifi_ssid) + "]";
  }
  doc["mode"] =
      isHeadless ? "Headless Mode" : (isTFTMode ? "TFT Mode" : "e-Paper Mode");
  doc["wifi_ssid"] = sysConfig.wifi_ssid; // Send SSID for client-side use

  // MQTT configuration and state details
  bool mqtt_configured = (strlen(sysConfig.mqtt_server) > 0);
  doc["mqtt_enabled"] = mqtt_configured;
  doc["mqtt_server"] = sysConfig.mqtt_server;
  doc["mqtt_port"] = sysConfig.mqtt_port;
  doc["mqtt_connected"] = mqtt_configured ? mqttClient.connected() : false;
  doc["mqtt_topic"] = stateTopic;

  JsonArray sensors = doc["sensors"].to<JsonArray>();
  for (int i = 0; i < 2; i++) {
    if (tempSensors[i].active) {
      JsonObject s = sensors.add<JsonObject>();
      s["type"] =
          (tempSensors[i].type == TempSensor::TYPE_BME280) ? "BME280" : "SHT3x";
      char addrHex[8];
      sprintf(addrHex, "0x%02X", tempSensors[i].address);
      s["address"] = addrHex;
      s["temperature"] = isnan(tempSensors[i].temperature)
                             ? JsonVariant()
                             : tempSensors[i].temperature;
      s["humidity"] = isnan(tempSensors[i].humidity) ? JsonVariant()
                                                     : tempSensors[i].humidity;
      s["pressure"] = isnan(tempSensors[i].pressure) ? JsonVariant()
                                                     : tempSensors[i].pressure;
      float dp = calculateDewPoint(tempSensors[i].temperature,
                                   tempSensors[i].humidity);
      s["dewpoint"] = isnan(dp) ? JsonVariant() : dp;
    }
  }

  JsonArray lightArr = doc["lights"].to<JsonArray>();
  for (int i = 0; i < 2; i++) {
    if (lightSensors[i].active) {
      JsonObject l = lightArr.add<JsonObject>();
      char addrHex[8];
      sprintf(addrHex, "0x%02X", lightSensors[i].address);
      l["address"] = addrHex;
      l["lux"] =
          isnan(lightSensors[i].lux) ? JsonVariant() : lightSensors[i].lux;
      l["broadband"] = lightSensors[i].broadband;
      l["ir"] = lightSensors[i].ir;
    }
  }

  JsonObject potis = doc["potentiometers"].to<JsonObject>();
  potis["poti_a_target_hum"] = potiAVal;
  potis["poti_b_gain"] = potiBVal;
  potis["poti_c_cal_offset"] = potiCVal;

  doc["rotor_position"] = rotorPosition;
  doc["rotor_offset"] = potiCVal;
  doc["rssi"] = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
  doc["espnow_role"] = sysConfig.espnow_role;
  doc["espnow_peer_mac"] = sysConfig.espnow_peer_mac;
  doc["espnow_channel"] =
      (sysConfig.espnow_role == 1)
          ? (WiFi.status() == WL_CONNECTED ? WiFi.channel() : 1)
          : (isPairingActive ? currentPairingChannel
                             : sysConfig.espnow_channel);
  doc["espnow_last_seen_ms"] =
      (lastEspNowRxTime == 0) ? -1 : (int)(millis() - lastEspNowRxTime);
  doc["espnow_interval_ms"] = avgEspNowIntervalMs;
  doc["espnow_pv_mismatch"] = (strlen(sysConfig.espnow_peer_mac) > 0) &&
                              (remoteProtocolVersion > 0) &&
                              (remoteProtocolVersion != localProtocolVersion);
  doc["espnow_remote_pv"] = remoteProtocolVersion;
  doc["espnow_local_pv"] = localProtocolVersion;
  doc["espnow_pairing"] = isPairingActive;
  doc["espnow_failsafe_mode"] = sysConfig.espnow_failsafe_mode;
  doc["wifi_mac"] = WiFi.macAddress();
  doc["wifi_channel"] = WiFi.status() == WL_CONNECTED ? WiFi.channel() : 1;
  doc["watchdog_reset_countdown"] = getWatchdogResetCountdown();
  doc["fw_version"] = "1." + String(localFirmwareVersion);
  doc["loops_per_sec"] = loopsPerSecond;

  String jsonResponse;
  serializeJson(doc, jsonResponse);
  server.send(200, "application/json", jsonResponse);
}

void handleGetHistory() {
  JsonDocument doc;

  // 120-Minute Array (1-minute resolution for mini preview cards & 2h RSSI
  // status panel)
  JsonArray samples120m = doc["h120m"].to<JsonArray>();
  int start120 = (history120mCount < HIST_120M_SIZE) ? 0 : history120mHead;
  for (int i = 0; i < history120mCount; i++) {
    int idx = (start120 + i) % HIST_120M_SIZE;
    JsonObject s = samples120m.add<JsonObject>();
    s["t0_min"] = isnan(history120mBuffer[idx].temp_0_min)
                      ? JsonVariant()
                      : history120mBuffer[idx].temp_0_min;
    s["t0"] = isnan(history120mBuffer[idx].temp_0_max)
                  ? JsonVariant()
                  : history120mBuffer[idx].temp_0_max;
    s["h0_min"] = isnan(history120mBuffer[idx].hum_0_min)
                      ? JsonVariant()
                      : history120mBuffer[idx].hum_0_min;
    s["h0"] = isnan(history120mBuffer[idx].hum_0_max)
                  ? JsonVariant()
                  : history120mBuffer[idx].hum_0_max;
    s["t1_min"] = isnan(history120mBuffer[idx].temp_1_min)
                      ? JsonVariant()
                      : history120mBuffer[idx].temp_1_min;
    s["t1"] = isnan(history120mBuffer[idx].temp_1_max)
                  ? JsonVariant()
                  : history120mBuffer[idx].temp_1_max;
    s["h1_min"] = isnan(history120mBuffer[idx].hum_1_min)
                      ? JsonVariant()
                      : history120mBuffer[idx].hum_1_min;
    s["h1"] = isnan(history120mBuffer[idx].hum_1_max)
                  ? JsonVariant()
                  : history120mBuffer[idx].hum_1_max;
    s["l0"] = history120mBuffer[idx].lux_0_max;
    s["l1"] = history120mBuffer[idx].lux_1_max;
    s["r"] = history120mBuffer[idx].rotor_max;
    s["el"] = history120mBuffer[idx].espnow_loss_sec;
    s["ml"] = history120mBuffer[idx].mqtt_loss_sec;
    s["rssi"] = history120mBuffer[idx].rssi_min;
  }
  // Active live 1-minute bucket
  JsonObject live1m = samples120m.add<JsonObject>();
  live1m["t0_min"] = isnan(b1m_temp_0_min) ? (isnan(tempSensors[0].temperature)
                                                  ? JsonVariant()
                                                  : tempSensors[0].temperature)
                                           : b1m_temp_0_min;
  live1m["t0"] = isnan(b1m_temp_0_max) ? (isnan(tempSensors[0].temperature)
                                              ? JsonVariant()
                                              : tempSensors[0].temperature)
                                       : b1m_temp_0_max;
  live1m["h0_min"] = isnan(b1m_hum_0_min) ? (isnan(tempSensors[0].humidity)
                                                 ? JsonVariant()
                                                 : tempSensors[0].humidity)
                                          : b1m_hum_0_min;
  live1m["h0"] = isnan(b1m_hum_0_max) ? (isnan(tempSensors[0].humidity)
                                             ? JsonVariant()
                                             : tempSensors[0].humidity)
                                      : b1m_hum_0_max;
  live1m["t1_min"] = isnan(b1m_temp_1_min) ? (isnan(tempSensors[1].temperature)
                                                  ? JsonVariant()
                                                  : tempSensors[1].temperature)
                                           : b1m_temp_1_min;
  live1m["t1"] = isnan(b1m_temp_1_max) ? (isnan(tempSensors[1].temperature)
                                              ? JsonVariant()
                                              : tempSensors[1].temperature)
                                       : b1m_temp_1_max;
  live1m["h1_min"] = isnan(b1m_hum_1_min) ? (isnan(tempSensors[1].humidity)
                                                 ? JsonVariant()
                                                 : tempSensors[1].humidity)
                                          : b1m_hum_1_min;
  live1m["h1"] = isnan(b1m_hum_1_max) ? (isnan(tempSensors[1].humidity)
                                             ? JsonVariant()
                                             : tempSensors[1].humidity)
                                      : b1m_hum_1_max;
  live1m["l0"] = b1m_lux_0_max;
  live1m["l1"] = b1m_lux_1_max;
  live1m["r"] = (rotorPosition > b1m_rotor_max) ? rotorPosition : b1m_rotor_max;
  live1m["el"] = b1m_espnow_loss_sec;
  live1m["ml"] = b1m_mqtt_loss_sec;
  int8_t activeRssi =
      (WiFi.status() == WL_CONNECTED) ? (int8_t)WiFi.RSSI() : -100;
  live1m["rssi"] = (b1m_rssi_min == 0) ? activeRssi : b1m_rssi_min;

  // 24-Hour Array (5-minute resolution for modal zoom)
  JsonArray samples24h = doc["h24h"].to<JsonArray>();
  int start24 = (history24hCount < HIST_24H_SIZE) ? 0 : history24hHead;
  for (int i = 0; i < history24hCount; i++) {
    int idx = (start24 + i) % HIST_24H_SIZE;
    JsonObject s = samples24h.add<JsonObject>();
    s["t0_min"] = isnan(history24hBuffer[idx].temp_0_min)
                      ? JsonVariant()
                      : history24hBuffer[idx].temp_0_min;
    s["t0"] = isnan(history24hBuffer[idx].temp_0_max)
                  ? JsonVariant()
                  : history24hBuffer[idx].temp_0_max;
    s["h0_min"] = isnan(history24hBuffer[idx].hum_0_min)
                      ? JsonVariant()
                      : history24hBuffer[idx].hum_0_min;
    s["h0"] = isnan(history24hBuffer[idx].hum_0_max)
                  ? JsonVariant()
                  : history24hBuffer[idx].hum_0_max;
    s["t1_min"] = isnan(history24hBuffer[idx].temp_1_min)
                      ? JsonVariant()
                      : history24hBuffer[idx].temp_1_min;
    s["t1"] = isnan(history24hBuffer[idx].temp_1_max)
                  ? JsonVariant()
                  : history24hBuffer[idx].temp_1_max;
    s["h1_min"] = isnan(history24hBuffer[idx].hum_1_min)
                      ? JsonVariant()
                      : history24hBuffer[idx].hum_1_min;
    s["h1"] = isnan(history24hBuffer[idx].hum_1_max)
                  ? JsonVariant()
                  : history24hBuffer[idx].hum_1_max;
    s["l0"] = history24hBuffer[idx].lux_0_max;
    s["l1"] = history24hBuffer[idx].lux_1_max;
    s["r"] = history24hBuffer[idx].rotor_max;
    s["el"] = history24hBuffer[idx].espnow_loss_sec;
    s["ml"] = history24hBuffer[idx].mqtt_loss_sec;
    s["rssi"] = history24hBuffer[idx].rssi_min;
  }
  // Active live 5-minute bucket
  JsonObject live5m = samples24h.add<JsonObject>();
  live5m["t0_min"] = isnan(b5m_temp_0_min) ? (isnan(tempSensors[0].temperature)
                                                  ? JsonVariant()
                                                  : tempSensors[0].temperature)
                                           : b5m_temp_0_min;
  live5m["t0"] = isnan(b5m_temp_0_max) ? (isnan(tempSensors[0].temperature)
                                              ? JsonVariant()
                                              : tempSensors[0].temperature)
                                       : b5m_temp_0_max;
  live5m["h0_min"] = isnan(b5m_hum_0_min) ? (isnan(tempSensors[0].humidity)
                                                 ? JsonVariant()
                                                 : tempSensors[0].humidity)
                                          : b5m_hum_0_min;
  live5m["h0"] = isnan(b5m_hum_0_max) ? (isnan(tempSensors[0].humidity)
                                             ? JsonVariant()
                                             : tempSensors[0].humidity)
                                      : b5m_hum_0_max;
  live5m["t1_min"] = isnan(b5m_temp_1_min) ? (isnan(tempSensors[1].temperature)
                                                  ? JsonVariant()
                                                  : tempSensors[1].temperature)
                                           : b5m_temp_1_min;
  live5m["t1"] = isnan(b5m_temp_1_max) ? (isnan(tempSensors[1].temperature)
                                              ? JsonVariant()
                                              : tempSensors[1].temperature)
                                       : b5m_temp_1_max;
  live5m["h1_min"] = isnan(b5m_hum_1_min) ? (isnan(tempSensors[1].humidity)
                                                 ? JsonVariant()
                                                 : tempSensors[1].humidity)
                                          : b5m_hum_1_min;
  live5m["h1"] = isnan(b5m_hum_1_max) ? (isnan(tempSensors[1].humidity)
                                             ? JsonVariant()
                                             : tempSensors[1].humidity)
                                      : b5m_hum_1_max;
  live5m["l0"] = b5m_lux_0_max;
  live5m["l1"] = b5m_lux_1_max;
  live5m["r"] = (rotorPosition > b5m_rotor_max) ? rotorPosition : b5m_rotor_max;
  live5m["el"] = b5m_espnow_loss_sec;
  live5m["ml"] = b5m_mqtt_loss_sec;
  live5m["rssi"] = (b5m_rssi_min == 0) ? activeRssi : b5m_rssi_min;

  String jsonResponse;
  serializeJson(doc, jsonResponse);
  server.send(200, "application/json", jsonResponse);
}

// Active connection check (TCP Handshake time heuristic to verify gateway is
// alive)
bool checkGatewayReachable() {
  IPAddress gw = WiFi.gatewayIP();
  if (gw[0] == 0)
    return false;

  WiFiClient client;
  client.setTimeout(500); // Set short 500ms timeout (setTimeout takes
                          // milliseconds on ESP32 Client class)
  unsigned long start = millis();
  bool ok = client.connect(gw, 80);
  unsigned long duration = millis() - start;

  if (ok) {
    client.stop();
    return true;
  }

  // Heuristic: If it failed immediately (duration < 150ms), it means the router
  // sent a TCP RST (refused). This means the router is physically ONLINE and
  // responding. If it took longer (> 400ms) to fail, it timed out (no
  // response), indicating the router is OFFLINE.
  if (duration < 150) {
    return true;
  }
  return false;
}

void handlePortalRoot() {
  if (WiFi.status() == WL_CONNECTED && !portalActive) {
    String pageTitle = String(sysConfig.mqtt_device_name);
    if (sysConfig.espnow_role == 1)
      pageTitle += " Master";
    else if (sysConfig.espnow_role == 2)
      pageTitle += " Slave";
    else
      pageTitle += " Dashboard";

    // Show Real-time Sensor Dashboard
    String html =
        R"rawhtml(
<!DOCTYPE html>
<html lang="de">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>)rawhtml" +
        pageTitle + R"rawhtml(</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; }
        body {
            background: linear-gradient(135deg, #0f172a 0%, #1e1b4b 100%);
            color: #f8fafc;
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            padding: 20px;
        }
        .container {
            background: rgba(30, 41, 59, 0.45);
            backdrop-filter: blur(12px);
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 20px;
            padding: 30px;
            width: 100%;
            max-width: 650px;
            box-shadow: 0 20px 25px -5px rgba(0, 0, 0, 0.5);
        }
        h1 { text-align: center; margin-bottom: 25px; font-size: 24px; font-weight: 600; letter-spacing: 1px; color: #818cf8; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-bottom: 20px; }
        @media(max-width: 500px) { .grid { grid-template-columns: 1fr; } }
        .card {
            background: rgba(15, 23, 42, 0.5);
            border: 1px solid rgba(255, 255, 255, 0.05);
            border-radius: 12px;
            padding: 20px;
        }
        .card-title { font-size: 13px; text-transform: uppercase; letter-spacing: 1px; color: #94a3b8; margin-bottom: 12px; font-weight: bold; border-bottom: 1px solid rgba(255,255,255,0.05); padding-bottom: 5px; }
        .value-row { display: flex; justify-content: space-between; margin-bottom: 8px; font-size: 15px; }
        .value-row:last-child { margin-bottom: 0; }
        .val { font-weight: 600; color: #38bdf8; }
        .tooltip {
            position: relative;
            display: inline-flex;
            align-items: center;
            cursor: pointer;
            margin-left: 6px;
        }
        .tooltip .tooltiptext {
            visibility: hidden;
            width: 220px;
            background-color: #ef4444;
            color: #fff;
            text-align: center;
            border-radius: 6px;
            padding: 8px;
            position: absolute;
            z-index: 10;
            bottom: 125%;
            left: 50%;
            transform: translateX(-50%);
            opacity: 0;
            transition: opacity 0.3s;
            font-size: 11px;
            font-weight: normal;
            line-height: 1.4;
            box-shadow: 0 4px 6px -1px rgba(0,0,0,0.3);
        }
        .tooltip:hover .tooltiptext {
            visibility: visible;
            opacity: 1;
        }
        .info-icon {
            width: 14px;
            height: 14px;
            border: 1px solid currentColor;
            border-radius: 50%;
            display: inline-flex;
            align-items: center;
            justify-content: center;
            font-size: 9px;
            font-weight: bold;
            font-family: serif;
        }
        .footer { text-align: center; margin-top: 20px; font-size: 11px; color: #64748b; }
        .moon-container { display: flex; justify-content: center; margin-top: 15px; }
        .moon {
          width: 80px;
          height: 80px;
          background: #191b28;
          border-radius: 50%;
          position: relative;
          overflow: hidden;
          box-shadow: inset -2px -2px 8px rgba(0,0,0,0.7);
          border: 1px solid rgba(255,255,255,0.1);
        }
        .moon::after {
          content: '';
          position: absolute;
          top: 0; 
          left: 0;
          width: 100%; 
          height: 100%;
          background: #38bdf8;
          border-radius: 50%;
          transform: var(--ts, translateX(-100%));
          transition: transform 0.2s ease-out;
        }
        details.hist-toggle { margin-top: 12px; border-top: 1px solid rgba(255,255,255,0.08); padding-top: 6px; }
        details.hist-toggle summary { font-size: 11px; color: #94a3b8; cursor: pointer; user-select: none; font-weight: 600; outline: none; margin-bottom: 4px; }
        .spark-box { position: relative; width: 100%; height: 50px; background: rgba(15,23,42,0.6); border-radius: 6px; border: 1px solid rgba(255,255,255,0.05); overflow: hidden; cursor: pointer; }
        .spark-box canvas { width: 100%; height: 50px; display: block; }
    </style>
</head>
<body>
    <div class="container">
        <h1 id="device-title">IDRY-26 Loading...</h1>
        <div class="grid">
            <div class="card" id="sensor-card-0" style="display:none;">
                <div class="card-title" id="sensor-title-0">Sensor 1</div>
                <div class="value-row"><span>Temperatur:</span><span class="val" id="temp-0">--</span></div>
                <div class="value-row"><span>Feuchtigkeit:</span><span class="val" id="hum-0">--</span></div>
                <div class="value-row" id="dp-row-0"><span>Taupunkt:</span><span class="val" id="dp-0">--</span></div>
                <div class="value-row" id="press-row-0"><span>Luftdruck:</span><span class="val" id="press-0">--</span></div>
                <details open class="hist-toggle" id="details-temp-0" ontoggle="renderAllCharts()">
                    <summary>60m Verlauf (Temperatur)</summary>
                    <div class="spark-box" onclick="openChartZoom('temp_0', 'Sensor 1 Temperatur')">
                        <canvas id="cv-temp-0"></canvas>
                    </div>
                </details>
                <details open class="hist-toggle" id="details-hum-0" ontoggle="renderAllCharts()">
                    <summary>60m Verlauf (Luftfeuchtigkeit)</summary>
                    <div class="spark-box" onclick="openChartZoom('hum_0', 'Sensor 1 Luftfeuchtigkeit')">
                        <canvas id="cv-hum-0"></canvas>
                    </div>
                </details>
            </div>
            <div class="card" id="sensor-card-1" style="display:none;">
                <div class="card-title" id="sensor-title-1">Sensor 2</div>
                <div class="value-row"><span>Temperatur:</span><span class="val" id="temp-1">--</span></div>
                <div class="value-row"><span>Feuchtigkeit:</span><span class="val" id="hum-1">--</span></div>
                <div class="value-row" id="dp-row-1"><span>Taupunkt:</span><span class="val" id="dp-1">--</span></div>
                <div class="value-row" id="press-row-1"><span>Luftdruck:</span><span class="val" id="press-1">--</span></div>
                <details open class="hist-toggle" id="details-temp-1" ontoggle="renderAllCharts()">
                    <summary>60m Verlauf (Temperatur)</summary>
                    <div class="spark-box" onclick="openChartZoom('temp_1', 'Sensor 2 Temperatur')">
                        <canvas id="cv-temp-1"></canvas>
                    </div>
                </details>
                <details open class="hist-toggle" id="details-hum-1" ontoggle="renderAllCharts()">
                    <summary>60m Verlauf (Luftfeuchtigkeit)</summary>
                    <div class="spark-box" onclick="openChartZoom('hum_1', 'Sensor 2 Luftfeuchtigkeit')">
                        <canvas id="cv-hum-1"></canvas>
                    </div>
                </details>
            </div>
            <div class="card" id="light-card-0" style="display:none;">
                <div class="card-title" id="light-title-0">TSL2561 (1)</div>
                <div class="value-row"><span>Helligkeit:</span><span class="val" id="lux-val-0">--</span></div>
                <div class="value-row"><span>Breitband:</span><span class="val" id="broadband-val-0">--</span></div>
                <div class="value-row"><span>Infrarot:</span><span class="val" id="ir-val-0">--</span></div>
                <details open class="hist-toggle" id="details-lux-0" ontoggle="renderAllCharts()">
                    <summary>60m Verlauf (Helligkeit)</summary>
                    <div class="spark-box" onclick="openChartZoom('lux_0', 'TSL2561 (1) Helligkeit')">
                        <canvas id="cv-lux-0"></canvas>
                    </div>
                </details>
            </div>
            <div class="card" id="light-card-1" style="display:none;">
                <div class="card-title" id="light-title-1">TSL2561 (2)</div>
                <div class="value-row"><span>Helligkeit:</span><span class="val" id="lux-val-1">--</span></div>
                <div class="value-row"><span>Breitband:</span><span class="val" id="broadband-val-1">--</span></div>
                <div class="value-row"><span>Infrarot:</span><span class="val" id="ir-val-1">--</span></div>
                <details open class="hist-toggle" id="details-lux-1" ontoggle="renderAllCharts()">
                    <summary>60m Verlauf (Helligkeit)</summary>
                    <div class="spark-box" onclick="openChartZoom('lux_1', 'TSL2561 (2) Helligkeit')">
                        <canvas id="cv-lux-1"></canvas>
                    </div>
                </details>
            </div>
            <div class="card">
                <div class="card-title">Potentiometer</div>
                <div class="value-row"><span>Sollwert Feuchte (A):</span><span class="val" id="poti-a">--</span></div>
                <div class="value-row"><span>Gain Faktor (B):</span><span class="val" id="poti-b">--</span></div>
                <div class="value-row"><span>Rotor-Offset (C):</span><span class="val" id="poti-c">--</span></div>
            </div>
            <div class="card">
                <div class="card-title">Rotor & Servo</div>
                <div class="value-row"><span>Rotor Stellung:</span><span class="val" id="rotor-pos">--</span></div>
                <div class="moon-container">
                    <div id="luna" class="moon"></div>
                </div>
                <details open class="hist-toggle" id="details-rotor" ontoggle="renderAllCharts()">
                    <summary>60m Verlauf (Rotor Öffnung)</summary>
                    <div class="spark-box" onclick="openChartZoom('rotor', 'Rotor Stellung Verlauf')">
                        <canvas id="cv-rotor"></canvas>
                    </div>
                </details>
            </div>
            <div class="card" id="espnow-card" style="display:none;">
                <div class="card-title">ESPNOW</div>
                <div class="value-row"><span>Rolle:</span><span class="val" id="espnow-val-role" style="font-weight: bold; text-transform: uppercase;">--</span></div>
                <div class="value-row"><span>Verbindung:</span><span class="val" id="espnow-val-conn">--</span></div>
                <div class="value-row"><span>Protokoll:</span><span class="val" id="espnow-val-pv">--</span></div>
                <details open class="hist-toggle" id="details-espnow" ontoggle="renderAllCharts()">
                    <summary>60m Verbindungsausfälle</summary>
                    <div class="spark-box" onclick="openChartZoom('espnow', 'ESP-NOW Link Loss Verlauf')">
                        <canvas id="cv-espnow"></canvas>
                    </div>
                </details>
            </div>
            <div class="card" id="mqtt-card" style="display:none;">
                <div class="card-title" id="mqtt-title">MQTT Dashboard</div>
                <div class="value-row"><span>Broker:</span><span class="val" id="mqtt-broker">--</span></div>
                <div class="value-row"><span>Status:</span><span class="val" id="mqtt-status">--</span></div>
                <div class="value-row"><span style="flex-shrink: 0; margin-right: 10px;">Topic:</span><span class="val" id="mqtt-topic" style="font-size:11px; text-align: right; word-break:break-all;">--</span></div>
                <details open class="hist-toggle" id="details-mqtt" ontoggle="renderAllCharts()">
                    <summary>60m Broker Ausfälle</summary>
                    <div class="spark-box" onclick="openChartZoom('mqtt', 'MQTT Link Loss Verlauf')">
                        <canvas id="cv-mqtt"></canvas>
                    </div>
                </details>
            </div>
        </div>
        <div class="card">
            <div class="card-title">System Status</div>
            <div class="value-row"><span>IP-Adresse:</span><span class="val" id="sys-ip">--</span></div>
            <div class="value-row"><span>Anzeige-Modus:</span><span class="val" id="sys-mode">--</span></div>
            <div class="value-row">
                <span style="display: flex; align-items: center; gap: 10px;">
                    Signalstärke RSSI:
                    <div style="width: 50px; height: 8px; background: rgba(255,255,255,0.15); border-radius: 4px; overflow: hidden; display: inline-block;">
                        <div id="sys-rssi-bar" style="width: 0%; height: 100%; transition: width 0.3s, background-color 0.3s; background: #ef4444;"></div>
                    </div>
                </span>
                <span class="val" id="sys-rssi">--</span>
            </div>
            <div class="value-row"><span>Watchdog reset weekly:</span><span class="val" id="sys-wd-reset" style="font-family: monospace;">--</span></div>
            <details open class="hist-toggle" id="details-rssi" ontoggle="renderAllCharts()">
                <summary>2h Signalstärke Verlauf (RSSI)</summary>
                <div class="spark-box" onclick="openChartZoom('rssi', 'WLAN Signalstärke Verlauf')">
                    <canvas id="cv-rssi"></canvas>
                </div>
            </details>
        </div>
        <div class="footer" style="display: flex; justify-content: space-between; align-items: center; margin-top: 25px; padding-top: 15px; border-top: 1px solid rgba(255,255,255,0.05);">
            <span id="footer-text">IDRY26 Live Monitor v)rawhtml" +
        String("1.") + String(localFirmwareVersion) +
        R"rawhtml( - (bench: <span id="footer-bench" style="font-family: monospace; color: #38bdf8; font-weight: bold;">--</span> loops/s)</span>
            <a href="/settings" style="color: #818cf8; text-decoration: none; display: inline-flex; align-items: center; gap: 5px; font-weight: 600; padding: 6px 12px; background: rgba(129, 140, 248, 0.1); border-radius: 8px; border: 1px solid rgba(129, 140, 248, 0.2); transition: all 0.2s;">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="3"></circle><path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 1 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-4 0v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 1 1-2.83-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1 0-4h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 1 1 2.83-2.83l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 4 0v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 1 1 2.83 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1z"></path></svg>
                Einstellungen
            </a>
        </div>
    </div>

    <div id="chart-modal" style="display:none; position:fixed; top:0; left:0; width:100vw; height:100vh; background:rgba(15,23,42,0.88); backdrop-filter:blur(10px); z-index:999; align-items:center; justify-content:center; padding:20px;">
        <div style="background:#1e293b; border:1px solid rgba(255,255,255,0.1); border-radius:16px; padding:24px; max-width:700px; width:100%; box-shadow:0 25px 50px -12px rgba(0,0,0,0.7); position:relative;">
            <h2 id="modal-title" style="font-size:18px; color:#818cf8; margin-bottom:15px; text-align:center;">Verlauf (24h Zoom)</h2>
            <div style="width:100%; overflow-x:auto; background:#0f172a; border-radius:8px; border:1px solid rgba(255,255,255,0.05); padding:10px; position:relative;" id="modal-canvas-container">
                <canvas id="modal-canvas" width="600" height="200" style="display:block; width:100%; height:200px; cursor:pointer;"></canvas>
                <div id="canvas-floating-popup" style="display:none; position:absolute; padding:5px 10px; background:#0f172a; border:1.5px solid #38bdf8; border-radius:6px; font-family:monospace; font-size:12px; color:#fff; pointer-events:none; z-index:10; white-space:nowrap; box-shadow:0 4px 14px rgba(0,0,0,0.7); transform:translate(-50%, -100%); transition: left 0.05s ease-out, top 0.05s ease-out;"></div>
            </div>
            <div id="modal-tooltip" style="font-family:monospace; font-size:13px; color:#38bdf8; margin-top:12px; text-align:center; min-height:18px;">Tippe oder fahre über eine Kerze für Details...</div>
            <button onclick="closeChartModal()" style="margin-top:18px; width:100%; padding:12px; border-radius:8px; border:none; background:#3b82f6; color:white; font-weight:bold; cursor:pointer; font-size:14px;">Schließen</button>
        </div>
    </div>
    <script>
        const wifiSSID = ")rawhtml";
    html += String(sysConfig.wifi_ssid);
    html += R"rawhtml(";
        let favCanvas = null;
        function updateFaviconMoon(p, isSlave) {
            if (!favCanvas) {
                favCanvas = document.createElement('canvas');
                favCanvas.width = 32;
                favCanvas.height = 32;
            }
            const ctx = favCanvas.getContext('2d');
            ctx.clearRect(0, 0, 32, 32);

            // 1. Theme Background (Rounded badge)
            ctx.fillStyle = isSlave ? '#3f0e0e' : '#171a33';
            ctx.beginPath();
            if (ctx.roundRect) {
                ctx.roundRect(0, 0, 32, 32, 6);
            } else {
                ctx.rect(0, 0, 32, 32);
            }
            ctx.fill();

            // 2. Base Dark Moon Circle (Center 16,16, Radius 11)
            const r = 11;
            ctx.save();
            ctx.beginPath();
            ctx.arc(16, 16, r, 0, Math.PI * 2);
            ctx.fillStyle = '#191b28';
            ctx.fill();
            ctx.clip();

            // 3. Light Blue Shutter Phase (translateX from -2r to 0)
            const shiftX = (p / 100.0) * (2 * r) - (2 * r);
            ctx.beginPath();
            ctx.arc(16 + shiftX, 16, r, 0, Math.PI * 2);
            ctx.fillStyle = '#38bdf8';
            ctx.fill();
            ctx.restore();

            // 4. Outer Bevel & Subtle Shadow Rings
            ctx.beginPath();
            ctx.arc(16, 16, r, 0, Math.PI * 2);
            ctx.strokeStyle = 'rgba(255,255,255,0.2)';
            ctx.lineWidth = 1;
            ctx.stroke();

            // 5. Update Favicon Link in DOM
            let link = document.getElementById('dynamic-favicon');
            if (!link) {
                link = document.createElement('link');
                link.id = 'dynamic-favicon';
                link.rel = 'icon';
                link.type = 'image/png';
                document.head.appendChild(link);
            }
            link.href = favCanvas.toDataURL('image/png');
        }

        function setMoon(val, isSlave) {
            const m = document.getElementById('luna');
            let p = parseInt(val);
            if (isNaN(p)) p = 0;
            if (p < 0) p = 0;
            if (p > 100) p = 100;
            if (m) m.style.setProperty('--ts', `translateX(${-100 + p}%)`);
            updateFaviconMoon(p, isSlave);
        }
        function fetchWithTimeout(resource, options = {}) {
            const { timeout = 1000 } = options;
            const controller = new AbortController();
            const id = setTimeout(() => controller.abort(), timeout);
            return fetch(resource, { ...options, signal: controller.signal })
                .then(response => { clearTimeout(id); return response; })
                .catch(err => { clearTimeout(id); throw err; });
        }
        function updateData() {
            fetchWithTimeout('/api/data', { timeout: 1000 })
                .then(response => {
                    if (!response.ok) throw new Error("Connection lost");
                    return response.json();
                })
                .then(data => {
                    let titleText = data.device_name;
                    let docTitle = data.device_name;
                    if (data.espnow_role === 1) {
                        titleText += " [MASTER]";
                        docTitle += " Master";
                        document.body.style.background = "linear-gradient(135deg, #0f172a 0%, #1e1b4b 100%)";
                    } else if (data.espnow_role === 2) {
                        titleText += " [SLAVE]";
                        docTitle += " Slave";
                        document.body.style.background = "linear-gradient(135deg, #1e1b1b 0%, #450a0a 100%)";
                    } else {
                        docTitle += " Dashboard";
                        document.body.style.background = "linear-gradient(135deg, #0f172a 0%, #1e1b4b 100%)";
                    }
                    document.getElementById('device-title').innerText = titleText;
                    document.title = docTitle;
                    document.getElementById('sys-ip').innerText = data.ip_address;
                    document.getElementById('sys-ip').style.color = data.ip_address.startsWith("try") ? "#f87171" : "#38bdf8";
                    document.getElementById('sys-mode').innerText = data.mode;
                    let rssi = parseInt(data.rssi) || 0;
                    if (rssi === 0) rssi = -100;
                    let pct = Math.round((rssi + 100) * 10 / 7);
                    if (pct < 0) pct = 0;
                    if (pct > 100) pct = 100;
                    const rssiBar = document.getElementById('sys-rssi-bar');
                    if (rssiBar) {
                        rssiBar.style.width = pct + "%";
                        if (rssi >= -50) {
                            rssiBar.style.backgroundColor = "#22c55e"; // Hellgrün
                        } else if (rssi >= -70) {
                            rssiBar.style.backgroundColor = "#84cc16"; // Grün
                        } else if (rssi >= -80) {
                            rssiBar.style.backgroundColor = "#eab308"; // Gelb
                        } else if (rssi >= -90) {
                            rssiBar.style.backgroundColor = "#f97316"; // Orange
                        } else {
                            rssiBar.style.backgroundColor = "#ef4444"; // Rot
                        }
                    }
                    document.getElementById('sys-rssi').innerText = rssi + " dBm";
                    document.getElementById('sys-rssi').style.color = "#38bdf8";
                    const wdResetEl = document.getElementById('sys-wd-reset');
                    if (wdResetEl) {
                        wdResetEl.innerText = data.watchdog_reset_countdown || "--";
                    }

                    // Update temperature sensors
                    for (let i = 0; i < 2; i++) {
                        let card = document.getElementById('sensor-card-' + i);
                        if (data.sensors && data.sensors[i]) {
                            card.style.display = 'block';
                            document.getElementById('sensor-title-' + i).innerText = data.sensors[i].type + " (" + data.sensors[i].address + ")" + (i === 0 ? " (innen)" : " (außen)");
                            document.getElementById('temp-' + i).innerText = data.sensors[i].temperature !== null ? data.sensors[i].temperature.toFixed(1) + " °C" : "--";
                            document.getElementById('hum-' + i).innerText = data.sensors[i].humidity !== null ? data.sensors[i].humidity.toFixed(1) + " %" : "--";
                            document.getElementById('dp-' + i).innerText = data.sensors[i].dewpoint !== null ? data.sensors[i].dewpoint.toFixed(1) + " °C" : "--";
                            if (data.sensors[i].type === "BME280" && data.sensors[i].pressure !== null && data.sensors[i].pressure !== undefined) {
                                document.getElementById('press-row-' + i).style.visibility = 'visible';
                                document.getElementById('press-' + i).innerText = data.sensors[i].pressure.toFixed(1) + " hPa";
                            } else {
                                document.getElementById('press-row-' + i).style.visibility = 'hidden';
                                document.getElementById('press-' + i).innerText = "--";
                            }
                        } else {
                            card.style.display = 'none';
                        }
                    }

                    // Update light sensors
                    for (let i = 0; i < 2; i++) {
                        let lightCard = document.getElementById('light-card-' + i);
                        if (data.lights && data.lights[i]) {
                            lightCard.style.display = 'block';
                            document.getElementById('light-title-' + i).innerText = "TSL2561 (" + data.lights[i].address + ")";
                            document.getElementById('lux-val-' + i).innerText = data.lights[i].lux !== null ? data.lights[i].lux.toFixed(1) + " Lux" : "--";
                            document.getElementById('broadband-val-' + i).innerText = data.lights[i].broadband;
                            document.getElementById('ir-val-' + i).innerText = data.lights[i].ir;
                        } else {
                            lightCard.style.display = 'none';
                        }
                    }

                    // Update potentiometers
                    let potValA = data.potentiometers.poti_a_target_hum;
                    let displayA = potValA.toFixed(0) + " %";
                    if (potValA <= 49.5) {
                        displayA = "Rigoros ZU";
                    } else if (potValA >= 70.5) {
                        displayA = "Rigoros AUF";
                    }
                    document.getElementById('poti-a').innerText = displayA;
                    document.getElementById('poti-b').innerText = data.potentiometers.poti_b_gain.toFixed(0) + " %";
                    document.getElementById('poti-c').innerText = data.potentiometers.poti_c_cal_offset.toFixed(0) + " °";

                    // Update Rotor & Servo card status dynamically
                    document.getElementById('rotor-pos').innerText = data.rotor_position.toFixed(0) + " %";
                    const m = document.getElementById('luna');
                    if (m) m.style.backgroundColor = '#191b28';
                    setMoon(data.rotor_position, data.espnow_role === 2);

                    // Update ESP-NOW card status dynamically
                    let espnowCard = document.getElementById('espnow-card');
                    if (data.espnow_role > 0) {
                        espnowCard.style.display = 'block';
                        let roleText = data.espnow_role === 1 ? "MASTER" : "SLAVE";
                        document.getElementById('espnow-val-role').innerHTML = "<strong>" + roleText + "</strong>";
                        
                        let connEl = document.getElementById('espnow-val-conn');
                        let lastSeenMs = data.espnow_last_seen_ms;
                        if (lastSeenMs === -1) {
                            connEl.innerText = "Keine Verbindung";
                            connEl.style.color = "#f87171";
                        } else if (lastSeenMs <= 15000) {
                            let intervalSec = ((data.espnow_interval_ms || 1000) / 1000).toFixed(3);
                            connEl.innerText = "Online (HB " + intervalSec + "s)";
                            connEl.style.color = "#4ade80";
                        } else {
                            connEl.innerText = "Offline (" + (lastSeenMs / 1000).toFixed(3) + "s)";
                            connEl.style.color = "#f87171";
                        }
                        
                        let pvEl = document.getElementById('espnow-val-pv');
                        if (data.espnow_pv_mismatch) {
                            pvEl.innerHTML = "<span style='color: #ef4444; font-weight: bold; display: inline-flex; align-items: center;'>V" + data.espnow_local_pv + 
                                             " <div class='tooltip'><span class='info-icon'>i</span><span class='tooltiptext'>Unterschiedliche Protokolle erkannt, bitte firmware auf gemeinsamen stand bringen.</span></div></span>";
                        } else {
                            pvEl.innerHTML = "<span>V" + data.espnow_local_pv + "</span>";
                        }
                    } else {
                        espnowCard.style.display = 'none';
                    }

                    // Update MQTT card status dynamically
                    let mqttCard = document.getElementById('mqtt-card');
                    if (data.mqtt_enabled) {
                        mqttCard.style.display = 'block';
                        document.getElementById('mqtt-title').innerText = "MQTT " + data.device_name;
                        document.getElementById('mqtt-broker').innerText = data.mqtt_server + ":" + data.mqtt_port;
                        
                        let statusEl = document.getElementById('mqtt-status');
                        if (data.mqtt_connected) {
                            statusEl.innerText = "connected";
                            statusEl.style.color = "#4ade80"; // green
                        } else {
                            statusEl.innerText = "try to connect";
                            statusEl.style.color = "#f87171"; // red
                        }
                        mqttCard.style.display = 'none';
                    }

                    const benchEl = document.getElementById('footer-bench');
                    if (benchEl) {
                        benchEl.innerText = data.loops_per_sec || 0;
                    }
                })
                .catch(err => {
                    // Connection lost to ESP32
                    document.getElementById('sys-ip').innerText = "try to reconnect to: [" + wifiSSID + "]";
                    document.getElementById('sys-ip').style.color = "#f87171";
                    document.getElementById('sys-rssi').innerText = "OFFLINE";
                    document.getElementById('sys-rssi').style.color = "#f87171";
                    
                    let mqttStatus = document.getElementById('mqtt-status');
                    if (mqttStatus) {
                        mqttStatus.innerText = "reconnecting";
                        mqttStatus.style.color = "#f87171";
                    }
                    let rotorPos = document.getElementById('rotor-pos');
                    if (rotorPos) rotorPos.innerText = "--";
                    setMoon(0);
                    const luna = document.getElementById('luna');
                    if (luna) luna.style.backgroundColor = '#f87171';
                });
        }

        let history120m = [];
        let history24h = [];

        function fetchHistory() {
            fetchWithTimeout('/api/history', { timeout: 2000 })
                .then(r => r.json())
                .then(data => {
                    if (data) {
                        history120m = data.h120m || [];
                        history24h = data.h24h || [];
                        renderAllCharts();
                    }
                }).catch(e => console.error("History fetch error", e));
        }

        function renderAllCharts() {
            renderCardChart('details-temp-0', 'cv-temp-0', 'temp', 0);
            renderCardChart('details-hum-0', 'cv-hum-0', 'hum', 0);
            renderCardChart('details-temp-1', 'cv-temp-1', 'temp', 1);
            renderCardChart('details-hum-1', 'cv-hum-1', 'hum', 1);
            renderCardChart('details-lux-0', 'cv-lux-0', 'lux', 0);
            renderCardChart('details-lux-1', 'cv-lux-1', 'lux', 1);
            renderCardChart('details-rotor', 'cv-rotor', 'rotor');
            renderCardChart('details-espnow', 'cv-espnow', 'espnow');
            renderCardChart('details-mqtt', 'cv-mqtt', 'mqtt');
            renderCardChart('details-rssi', 'cv-rssi', 'rssi');
        }

        function renderCardChart(detailsId, canvasId, type, index) {
            const details = document.getElementById(detailsId);
            if (details && !details.open) return;

            const canvas = document.getElementById(canvasId);
            if (!canvas) return;

            const boxW = canvas.offsetWidth || (canvas.parentElement ? canvas.parentElement.offsetWidth : 250);
            const boxH = canvas.offsetHeight || 50;
            if (boxW < 10) return;

            const ctx = canvas.getContext('2d');
            const dpr = window.devicePixelRatio || 1;
            const w = canvas.width = boxW * dpr;
            const h = canvas.height = boxH * dpr;

            ctx.clearRect(0, 0, w, h);
            if (!history120m || history120m.length === 0) return;

            const count = (type === 'rssi') ? 120 : 60;
            const data120m = history120m.slice(-count);

            let minY = 0, maxY = 100, midY = 50;
            let labelMax = "100", labelMid = "50", labelMin = "0";
            let greenLineVal = null;

            if (type === 'temp') {
                maxY = 50; minY = 0; midY = 25;
                labelMax = "50"; labelMid = "25"; labelMin = "0";
                greenLineVal = 25;
            } else if (type === 'hum') {
                maxY = 100; minY = 0; midY = 50;
                labelMax = "100"; labelMid = "50"; labelMin = "0";
                greenLineVal = 50;
            } else if (type === 'lux') {
                maxY = 1000; minY = 0; midY = 500;
                labelMax = "1000"; labelMid = "500"; labelMin = "0";
            } else if (type === 'rotor' || type === 'rssi') {
                maxY = 100; minY = 0; midY = 50;
                labelMax = "100"; labelMid = "50"; labelMin = "0";
            } else if (type === 'espnow' || type === 'mqtt') {
                maxY = 60; minY = 0; midY = 30;
                labelMax = "60"; labelMid = "30"; labelMin = "0";
            }

            ctx.fillStyle = '#64748b';
            ctx.font = `${9 * dpr}px monospace`;
            ctx.textBaseline = 'top';
            ctx.fillText(labelMax, 2 * dpr, 2 * dpr);
            ctx.textBaseline = 'middle';
            ctx.fillText(labelMid, 2 * dpr, h / 2);
            ctx.textBaseline = 'bottom';
            ctx.fillText(labelMin, 2 * dpr, h - 6 * dpr);

            const marginL = 28 * dpr;
            const chartW = w - marginL;
            const chartH = h - 6 * dpr;

            const candleW = chartW / count;
            const offsetIndex = count - data120m.length;

            for (let i = 0; i < data120m.length; i++) {
                const d = data120m[i];
                let valMax = 0, valMin = 0;

                if (type === 'temp') {
                    valMax = (index === 0 ? d.t0 : d.t1);
                    valMin = (index === 0 ? d.t0_min : d.t1_min);
                } else if (type === 'hum') {
                    valMax = (index === 0 ? d.h0 : d.h1);
                    valMin = (index === 0 ? d.h0_min : d.h1_min);
                } else if (type === 'lux') {
                    valMax = (index === 0 ? d.l0 : d.l1);
                } else if (type === 'rotor') valMax = d.r;
                else if (type === 'espnow') valMax = d.el;
                else if (type === 'mqtt') valMax = d.ml;
                else if (type === 'rssi') {
                    let r = (d.rssi !== undefined && d.rssi !== null && d.rssi !== 0) ? d.rssi : -100;
                    valMax = Math.round((r + 100) * 10 / 7);
                }

                if (valMax === null || valMax === undefined || isNaN(valMax)) continue;
                if (valMin === null || valMin === undefined || isNaN(valMin)) valMin = valMax;

                if (valMax < minY) valMax = minY; if (valMax > maxY) valMax = maxY;
                if (valMin < minY) valMin = minY; if (valMin > maxY) valMin = maxY;

                const candleIndex = offsetIndex + i;
                const x1 = marginL + candleIndex * candleW;
                const x2 = marginL + (candleIndex + 1) * candleW;
                const barW = Math.max(1, x2 - x1);

                if (type === 'temp' || type === 'hum') {
                    const minH = ((valMin - minY) / (maxY - minY)) * chartH;
                    const maxH = ((valMax - minY) / (maxY - minY)) * chartH;
                    const yBase = chartH - minH;
                    const yTop = chartH - maxH;

                    // Base light-blue candle up to min value
                    ctx.fillStyle = '#38bdf8';
                    ctx.fillRect(x1, yBase, barW, minH);

                    // Yellow spike candle top segment (delta max-min within minute)
                    const spikeH = Math.max(2 * dpr, yBase - yTop);
                    ctx.fillStyle = '#facc15';
                    ctx.fillRect(x1, yTop, barW, spikeH);
                } else {
                    const valH = ((valMax - minY) / (maxY - minY)) * chartH;
                    const y = chartH - valH;
                    if (type === 'espnow' || type === 'mqtt') {
                        ctx.fillStyle = '#ef4444';
                    } else if (type === 'rssi') {
                        const rssiGrad = ctx.createLinearGradient(0, chartH, 0, 0);
                        rssiGrad.addColorStop(0.0, '#ef4444');  // Rot (Schwacher Empfang)
                        rssiGrad.addColorStop(0.35, '#f97316'); // Orange
                        rssiGrad.addColorStop(0.65, '#eab308'); // Gelb
                        rssiGrad.addColorStop(1.0, '#22c55e');  // Grün (Starker Empfang)
                        ctx.fillStyle = rssiGrad;
                    } else {
                        ctx.fillStyle = '#38bdf8';
                    }
                    ctx.fillRect(x1, y, barW, valH);
                }
            }

            if (greenLineVal !== null) {
                const greenY = chartH - ((greenLineVal - minY) / (maxY - minY)) * chartH;
                ctx.strokeStyle = '#22c55e';
                ctx.lineWidth = 1.5 * dpr;
                ctx.setLineDash([3 * dpr, 3 * dpr]);
                ctx.beginPath();
                ctx.moveTo(marginL, greenY);
                ctx.lineTo(w, greenY);
                ctx.stroke();
                ctx.setLineDash([]);
            }

            const baseY = chartH + 1;
            ctx.strokeStyle = 'rgba(255,255,255,0.2)';
            ctx.lineWidth = 1 * dpr;
            ctx.beginPath();
            ctx.moveTo(marginL, baseY);
            ctx.lineTo(w, baseY);
            ctx.stroke();

            const tickStep = (type === 'rssi') ? 30 : 15;
            for (let i = 0; i <= count; i += tickStep) {
                const tx = marginL + i * candleW;
                const tickH = (i % (tickStep * 2) === 0) ? 4 * dpr : 2 * dpr;
                ctx.lineWidth = (i % (tickStep * 2) === 0) ? 2 * dpr : 1 * dpr;
                ctx.beginPath();
                ctx.moveTo(tx, baseY);
                ctx.lineTo(tx, baseY + tickH);
                ctx.stroke();
            }
        }

        let currentZoomType = '', currentZoomTitle = '';
        function openChartZoom(type, title) {
            currentZoomType = type;
            currentZoomTitle = title;
            document.getElementById('modal-title').innerText = title + " (24h Zoom)";
            document.getElementById('chart-modal').style.display = 'flex';
            requestAnimationFrame(() => {
                renderModalZoom();
                setTimeout(() => {
                    const canvas = document.getElementById('modal-canvas');
                    if (canvas && canvas.parentElement) {
                        canvas.parentElement.scrollLeft = canvas.parentElement.scrollWidth;
                    }
                }, 50);
            });
        }

        function closeChartModal() {
            document.getElementById('chart-modal').style.display = 'none';
        }

        function renderModalZoom() {
            const canvas = document.getElementById('modal-canvas');
            if (!canvas) return;

            const ctx = canvas.getContext('2d');
            const dpr = window.devicePixelRatio || 1;
            const container = canvas.parentElement;

            const totalSamples = 288;
            const containerWidth = container.offsetWidth || 600;
            const canvasW = Math.max(containerWidth, totalSamples * 5);
            const canvasH = 200;

            const w = canvas.width = canvasW * dpr;
            const h = canvas.height = canvasH * dpr;
            canvas.style.width = canvasW + "px";
            canvas.style.height = canvasH + "px";

            ctx.clearRect(0, 0, w, h);
            if (!history24h || history24h.length === 0) return;

            let type = currentZoomType, index = 0;
            if (type.startsWith('temp_')) { index = parseInt(type.split('_')[1]); type = 'temp'; }
            if (type.startsWith('hum_')) { index = parseInt(type.split('_')[1]); type = 'hum'; }
            if (type.startsWith('lux_')) { index = parseInt(type.split('_')[1]); type = 'lux'; }

            let minY = 0, maxY = 100, labelMax = "100", labelMid = "50", labelMin = "0", greenLineVal = null;
            if (type === 'temp') { maxY = 50; labelMax = "50"; labelMid = "25"; labelMin = "0"; greenLineVal = 25; }
            else if (type === 'hum') { maxY = 100; labelMax = "100"; labelMid = "50"; labelMin = "0"; greenLineVal = 50; }
            else if (type === 'lux') { maxY = 1000; labelMax = "1000"; labelMid = "500"; labelMin = "0"; }
            else if (type === 'rotor' || type === 'rssi') { maxY = 100; labelMax = "100"; labelMid = "50"; labelMin = "0"; }
            else if (type === 'espnow' || type === 'mqtt') { maxY = 60; labelMax = "60"; labelMid = "30"; labelMin = "0"; }

            ctx.fillStyle = '#94a3b8';
            ctx.font = `${11 * dpr}px monospace`;
            ctx.textBaseline = 'top'; ctx.fillText(labelMax, 6 * dpr, 6 * dpr);
            ctx.textBaseline = 'middle'; ctx.fillText(labelMid, 6 * dpr, h / 2);
            ctx.textBaseline = 'bottom'; ctx.fillText(labelMin, 6 * dpr, h - 12 * dpr);

            const marginL = 40 * dpr;
            const chartW = w - marginL;
            const chartH = h - 15 * dpr;
            const candleW = chartW / totalSamples;
            const offsetIndex = totalSamples - history24h.length;

            for (let i = 0; i < history24h.length; i++) {
                const d = history24h[i];
                let valMax = 0, valMin = 0;

                if (type === 'temp') {
                    valMax = (index === 0 ? d.t0 : d.t1);
                    valMin = (index === 0 ? d.t0_min : d.t1_min);
                } else if (type === 'hum') {
                    valMax = (index === 0 ? d.h0 : d.h1);
                    valMin = (index === 0 ? d.h0_min : d.h1_min);
                } else if (type === 'lux') {
                    valMax = (index === 0 ? d.l0 : d.l1);
                } else if (type === 'rotor') valMax = d.r;
                else if (type === 'espnow') valMax = d.el;
                else if (type === 'mqtt') valMax = d.ml;
                else if (type === 'rssi') {
                    let r = (d.rssi !== undefined && d.rssi !== null && d.rssi !== 0) ? d.rssi : -100;
                    valMax = Math.round((r + 100) * 10 / 7);
                }

                if (valMax === null || valMax === undefined || isNaN(valMax)) continue;
                if (valMin === null || valMin === undefined || isNaN(valMin)) valMin = valMax;

                if (valMax < minY) valMax = minY; if (valMax > maxY) valMax = maxY;
                if (valMin < minY) valMin = minY; if (valMin > maxY) valMin = maxY;

                const candleIndex = offsetIndex + i;
                const x1 = marginL + candleIndex * candleW;
                const x2 = marginL + (candleIndex + 1) * candleW;
                const barW = Math.max(1, x2 - x1);

                if (type === 'temp' || type === 'hum') {
                    const minH = ((valMin - minY) / (maxY - minY)) * chartH;
                    const maxH = ((valMax - minY) / (maxY - minY)) * chartH;
                    const yBase = chartH - minH;
                    const yTop = chartH - maxH;

                    // Base light-blue candle body up to min value
                    ctx.fillStyle = '#38bdf8';
                    ctx.fillRect(x1, yBase, barW, minH);

                    // Yellow spike top segment (delta max-min within 5-min bucket)
                    const spikeH = Math.max(2 * dpr, yBase - yTop);
                    ctx.fillStyle = '#facc15';
                    ctx.fillRect(x1, yTop, barW, spikeH);
                } else {
                    const valH = ((valMax - minY) / (maxY - minY)) * chartH;
                    const y = chartH - valH;
                    if (type === 'espnow' || type === 'mqtt') {
                        ctx.fillStyle = '#ef4444';
                    } else if (type === 'rssi') {
                        const rssiGrad = ctx.createLinearGradient(0, chartH, 0, 0);
                        rssiGrad.addColorStop(0.0, '#ef4444');  // Rot
                        rssiGrad.addColorStop(0.35, '#f97316'); // Orange
                        rssiGrad.addColorStop(0.65, '#eab308'); // Gelb
                        rssiGrad.addColorStop(1.0, '#22c55e');  // Grün
                        ctx.fillStyle = rssiGrad;
                    } else {
                        ctx.fillStyle = '#38bdf8';
                    }
                    ctx.fillRect(x1, y, barW, valH);
                }
            }

            if (greenLineVal !== null) {
                const greenY = chartH - ((greenLineVal - minY) / (maxY - minY)) * chartH;
                ctx.strokeStyle = '#22c55e';
                ctx.lineWidth = 2 * dpr;
                ctx.setLineDash([5 * dpr, 5 * dpr]);
                ctx.beginPath(); ctx.moveTo(marginL, greenY); ctx.lineTo(w, greenY); ctx.stroke();
                ctx.setLineDash([]);
            }

            const baseY = chartH + 1;
            ctx.strokeStyle = 'rgba(255,255,255,0.2)';
            ctx.lineWidth = 1 * dpr;
            ctx.beginPath(); ctx.moveTo(marginL, baseY); ctx.lineTo(w, baseY); ctx.stroke();

            for (let i = 0; i <= totalSamples; i += 12) {
                const tx = marginL + i * candleW;
                ctx.lineWidth = 2 * dpr;
                ctx.beginPath(); ctx.moveTo(tx, baseY); ctx.lineTo(tx, baseY + 6 * dpr); ctx.stroke();
            }
        }

        const modalCanvas = document.getElementById('modal-canvas');
        if (modalCanvas) {
            function handleModalPointer(e) {
                if (!history24h || history24h.length === 0) return;
                const rect = modalCanvas.getBoundingClientRect();
                const clientX = e.clientX || (e.touches && e.touches[0] ? e.touches[0].clientX : 0);
                const clientY = e.clientY || (e.touches && e.touches[0] ? e.touches[0].clientY : 0);
                if (!clientX) return;

                const dpr = window.devicePixelRatio || 1;
                const cssX = clientX - rect.left;
                const cssY = clientY - rect.top;
                const mouseX = cssX * dpr;
                const mouseY = cssY * dpr;

                const totalSamples = 288;
                const marginL = 40 * dpr;
                const chartW = modalCanvas.width - marginL;
                const chartH = modalCanvas.height - 15 * dpr;
                const candleW = chartW / totalSamples;
                const offsetIndex = totalSamples - history24h.length;

                const candleIndex = Math.floor((mouseX - marginL) / candleW);
                const arrayIdx = candleIndex - offsetIndex;

                const popup = document.getElementById('canvas-floating-popup');
                const tooltipEl = document.getElementById('modal-tooltip');

                if (arrayIdx >= 0 && arrayIdx < history24h.length) {
                    const d = history24h[arrayIdx];
                    let type = currentZoomType, index = 0;
                    if (type.startsWith('temp_')) { index = parseInt(type.split('_')[1]); type = 'temp'; }
                    if (type.startsWith('hum_')) { index = parseInt(type.split('_')[1]); type = 'hum'; }
                    if (type.startsWith('lux_')) { index = parseInt(type.split('_')[1]); type = 'lux'; }

                    let valMax = 0, valMin = 0, minY = 0, maxY = 100, unit = "%";
                    if (type === 'temp') {
                        valMax = (index === 0 ? d.t0 : d.t1);
                        valMin = (index === 0 ? d.t0_min : d.t1_min);
                        maxY = 50; unit = " °C";
                    } else if (type === 'hum') {
                        valMax = (index === 0 ? d.h0 : d.h1);
                        valMin = (index === 0 ? d.h0_min : d.h1_min);
                        maxY = 100; unit = " %";
                    } else if (type === 'lux') {
                        valMax = (index === 0 ? d.l0 : d.l1); valMin = valMax;
                        maxY = 1000; unit = " Lux";
                    } else if (type === 'rotor') {
                        valMax = d.r; valMin = valMax; maxY = 100; unit = " %";
                    } else if (type === 'espnow') {
                        valMax = d.el; valMin = valMax; maxY = 49; unit = "s Loss";
                    } else if (type === 'mqtt') {
                        valMax = d.ml; valMin = valMax; maxY = 49; unit = "s Loss";
                    } else if (type === 'rssi') {
                        let r = (d.rssi !== undefined && d.rssi !== null && d.rssi !== 0) ? d.rssi : -100;
                        valMax = Math.round((r + 100) * 10 / 7); valMin = valMax; maxY = 100; unit = "% (" + r + " dBm)";
                    }

                    if (valMax === null || valMax === undefined || isNaN(valMax)) return;
                    if (valMin === null || valMin === undefined || isNaN(valMin)) valMin = valMax;

                    const minH = ((valMin - minY) / (maxY - minY)) * chartH;
                    const maxH = ((valMax - minY) / (maxY - minY)) * chartH;
                    const yBase = chartH - minH;
                    const yTop = chartH - maxH;

                    const minutesAgo = (history24h.length - 1 - arrayIdx) * 5;
                    let timeStr = "JETZT";
                    if (minutesAgo > 0) {
                        const hrs = Math.floor(minutesAgo / 60);
                        const mins = minutesAgo % 60;
                        timeStr = "-" + (hrs > 0 ? hrs + "h " : "") + mins + "m";
                    }

                    let isYellowSegment = (type === 'temp' || type === 'hum') && (mouseY <= yBase && mouseY >= yTop);
                    let detailTxt = "", badgeBorderColor = "#38bdf8";

                    if (isYellowSegment && Math.abs(valMax - valMin) > 0.05) {
                        let delta = (valMax - valMin).toFixed(1);
                        detailTxt = `Max: ${valMax.toFixed(1)}${unit} (Spike Delta: +${delta}${unit}) [${timeStr}]`;
                        badgeBorderColor = "#facc15";
                    } else if (type === 'temp' || type === 'hum') {
                        detailTxt = `Min: ${valMin.toFixed(1)}${unit} (Max: ${valMax.toFixed(1)}${unit}) [${timeStr}]`;
                        badgeBorderColor = "#38bdf8";
                    } else {
                        detailTxt = `${valMax.toFixed(1)}${unit} [${timeStr}]`;
                        badgeBorderColor = (type === 'espnow' || type === 'mqtt') ? "#f87171" : (type === 'rssi' ? "#22c55e" : "#38bdf8");
                    }

                    if (popup) {
                        const targetY = isYellowSegment ? yTop : (type === 'temp' || type === 'hum' ? yBase : chartH - maxH);
                        const cssTargetX = (marginL + (candleIndex + 0.5) * candleW) / dpr;
                        const cssTargetY = (targetY / dpr) - 15;

                        popup.style.display = 'block';
                        popup.style.left = cssTargetX + 'px';
                        popup.style.top = cssTargetY + 'px';
                        popup.style.borderColor = badgeBorderColor;
                        popup.innerText = detailTxt;
                    }

                    if (tooltipEl) {
                        tooltipEl.innerText = detailTxt;
                        tooltipEl.style.color = badgeBorderColor;
                    }
                }
            }

            modalCanvas.addEventListener('pointerdown', handleModalPointer);
            modalCanvas.addEventListener('pointermove', handleModalPointer);
        }

        setInterval(updateData, 1000);
        setInterval(fetchHistory, 1000);
        updateData();
        fetchHistory();
    </script>
</body>
</html>
)rawhtml";
    server.send(200, "text/html", html);
  } else {
    // Show Wi-Fi setup captive portal
    int n = WiFi.scanNetworks();
    String wifiOptions = "";
    for (int i = 0; i < n; ++i) {
      wifiOptions += "<option value=\"" + WiFi.SSID(i) + "\">" + WiFi.SSID(i) +
                     " (" + String(WiFi.RSSI(i)) + " dBm)</option>";
    }

    String html = R"rawhtml(
<!DOCTYPE html>
<html lang="de">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>IDRY-26 Device Setup</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; }
        body {
            background: linear-gradient(135deg, #0f172a 0%, #1e1b4b 100%);
            color: #f8fafc;
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            padding: 20px;
        }
        .container {
            background: rgba(30, 41, 59, 0.45);
            backdrop-filter: blur(12px);
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 20px;
            padding: 30px;
            width: 100%;
            max-width: 500px;
            box-shadow: 0 20px 25px -5px rgba(0, 0, 0, 0.5);
        }
        h1 { text-align: center; margin-bottom: 25px; font-size: 24px; font-weight: 600; letter-spacing: 1px; color: #818cf8; }
        .section-title { font-size: 14px; text-transform: uppercase; letter-spacing: 2px; color: #94a3b8; margin: 15px 0 10px 0; font-weight: bold; border-bottom: 1px solid rgba(255,255,255,0.05); padding-bottom: 5px;}
        .form-group { margin-bottom: 18px; }
        label { display: block; font-size: 13px; color: #cbd5e1; margin-bottom: 6px; }
        input, select {
            width: 100%;
            padding: 12px 16px;
            background: rgba(15, 23, 42, 0.6);
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 10px;
            color: white;
            font-size: 14px;
            outline: none;
        }
        input:focus, select:focus { border-color: #6366f1; }
        .btn {
            width: 100%;
            padding: 14px;
            background: linear-gradient(135deg, #6366f1 0%, #4f46e5 100%);
            border: none;
            border-radius: 10px;
            color: white;
            font-size: 16px;
            font-weight: 600;
            cursor: pointer;
            margin-top: 10px;
        }
        .footer { text-align: center; margin-top: 20px; font-size: 11px; color: #64748b; }
    </style>
</head>
<body>
    <div class="container">
        <h1>IDRY-26 Configuration</h1>
        <form action="/save" method="POST">
            <div class="section-title">Wi-Fi Verbindung</div>
            <div class="form-group">
                <label for="ssid">Netzwerk (SSID)</label>
                <select name="ssid" id="ssid">
                    <option value="">Wähle ein Netzwerk...</option>
)rawhtml";
    html += wifiOptions;
    html += R"rawhtml(
                </select>
                <input type="text" name="ssid_custom" placeholder="Oder manuell eingeben..." style="margin-top: 8px;">
            </div>
            <div class="form-group">
                <label for="pass">Wi-Fi Passwort</label>
                <input type="password" name="pass" id="pass" placeholder="Passwort eingeben">
            </div>

            <div class="section-title">MQTT Konfiguration</div>
            <div class="form-group">
                <label for="mqtt_server">MQTT Broker Adresse</label>
                <input type="text" name="mqtt_server" id="mqtt_server" placeholder="z.B. 192.168.1.100" required>
            </div>
            <div class="form-group">
                <label for="mqtt_port">MQTT Port</label>
                <number name="mqtt_port" id="mqtt_port" value="1883" required>
            </div>
            <div class="form-group">
                <label for="mqtt_user">MQTT Benutzername (optional)</label>
                <input type="text" name="mqtt_user" id="mqtt_user" placeholder="Benutzername">
            </div>
            <div class="form-group">
                <label for="mqtt_pass">MQTT Passwort (optional)</label>
                <input type="password" name="mqtt_pass" id="mqtt_pass" placeholder="Passwort">
            </div>
            <div class="form-group">
                <label for="mqtt_device">Gerätename in Home Assistant</label>
                <input type="text" name="mqtt_device" id="mqtt_device" placeholder="z.B. growbox_display">
            </div>

            <button type="submit" class="btn">Speichern & Verbinden</button>
        </form>
        <div class="footer">IDRY26 IoT Device Config Portal</div>
    </div>
</body>
</html>
)rawhtml";
    server.send(200, "text/html", html);
  }
}

void handlePortalSave() {
  String ssid = server.arg("ssid");
  String custom_ssid = server.arg("ssid_custom");
  if (custom_ssid.length() > 0) {
    ssid = custom_ssid;
  }
  String pass = server.arg("pass");
  String mqtt_server = server.arg("mqtt_server");
  int mqtt_port = server.arg("mqtt_port").toInt();
  String mqtt_user = server.arg("mqtt_user");
  String mqtt_pass = server.arg("mqtt_pass");
  String mqtt_device = server.arg("mqtt_device");

  strlcpy(sysConfig.wifi_ssid, ssid.c_str(), sizeof(sysConfig.wifi_ssid));
  strlcpy(sysConfig.wifi_pass, pass.c_str(), sizeof(sysConfig.wifi_pass));
  strlcpy(sysConfig.mqtt_server, mqtt_server.c_str(),
          sizeof(sysConfig.mqtt_server));
  sysConfig.mqtt_port = (mqtt_port > 0) ? mqtt_port : 1883;
  strlcpy(sysConfig.mqtt_user, mqtt_user.c_str(), sizeof(sysConfig.mqtt_user));
  strlcpy(sysConfig.mqtt_pass, mqtt_pass.c_str(), sizeof(sysConfig.mqtt_pass));

  if (mqtt_device.length() > 0) {
    strlcpy(sysConfig.mqtt_device_name, mqtt_device.c_str(),
            sizeof(sysConfig.mqtt_device_name));
  } else {
    strlcpy(sysConfig.mqtt_device_name, apSSID.c_str(),
            sizeof(sysConfig.mqtt_device_name));
  }

  saveConfiguration();

  String html = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Einstellungen gespeichert</title>
    <style>
        body { background: #0f172a; color: white; text-align: center; padding-top: 100px; font-family: sans-serif; }
        .box { background: #1e293b; padding: 40px; border-radius: 15px; display: inline-block; }
        h1 { color: #818cf8; margin-bottom: 20px; }
    </style>
</head>
<body>
    <div class="box">
        <h1>Einstellungen gespeichert!</h1>
        <p>Der ESP32 startet nun neu und verbindet sich mit <strong>)rawhtml";
  html += ssid + R"rawhtml(</strong>.</p>
        <p>Bitte verbinde dein Gerät wieder mit deinem Heimnetzwerk.</p>
    </div>
    <script>setTimeout(function(){ window.location.href = '/'; }, 5000);</script>
</body>
</html>
)rawhtml";
  server.send(200, "text/html", html);
  delay(2000);
  ESP.restart();
}

void handleSettingsPage() {
  bool hasLocalSensor = (detectedTempSensors > 0) ||
                        (tempSensors[0].active || tempSensors[1].active);
  String html = R"rawhtml(
<!DOCTYPE html>
<html lang="de">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Settings - IDRY-26</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; }
        body {
            background: )rawhtml";
  if (sysConfig.espnow_role == 2) {
    html += "linear-gradient(135deg, #1e1b1b 0%, #450a0a 100%);";
  } else {
    html += "linear-gradient(135deg, #0f172a 0%, #1e1b4b 100%);";
  }
  html += R"rawhtml(
            color: #f8fafc;
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            padding: 20px;
        }
        .container {
            width: 100%;
            max-width: 550px;
        }
        .header-title {
            text-align: center;
            margin-bottom: 25px;
            font-size: 26px;
            font-weight: 600;
            letter-spacing: 1px;
            color: #818cf8;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 10px;
        }
        .settings-card {
            background: rgba(30, 41, 59, 0.45);
            backdrop-filter: blur(12px);
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 20px;
            padding: 25px;
            margin-bottom: 20px;
            box-shadow: 0 15px 20px -5px rgba(0, 0, 0, 0.4);
        }
        .section-title {
            font-size: 14px;
            text-transform: uppercase;
            letter-spacing: 2px;
            color: #94a3b8;
            margin-bottom: 18px;
            font-weight: bold;
            border-bottom: 1px solid rgba(255,255,255,0.05);
            padding-bottom: 6px;
            display: flex;
            align-items: center;
            gap: 8px;
        }
        .form-group { margin-bottom: 18px; }
        .form-group:last-child { margin-bottom: 0; }
        label { display: block; font-size: 13px; color: #cbd5e1; margin-bottom: 6px; }
        input, select {
            width: 100%;
            padding: 12px 16px;
            background: rgba(15, 23, 42, 0.6);
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 10px;
            color: white;
            font-size: 14px;
            outline: none;
            transition: border-color 0.2s;
        }
        input:focus, select:focus { border-color: #6366f1; }
        .slider-container { display: flex; align-items: center; gap: 15px; }
        .slider { flex-grow: 1; height: 6px; background: rgba(15, 23, 42, 0.6); outline: none; border-radius: 3px; -webkit-appearance: none; }
        .slider::-webkit-slider-thumb { -webkit-appearance: none; width: 18px; height: 18px; border-radius: 50%; background: #6366f1; cursor: pointer; transition: background 0.15s; }
        .slider::-webkit-slider-thumb:hover { background: #818cf8; }
        .btn-row { display: flex; gap: 10px; margin-top: 25px; }
        .btn {
            flex: 1;
            padding: 14px;
            border: none;
            border-radius: 10px;
            color: white;
            font-size: 15px;
            font-weight: 600;
            cursor: pointer;
            text-align: center;
            text-decoration: none;
            transition: all 0.2s;
            display: inline-block;
        }
        .btn-save { background: rgba(30, 41, 59, 0.6); border: 1px solid #f87171; color: #f87171 !important; }
        .btn-save:hover { background: #f87171; color: white !important; }
        .btn-back { background: rgba(255, 255, 255, 0.1); border: 1px solid rgba(255, 255, 255, 0.15); display: flex; align-items: center; justify-content: center; }
        .btn-back:hover { background: rgba(255, 255, 255, 0.2); }
        .btn-secondary { background: rgba(129, 140, 248, 0.15); border: 1px solid rgba(129, 140, 248, 0.3); color: #818cf8; }
        .btn-secondary:hover { background: rgba(129, 140, 248, 0.3); }
        .btn-danger { background: rgba(239, 68, 68, 0.15); border: 1px solid rgba(239, 68, 68, 0.3); color: #ef4444; }
        .btn-danger:hover { background: rgba(239, 68, 68, 0.35); }
        .btn-danger.confirm-step { background: #dc2626 !important; border-color: #ef4444 !important; color: white !important; animation: pulse-border 1.5s infinite; }
        .hint-text { font-size: 11px; color: #94a3b8; margin-top: 5px; display: block; font-family: monospace; }
        .footer { text-align: center; margin-top: 25px; font-size: 11px; color: #64748b; }
        @keyframes pulse-border {
            0% { box-shadow: 0 0 0 0 rgba(239, 68, 68, 0.7); }
            70% { box-shadow: 0 0 0 10px rgba(239, 68, 68, 0); }
            100% { box-shadow: 0 0 0 0 rgba(239, 68, 68, 0); }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1 class="header-title">
            <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="3"></circle><path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 1 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-4 0v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 1 1-2.83-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1 0-4h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 1 1 2.83-2.83l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 4 0v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 1 1 2.83 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1z"></path></svg>
            Einstellungen
        </h1>
        
        <form action="/settings/save" method="POST" id="settings-form">
            <!-- WLAN Einstellungen Panel -->
            <div class="settings-card">
                <div class="section-title">
                    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M5 12.55a11 11 0 0 1 14.08 0"></path><path d="M1.42 9a16 16 0 0 1 21.16 0"></path><path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path><circle cx="12" cy="20" r="1"></circle></svg>
                    WLAN Verbindung
                </div>
                <div class="form-group">
                    <label for="wifi_ssid">Netzwerk (SSID)</label>
                    <input type="text" name="wifi_ssid" id="wifi_ssid" value=")rawhtml";
  html += String(sysConfig.wifi_ssid);
  html += R"rawhtml(" required>
                </div>
                <div class="form-group">
                    <label for="wifi_pass">Wi-Fi Passwort</label>
                    <input type="password" name="wifi_pass" id="wifi_pass" value=")rawhtml";
  html += String(sysConfig.wifi_pass);
  html += R"rawhtml(" placeholder="Passwort eingeben">
                </div>
                <div class="form-group">
                    <label for="wifi_tx_power">Sendeleistung (RF TX Power)</label>
                    <select name="wifi_tx_power" id="wifi_tx_power">
                        <option value="78" style="color: #f87171;")rawhtml";
  if (sysConfig.wifi_tx_power == 78)
    html += " selected";
  html += R"rawhtml(>19.5 dBm (Maximum - Risiko!)</option>
                        <option value="68" style="color: #f87171;")rawhtml";
  if (sysConfig.wifi_tx_power == 68)
    html += " selected";
  html += R"rawhtml(>17.0 dBm (Hoch - Risiko!)</option>
                        <option value="60" style="color: #f87171;")rawhtml";
  if (sysConfig.wifi_tx_power == 60)
    html += " selected";
  html += R"rawhtml(>15.0 dBm (Mittel - Warnung)</option>
                        <option value="52" style="color: #4ade80;")rawhtml";
  if (sysConfig.wifi_tx_power == 52)
    html += " selected";
  html += R"rawhtml(>13.0 dBm (Standard - Empfohlen)</option>
                        <option value="44")rawhtml";
  if (sysConfig.wifi_tx_power == 44)
    html += " selected";
  html += R"rawhtml(>11.0 dBm (Sehr Niedrig)</option>
                        <option value="34")rawhtml";
  if (sysConfig.wifi_tx_power == 34)
    html += " selected";
  html += R"rawhtml(>8.5 dBm (Minimum)</option>
                    </select>
                </div>
            </div>

            <!-- MQTT Einstellungen Panel -->
            <div class="settings-card">
                <div class="section-title">
                    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect x="3" y="3" width="18" height="18" rx="2" ry="2"></rect><line x1="9" y1="3" x2="9" y2="21"></line></svg>
                    MQTT Konfiguration
                </div>
                <div class="form-group">
                    <label for="mqtt_server">MQTT Broker Adresse</label>
                    <input type="text" name="mqtt_server" id="mqtt_server" value=")rawhtml";
  html += String(sysConfig.mqtt_server);
  html += R"rawhtml(" placeholder="z.B. 192.168.1.100">
                </div>
                <div class="form-group">
                    <label for="mqtt_port">MQTT Port</label>
                    <input type="number" name="mqtt_port" id="mqtt_port" value=")rawhtml";
  html += String(sysConfig.mqtt_port);
  html += R"rawhtml(" required>
                </div>
                <div class="form-group">
                    <label for="mqtt_user">MQTT Benutzername</label>
                    <input type="text" name="mqtt_user" id="mqtt_user" value=")rawhtml";
  html += String(sysConfig.mqtt_user);
  html += R"rawhtml(" placeholder="optional">
                </div>
                <div class="form-group">
                    <label for="mqtt_pass">MQTT Passwort</label>
                    <input type="password" name="mqtt_pass" id="mqtt_pass" value=")rawhtml";
  html += String(sysConfig.mqtt_pass);
  html += R"rawhtml(" placeholder="optional">
                </div>
                <div class="form-group">
                    <label for="mqtt_device_name">Gerätename (HA Discovery Name)</label>
                    <input type="text" name="mqtt_device_name" id="mqtt_device_name" value=")rawhtml";
  html += String(sysConfig.mqtt_device_name);
  html += R"rawhtml(" required>
                    <span class="hint-text">Publish Topic: <span id="topic-preview">idry/)rawhtml";
  html += String(sysConfig.mqtt_device_name);
  html += R"rawhtml(/state</span></span>
                </div>
                <div class="form-group">
                    <label for="interval-slider">MQTT Sende-Intervall: <span id="interval-label">)rawhtml";
  html += String(sysConfig.mqtt_report_interval);
  html += R"rawhtml( Minuten</span></label>
                    <div class="slider-container">
                        <input type="range" name="mqtt_report_interval" min="1" max="60" class="slider" id="interval-slider" value=")rawhtml";
  html += String(sysConfig.mqtt_report_interval);
  html += R"rawhtml(" required>
                    </div>
                </div>
            </div>

            <!-- ESP-NOW Einstellungen Panel -->
            <div class="settings-card">
                <div class="section-title">
                    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M18 8A6 6 0 0 0 6 8c0 7-3 9-3 9h18s-3-2-3-9"></path><path d="M13.73 21a2 2 0 0 1-3.46 0"></path></svg>
                    ESPNOW &nbsp;<span id="espnow-local-mac" style="font-family: monospace; text-transform: none; color: #94a3b8;">[Laden...]</span>
                </div>
                <div class="form-group">
                    <label for="espnow_role">Status / Rolle</label>
                    <select name="espnow_role" id="espnow_role" onchange="toggleEspNowFields()")rawhtml";
  if (strlen(sysConfig.espnow_peer_mac) > 0) {
    html += " disabled";
  }
  html += R"rawhtml(>
                        <option value="0")rawhtml";
  if (sysConfig.espnow_role == 0)
    html += " selected";
  html += R"rawhtml(>Deaktiviert</option>
                        <option value="1")rawhtml";
  if (sysConfig.espnow_role == 1)
    html += " selected";
  html += R"rawhtml(>Master</option>
                        <option value="2")rawhtml";
  if (sysConfig.espnow_role == 2)
    html += " selected";
  html += R"rawhtml(>Slave</option>
                    </select>
                </div>
                <div class="form-group" id="espnow-channel-group">
                    <label for="espnow_channel">Kanal (nur für Slave relevant)</label>
                    <select name="espnow_channel" id="espnow_channel")rawhtml";
  if (strlen(sysConfig.espnow_peer_mac) > 0) {
    html += " disabled";
  }
  html += R"rawhtml(>
)rawhtml";
  for (int c = 1; c <= 13; c++) {
    html += "                        <option value=\"" + String(c) + "\"";
    if (sysConfig.espnow_channel == c)
      html += " selected";
    html += ">Kanal " + String(c) + "</option>";
  }
  html += R"rawhtml(
                    </select>
                </div>
                <div class="form-group">
                    <label for="espnow_peer_mac">Partner MAC-Adresse</label>
                    <input type="text" name="espnow_peer_mac" id="espnow_peer_mac" value=")rawhtml";
  html += String(sysConfig.espnow_peer_mac);
  html +=
      R"rawhtml(" placeholder="XX:XX:XX:XX:XX:XX" pattern="^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$")rawhtml";
  if (strlen(sysConfig.espnow_peer_mac) > 0) {
    html += " readonly";
  }
  html += R"rawhtml(>
                    <span class="hint-text" id="espnow-scan-hint")rawhtml";
  if (strlen(sysConfig.espnow_peer_mac) > 0) {
    html += " style=\"display:none;\"";
  }
  html += R"rawhtml(>Leer lassen für automatischen Scan</span>
                </div>
                <div class="form-group">
                    <label for="espnow_failsafe_mode">Connection-Loss Fail-Safe (Slave)</label>)rawhtml";
  if (!hasLocalSensor) {
    html += R"rawhtml(
                    <div style="color: #f87171; font-size: 12px; margin-top: 6px; margin-bottom: 8px; font-weight: 500; line-height: 1.4;">
                        (Kein Sensor angeschlossen! -> Notfall 50% erzwungen)<br>
                        Bei Bedarf BME280 oder SHT31 anschließen<br>
                        Und dann nochmal hier im Menu aktivieren.
                    </div>)rawhtml";
  }
  html += R"rawhtml(
                    <select name="espnow_failsafe_mode" id="espnow_failsafe_mode">
                        <option value="0")rawhtml";
  if (sysConfig.espnow_failsafe_mode == 0 || !hasLocalSensor)
    html += " selected";
  html += R"rawhtml(>50% Rotor-Position (Notfall-Öffnung)</option>
                        <option value="1")rawhtml";
  if (sysConfig.espnow_failsafe_mode == 1 && hasLocalSensor)
    html += " selected";
  if (!hasLocalSensor)
    html += " disabled style=\"color: #64748b;\"";
  html += R"rawhtml(>Lokale Steuerung (Slave Poti A & Sensor) )rawhtml";
  if (!hasLocalSensor)
    html += "[Kein Sensor]";
  html += R"rawhtml(</option>
                    </select>
                    <span class="hint-text">Verhalten des Slaves bei Verbindungsverlust (>60s) zum Master</span>
                </div>
                <div class="form-group">
                    <label>Verbindungs-Status</label>
                    <div id="espnow-status" style="font-size: 13px; font-weight: 600; font-family: monospace; color: #f87171; margin-bottom: 8px;">
                        Warte auf Verbindung...
                    </div>
                    <span class="hint-text" id="espnow-pv-info" style="color: #38bdf8; display: block;">Protocol Version Local [v1]</span>
                    <span class="hint-text" id="espnow-pv-warning" style="color: #f87171; display: none; margin-top: 4px;">Protokoll-Unterschiede erkannt, bitte Firmware updaten auf eine gemeinsame Version.</span>
                </div>
                <div class="btn-row" style="margin-top: 15px;">
                    <button type="button" id="pair-btn" onclick="togglePairing()" class="btn btn-secondary")rawhtml";
  if (strlen(sysConfig.espnow_peer_mac) > 0) {
    html += " style=\"display:none;\"";
  }
  html += R"rawhtml(>Pairing starten</button>
                </div>
            </div>

            <!-- Buzzer Test Panel -->
            <div class="settings-card">
                <div class="section-title">
                    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M11 5L6 9H2v6h4l5 4V5z"></path><path d="M19.07 4.93a10 10 0 0 1 0 14.14M15.54 8.46a5 5 0 0 1 0 7.07"></path></svg>
                    Buzzer Test
                </div>
                <div class="btn-row" style="margin-top: 5px;">
                    <button type="button" onclick="testBuzzer('local')" class="btn btn-secondary">Lokal abspielen</button>
                    <button type="button" id="remote-buzz-btn" onclick="testBuzzer('remote')" class="btn btn-secondary" style="display: none;">Remote abspielen</button>
                </div>
            </div>

            <!-- Systemeinstellungen Panel -->
            <div class="settings-card">
                <div class="section-title">
                    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect x="2" y="3" width="20" height="14" rx="2" ry="2"></rect><line x1="8" y1="21" x2="16" y2="21"></line><line x1="12" y1="17" x2="12" y2="21"></line></svg>
                    System & Anzeige
                </div>
                <div class="form-group">
                    <label for="brightness-slider">Display-Helligkeit: <span id="brightness-label">)rawhtml";
  html += String(sysConfig.display_brightness);
  html += R"rawhtml(%</span></label>
                    <div class="slider-container">
                        <input type="range" name="display_brightness" min="0" max="100" class="slider" id="brightness-slider" value=")rawhtml";
  html += String(sysConfig.display_brightness);
  html += R"rawhtml(">
                    </div>
                    <span class="hint-text" style="font-family: inherit;">Natürliches Dimmverhalten über Gamma 2.2 Korrektur</span>
                </div>
                <div class="form-group">
                    <label for="servo-interval-slider">Servo Update-Intervall: <span id="servo-interval-label">)rawhtml";
  html += String(sysConfig.servo_update_interval);
  html += R"rawhtml( Sekunden</span></label>
                    <div class="slider-container">
                        <input type="range" name="servo_update_interval" min="1" max="30" class="slider" id="servo-interval-slider" value=")rawhtml";
  html += String(sysConfig.servo_update_interval);
  html += R"rawhtml(" required>
                    </div>
                </div>
                <div class="form-group">
                    <label for="wlan-time-trap-slider">WLAN connection watchdog time: <span id="wlan-time-trap-label">)rawhtml";
  if (sysConfig.wlan_time_trap == 0) {
    html += "0 <span style='color: #ef4444; font-weight: bold;'> "
            "(deaktiviert)</span>";
  } else {
    html += String(sysConfig.wlan_time_trap) + " Sekunden";
  }
  html += R"rawhtml(</span></label>
                    <div class="slider-container">
                        <input type="range" name="wlan_time_trap" min="0" max="330" class="slider" id="wlan-time-trap-slider" value=")rawhtml";
  html += String(sysConfig.wlan_time_trap);
  html +=
      R"rawhtml(" required>
                    </div>
                </div>
            </div>

            <div class="btn-row">
                <button type="submit" class="btn btn-save">Save</button>
                <a href="/" class="btn btn-back">Back</a>
            </div>
        </form>

        <!-- System Status Panel -->
        <div class="settings-card" style="margin-top: 25px;">
            <div class="section-title">
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect x="2" y="3" width="20" height="14" rx="2" ry="2"></rect><line x1="8" y1="21" x2="16" y2="21"></line><line x1="12" y1="17" x2="12" y2="21"></line></svg>
                System Status
            </div>
            <div class="value-row" style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; font-size: 13px;">
                <span>IP-Adresse:</span>
                <span class="val" id="sys-ip" style="font-family: monospace; color: #38bdf8; font-weight: 600;">--</span>
            </div>
            <div class="value-row" style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; font-size: 13px;">
                <span>Anzeige-Modus:</span>
                <span class="val" id="sys-mode" style="font-family: monospace; font-weight: 600;">--</span>
            </div>
            <div class="value-row" style="display: flex; justify-content: space-between; align-items: center; font-size: 13px;">
                <span style="display: flex; align-items: center; gap: 10px;">
                    Signalstärke RSSI:
                    <div style="width: 50px; height: 8px; background: rgba(255,255,255,0.15); border-radius: 4px; overflow: hidden; display: inline-block;">
                        <div id="sys-rssi-bar" style="width: 0%; height: 100%; transition: width 0.3s, background-color 0.3s; background: #ef4444;"></div>
                    </div>
                </span>
                <span class="val" id="sys-rssi" style="font-family: monospace; color: #38bdf8; font-weight: 600;">--</span>
            </div>
            <div class="value-row" style="display: flex; justify-content: space-between; align-items: center; margin-top: 12px; font-size: 13px;">
                <span>Watchdog reset weekly:</span>
                <span class="val" id="settings-wd-reset" style="font-family: monospace; font-weight: 600;">--</span>
            </div>
        </div>

        <!-- Geräte-Management Panel -->
        <div class="settings-card" style="margin-top: 25px;">
            <div class="section-title" style="color: #f87171;">
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"></path><line x1="12" y1="9" x2="12" y2="13"></line><line x1="12" y1="17" x2="12.01" y2="17"></line></svg>
                Geräte-Management
            </div>
            <form id="reset-form" action="/settings/reset" method="POST">
                <div class="btn-row" style="margin-top: 5px; flex-direction: column; gap: 12px;">
                    <a href="/firmware" class="btn btn-secondary" style="width:100%; border-color: rgba(129, 140, 248, 0.4); color: #818cf8; text-decoration: none; text-align: center; display: block; box-sizing: border-box;">Firmware & OTA Update</a>
                    <button type="submit" name="action" value="reboot" class="btn btn-secondary" style="width:100%; border-color: rgba(74, 222, 128, 0.4); color: #4ade80;">Reboot Device</button>
                    <button type="submit" name="action" value="defaults" class="btn btn-secondary" style="width:100%;">Restore Defaults (ohne WLAN/MQTT)</button>
                    <button type="submit" name="action" value="delete_espnow" class="btn btn-secondary" style="width:100%; border-color: rgba(239, 68, 68, 0.4); color: #f87171;">Delete ESPNOW connections</button>
                    <button type="button" id="complete-reset-btn" class="btn btn-danger" style="width:100%;">Complete Reset</button>
                    <input type="hidden" name="action" id="reset-action" value="">
                </div>
            </form>
        </div>

        <div class="footer" id="footer-text">IDRY26 Live Monitor v)rawhtml" +
      String("1.") + String(localFirmwareVersion) +
      R"rawhtml( - (bench: <span id="footer-bench-settings" style="font-family: monospace; color: #38bdf8; font-weight: bold;">--</span> loops/s)</div>
    </div>

    <script>
        // Real-time brightness slider update
        const brightnessSlider = document.getElementById('brightness-slider');
        const brightnessLabel = document.getElementById('brightness-label');
        brightnessSlider.oninput = function() {
            brightnessLabel.innerText = this.value + "%";
        }

        // Real-time report interval slider update
        const intervalSlider = document.getElementById('interval-slider');
        const intervalLabel = document.getElementById('interval-label');
        intervalSlider.oninput = function() {
            intervalLabel.innerText = this.value + " Minuten";
        }

        // Real-time servo update interval slider update
        const servoIntervalSlider = document.getElementById('servo-interval-slider');
        const servoIntervalLabel = document.getElementById('servo-interval-label');
        servoIntervalSlider.oninput = function() {
            servoIntervalLabel.innerText = this.value + " Sekunden";
        }

        // Real-time WLAN Time Trap slider update
        const trapSlider = document.getElementById('wlan-time-trap-slider');
        const trapLabel = document.getElementById('wlan-time-trap-label');
        trapSlider.oninput = function() {
            if (parseInt(this.value) === 0) {
                trapLabel.innerHTML = this.value + " <span style='color: #ef4444; font-weight: bold;'> (deaktiviert)</span>";
            } else {
                trapLabel.innerHTML = this.value + " Sekunden";
            }
        }

        // Real-time HA topic preview path update
        const deviceInput = document.getElementById('mqtt_device_name');
        const topicPreview = document.getElementById('topic-preview');
        deviceInput.oninput = function() {
            const cleanVal = this.value.trim() || "device_name";
            topicPreview.innerText = "idry/" + cleanVal + "/state";
        }

        // ESP-NOW UI State Updates
        function toggleEspNowFields() {
            const role = document.getElementById('espnow_role').value;
            const chanGroup = document.getElementById('espnow-channel-group');
            const chanSelect = document.getElementById('espnow_channel');
            if (role === "1" || role === "0") {
                chanSelect.disabled = true;
                chanGroup.style.opacity = "0.5";
            } else {
                chanSelect.disabled = false;
                chanGroup.style.opacity = "1";
            }
        }
        
        let pairingActive = false;
        let lastMismatchTime = 0;
        function togglePairing() {
            const btn = document.getElementById('pair-btn');
            const action = pairingActive ? 'stop' : 'start';
            const role = document.getElementById('espnow_role').value;
            const channel = document.getElementById('espnow_channel').value;
            let url = '/api/espnow/pair?action=' + action;
            if (action === 'start') {
                url += '&role=' + role + '&channel=' + channel;
            }
            fetch(url)
                .then(r => r.json())
                .then(data => {
                    if (data.status === 'ok') {
                        pairingActive = !pairingActive;
                        btn.innerText = pairingActive ? 'Pairing abbrechen' : 'Pairing starten';
                        if (pairingActive) {
                            btn.classList.add('confirm-step');
                        } else {
                            btn.classList.remove('confirm-step');
                        }
                    } else {
                        alert(data.message || 'Error executing action');
                    }
                }).catch(err => console.error(err));
        }

        function testBuzzer(type) {
            fetch('/api/espnow/buzzer_test?type=' + type)
                .then(r => r.json())
                .then(data => {
                    if (data.status !== 'ok') {
                        alert(data.message || 'Fehler beim Buzzer-Test');
                    }
                }).catch(err => console.error(err));
        }

        // Poll real-time data for ESP-NOW and MAC Addresses
        function pollEspNowStatus() {
            fetch('/api/data')
                .then(r => r.json())
                .then(data => {
                    const wifiChannel = data.wifi_channel || 1;
                    const role = parseInt(document.getElementById('espnow_role').value) || 0;
                    
                    if (role === 2) {
                        document.body.style.background = "linear-gradient(135deg, #1e1b1b 0%, #450a0a 100%)";
                    } else {
                        document.body.style.background = "linear-gradient(135deg, #0f172a 0%, #1e1b4b 100%)";
                    }
                    
                    document.getElementById('espnow-local-mac').innerText = "[" + (data.wifi_mac || "") + "]";
                    
                    if (role === 1) {
                        document.getElementById('espnow_channel').value = wifiChannel;
                    }
                    
                    const statusDiv = document.getElementById('espnow-status');
                    const lastSeenMs = data.espnow_last_seen_ms;
                    
                    if (role === 0) {
                        statusDiv.innerText = "ESP-NOW deaktiviert";
                        statusDiv.style.color = "#94a3b8";
                    } else if (lastSeenMs === -1) {
                        statusDiv.innerText = "Nie gesehen / Keine Verbindung";
                        statusDiv.style.color = "#f87171";
                    } else if (lastSeenMs <= 15000) {
                        const intervalSec = ((data.espnow_interval_ms || 1000) / 1000).toFixed(3);
                        statusDiv.innerText = "Online (HB " + intervalSec + "s)";
                        statusDiv.style.color = "#4ade80";
                    } else {
                        statusDiv.innerText = "Offline (Kontakt: " + (lastSeenMs / 1000).toFixed(3) + "s)";
                        statusDiv.style.color = "#f87171";
                    }
                    
                    // Update Protocol Version Info static line
                    const pvInfoEl = document.getElementById('espnow-pv-info');
                    const pvWarnEl = document.getElementById('espnow-pv-warning');
                    const peerMac = data.espnow_peer_mac || "";
                    
                    if (peerMac.length > 0) {
                        pvInfoEl.innerText = "Protocol Version Local [v" + data.espnow_local_pv + "] Partner [v" + data.espnow_remote_pv + "]";
                    } else {
                        pvInfoEl.innerText = "Protocol Version Local [v" + data.espnow_local_pv + "]";
                    }
                    
                    if (data.espnow_pv_mismatch) {
                        pvWarnEl.style.display = "block";
                        lastMismatchTime = Date.now();
                    } else {
                        // Hold the warning visible for at least 2000ms
                        if (!lastMismatchTime || (Date.now() - lastMismatchTime > 2000)) {
                            pvWarnEl.style.display = "none";
                        }
                    }
                    
                    const roleSelect = document.getElementById('espnow_role');
                    const chanSelect = document.getElementById('espnow_channel');
                    if (peerMac.length > 0) {
                        roleSelect.disabled = true;
                        chanSelect.disabled = true;
                    } else {
                        roleSelect.disabled = false;
                        toggleEspNowFields();
                    }
                    
                    const peerInput = document.getElementById('espnow_peer_mac');
                    if (peerInput) {
                        if (peerMac.length > 0) {
                            peerInput.readOnly = true;
                        } else {
                            peerInput.readOnly = false;
                        }
                        if (document.activeElement !== peerInput) {
                            peerInput.value = peerMac;
                        }
                    }
                    const scanHint = document.getElementById('espnow-scan-hint');
                    if (scanHint) {
                        if (peerMac.length > 0) {
                            scanHint.style.display = 'none';
                        } else {
                            scanHint.style.display = 'inline';
                        }
                    }
                    const remoteBtn = document.getElementById('remote-buzz-btn');
                    if (peerMac.length > 0) {
                        remoteBtn.style.display = "inline-block";
                    } else {
                        remoteBtn.style.display = "none";
                    }
                    
                    pairingActive = data.espnow_pairing || false;
                    const pairBtn = document.getElementById('pair-btn');
                    if (peerMac.length > 0 && !pairingActive) {
                        pairBtn.style.display = 'none';
                    } else {
                        pairBtn.style.display = 'inline-block';
                        if (pairingActive) {
                            pairBtn.innerText = 'Pairing abbrechen';
                            pairBtn.classList.add('confirm-step');
                        } else {
                            pairBtn.innerText = 'Pairing starten';
                            pairBtn.classList.remove('confirm-step');
                        }
                    }

                    // Update System Status card on Settings page
                    const sysIpEl = document.getElementById('sys-ip');
                    if (sysIpEl) {
                        sysIpEl.innerText = data.ip_address || "--";
                        sysIpEl.style.color = (data.ip_address && data.ip_address.startsWith("try")) ? "#f87171" : "#38bdf8";
                    }
                    const sysModeEl = document.getElementById('sys-mode');
                    if (sysModeEl) {
                        sysModeEl.innerText = data.mode || "--";
                    }
                    const settingsRssiEl = document.getElementById('sys-rssi');
                    if (settingsRssiEl) {
                        let rssi = parseInt(data.rssi) || 0;
                        if (rssi === 0) rssi = -100;
                        settingsRssiEl.innerText = rssi + " dBm";
                        
                        let pct = Math.round((rssi + 100) * 10 / 7);
                        if (pct < 0) pct = 0;
                        if (pct > 100) pct = 100;
                        
                        const rssiBar = document.getElementById('sys-rssi-bar');
                        if (rssiBar) {
                            rssiBar.style.width = pct + "%";
                            if (rssi >= -50) {
                                rssiBar.style.backgroundColor = "#22c55e";
                            } else if (rssi >= -70) {
                                rssiBar.style.backgroundColor = "#84cc16";
                            } else if (rssi >= -80) {
                                rssiBar.style.backgroundColor = "#eab308";
                            } else if (rssi >= -90) {
                                rssiBar.style.backgroundColor = "#f97316";
                            } else {
                                rssiBar.style.backgroundColor = "#ef4444";
                            }
                        }
                    }
                    const settingsWdEl = document.getElementById('settings-wd-reset');
                    if (settingsWdEl) {
                        settingsWdEl.innerText = data.watchdog_reset_countdown || "--";
                    }
                    const settingsBenchEl = document.getElementById('footer-bench-settings');
                    if (settingsBenchEl) {
                        settingsBenchEl.innerText = data.loops_per_sec || 0;
                    }
                }).catch(err => console.error(err));
        }

        // Initialize and poll
        toggleEspNowFields();
        pollEspNowStatus();
        setInterval(pollEspNowStatus, 250);

        document.getElementById('settings-form').onsubmit = function() {
            document.getElementById('espnow_role').disabled = false;
            document.getElementById('espnow_channel').disabled = false;
        };

        // Two-stage confirmation for Complete Reset button
        const resetBtn = document.getElementById('complete-reset-btn');
        const resetForm = document.getElementById('reset-form');
        const resetAction = document.getElementById('reset-action');
        let confirmStage = false;

        resetBtn.onclick = function() {
            if (!confirmStage) {
                confirmStage = true;
                resetBtn.innerText = "Sicher? Alle Daten loeschen!";
                resetBtn.classList.add('confirm-step');
                
                setTimeout(function() {
                    confirmStage = false;
                    resetBtn.innerText = "Complete Reset";
                    resetBtn.classList.remove('confirm-step');
                }, 5000);
            } else {
                resetAction.value = "clear";
                resetForm.submit();
            }
        }
    </script>
</body>
</html>
)rawhtml";
  server.send(200, "text/html", html);
}

void handleSettingsSave() {
  String ssid = server.arg("wifi_ssid");
  String pass = server.arg("wifi_pass");
  String mqtt_server = server.arg("mqtt_server");
  int mqtt_port = server.arg("mqtt_port").toInt();
  String mqtt_user = server.arg("mqtt_user");
  String mqtt_pass = server.arg("mqtt_pass");
  String mqtt_device = server.arg("mqtt_device_name");
  int interval = server.arg("mqtt_report_interval").toInt();
  int brightness = server.arg("display_brightness").toInt();
  int tx_power = server.arg("wifi_tx_power").toInt();
  int servo_up_int = server.arg("servo_update_interval").toInt();
  int trap_val = server.arg("wlan_time_trap").toInt();

  int esp_role = server.arg("espnow_role").toInt();
  int esp_channel = server.arg("espnow_channel").toInt();
  int esp_failsafe = server.arg("espnow_failsafe_mode").toInt();
  String esp_peer_mac = server.arg("espnow_peer_mac");
  esp_peer_mac.trim();
  esp_peer_mac.toUpperCase();

  // Protect existing paired MAC from being wiped by form submit of
  // empty/disabled field
  if (esp_peer_mac.length() == 0 && strlen(sysConfig.espnow_peer_mac) > 0) {
    esp_peer_mac = String(sysConfig.espnow_peer_mac);
  }

  if (interval < 1)
    interval = 1;
  if (interval > 60)
    interval = 60;
  if (brightness < 0)
    brightness = 0;
  if (brightness > 100)
    brightness = 100;
  if (esp_channel < 1)
    esp_channel = 1;
  if (esp_channel > 13)
    esp_channel = 13;
  if (servo_up_int < 1)
    servo_up_int = 1;
  if (servo_up_int > 30)
    servo_up_int = 30;
  if (trap_val < 0)
    trap_val = 0;
  if (trap_val > 330)
    trap_val = 330;
  if (esp_failsafe < 0 || esp_failsafe > 1)
    esp_failsafe = 0;

  // Check if any configuration parameters actually changed
  bool hasChanges =
      (strcmp(sysConfig.wifi_ssid, ssid.c_str()) != 0 ||
       strcmp(sysConfig.wifi_pass, pass.c_str()) != 0 ||
       strcmp(sysConfig.mqtt_server, mqtt_server.c_str()) != 0 ||
       sysConfig.mqtt_port != mqtt_port ||
       strcmp(sysConfig.mqtt_user, mqtt_user.c_str()) != 0 ||
       strcmp(sysConfig.mqtt_pass, mqtt_pass.c_str()) != 0 ||
       strcmp(sysConfig.mqtt_device_name, mqtt_device.c_str()) != 0 ||
       sysConfig.mqtt_report_interval != interval ||
       sysConfig.display_brightness != brightness ||
       sysConfig.wifi_tx_power != tx_power ||
       sysConfig.espnow_role != esp_role ||
       sysConfig.espnow_channel != esp_channel ||
       strcmp(sysConfig.espnow_peer_mac, esp_peer_mac.c_str()) != 0 ||
       sysConfig.servo_update_interval != servo_up_int ||
       sysConfig.wlan_time_trap != trap_val ||
       sysConfig.espnow_failsafe_mode != esp_failsafe);

  bool wifiChanged = (strcmp(sysConfig.wifi_ssid, ssid.c_str()) != 0 ||
                      strcmp(sysConfig.wifi_pass, pass.c_str()) != 0);
  bool deviceNameChanged =
      (strcmp(sysConfig.mqtt_device_name, mqtt_device.c_str()) != 0);
  bool espnowChanged =
      (sysConfig.espnow_role != esp_role ||
       sysConfig.espnow_channel != esp_channel ||
       strcmp(sysConfig.espnow_peer_mac, esp_peer_mac.c_str()) != 0);

  if (hasChanges) {
    strlcpy(sysConfig.wifi_ssid, ssid.c_str(), sizeof(sysConfig.wifi_ssid));
    strlcpy(sysConfig.wifi_pass, pass.c_str(), sizeof(sysConfig.wifi_pass));
    strlcpy(sysConfig.mqtt_server, mqtt_server.c_str(),
            sizeof(sysConfig.mqtt_server));
    sysConfig.mqtt_port = (mqtt_port > 0) ? mqtt_port : 1883;
    strlcpy(sysConfig.mqtt_user, mqtt_user.c_str(),
            sizeof(sysConfig.mqtt_user));
    strlcpy(sysConfig.mqtt_pass, mqtt_pass.c_str(),
            sizeof(sysConfig.mqtt_pass));
    strlcpy(sysConfig.mqtt_device_name, mqtt_device.c_str(),
            sizeof(sysConfig.mqtt_device_name));
    sysConfig.mqtt_report_interval = interval;
    sysConfig.display_brightness = brightness;
    sysConfig.wifi_tx_power = tx_power;
    sysConfig.espnow_role = esp_role;
    sysConfig.espnow_channel = esp_channel;
    strlcpy(sysConfig.espnow_peer_mac, esp_peer_mac.c_str(),
            sizeof(sysConfig.espnow_peer_mac));
    sysConfig.servo_update_interval = servo_up_int;
    sysConfig.wlan_time_trap = trap_val;
    sysConfig.espnow_failsafe_mode = esp_failsafe;

    // Clear LMK if role disabled or partner MAC cleared
    if (esp_role == 0 || esp_peer_mac.length() == 0) {
      memset(sysConfig.espnow_lmk, 0, sizeof(sysConfig.espnow_lmk));
    }

    saveConfiguration(); // Saves to LittleFS JSON
  } else {
    Serial.println(
        "[LittleFS] No changes detected. Skipping write to avoid flash wear.");
  }

  // Re-init ESP-NOW if configured values changed
  if (espnowChanged) {
    initEspNow();
  }

  // Apply non-reboot settings immediately
  if (isTFTMode) {
    uint8_t rawBrightness =
        (uint8_t)round(pow(sysConfig.display_brightness / 100.0, 2.2) * 255.0);
    tft.setBrightness(rawBrightness);
  }
  WiFi.setTxPower((wifi_power_t)sysConfig.wifi_tx_power);

  if (hasChanges && (wifiChanged || deviceNameChanged)) {
    String html = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Einstellungen gespeichert</title>
    <style>
        body { background: #0f172a; color: white; text-align: center; padding-top: 100px; font-family: sans-serif; }
        .box { background: #1e293b; padding: 40px; border-radius: 15px; display: inline-block; border: 1px solid rgba(255,255,255,0.1); }
        h1 { color: #f87171; margin-bottom: 20px; }
    </style>
</head>
<body>
    <div class="box">
        <h1>Einstellungen gespeichert!</h1>
        <p>Der ESP32 startet nun neu, um die geänderten Netzwerk- oder Gerätenamen-Einstellungen anzuwenden.</p>
        <p>Bitte verbinde dein Gerät wieder mit deinem Heimnetzwerk.</p>
    </div>
    <script>setTimeout(function(){ window.location.href = '/'; }, 5000);</script>
</body>
</html>
)rawhtml";
    server.send(200, "text/html", html);
    delay(2000);
    ESP.restart();
  } else {
    String html = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Gespeichert</title>
    <style>
        body { background: #0f172a; color: white; text-align: center; padding-top: 100px; font-family: sans-serif; }
        .box { background: #1e293b; padding: 40px; border-radius: 15px; display: inline-block; border: 1px solid rgba(255,255,255,0.1); }
        h1 { color: #818cf8; margin-bottom: 20px; }
    </style>
</head>
<body>
    <div class="box">
        <h1>Einstellungen gespeichert!</h1>
        <p>Die Einstellungen (Sendeleistung, Helligkeit, MQTT-Sende-Intervall) wurden im laufenden Betrieb angewendet.</p>
        <p>Du wirst gleich zurückgeleitet...</p>
    </div>
    <script>setTimeout(function(){ window.location.href = '/settings'; }, 2000);</script>
</body>
</html>
)rawhtml";
    server.send(200, "text/html", html);
  }
}

void handleSettingsReset() {
  String action = server.arg("action");
  if (action == "reboot") {
    String html = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Gerät startet neu</title>
    <style>
        body { background: #0f172a; color: white; text-align: center; padding-top: 100px; font-family: sans-serif; }
        .box { background: #1e293b; padding: 40px; border-radius: 15px; display: inline-block; border: 1px solid rgba(255,255,255,0.1); }
        h1 { color: #4ade80; margin-bottom: 20px; }
    </style>
</head>
<body>
    <div class="box">
        <h1>iDry 26 reboot.</h1>
        <p>Stay calm, we are back online in a second :-)</p>
    </div>
    <script>setTimeout(function(){ window.location.href = '/settings'; }, 5000);</script>
</body>
</html>
)rawhtml";
    server.send(200, "text/html", html);
    delay(1000);
    ESP.restart();
  } else if (action == "defaults") {
    bool hasChanges =
        (sysConfig.mqtt_report_interval != 5 ||
         sysConfig.display_brightness != 80 || sysConfig.wifi_tx_power != 52 ||
         sysConfig.servo_update_interval != 5 ||
         sysConfig.wlan_time_trap != 120);

    if (hasChanges) {
      sysConfig.mqtt_report_interval = 5;
      sysConfig.display_brightness = 80;
      sysConfig.wifi_tx_power = 52;
      sysConfig.servo_update_interval = 5;
      sysConfig.wlan_time_trap = 120;
      saveConfiguration();
    } else {
      Serial.println("[LittleFS] Configuration already at default values. "
                     "Skipping write.");
    }

    if (isTFTMode) {
      uint8_t rawBrightness = (uint8_t)round(
          pow(sysConfig.display_brightness / 100.0, 2.2) * 255.0);
      tft.setBrightness(rawBrightness);
    }
    WiFi.setTxPower((wifi_power_t)sysConfig.wifi_tx_power);

    server.sendHeader("Location", "/settings");
    server.send(303);
  } else if (action == "clear") {
    LittleFS.remove("/config.json");
    String html = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Gerät zurückgesetzt</title>
    <style>
        body { background: #0f172a; color: white; text-align: center; padding-top: 100px; font-family: sans-serif; }
        .box { background: #1e293b; padding: 40px; border-radius: 15px; display: inline-block; border: 1px solid rgba(255,255,255,0.1); }
        h1 { color: #ef4444; margin-bottom: 20px; }
    </style>
</head>
<body>
    <div class="box">
        <h1>Gerät komplett zurückgesetzt!</h1>
        <p>Der ESP32 startet nun neu und öffnet das Konfigurations-Portal.</p>
    </div>
    <script>setTimeout(function(){ window.location.href = '/'; }, 3000);</script>
</body>
</html>
)rawhtml";
    server.send(200, "text/html", html);
    delay(2000);
    ESP.restart();
  } else if (action == "delete_espnow") {
    memset(sysConfig.espnow_peer_mac, 0, sizeof(sysConfig.espnow_peer_mac));
    memset(sysConfig.espnow_lmk, 0, sizeof(sysConfig.espnow_lmk));
    protocolVersionMismatch = false;
    remoteProtocolVersion = 0;
    lastEspNowRxTime = 0;
    saveConfiguration();
    initEspNow(); // Remove peer from driver

    // Play double error beep
    tone(BUZZER_PIN, 300, 80);
    delay(100);
    tone(BUZZER_PIN, 200, 150);
    delay(200);
    noTone(BUZZER_PIN);

    server.sendHeader("Location", "/settings");
    server.send(303);
  } else {
    server.sendHeader("Location", "/settings");
    server.send(303);
  }
}

void handleEspNowPairApi() {
  String action = server.arg("action");

  if (action == "start") {
    if (server.hasArg("role")) {
      sysConfig.espnow_role = server.arg("role").toInt();
    }
    if (server.hasArg("channel")) {
      sysConfig.espnow_channel = server.arg("channel").toInt();
    }

    if (sysConfig.espnow_role == 0) {
      server.send(400, "application/json",
                  "{\"status\":\"error\",\"message\":\"Rolle Master oder Slave "
                  "zuerst auswaehlen.\"}");
      return;
    }

    // Dynamically initialize ESP-NOW for the selected role
    initEspNow();

    isPairingActive = true;
    pairingStartTime = millis();
    lastPairingBeaconTime = 0;

    if (sysConfig.espnow_role == 1) { // Master
      // Generate random LMK
      uint8_t rawLmk[16];
      for (int i = 0; i < 16; i++) {
        rawLmk[i] = (uint8_t)(esp_random() & 0xFF);
      }
      for (int i = 0; i < 16; i++) {
        sprintf(proposedLmk + 2 * i, "%02x", rawLmk[i]);
      }
      proposedLmk[32] = '\0';
      originalWifiChannel = WiFi.status() == WL_CONNECTED ? WiFi.channel() : 1;
      Serial.printf("[Pairing] Master pairing started. Proposed LMK: %s\n",
                    proposedLmk);
    } else { // Slave
      currentPairingChannel = sysConfig.espnow_channel;
      lastChannelHopTime = millis();
      originalWifiChannel = WiFi.status() == WL_CONNECTED ? WiFi.channel() : 1;
      esp_wifi_set_channel(currentPairingChannel, WIFI_SECOND_CHAN_NONE);
      Serial.printf(
          "[Pairing] Slave pairing started on channel %d (Fast Track)\n",
          currentPairingChannel);
    }

    tone(BUZZER_PIN, 880, 80);
    delay(100);
    tone(BUZZER_PIN, 1047, 80);
    delay(100);
    noTone(BUZZER_PIN);

    server.send(200, "application/json",
                "{\"status\":\"ok\",\"message\":\"Pairing gestartet.\"}");
  } else {
    isPairingActive = false;
    if (sysConfig.espnow_role == 2) {
      esp_wifi_set_channel(originalWifiChannel, WIFI_SECOND_CHAN_NONE);
    }
    server.send(200, "application/json",
                "{\"status\":\"ok\",\"message\":\"Pairing gestoppt.\"}");
  }
}

void handleBuzzerTestApi() {
  String type = server.arg("type");
  if (type == "local") {
    server.send(200, "application/json", "{\"status\":\"ok\"}");
    playWinnerMelody();
  } else if (type == "remote") {
    if (strlen(sysConfig.espnow_peer_mac) == 0) {
      server.send(
          400, "application/json",
          "{\"status\":\"error\",\"message\":\"Kein Partner gekoppelt.\"}");
      return;
    }

    EspNowMessage msg;
    msg.pv = localProtocolVersion;
    msg.type = 2; // Command/Data
    strlcpy(msg.key, sysConfig.espnow_lmk, sizeof(msg.key));
    msg.command = 1; // Play winner melody
    msg.value = 0;

    uint8_t peerMac[6];
    sscanf(sysConfig.espnow_peer_mac, "%x:%x:%x:%x:%x:%x", &peerMac[0],
           &peerMac[1], &peerMac[2], &peerMac[3], &peerMac[4], &peerMac[5]);

    esp_err_t result =
        esp_now_send(peerMac, (uint8_t *)&msg, sizeof(EspNowMessage));
    if (result == ESP_OK) {
      server.send(200, "application/json", "{\"status\":\"ok\"}");
    } else {
      server.send(
          500, "application/json",
          "{\"status\":\"error\",\"message\":\"Senden fehlgeschlagen.\"}");
    }
  } else {
    server.send(400, "application/json",
                "{\"status\":\"error\",\"message\":\"Ungueltiger Typ.\"}");
  }
}

void handleFavicon() {
  server.sendHeader("Cache-Control", "public, max-age=31536000");
  if (server.uri().endsWith(".ico")) {
    server.send_P(200, "image/x-icon", (const char *)favicon_png,
                  sizeof(favicon_png));
  } else {
    server.send_P(200, "image/png", (const char *)favicon_png,
                  sizeof(favicon_png));
  }
}

// =====================================================================
// FIRMWARE & ONLINE / MANUAL OTA UPDATE HANDLERS
// =====================================================================

int fetchGithubFirmwareVersion() {
  if (WiFi.status() != WL_CONNECTED)
    return -1;
  WiFiClientSecure client;
  client.setInsecure(); // Disable SSL cert check for ESP32 GitHub requests
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(4000);
  if (http.begin(client, "https://raw.githubusercontent.com/VR-addicted/"
                         "grow-zone-iDry/main/FIRMWARE/version.txt")) {
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      payload.trim();
      int version = payload.toInt();
      http.end();
      return version;
    }
    http.end();
  }
  return -1;
}

void handleFirmwarePage() {
  int onlineVersion = fetchGithubFirmwareVersion();
  String html = R"rawhtml(
<!DOCTYPE html>
<html lang="de">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="icon" type="image/png" sizes="32x32" href="/favicon-32x32.png">
    <link rel="icon" type="image/x-icon" href="/favicon.ico">
    <link rel="shortcut icon" type="image/x-icon" href="/favicon.ico">
    <title>Firmware Update - IDRY-26</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; }
        body {
            background: linear-gradient(135deg, #0f172a 0%, #1e1b4b 100%);
            color: #f8fafc;
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            padding: 20px;
        }
        .container {
            background: rgba(30, 41, 59, 0.45);
            backdrop-filter: blur(12px);
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 20px;
            padding: 30px;
            width: 100%;
            max-width: 550px;
            box-shadow: 0 20px 25px -5px rgba(0, 0, 0, 0.5);
        }
        h1 { text-align: center; margin-bottom: 20px; font-size: 22px; font-weight: 600; color: #818cf8; }
        .card {
            background: rgba(15, 23, 42, 0.5);
            border: 1px solid rgba(255, 255, 255, 0.05);
            border-radius: 12px;
            padding: 20px;
            margin-bottom: 20px;
        }
        .card-title { font-size: 13px; text-transform: uppercase; letter-spacing: 1px; color: #94a3b8; margin-bottom: 12px; font-weight: bold; border-bottom: 1px solid rgba(255,255,255,0.05); padding-bottom: 5px; }
        .info-text { font-size: 14px; line-height: 1.6; color: #cbd5e1; margin-bottom: 15px; }
        .btn {
            display: block;
            width: 100%;
            padding: 12px 18px;
            border-radius: 10px;
            font-size: 14px;
            font-weight: 600;
            cursor: pointer;
            text-align: center;
            text-decoration: none;
            border: none;
            transition: all 0.2s;
            margin-top: 10px;
        }
        .btn-update { background: linear-gradient(135deg, #10b981 0%, #059669 100%); color: white; box-shadow: 0 4px 12px rgba(16, 185, 129, 0.3); }
        .btn-update:hover { transform: translateY(-1px); box-shadow: 0 6px 16px rgba(16, 185, 129, 0.4); }
        .btn-nav { background: rgba(255, 255, 255, 0.05); border: 1px solid rgba(255, 255, 255, 0.1); color: #cbd5e1; }
        .btn-nav:hover { background: rgba(255, 255, 255, 0.1); }
        .badge-up-to-date { background: rgba(52, 211, 153, 0.15); border: 1px solid rgba(52, 211, 153, 0.3); color: #34d399; padding: 10px 14px; border-radius: 8px; font-size: 13px; text-align: center; margin-bottom: 15px; }
        .badge-update-avail { background: rgba(251, 191, 36, 0.15); border: 1px solid rgba(251, 191, 36, 0.3); color: #fbbf24; padding: 10px 14px; border-radius: 8px; font-size: 13px; text-align: center; margin-bottom: 15px; }
        input[type="file"] { display: block; width: 100%; padding: 10px; background: rgba(15, 23, 42, 0.6); border: 1px solid rgba(255, 255, 255, 0.1); border-radius: 8px; color: #f8fafc; font-size: 13px; margin-bottom: 12px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Firmware & OTA Update</h1>
        <div class="card">
            <div class="card-title">Versions-Status</div>
            <div class="info-text">
                Installierte Version: <strong>)rawhtml";
  html += String(localFirmwareVersion);
  html +=
      R"rawhtml(</strong> &nbsp;|&nbsp; Aktuellste Version: <strong>)rawhtml";
  if (onlineVersion > 0) {
    html += String(onlineVersion);
  } else {
    html += "Nicht erreichbar";
  }
  html += R"rawhtml(</strong>
            </div>
)rawhtml";

  if (onlineVersion > localFirmwareVersion) {
    html += R"rawhtml(
            <div class="badge-update-avail">
                ⚡ Neue Firmware-Version )rawhtml";
    html += String(onlineVersion);
    html += R"rawhtml( auf GitHub verfuegbar!
            </div>
            <a href="/firmware/autoupdate" class="btn btn-update">
                🚀 Automatisch Online Updaten (v)rawhtml";
    html += String(onlineVersion);
    html += R"rawhtml()
            </a>
)rawhtml";
  } else if (onlineVersion > 0) {
    html += R"rawhtml(
            <div class="badge-up-to-date">
                ✓ Deine Firmware ist auf dem neuesten Stand.
            </div>
)rawhtml";
  }

  html += R"rawhtml(
        </div>

        <div class="card">
            <div class="card-title">Manuelles Firmware File Flash</div>
            <p class="info-text">Lokale Firmware-Datei (.bin) auswählen und direkt auf den ESP32 hochladen:</p>
            <form method="POST" action="/firmware/upload" enctype="multipart/form-data">
                <input type="file" name="update" accept=".bin" required>
                <button type="submit" class="btn btn-nav" style="background: rgba(99, 102, 241, 0.2); border-color: rgba(99, 102, 241, 0.4); color: #a5b4fc;">
                    Firmware .bin Flashen
                </button>
            </form>
        </div>

        <div style="display: flex; gap: 10px;">
            <a href="/settings" class="btn btn-nav" style="flex: 1;">Zurueck zu Settings</a>
            <a href="/" class="btn btn-nav" style="flex: 1;">Zurueck zum Monitor</a>
        </div>
    </div>
</body>
</html>
)rawhtml";
  server.send(200, "text/html", html);
}

void handleAutoUpdate() {
  if (WiFi.status() != WL_CONNECTED) {
    server.send(500, "text/html",
                "<html><body><h1>Keine WLAN-Verbindung zum Internet!</h1><p><a "
                "href='/firmware'>Zurueck</a></p></body></html>");
    return;
  }

  String html = R"rawhtml(
<!DOCTYPE html>
<html lang="de">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="icon" type="image/png" sizes="32x32" href="/favicon-32x32.png">
    <link rel="icon" type="image/x-icon" href="/favicon.ico">
    <link rel="shortcut icon" type="image/x-icon" href="/favicon.ico">
    <title>GitHub OTA Online-Update - IDRY-26</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; }
        body {
            background: linear-gradient(135deg, #0f172a 0%, #1e1b4b 100%);
            color: #f8fafc;
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            padding: 20px;
        }
        .container {
            background: rgba(30, 41, 59, 0.5);
            backdrop-filter: blur(12px);
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 20px;
            padding: 25px;
            width: 100%;
            max-width: 600px;
            box-shadow: 0 20px 25px -5px rgba(0, 0, 0, 0.5);
        }
        h1 { text-align: center; font-size: 20px; color: #38bdf8; margin-bottom: 15px; font-weight: 600; }
        .progress-bar-bg {
            background: rgba(15, 23, 42, 0.8);
            border: 1px solid rgba(255,255,255,0.1);
            border-radius: 10px;
            height: 22px;
            overflow: hidden;
            margin-bottom: 15px;
            position: relative;
        }
        .progress-bar-fill {
            background: linear-gradient(90deg, #38bdf8 0%, #818cf8 100%);
            height: 100%;
            width: 0%;
            transition: width 0.4s ease;
        }
        .progress-text {
            position: absolute;
            top: 0; left: 0; width: 100%; height: 100%;
            display: flex; align-items: center; justify-content: center;
            font-size: 11px; font-weight: bold; color: #ffffff;
            text-shadow: 0 1px 2px rgba(0,0,0,0.8);
        }
        .console {
            background: #020617;
            border: 1px solid rgba(56, 189, 248, 0.2);
            border-radius: 10px;
            padding: 15px;
            font-family: 'Consolas', 'Courier New', monospace;
            font-size: 12px;
            height: 230px;
            overflow-y: auto;
            color: #38bdf8;
            line-height: 1.6;
            box-shadow: inset 0 2px 4px rgba(0,0,0,0.5);
        }
        .log-line { margin-bottom: 4px; word-break: break-all; }
        .log-error { color: #f87171; font-weight: bold; }
        .log-success { color: #4ade80; font-weight: bold; }
        .log-header { color: #fbbf24; }
        .btn-nav {
            display: inline-block; width: 100%; padding: 12px;
            border-radius: 10px; font-size: 14px; font-weight: 600;
            text-align: center; text-decoration: none; border: none;
            margin-top: 15px; cursor: pointer;
        }
        .btn-back { background: rgba(255, 255, 255, 0.05); border: 1px solid rgba(255, 255, 255, 0.1); color: #cbd5e1; }
        .btn-back:hover { background: rgba(255, 255, 255, 0.1); }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚀 GitHub OTA Online-Update Terminal</h1>
        <div class="progress-bar-bg">
            <div id="progress-fill" class="progress-bar-fill"></div>
            <div id="progress-text" class="progress-text">0%</div>
        </div>
        <div id="console" class="console">
            <div class="log-line log-header">[SYSTEM] Starte Online-Update von GitHub...</div>
            <div class="log-line">[TARGET] https://raw.githubusercontent.com/VR-addicted/grow-zone-iDry/main/FIRMWARE/firmware.bin</div>
        </div>
        <a id="back-btn" href="/firmware" class="btn-nav btn-back" style="display: none;">Zurueck zu Firmware Update</a>
    </div>

    <script>
        const consoleEl = document.getElementById('console');
        const fillEl = document.getElementById('progress-fill');
        const textEl = document.getElementById('progress-text');
        const backBtn = document.getElementById('back-btn');

        function appendLog(text, isError = false, isSuccess = false) {
            const line = document.createElement('div');
            line.className = 'log-line' + (isError ? ' log-error' : (isSuccess ? ' log-success' : ''));
            line.innerText = text;
            consoleEl.appendChild(line);
            consoleEl.scrollTop = consoleEl.scrollHeight;
        }

        appendLog('[CONNECT] Verbinde mit GitHub raw.githubusercontent.com...');
        fillEl.style.width = '20%';
        textEl.innerText = '20%';

        fetch('/api/firmware/autoupdate_start')
            .then(r => r.json())
            .then(data => {
                if (data.status === 'ok') {
                    fillEl.style.width = '100%';
                    textEl.innerText = '100%';
                    appendLog('[DOWNLOAD] Datei FIRMWARE/firmware.bin erfolgreich geladen (' + (data.written || 0) + ' Bytes)!', false, true);
                    appendLog('[HEADER VERIFY] ESP32 Magic Byte (0xE9) und Header gueltig!', false, true);
                    appendLog('[FLASH] Inaktive OTA-Bank (app0/app1) erfolgreich beschrieben!', false, true);
                    appendLog('[REBOOT] iDry 26 reboot. Stay calm, we are back online in a second :-)', false, true);
                    setTimeout(() => { window.location.href = '/'; }, 6000);
                } else {
                    fillEl.style.width = '0%';
                    textEl.innerText = 'Fehler';
                    appendLog('[FEHLER] ' + (data.message || 'Update abgebrochen'), true);
                    backBtn.style.display = 'block';
                }
            })
            .catch(err => {
                fillEl.style.width = '0%';
                textEl.innerText = 'Fehler';
                appendLog('[FEHLER] Verbindungsabbruch beim Flashen: ' + err, true);
                backBtn.style.display = 'block';
            });
    </script>
</body>
</html>
)rawhtml";
  server.send(200, "text/html", html);
}

void handleAutoUpdateApi() {
  if (WiFi.status() != WL_CONNECTED) {
    server.send(400, "application/json",
                "{\"status\":\"error\",\"message\":\"Keine aktive "
                "WLAN-Verbindung zum Internet.\"}");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure(); // Disable SSL cert check for ESP32 raw GitHub download
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(8000);

  const char *binUrl = "https://raw.githubusercontent.com/VR-addicted/"
                       "grow-zone-iDry/main/FIRMWARE/firmware.bin";
  Serial.printf("[OTA] Connecting to GitHub RAW URL: %s\n", binUrl);

  if (!http.begin(client, binUrl)) {
    server.send(500, "application/json",
                "{\"status\":\"error\",\"message\":\"HTTP Verbindungsaufbau zu "
                "GitHub fehlgeschlagen.\"}");
    return;
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("[OTA] GitHub HTTP Error Code: %d\n", httpCode);
    http.end();
    server.send(500, "application/json",
                "{\"status\":\"error\",\"message\":\"GitHub Datei "
                "FIRMWARE/firmware.bin nicht gefunden (HTTP " +
                    String(httpCode) + ").\"}");
    return;
  }

  int contentLength = http.getSize();
  Serial.printf("[OTA] GitHub firmware.bin Content Length: %d bytes\n",
                contentLength);

  WiFiClient *stream = http.getStreamPtr();
  if (!stream) {
    http.end();
    server.send(500, "application/json",
                "{\"status\":\"error\",\"message\":\"HTTP Stream von GitHub "
                "konnte nicht geoeffnet werden.\"}");
    return;
  }

  // Read first chunk to inspect header and magic byte
  uint8_t firstBuf[512];
  size_t firstRead = 0;
  unsigned long startWait = millis();
  while (stream->available() == 0 && (millis() - startWait < 5000)) {
    delay(10);
  }

  firstRead = stream->readBytes(firstBuf, sizeof(firstBuf));
  if (firstRead < 4) {
    http.end();
    server.send(
        500, "application/json",
        "{\"status\":\"error\",\"message\":\"Dateikopf zu klein oder leer!\"}");
    return;
  }

  // ESP32 Image Magic Byte Check: 0xE9 (233)
  if (firstBuf[0] != 0xE9) {
    char hexErr[128];
    snprintf(hexErr, sizeof(hexErr),
             "Ungueltiges ESP32 Binary! Magic Byte 0x%02X != 0xE9 (Kein ESP32 "
             "Image). Abbruch!",
             firstBuf[0]);
    Serial.printf("[OTA] %s\n", hexErr);
    http.end();
    server.send(400, "application/json",
                "{\"status\":\"error\",\"message\":\"" + String(hexErr) +
                    "\"}");
    return;
  }

  Serial.println("[OTA] Magic byte 0xE9 verified! Valid ESP32 binary header.");

  // Begin OTA Partition Write
  size_t updateSize = (contentLength > 0) ? contentLength : UPDATE_SIZE_UNKNOWN;
  if (!Update.begin(updateSize)) {
    http.end();
    server.send(500, "application/json",
                "{\"status\":\"error\",\"message\":\"Partition Flash Start "
                "fehlgeschlagen!\"}");
    return;
  }

  // Write first chunk
  if (Update.write(firstBuf, firstRead) != firstRead) {
    Update.abort();
    http.end();
    server.send(500, "application/json",
                "{\"status\":\"error\",\"message\":\"Fehler beim Schreiben des "
                "ersten Datenblocks.\"}");
    return;
  }

  // Stream remaining bytes
  uint8_t buffer[2048];
  size_t writtenBytes = firstRead;

  while (http.connected() &&
         (writtenBytes < (size_t)contentLength || contentLength <= 0)) {
    size_t sizeAvailable = stream->available();
    if (sizeAvailable > 0) {
      size_t readLen =
          stream->readBytes(buffer, min(sizeAvailable, sizeof(buffer)));
      if (readLen > 0) {
        if (Update.write(buffer, readLen) != readLen) {
          Update.abort();
          http.end();
          server.send(500, "application/json",
                      "{\"status\":\"error\",\"message\":\"Fehler beim "
                      "Schreiben in die Partition.\"}");
          return;
        }
        writtenBytes += readLen;
      }
    } else {
      delay(1);
    }
  }

  http.end();

  if (contentLength > 0 && writtenBytes < (size_t)contentLength) {
    Update.abort();
    server.send(
        500, "application/json",
        "{\"status\":\"error\",\"message\":\"Download unvollstaendig (" +
            String(writtenBytes) + "/" + String(contentLength) + " Bytes).\"}");
    return;
  }

  if (!Update.end(true)) {
    server.send(500, "application/json",
                "{\"status\":\"error\",\"message\":\"OTA Abschlussfehler "
                "(Update.end failed).\"}");
    return;
  }

  Serial.printf(
      "[OTA] Update.end() SUCCESS! Written total: %u bytes. Rebooting...\n",
      (unsigned int)writtenBytes);
  server.send(200, "application/json",
              "{\"status\":\"ok\",\"message\":\"Update erfolgreich! iDry 26 "
              "reboot...\",\"written\":" +
                  String(writtenBytes) + "}");

  delay(1000);
  ESP.restart();
}

static bool g_manualUploadError = false;

void handleUploadProgress() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("[OTA] Manual Firmware Upload Start: %s\n",
                  upload.filename.c_str());
    g_manualUploadError = false;
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
      g_manualUploadError = true;
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!g_manualUploadError) {
      // Magic Byte check on first block
      if (upload.totalSize == 0 && upload.currentSize >= 1) {
        if (upload.buf[0] != 0xE9) {
          Serial.printf("[OTA] Manual Upload Aborted: Magic byte 0x%02X != "
                        "0xE9 (Invalid ESP32 binary)\n",
                        upload.buf[0]);
          g_manualUploadError = true;
          Update.abort();
          return;
        }
      }
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
        g_manualUploadError = true;
      }
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!g_manualUploadError) {
      if (upload.totalSize < 100000) {
        Serial.printf(
            "[OTA] Manual Upload Aborted: Size too small (%u bytes < 100KB)\n",
            (unsigned int)upload.totalSize);
        g_manualUploadError = true;
        Update.abort();
      } else if (Update.end(true)) {
        Serial.printf(
            "[OTA] Manual Firmware Upload Finished! Total bytes: %u\n",
            (unsigned int)upload.totalSize);
      } else {
        Update.printError(Serial);
        g_manualUploadError = true;
      }
    }
  }
}

void handleUploadFinish() {
  if (g_manualUploadError || Update.hasError()) {
    server.send(400, "text/html", R"rawhtml(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <link rel="icon" type="image/png" sizes="32x32" href="/favicon-32x32.png">
    <title>Upload Fehler - IDRY-26</title>
    <style>
        body { background: #0f172a; color: white; text-align: center; padding-top: 80px; font-family: sans-serif; }
        .box { background: #1e293b; padding: 30px; border-radius: 15px; display: inline-block; border: 1px solid rgba(255,255,255,0.1); max-width: 500px; box-shadow: 0 10px 20px rgba(0,0,0,0.5); }
        h1 { color: #f87171; margin-bottom: 15px; font-size: 20px; }
        p { color: #cbd5e1; font-size: 14px; margin-bottom: 20px; line-height: 1.6; }
        .btn { display: inline-block; padding: 10px 20px; background: rgba(255,255,255,0.1); border: 1px solid rgba(255,255,255,0.2); color: white; text-decoration: none; border-radius: 8px; font-size: 13px; }
        .btn:hover { background: rgba(255,255,255,0.2); }
    </style>
</head>
<body>
    <div class="box">
        <h1>Firmware-Upload abgelehnt!</h1>
        <p>Die hochgeladene Datei ist kein gültiges ESP32 Binary (Magic Byte 0xE9 fehlt oder Dateigröße kleiner als 100 KB).</p>
        <a href="/firmware" class="btn">Zurueck zu Firmware Update</a>
    </div>
</body>
</html>
)rawhtml");
  } else {
    String html = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <link rel="icon" type="image/png" sizes="32x32" href="/favicon-32x32.png">
    <link rel="icon" type="image/x-icon" href="/favicon.ico">
    <link rel="shortcut icon" type="image/x-icon" href="/favicon.ico">
    <title>Firmware-Update erfolgreich</title>
    <style>
        body { background: #0f172a; color: white; text-align: center; padding-top: 100px; font-family: sans-serif; }
        .box { background: #1e293b; padding: 40px; border-radius: 15px; display: inline-block; border: 1px solid rgba(255,255,255,0.1); }
        h1 { color: #4ade80; margin-bottom: 20px; }
    </style>
</head>
<body>
    <div class="box">
        <h1>iDry 26 reboot.</h1>
        <p>Stay calm, we are back online in a second :-)</p>
    </div>
    <script>setTimeout(function(){ window.location.href = '/'; }, 6000);</script>
</body>
</html>
)rawhtml";
    server.send(200, "text/html", html);
    delay(1000);
    ESP.restart();
  }
}

void startCaptivePortal() {
  portalActive = true;
  generateUniqueSSID();

  Serial.println("\n--- WiFi / MQTT Portal Mode ---");
  Serial.printf("Config SSID: %s\n", apSSID.c_str());

  // Shut down E-ink display power draw before activating SoftAP
  if (!isTFTMode) {
    Serial.println("[Power] Powering off E-Ink display to stabilize voltage "
                   "for SoftAP...");
    display.powerOff();
  }
  delay(500); // Allow LDO voltage rail to recover and settle

  // Stop background STA connection scanning to prevent AP signal disruption
  WiFi.persistent(
      false); // Prevent NVS flash writes which can corrupt Wi-Fi driver state
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(true);
  WiFi.softAPdisconnect(true);
  delay(200);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false); // Disable sleep mode to prevent transmitter power-down

  // Force medium-low RF transmission power (11dBm is safe) to test the next
  // physical threshold
  WiFi.setTxPower(WIFI_POWER_13dBm); // limit without interference
  delay(200);

  // Start SoftAP on Channel 6 (standard stable channel, visible, max 4 clients)
  bool ok = WiFi.softAP(apSSID.c_str(), apPassword, 6, 0, 4);

  // Diagnostic Prints
  Serial.printf("[AP Debug] softAP startup return: %s\n",
                ok ? "SUCCESS" : "FAILED");
  Serial.printf("[AP Debug] Current WiFi Mode: %d (1=STA, 2=AP, 3=AP_STA)\n",
                (int)WiFi.getMode());
  Serial.printf("[AP Debug] SoftAP IP Address: %s\n",
                WiFi.softAPIP().toString().c_str());
  Serial.printf("[AP Debug] Target SSID: %s (Channel 6)\n", apSSID.c_str());
  Serial.printf("[AP Debug] Target Password: %s\n", apPassword);
  Serial.printf("[AP Debug] TX Power Level: %d\n", (int)WiFi.getTxPower());

  dnsServer.start(53, "*", WiFi.softAPIP());

  server.on("/", handlePortalRoot);
  server.on("/save", handlePortalSave);
  server.on("/api/data", handleGetData);
  server.on("/api/history", handleGetHistory);
  server.on("/settings", handleSettingsPage);
  server.on("/settings/save", handleSettingsSave);
  server.on("/settings/reset", handleSettingsReset);
  server.on("/api/espnow/pair", handleEspNowPairApi);
  server.on("/api/espnow/buzzer_test", handleBuzzerTestApi);
  server.on("/firmware", handleFirmwarePage);
  server.on("/firmware/autoupdate", handleAutoUpdate);
  server.on("/api/firmware/autoupdate_start", handleAutoUpdateApi);
  server.on("/firmware/upload", HTTP_POST, handleUploadFinish,
            handleUploadProgress);
  server.on("/favicon.ico", handleFavicon);
  server.on("/favicon-32x32.png", handleFavicon);
  server.onNotFound(handlePortalRoot);
  server.begin();
  initEspNow();
}

// =====================================================================
// DISPLAY VISUAL FEEDBACK (DURING BOOT)
// =====================================================================
void updateBootScreen(const char *line1, const char *line2) {
  if (isHeadless)
    return;
  if (isTFTMode) {
    tft.startWrite();
    tft.clear(TFT_NAVY);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(15, 40);
    tft.print("Boot System...");

    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(15, 90);
    tft.print(line1);

    tft.setTextColor(TFT_CYAN);
    tft.setCursor(15, 140);
    tft.print(line2);
    tft.endWrite();
  } else {
    display.setRotation(1);
    display.setFont(&::FreeMonoBold9pt7b);
    display.firstPage();
    do {
      display.fillScreen(GxEPD_WHITE);
      display.drawRect(0, 0, display.width(), display.height(), GxEPD_BLACK);
      display.drawRect(4, 4, display.width() - 8, display.height() - 8,
                       GxEPD_RED);

      display.setTextColor(GxEPD_BLACK);
      display.setCursor(20, 50);
      display.print("IDRY26 Bootstrap Config");

      display.setTextColor(GxEPD_RED);
      display.setCursor(20, 100);
      display.print(line1);

      display.setTextColor(GxEPD_BLACK);
      display.setCursor(20, 150);
      display.print(line2);
    } while (display.nextPage());
  }
}

// =====================================================================
// MQTT CLIENT & HA AUTO-DISCOVERY SETUP
// =====================================================================

void sendHADiscoveryConfig(const char *sensorName, const char *displayName,
                           const char *unit, const char *icon,
                           const char *deviceClass) {
  String discoveryTopic = "homeassistant/sensor/" +
                          String(sysConfig.mqtt_device_name) + "/" +
                          String(sensorName) + "/config";
  JsonDocument doc;
  doc["name"] = displayName;
  doc["state_topic"] = stateTopic;
  doc["value_template"] = "{{ value_json." + String(sensorName) + " }}";
  doc["unique_id"] =
      String(sysConfig.mqtt_device_name) + "_" + String(sensorName);

  if (unit && strlen(unit) > 0)
    doc["unit_of_measurement"] = unit;
  if (icon && strlen(icon) > 0)
    doc["icon"] = icon;
  if (deviceClass && strlen(deviceClass) > 0)
    doc["device_class"] = deviceClass;

  JsonObject dev = doc["device"].to<JsonObject>();
  dev["identifiers"][0] = String(sysConfig.mqtt_device_name);
  dev["name"] = String(sysConfig.mqtt_device_name);
  dev["model"] = "IDRY-26 Multi-Sensor Display";
  dev["sw_version"] = "2026.07.02";
  dev["manufacturer"] = "Growblox";

  String payload;
  serializeJson(doc, payload);
  mqttClient.publish(discoveryTopic.c_str(), payload.c_str(), true);
}

void registerHomeAssistantDevices() {
  Serial.println("[MQTT] Registering entities via HA Auto-Discovery...");

  // Primary System & Calculated Entities (Always Available)
  sendHADiscoveryConfig("rotor_pos", "Rotor Position", "%", "mdi:fan", "");
  sendHADiscoveryConfig("servo_angle", "Servo Winkel", "°", "mdi:angle-acute",
                        "");
  sendHADiscoveryConfig("temperature", "Temperatur", "°C", "mdi:thermometer",
                        "temperature");
  sendHADiscoveryConfig("humidity", "Luftfeuchtigkeit", "%",
                        "mdi:water-percent", "humidity");
  sendHADiscoveryConfig("dewpoint", "Taupunkt", "°C", "mdi:thermometer-alert",
                        "temperature");
  sendHADiscoveryConfig("vpd", "VPD", "kPa", "mdi:gauge", "");

  // Register active temperature sensors dynamically
  for (int i = 0; i < 2; i++) {
    if (tempSensors[i].active) {
      String idStr = "sensor_" + String(i);
      String nameStr =
          String((tempSensors[i].type == TempSensor::TYPE_BME280) ? "BME280"
                                                                  : "SHT3x") +
          " (" + String(i + 1) + ")";

      sendHADiscoveryConfig((idStr + "_temp").c_str(),
                            (nameStr + " Temp").c_str(), "°C",
                            "mdi:thermometer", "temperature");
      sendHADiscoveryConfig((idStr + "_hum").c_str(),
                            (nameStr + " Feuchte").c_str(), "%",
                            "mdi:water-percent", "humidity");
      sendHADiscoveryConfig((idStr + "_dewpoint").c_str(),
                            (nameStr + " Taupunkt").c_str(), "°C",
                            "mdi:thermometer-alert", "temperature");

      if (tempSensors[i].type == TempSensor::TYPE_BME280) {
        sendHADiscoveryConfig((idStr + "_press").c_str(),
                              (nameStr + " Druck").c_str(), "hPa", "mdi:gauge",
                              "pressure");
      }
    }
  }

  // Register active light sensors dynamically
  for (int i = 0; i < 2; i++) {
    if (lightSensors[i].active) {
      String idStr = "light_" + String(i);
      String nameStr = "TSL2561 (" + String(i + 1) + ")";

      sendHADiscoveryConfig((idStr + "_lux").c_str(),
                            (nameStr + " Helligkeit").c_str(), "lx",
                            "mdi:weather-sunny", "illuminance");
      sendHADiscoveryConfig((idStr + "_broadband").c_str(),
                            (nameStr + " Breitband").c_str(), "",
                            "mdi:solar-power", "");
      sendHADiscoveryConfig((idStr + "_ir").c_str(),
                            (nameStr + " Infrarot").c_str(), "",
                            "mdi:brightness-5", "");
    }
  }

  sendHADiscoveryConfig("poti_a", "Poti A (Sollwert)", "%", "mdi:knob", "");
  sendHADiscoveryConfig("poti_b", "Poti B (Gain)", "%", "mdi:knob", "");
  sendHADiscoveryConfig("poti_c", "Poti C (Cal Offset)", "°", "mdi:knob", "");
  sendHADiscoveryConfig("linkquality", "Signalstärke", "lqi", "mdi:signal", "");
  sendHADiscoveryConfig("rssi", "WLAN Signalstärke", "dBm", "mdi:wifi",
                        "signal_strength");
}

void publishMqttState() {
  if (!mqttClient.connected())
    return;

  JsonDocument doc;

  doc["rotor_pos"] = (int)round(rotorPosition);
  doc["servo_angle"] = (int)round(currentServoAngle);

  // Extract primary temperature and humidity
  float primaryTemp = NAN;
  float primaryHum = NAN;
  for (int i = 0; i < 2; i++) {
    if (tempSensors[i].active && !isnan(tempSensors[i].temperature)) {
      primaryTemp = tempSensors[i].temperature;
      primaryHum = tempSensors[i].humidity;
      break;
    }
  }

  if (!isnan(primaryTemp)) {
    doc["temperature"] = primaryTemp;
    doc["humidity"] = primaryHum;
    float dp = calculateDewPoint(primaryTemp, primaryHum);
    doc["dewpoint"] = isnan(dp) ? JsonVariant() : dp;
    float vpd = calculateVPD(primaryTemp, primaryHum);
    doc["vpd"] = isnan(vpd) ? JsonVariant() : vpd;
  }

  for (int i = 0; i < 2; i++) {
    if (tempSensors[i].active) {
      String idStr = "sensor_" + String(i);
      doc[idStr + "_temp"] = isnan(tempSensors[i].temperature)
                                 ? JsonVariant()
                                 : tempSensors[i].temperature;
      doc[idStr + "_hum"] = isnan(tempSensors[i].humidity)
                                ? JsonVariant()
                                : tempSensors[i].humidity;
      float dp = calculateDewPoint(tempSensors[i].temperature,
                                   tempSensors[i].humidity);
      doc[idStr + "_dewpoint"] = isnan(dp) ? JsonVariant() : dp;

      if (tempSensors[i].type == TempSensor::TYPE_BME280) {
        doc[idStr + "_press"] = isnan(tempSensors[i].pressure)
                                    ? JsonVariant()
                                    : tempSensors[i].pressure;
      }
    }
  }

  for (int i = 0; i < 2; i++) {
    if (lightSensors[i].active) {
      String idStr = "light_" + String(i);
      doc[idStr + "_lux"] =
          isnan(lightSensors[i].lux) ? JsonVariant() : lightSensors[i].lux;
      doc[idStr + "_broadband"] = lightSensors[i].broadband;
      doc[idStr + "_ir"] = lightSensors[i].ir;
    }
  }

  doc["poti_a"] = potiAVal;
  doc["poti_b"] = potiBVal;
  doc["poti_c"] = potiCVal;
  doc["servo_update_interval"] = sysConfig.servo_update_interval;
  doc["espnow_role"] = sysConfig.espnow_role;
  doc["espnow_last_seen_ms"] =
      (sysConfig.espnow_role > 0 && strlen(sysConfig.espnow_peer_mac) > 0)
          ? ((lastEspNowRxTime == 0) ? -1 : (long)(millis() - lastEspNowRxTime))
          : -1;
  doc["espnow_interval_ms"] =
      (sysConfig.espnow_role > 0 && strlen(sysConfig.espnow_peer_mac) > 0)
          ? avgEspNowIntervalMs
          : 0;
  doc["linkquality"] =
      (WiFi.status() == WL_CONNECTED) ? map(WiFi.RSSI(), -100, -30, 0, 255) : 0;
  doc["rssi"] = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
  doc["watchdog_reset_countdown"] = getWatchdogResetCountdown();

  String statePayload;
  serializeJson(doc, statePayload);
  mqttClient.publish(stateTopic.c_str(), statePayload.c_str());
  Serial.println("[MQTT] Published live sensor state data.");
}

// =====================================================================
// AUTO-DETECTION VIA RESET-INDUCED STATE CHANGE
// =====================================================================
bool detectDisplayType() {
  Serial.println(
      "[Auto-Detect] Starting display presence and type diagnostics...");

  pinMode(EPD_BUSY, INPUT_PULLUP);
  delay(50); // Let the levels settle
  int busy_initial = digitalRead(EPD_BUSY);

  Serial.printf("[Auto-Detect] Initial BUSY line state (with Pull-Up): %d\n",
                busy_initial);

  // Pulse EPD_RST to verify pin behavior
  pinMode(EPD_RST, OUTPUT);
  digitalWrite(EPD_RST, HIGH);
  delay(10);
  int busy_rst_high_before = digitalRead(EPD_BUSY);

  digitalWrite(EPD_RST, LOW);
  delay(20);
  int busy_rst_low = digitalRead(EPD_BUSY);

  digitalWrite(EPD_RST, HIGH);
  delay(10);

  Serial.printf("[Auto-Detect] Reset diagnostic: before=%d, during=%d\n",
                busy_rst_high_before, busy_rst_low);

  if (busy_initial == LOW && busy_rst_high_before == LOW &&
      busy_rst_low == LOW) {
    Serial.println("[Auto-Detect] Result: ILI9341 TFT Display detected "
                   "(Backlight line pulled LOW)");
    isHeadless = false;
    return true; // TFT Mode
  } else if (busy_rst_high_before != busy_rst_low) {
    Serial.println("[Auto-Detect] Result: e-Paper Display detected (BUSY state "
                   "change active)");
    isHeadless = false;
    return false; // e-Paper Mode
  } else {
    Serial.println("[Auto-Detect] Result: No Display detected (Headless Mode)");
    isHeadless = true;
    return false; // Headless Mode (isTFTMode = false)
  }
}

// WiFi Event Handler for Instant Reconnection
void WiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Serial.println("[WLAN] Event: WiFi connection lost.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== Multi-Display Bootstrap Boot ===");

  // Initialize Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Startup melody (Ascending arpeggio)
  tone(BUZZER_PIN, 523, 100); // C5
  delay(120);
  tone(BUZZER_PIN, 659, 100); // E5
  delay(120);
  tone(BUZZER_PIN, 784, 100); // G5
  delay(120);
  tone(BUZZER_PIN, 1047, 120); // C6
  delay(140);
  tone(BUZZER_PIN, 1319, 150); // E6
  delay(170);
  tone(BUZZER_PIN, 1568, 300); // G6
  delay(350);
  noTone(BUZZER_PIN);

  // Initialize LEDC PWM channel for Servo control on GPIO 18 (50 Hz, 14-bit
  // resolution)
  ledcSetup(SERVO_LEDC_CHANNEL, 50, 14);
  ledcAttachPin(SERVO_PIN, SERVO_LEDC_CHANNEL);

  // Set espClient socket connection timeout to 500ms to prevent blocking MQTT
  // client connects
  espClient.setTimeout(500);

  // Register WiFi Event handler for instant reconnects
  WiFi.onEvent(WiFiEvent);

  // Load Configuration first to retrieve display and network preferences
  isConfigLoaded = loadConfiguration();

  // Perform display type autodetect
  isTFTMode = detectDisplayType();

  // Fast-boot active display driver
  if (isHeadless) {
    Serial.println("Starting Headless Mode (No Display Connected)...");
  } else if (isTFTMode) {
    Serial.println("Starting TFT Mode (LovyanGFX)...");
    tft.init();
    tft.setRotation(1);
    uint8_t rawBrightness =
        (uint8_t)round(pow(sysConfig.display_brightness / 100.0, 2.2) * 255.0);
    tft.setBrightness(rawBrightness);
    tft.clear(TFT_NAVY);
  } else {
    Serial.println("Starting e-Paper Mode (GxEPD2)...");
    pinMode(EPD_BUSY, INPUT_PULLUP);
    SPI.begin(EPD_SCK, EPD_MISO, EPD_MOSI, -1);
    display.init(115200, true, 2, false);
    SPI.begin(EPD_SCK, EPD_MISO, EPD_MOSI, -1);
    pinMode(EPD_CS, OUTPUT);
    digitalWrite(EPD_CS, HIGH);
  }

  if (isConfigLoaded && strlen(sysConfig.wifi_ssid) > 0) {
    Serial.printf("[WLAN] Connecting to: %s\n", sysConfig.wifi_ssid);
    updateBootScreen("WLAN Verbinden...", sysConfig.wifi_ssid);

    WiFi.persistent(false); // Prevent flash wear and config corruption
    WiFi.disconnect(false); // DO NOT turn off the radio!
    WiFi.mode(WIFI_OFF);
    delay(200);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(
        false); // Disable power-save mode for maximum RF stability/speed
    delay(200);
    WiFi.begin(sysConfig.wifi_ssid, sysConfig.wifi_pass);
    WiFi.setTxPower((wifi_power_t)sysConfig.wifi_tx_power);

    // Timeout verification (Wait 15 seconds maximum)
    unsigned long connStart = millis();
    bool isConnected = false;
    while (millis() - connStart < 15000) {
      if (WiFi.status() == WL_CONNECTED) {
        isConnected = true;
        break;
      }
      delay(100);
    }

    if (isConnected) {
      Serial.printf("[WLAN] Connected! IP: %s\n",
                    WiFi.localIP().toString().c_str());
      updateBootScreen("WLAN Verbunden!", WiFi.localIP().toString().c_str());

      // Scan I2C Devices
      scanI2C();

      // Web Server Init (Real-time monitor)
      server.on("/", handlePortalRoot);
      server.on("/api/data", handleGetData);
      server.on("/api/history", handleGetHistory);
      server.on("/settings", handleSettingsPage);
      server.on("/settings/save", handleSettingsSave);
      server.on("/settings/reset", handleSettingsReset);
      server.on("/api/espnow/pair", handleEspNowPairApi);
      server.on("/api/espnow/buzzer_test", handleBuzzerTestApi);
      server.on("/firmware", handleFirmwarePage);
      server.on("/firmware/autoupdate", handleAutoUpdate);
      server.on("/api/firmware/autoupdate_start", handleAutoUpdateApi);
      server.on("/firmware/upload", HTTP_POST, handleUploadFinish,
                handleUploadProgress);
      server.on("/favicon.ico", handleFavicon);
      server.on("/favicon-32x32.png", handleFavicon);
      server.begin();
      initEspNow();

      // Setup MQTT Settings
      mqttClient.setServer(sysConfig.mqtt_server, sysConfig.mqtt_port);
      mqttClient.setBufferSize(
          2048); // Expand buffer from default 256 bytes for HA Discovery JSON
      baseTopic = "idry/" + String(sysConfig.mqtt_device_name);
      stateTopic = baseTopic + "/state";

      delay(1000);
    } else {
      Serial.println(
          "[WLAN] Connection timed out! Launching Captive Config Portal...");
      updateBootScreen("WLAN Timeout!", "Starte Portal...");
      delay(1000);
      startCaptivePortal();
    }
  } else {
    Serial.println(
        "[WLAN] No configuration stored. Starting Captive Config Portal...");
    updateBootScreen("Kein Setup!", "Starte Portal...");
    delay(1000);
    startCaptivePortal();
  }
}

void loop() {
  loopCounter++;
  if (millis() - lastLoopBenchTime >= 1000) {
    loopsPerSecond = loopCounter;
    loopCounter = 0;
    lastLoopBenchTime = millis();
  }

  // =====================================================================
  // ESP-NOW PAIRING TICK
  // =====================================================================
  if (isPairingActive) {
    if (millis() - pairingStartTime >= 30000) {
      Serial.println("[Pairing] Timeout! Exiting pairing mode.");
      isPairingActive = false;
      if (sysConfig.espnow_role == 2) {
        esp_wifi_set_channel(originalWifiChannel, WIFI_SECOND_CHAN_NONE);
      }
      tone(BUZZER_PIN, 150, 400);
      delay(450);
      noTone(BUZZER_PIN);
      initEspNow();
    } else {
      if (sysConfig.espnow_role == 1) { // Master broadcasts every 500ms
        if (millis() - lastPairingBeaconTime >= 500) {
          lastPairingBeaconTime = millis();

          EspNowMessage msg;
          msg.pv = localProtocolVersion;
          msg.type = 0; // Beacon
          strlcpy(msg.key, proposedLmk, sizeof(msg.key));
          msg.command = 0;
          msg.value = 0;

          uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

          esp_now_peer_info_t peerInfo;
          memset(&peerInfo, 0, sizeof(peerInfo));
          memcpy(peerInfo.peer_addr, broadcastMac, 6);
          peerInfo.channel = WiFi.status() == WL_CONNECTED ? WiFi.channel() : 1;
          peerInfo.encrypt = false;

          if (!esp_now_is_peer_exist(broadcastMac)) {
            esp_now_add_peer(&peerInfo);
          }

          esp_now_send(broadcastMac, (uint8_t *)&msg, sizeof(EspNowMessage));
          Serial.println("[Pairing] Master sending beacon...");
        }
      } else if (sysConfig.espnow_role == 2) { // Slave channel hopping
        unsigned long timeInPairing = millis() - pairingStartTime;
        if (timeInPairing >= 1200) {
          if (millis() - lastChannelHopTime >= 1200) {
            lastChannelHopTime = millis();

            currentPairingChannel++;
            if (currentPairingChannel > 13) {
              currentPairingChannel = 1;
            }
            if (currentPairingChannel == sysConfig.espnow_channel) {
              currentPairingChannel++;
              if (currentPairingChannel > 13) {
                currentPairingChannel = 1;
              }
            }
            esp_wifi_set_channel(currentPairingChannel, WIFI_SECOND_CHAN_NONE);
            Serial.printf("[Pairing] Slave hopping to channel %d...\n",
                          currentPairingChannel);
          }
        }
      }
    }
  }

  // =====================================================================
  // ESP-NOW MASTER KEEP-ALIVE PING & BROADCAST FALLBACK (Every 1s)
  // =====================================================================
  static unsigned long lastEspNowPingTime = 0;
  if (!isPairingActive && sysConfig.espnow_role == 1 &&
      strlen(sysConfig.espnow_peer_mac) > 0) {
    if (millis() - lastEspNowPingTime >= 1000) {
      lastEspNowPingTime = millis();

      // Ensure Master peer is registered on current Wi-Fi channel
      uint8_t masterChan = (WiFi.status() == WL_CONNECTED) ? WiFi.channel() : 1;
      uint8_t peerMac[6];
      if (sscanf(sysConfig.espnow_peer_mac, "%x:%x:%x:%x:%x:%x", &peerMac[0],
                 &peerMac[1], &peerMac[2], &peerMac[3], &peerMac[4],
                 &peerMac[5]) == 6) {
        esp_now_peer_info_t peerInfo;
        memset(&peerInfo, 0, sizeof(peerInfo));
        memcpy(peerInfo.peer_addr, peerMac, 6);
        peerInfo.channel = masterChan;
        peerInfo.ifidx = WIFI_IF_STA;
        peerInfo.encrypt = false;

        if (esp_now_is_peer_exist(peerMac)) {
          esp_now_mod_peer(&peerInfo);
        } else {
          esp_now_add_peer(&peerInfo);
        }

        EspNowMessage pingMsg;
        pingMsg.pv = localProtocolVersion;
        pingMsg.type = 2; // Data/Command
        strlcpy(pingMsg.key, sysConfig.espnow_lmk, sizeof(pingMsg.key));
        pingMsg.command = 2; // Ping-Request
        pingMsg.value = rotorPosition;

        esp_now_send(peerMac, (uint8_t *)&pingMsg, sizeof(EspNowMessage));

        // If Slave hasn't responded for >3s, also broadcast ping on Master's
        // channel
        if (lastEspNowRxTime == 0 || (millis() - lastEspNowRxTime > 3000)) {
          uint8_t bcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
          esp_now_peer_info_t bcastInfo;
          memset(&bcastInfo, 0, sizeof(bcastInfo));
          memcpy(bcastInfo.peer_addr, bcastMac, 6);
          bcastInfo.channel = masterChan;
          bcastInfo.ifidx = WIFI_IF_STA;
          bcastInfo.encrypt = false;
          if (!esp_now_is_peer_exist(bcastMac)) {
            esp_now_add_peer(&bcastInfo);
          }
          esp_now_send(bcastMac, (uint8_t *)&pingMsg, sizeof(EspNowMessage));
        }
      }
    }
  }

  // =====================================================================
  // ESP-NOW SLAVE AUTOMATIC CHANNEL HOPPING & RESYNCHRONIZATION (>3s Loss)
  // =====================================================================
  static unsigned long lastSlaveScanHopTime = 0;
  static uint8_t slaveScanChan = 1;
  if (!isPairingActive && sysConfig.espnow_role == 2 &&
      strlen(sysConfig.espnow_peer_mac) > 0) {
    if (lastEspNowRxTime == 0 || (millis() - lastEspNowRxTime > 3000)) {
      if (millis() - lastSlaveScanHopTime >= 350) {
        lastSlaveScanHopTime = millis();
        slaveScanChan = (slaveScanChan % 13) + 1;
        esp_wifi_set_channel(slaveScanChan, WIFI_SECOND_CHAN_NONE);
      }
    }
  }

  if (portalActive) {
    dnsServer.processNextRequest();
    server.handleClient();
    return;
  }

  server.handleClient();

  // Run potentiometer check and servo target angle calculations continuously in
  // the loop
  updateServoRamping(false);

  // Continuous non-blocking Servo Motion Profiling (Sine Ease-In-Ease-Out
  // Softstart/Stop Ramping)
  if (servoMoving) {
    unsigned long elapsed = millis() - servoMoveStartTime;
    if (elapsed >= (unsigned long)servoMoveDuration) {
      currentServoAngle = targetServoAngle;
      servoMoving = false;
      servoFinishedPending = true;
      servoFinishedTime = millis();
    } else {
      float t = (float)elapsed / servoMoveDuration;
      // Sine ease-in-ease-out curve
      float smooth_t = 0.5f * (1.0f - cos(t * PI));
      currentServoAngle =
          startServoAngle + smooth_t * (targetServoAngle - startServoAngle);
    }

    // Rate limit physical servo updates (LEDC register writes) to 50Hz (every
    // 20ms) or when movement finishes. This prevents register congestion /
    // driver lockup on the ESP32.
    static unsigned long lastServoWriteTime = 0;
    static float lastWrittenAngle = -1.0f;
    if (millis() - lastServoWriteTime >= 20 || !servoMoving ||
        fabs(currentServoAngle - lastWrittenAngle) > 0.05f) {
      lastServoWriteTime = millis();
      lastWrittenAngle = currentServoAngle;

      // Convert angle (0 to 180 deg) to duty cycle ticks (500us to 2500us pulse
      // width)
      float pulseWidthUs = 500.0f + (currentServoAngle / 180.0f) * 2000.0f;
      uint32_t duty = (pulseWidthUs / 20000.0f) * 16384.0f;
      ledcWrite(SERVO_LEDC_CHANNEL, duty);
    }
  }

  if (servoFinishedPending && (millis() - servoFinishedTime >= 1000)) {
    servoFinishedPending = false;
    ledcWrite(SERVO_LEDC_CHANNEL, 0);
    Serial.println("[Servo] Idle. Detached power to stop buzzing.");
  }

  // Connect / Maintain Wi-Fi Connection & Active Link Watchdog
  static unsigned long lastWifiCheck = 0;
  static unsigned long connectedSince = 0;
  static unsigned long lastMqttOk = 0;

  // WLAN Watchdog Time Trap
  static unsigned long disconnectStartTime = 0;
  static bool timeTrapAlarmTriggered = false;

  // Check weekly watchdog reset timer
  checkWeeklyWatchdogReset();

  if (WiFi.status() == WL_CONNECTED) {
    disconnectStartTime = 0;
    timeTrapAlarmTriggered = false;

    if (!ntpInitialized) {
      ntpInitialized = true;
      configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org",
                   "time.nist.gov", "time.google.com");
      Serial.println(
          "[NTP] Initialized SNTP client for Europe/Berlin time zone.");
    }

    if (connectedSince == 0) {
      connectedSince = millis();
      lastMqttOk = millis();
    }

    // Watchdog 1: Check RSSI and test gateway reachability. If the router goes
    // offline, RSSI will report 0 or gateway TCP test fails.
    static unsigned long lastGatewayCheck = 0;
    if (millis() - connectedSince > 8000) { // Allow 8s post-connection buffer
      int rssi = WiFi.RSSI();
      if (rssi == 0 || rssi < -96) {
        Serial.printf("[WLAN] Watchdog: Router disappeared (RSSI = %d dBm). "
                      "Forcing disconnect...\n",
                      rssi);
        WiFi.disconnect(true);
        connectedSince = 0;
      } else if (millis() - lastGatewayCheck >=
                 2000) { // Verify gateway status every 2 seconds
        lastGatewayCheck = millis();
        if (!checkGatewayReachable()) {
          Serial.println("[WLAN] Watchdog: Gateway unreachable (Active TCP "
                         "check failed). Forcing disconnect...");
          WiFi.disconnect(true);
          connectedSince = 0;
        }
      }
    }
  } else {
    connectedSince = 0;

    // Time Trap Watchdog evaluation
    if (sysConfig.wlan_time_trap > 0) {
      if (disconnectStartTime == 0) {
        disconnectStartTime = millis();
        Serial.println(
            "[WLAN] Immediate connection loss alarm! Playing buzzer melody.");
        tone(BUZZER_PIN, 500, 250);
        delay(350);
        tone(BUZZER_PIN, 500, 250);
        delay(250);
        noTone(BUZZER_PIN);
      } else if (millis() - disconnectStartTime >=
                 (unsigned long)(sysConfig.wlan_time_trap * 1000)) {
        disconnectStartTime =
            millis(); // Reset timer to repeat alarm at interval
        Serial.println(
            "[WLAN] Watchdog repeat alarm triggered! Playing buzzer melody.");
        tone(BUZZER_PIN, 500, 250);
        delay(350);
        tone(BUZZER_PIN, 500, 250);
        delay(250);
        noTone(BUZZER_PIN);
      }
    }

    if (millis() - lastWifiCheck >=
        5000) { // Try to reconnect every 5s if disconnected
      lastWifiCheck = millis();
      Serial.printf("[WLAN] Connection lost. Reconnecting to %s...\n",
                    sysConfig.wifi_ssid);
      WiFi.begin(sysConfig.wifi_ssid, sysConfig.wifi_pass);
    }
  }

  // Connect / Maintain MQTT Connection
  if (WiFi.status() == WL_CONNECTED && strlen(sysConfig.mqtt_server) > 0) {
    if (mqttClient.connected()) {
      lastMqttOk = millis();
    } else {
      // Watchdog 2: Zombie Connection check. If WiFi reports connected but MQTT
      // cannot connect for 25s
      if (millis() - lastMqttOk > 25000) {
        Serial.println("[WLAN] Watchdog: Zombie link detected (MQTT "
                       "unreachable for 25s). Resetting WiFi...");
        WiFi.disconnect(true);
        connectedSince = 0;
        lastMqttOk = millis();
      }
    }
    static unsigned long lastMqttConnectAttempt = 0;
    if (!mqttClient.connected() && millis() - lastMqttConnectAttempt >= 10000) {
      lastMqttConnectAttempt = millis();
      Serial.println("[MQTT] Connecting to broker...");
      String clientID = String(sysConfig.mqtt_device_name) + "-" +
                        String(random(0xffff), HEX);

      bool mqttConnected = false;
      if (strlen(sysConfig.mqtt_user) > 0) {
        mqttConnected = mqttClient.connect(
            clientID.c_str(), sysConfig.mqtt_user, sysConfig.mqtt_pass);
      } else {
        mqttConnected = mqttClient.connect(clientID.c_str());
      }

      if (mqttConnected) {
        Serial.println("[MQTT] Broker Connected!");
        registerHomeAssistantDevices();
        publishMqttState(); // Publish initial state telemetry immediately!
      } else {
        Serial.printf("[MQTT] Connection failed, rc=%d. Retrying in 10s.\n",
                      mqttClient.state());
      }
    }
    mqttClient.loop();
  }

  static unsigned long lastUpdate = 0;
  static int updateCount = 0;

  // Run Display and Sensor Loop every 1 second
  unsigned long interval = 1000;

  if (millis() - lastUpdate >= interval) {
    lastUpdate = millis();
    updateCount++;

    // Read real sensors every 1 second (keeps Web UI and MQTT fresh)
    readSensors();
    updateHistoryAccumulators1s();

    // Trigger a closed-loop servo update only at the configured interval
    static unsigned long lastServoUpdateCall = 0;
    if (millis() - lastServoUpdateCall >=
        (unsigned long)(sysConfig.servo_update_interval * 1000)) {
      lastServoUpdateCall = millis();
      updateServoRamping(true);
    }

    // Publish to MQTT State based on configured interval (converted to seconds)
    if (updateCount % (sysConfig.mqtt_report_interval * 60) == 0) {
      publishMqttState();
    }

    if (isHeadless) {
      // Headless Mode: skip drawing to display to conserve power/speed
    } else if (isTFTMode) {
      // --- NATIVE TFT INTERFACE (Outlined UI with font size 1 for compact
      // display) ---
      tft.startWrite();
      tft.clear(TFT_BLACK);

      tft.drawRect(0, 0, tft.width(), tft.height(), TFT_GREEN);
      tft.drawRect(4, 4, tft.width() - 8, tft.height() - 8, TFT_BLUE);

      // Header (size 2)
      tft.setTextColor(TFT_YELLOW);
      tft.setTextSize(2);
      tft.setCursor(15, 20);
      tft.printf(sysConfig.mqtt_device_name);

      tft.setTextColor(TFT_WHITE);
      tft.setTextSize(1);
      tft.setCursor(15, 50);
      if (WiFi.status() == WL_CONNECTED) {
        tft.printf("IP: %s", WiFi.localIP().toString().c_str());
      } else {
        tft.printf("reconnecting [%s]", sysConfig.wifi_ssid);
      }

      // Details section (size 1)
      int cursorY = 70;
      tft.setTextColor(TFT_ORANGE);

      for (int i = 0; i < 2; i++) {
        if (tempSensors[i].active) {
          tft.setCursor(15, cursorY);
          String sType = (tempSensors[i].type == TempSensor::TYPE_BME280)
                             ? "BME280"
                             : "SHT3x";
          float dp = calculateDewPoint(tempSensors[i].temperature,
                                       tempSensors[i].humidity);
          tft.printf("%s [0x%02X]: %.1f C, %.1f %% (Taup: %.1f C)",
                     sType.c_str(), tempSensors[i].address,
                     tempSensors[i].temperature, tempSensors[i].humidity, dp);
          cursorY += 16;
          if (tempSensors[i].type == TempSensor::TYPE_BME280) {
            tft.setCursor(15, cursorY);
            tft.printf("  Druck: %.1f hPa", tempSensors[i].pressure);
            cursorY += 16;
          }
        }
      }

      for (int i = 0; i < 2; i++) {
        if (lightSensors[i].active) {
          tft.setTextColor(TFT_CYAN);
          tft.setCursor(15, cursorY);
          tft.printf("TSL2561 [0x%02X]: %.1f lx (B:%u IR:%u)",
                     lightSensors[i].address, lightSensors[i].lux,
                     lightSensors[i].broadband, lightSensors[i].ir);
          cursorY += 16;
        }
      }

      tft.setTextColor(TFT_GREEN);
      tft.setCursor(15, cursorY);
      tft.printf("Poti A (Sollwert): %.1f %%", potiAVal);
      cursorY += 14;
      tft.setCursor(15, cursorY);
      tft.printf("Poti B (Gain):     %.1f %%", potiBVal);
      cursorY += 14;
      tft.setCursor(15, cursorY);
      tft.printf("Poti C (Cal Off):  %.0f Grad", potiCVal);
      cursorY += 14;
      tft.setCursor(15, cursorY);
      tft.printf("Rotor-Stellung:    %.0f %%", rotorPosition);

      // Dynamic activity dot
      tft.fillCircle(210, 20, 6,
                     tft.color888(random(255), random(255), random(255)));

      tft.endWrite();
    } else {
      // --- NATIVE E-PAPER INTERFACE (Refreshed every 10 loop cycles to prevent
      // burnout) ---
      if (updateCount % 10 == 0) {
        Serial.println("[e-Paper] Re-drawing screen...");
        display.setRotation(1);
        display.setFont(&::FreeMonoBold9pt7b);

        display.firstPage();
        do {
          display.fillScreen(GxEPD_WHITE);
          display.drawRect(0, 0, display.width(), display.height(),
                           GxEPD_BLACK);
          display.drawRect(4, 4, display.width() - 8, display.height() - 8,
                           GxEPD_RED);

          // Title
          display.setTextColor(GxEPD_BLACK);
          display.setCursor(15, 30);
          if (WiFi.status() == WL_CONNECTED) {
            display.printf("%s E-Ink", sysConfig.mqtt_device_name);
          } else {
            display.printf("recon [%s]", sysConfig.wifi_ssid);
          }

          int epY = 60;
          for (int i = 0; i < 2; i++) {
            if (tempSensors[i].active) {
              display.setCursor(15, epY);
              String sType = (tempSensors[i].type == TempSensor::TYPE_BME280)
                                 ? "BME280"
                                 : "SHT3x";
              float dp = calculateDewPoint(tempSensors[i].temperature,
                                           tempSensors[i].humidity);
              display.printf("%s: %.1fC %.1f%% (T:%.1fC)", sType.c_str(),
                             tempSensors[i].temperature,
                             tempSensors[i].humidity, dp);
              epY += 28;
            }
          }

          for (int i = 0; i < 2; i++) {
            if (lightSensors[i].active) {
              display.setTextColor(GxEPD_RED);
              display.setCursor(15, epY);
              display.printf("L%d: %.1flx B:%u", i, lightSensors[i].lux,
                             lightSensors[i].broadband);
              epY += 28;
            }
          }

          display.setTextColor(GxEPD_BLACK);
          display.setCursor(15, epY);
          display.printf("A:%.0f%% B:%.0f%% C:%.0f", potiAVal, potiBVal,
                         potiCVal);
          epY += 28;
          display.setCursor(15, epY);
          display.printf("Rotor: %.0f %%", rotorPosition);

        } while (display.nextPage());
      }
    }
  }

  // 5-Minute Millis Time Trap for Low Humidity Alarm Check
  static unsigned long lastAlarmCheckTime = 0;
  if (millis() - lastAlarmCheckTime >= 300000) { // 5 minutes (300,000 ms)
    lastAlarmCheckTime = millis();

    // Use the promoted master inside sensor (tempSensors[0]) for the primary
    // check. If not active, fall back to tempSensors[1] if active.
    float hum_inside = NAN;
    if (tempSensors[0].active && !isnan(tempSensors[0].humidity)) {
      hum_inside = tempSensors[0].humidity;
    } else if (tempSensors[1].active && !isnan(tempSensors[1].humidity)) {
      hum_inside = tempSensors[1].humidity;
    }

    if (!isnan(hum_inside) && hum_inside < potiAVal) {
      Serial.printf("[Alarm] Inside humidity (%.1f%%) is below target "
                    "(%.1f%%). Playing warning chime.\n",
                    hum_inside, potiAVal);
      // Play 3 pleasant descending tones, 500ms each, no pause
      tone(BUZZER_PIN, 523, 500); // C5 (523 Hz)
      delay(500);
      tone(BUZZER_PIN, 440, 500); // A4 (440 Hz)
      delay(500);
      tone(BUZZER_PIN, 349, 500); // F4 (349 Hz)
      delay(500);
      noTone(BUZZER_PIN);
    }
  }

  // 5-Minute Millis Time Trap for Thermodynamic Bypass Alarm Check (offset by 5
  // seconds to prevent collision)
  static unsigned long lastBypassAlarmCheckTime =
      5000; // start with 5 seconds offset
  if (millis() - lastBypassAlarmCheckTime >= 300000) { // 5 minutes (300,000 ms)
    lastBypassAlarmCheckTime = millis();

    if (bypassModeActive) {
      float hum_inside = NAN;
      if (tempSensors[0].active && !isnan(tempSensors[0].humidity)) {
        hum_inside = tempSensors[0].humidity;
      } else if (tempSensors[1].active && !isnan(tempSensors[1].humidity)) {
        hum_inside = tempSensors[1].humidity;
      }

      // Only play the bypass alarm if the inside is still too wet (above/equal
      // to target humidity)
      if (isnan(hum_inside) || hum_inside >= potiAVal) {
        Serial.println("[Alarm] Thermodynamic bypass is active (Outside "
                       "humidity too high). Playing warning chime.");
        // Play 3 very short tones of 500 Hz with a short pause, 1s long pause,
        // and then repeat
        for (int repeat = 0; repeat < 2; repeat++) {
          for (int note = 0; note < 3; note++) {
            tone(BUZZER_PIN, 500, 80); // 500 Hz, 80ms duration
            delay(160);                // 80ms sound + 80ms pause
          }
          if (repeat == 0) {
            delay(840); // 1000ms total pause between sequences (1000 - 160 =
                        // 840ms extra delay)
          }
        }
        noTone(BUZZER_PIN);
      }
    }
  }
}
