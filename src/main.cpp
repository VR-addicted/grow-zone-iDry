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

// OTA Firmware Update & NVS Preferences Libraries
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Preferences.h>
#include <Update.h>
#include <WiFiClientSecure.h>

// Firmware Version (automatically incremented on each build via version_increment.py)
#ifndef BUILD_NUMBER
#define BUILD_NUMBER 61
#endif
const int localFirmwareVersion = BUILD_NUMBER;
extern int cachedOnlineVersion;
void checkGithubUpdateAsync(bool force = false);

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
  char web_password[33] = "";    // Optional Web UI protection password (empty = open access)
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
  int dry_strategy = 0;         // Dry Strategy: 0 = 60/60 Mode, 1 = VPD Mode, 2 = VPD AUTO
  int hygro_limit = 70;         // Hygro-Limit Safety Cap: 70 or 75 (%)
  int vpd_auto_day = 1;         // Active day index for VPD AUTO mode (1 to 14)
  unsigned long vpd_auto_start_time = 0; // Timestamp when active day was set
  int log_level = 3;            // Log Level: 1 = Status/Telemetry, 2 = Warn/Alarm, 3 = Verbose Debug (Default Level 3)
  int purge_interval_min = 240; // Purge interval in minutes (0 = disabled, 10 to 1440 min)
  int purge_duration_sec = 30;   // Purge opening duration in seconds (10 to 600 sec)
  float servo_total_meters = 0.0f; // Total Servo travel distance in meters (r=27mm)
};

// --- T-PIPE LIVE SYSTEM LOGGING ARCHITECTURE ---
#define APP_LOG_BUFFER_SIZE 16
#define APP_LOG_LINE_MAX 120

struct AppLogLine {
  char text[APP_LOG_LINE_MAX];
};

AppLogLine appLogBuffer[APP_LOG_BUFFER_SIZE];
int appLogHead = 0;
int appLogCount = 0;

AppLogLine remoteAppLogBuffer[APP_LOG_BUFFER_SIZE];
int remoteAppLogHead = 0;
int remoteAppLogCount = 0;

void addRemoteAppLog(const char* logLine) {
  AppLogLine& line = remoteAppLogBuffer[remoteAppLogHead];
  snprintf(line.text, sizeof(line.text), "%s", logLine);
  remoteAppLogHead = (remoteAppLogHead + 1) % APP_LOG_BUFFER_SIZE;
  if (remoteAppLogCount < APP_LOG_BUFFER_SIZE) {
    remoteAppLogCount++;
  }
}

void sendEspNowLogLine(const char* logLine);
extern Config sysConfig;

void addAppLogEx(uint8_t level, const char* format, ...) {
  if (level > (uint8_t)sysConfig.log_level) return;

  char rawMsg[APP_LOG_LINE_MAX];
  va_list args;
  va_start(args, format);
  vsnprintf(rawMsg, sizeof(rawMsg), format, args);
  va_end(args);

  // 1. Output to USB UART Serial
  Serial.println(rawMsg);

  // 2. Add timestamp [HH:MM:SS] and tag [L1]/[L2]/[L3]
  uint32_t sec = millis() / 1000;
  uint32_t hrs = (sec / 3600) % 24;
  uint32_t mins = (sec % 3600) / 60;
  uint32_t secs = sec % 60;

  const char* lvlTag = (level == 1) ? "STAT" : ((level == 2) ? "WARN" : "DBG ");

  AppLogLine& line = appLogBuffer[appLogHead];
  snprintf(line.text, sizeof(line.text), "[%02u:%02u:%02u] [%s] %s", hrs, mins, secs, lvlTag, rawMsg);

  appLogHead = (appLogHead + 1) % APP_LOG_BUFFER_SIZE;
  if (appLogCount < APP_LOG_BUFFER_SIZE) {
    appLogCount++;
  }

  // 3. Stream log line to peer over ESP-NOW
  sendEspNowLogLine(line.text);
}

void addAppLog(const char* format, ...) {
  char rawMsg[APP_LOG_LINE_MAX];
  va_list args;
  va_start(args, format);
  vsnprintf(rawMsg, sizeof(rawMsg), format, args);
  va_end(args);
  addAppLogEx(1, "%s", rawMsg);
}

Config sysConfig;
bool isConfigLoaded = false;

// Potentiometer States
float potiAVal = 0.0;      // Target Humidity / Calculated Target RH: 0 - 100%
float potiBVal = 0.0;      // Gain: 0 - 400%
float potiCVal = 0.0;      // Calibration Offset: 0 to 120 deg
float targetVpdVal = 0.0f; // Target VPD in kPa (0.60 to 1.40)
float rawCalculatedRh =
    0.0f; // Raw mathematically calculated target RH (unlimited)
float effectiveTargetRh = 0.0f; // Calculated target RH clamped to hygro_limit
float rotorPosition = 0.0;      // Logical Rotor opening: 0 - 100%
bool bypassModeActive = false;  // Thermodynamic bypass (notschließen) is active

// Servo Motion Profiling (Ease-In-Ease-Out Softstart/Stop Ramping)
float targetServoAngle = 0.0f;
float startServoAngle = 0.0f;
float currentServoAngle = 0.0f;
unsigned long servoMoveStartTime = 0;
float servoMoveDuration = 0.0f; // in milliseconds
bool servoMoving = false;
bool servoFinishedPending = false;
unsigned long servoFinishedTime = 0;

// Servo Odometer & Lifetime Tracking Globals (r = 27mm, 0.47124 mm/deg)
float servoTotalMeters = 0.0f;
float lastSavedServoMeters = 0.0f;
unsigned long lastOdometerSaveTime = 0;
float lastTrackedServoAngle = -1.0f;

// Display Ambient Light Backlight Auto-Dimmer
unsigned long darknessStartTime = 0;
bool isDisplayDarkened = false;

// 21x14 Scientific Temperature-VPD Matrix for Temperatures 15°C to 35°C across Days 1 to 14
const float vpdTempMatrix[21][14] = {
  // 15°C
  {0.56f, 0.58f, 0.59f, 0.61f, 0.62f, 0.64f, 0.65f, 0.66f, 0.66f, 0.67f, 0.68f, 0.68f, 0.68f, 0.68f},
  // 16°C
  {0.59f, 0.60f, 0.62f, 0.64f, 0.66f, 0.67f, 0.68f, 0.69f, 0.70f, 0.71f, 0.71f, 0.71f, 0.71f, 0.71f},
  // 17°C
  {0.62f, 0.63f, 0.65f, 0.67f, 0.69f, 0.70f, 0.71f, 0.72f, 0.73f, 0.74f, 0.75f, 0.75f, 0.75f, 0.75f},
  // 18°C
  {0.64f, 0.66f, 0.68f, 0.70f, 0.72f, 0.74f, 0.75f, 0.75f, 0.76f, 0.77f, 0.78f, 0.78f, 0.78f, 0.78f},
  // 19°C
  {0.67f, 0.69f, 0.71f, 0.73f, 0.75f, 0.77f, 0.78f, 0.79f, 0.80f, 0.81f, 0.82f, 0.82f, 0.82f, 0.82f},
  // 20°C (Baseline)
  {0.70f, 0.72f, 0.74f, 0.76f, 0.78f, 0.80f, 0.81f, 0.82f, 0.83f, 0.84f, 0.85f, 0.85f, 0.85f, 0.85f},
  // 21°C
  {0.73f, 0.75f, 0.77f, 0.79f, 0.81f, 0.83f, 0.84f, 0.85f, 0.86f, 0.87f, 0.88f, 0.88f, 0.88f, 0.88f},
  // 22°C
  {0.76f, 0.78f, 0.80f, 0.82f, 0.84f, 0.86f, 0.87f, 0.89f, 0.90f, 0.91f, 0.92f, 0.92f, 0.92f, 0.92f},
  // 23°C
  {0.78f, 0.81f, 0.83f, 0.85f, 0.87f, 0.90f, 0.91f, 0.92f, 0.93f, 0.94f, 0.95f, 0.95f, 0.95f, 0.95f},
  // 24°C
  {0.81f, 0.84f, 0.86f, 0.88f, 0.90f, 0.93f, 0.94f, 0.95f, 0.96f, 0.97f, 0.99f, 0.99f, 0.99f, 0.99f},
  // 25°C
  {0.84f, 0.86f, 0.89f, 0.91f, 0.94f, 0.96f, 0.97f, 0.98f, 1.00f, 1.01f, 1.02f, 1.02f, 1.02f, 1.02f},
  // 26°C
  {0.87f, 0.89f, 0.92f, 0.94f, 0.97f, 0.99f, 1.00f, 1.02f, 1.03f, 1.04f, 1.05f, 1.05f, 1.05f, 1.05f},
  // 27°C
  {0.90f, 0.92f, 0.95f, 0.97f, 1.00f, 1.02f, 1.04f, 1.05f, 1.06f, 1.07f, 1.09f, 1.09f, 1.09f, 1.09f},
  // 28°C
  {0.92f, 0.95f, 0.98f, 1.00f, 1.03f, 1.06f, 1.07f, 1.08f, 1.10f, 1.11f, 1.12f, 1.12f, 1.12f, 1.12f},
  // 29°C
  {0.95f, 0.98f, 1.01f, 1.03f, 1.06f, 1.09f, 1.10f, 1.12f, 1.13f, 1.14f, 1.16f, 1.16f, 1.16f, 1.16f},
  // 30°C
  {0.98f, 1.01f, 1.04f, 1.06f, 1.10f, 1.12f, 1.13f, 1.15f, 1.16f, 1.18f, 1.19f, 1.19f, 1.19f, 1.19f},
  // 31°C
  {1.01f, 1.04f, 1.07f, 1.10f, 1.13f, 1.16f, 1.17f, 1.19f, 1.20f, 1.21f, 1.23f, 1.23f, 1.23f, 1.23f},
  // 32°C
  {1.04f, 1.07f, 1.10f, 1.13f, 1.16f, 1.19f, 1.20f, 1.22f, 1.23f, 1.25f, 1.26f, 1.26f, 1.26f, 1.26f},
  // 33°C
  {1.06f, 1.10f, 1.13f, 1.16f, 1.19f, 1.22f, 1.24f, 1.25f, 1.27f, 1.28f, 1.30f, 1.30f, 1.30f, 1.30f},
  // 34°C
  {1.09f, 1.13f, 1.16f, 1.19f, 1.23f, 1.26f, 1.27f, 1.29f, 1.30f, 1.32f, 1.33f, 1.33f, 1.33f, 1.33f},
  // 35°C
  {1.12f, 1.16f, 1.19f, 1.22f, 1.26f, 1.29f, 1.30f, 1.32f, 1.34f, 1.35f, 1.37f, 1.37f, 1.37f, 1.37f}
};

// =====================================================================
// ESP-NOW & PAIRING STATE MACHINE & WI-FI CHANNEL HOPS
// =====================================================================
struct __attribute__((packed)) EspNowMessage {
  uint8_t pv;      // Protocol version (currently 5)
  uint8_t type;    // 0 = Pairing Beacon, 1 = Pairing Response, 2 = Command/Data
  char key[33];    // LMK hex string exchanged during pairing
  uint8_t command; // 0 = None, 1 = Play Winner Melody, 2 = Ping-Request/Data
                   // Sync, 3 = Ping-Reply, 99 = Remote Reboot Request
  float value;     // Numeric payload value (rotorPosition)
  uint8_t dry_strategy; // Dry Strategy: 0 = 60/60 Mode, 1 = VPD Mode
};

struct __attribute__((packed)) EspNowLogMessage {
  uint8_t pv;        // Protocol version (5)
  uint8_t type;      // 3 = Log Line Stream
  char logText[180]; // Timestamped log message string
};

const uint8_t localProtocolVersion = 5;
uint8_t remoteProtocolVersion = 0;
bool protocolVersionMismatch = false;
uint32_t avgEspNowIntervalMs = 1000;
uint8_t remoteMasterDryStrategy = 0;

static bool isSendingLogPacket = false;
static bool isEspNowInitialized = false;

void sendEspNowLogLine(const char* logLine) {
  if (isSendingLogPacket) return;
  if (!isEspNowInitialized) return;
  if (sysConfig.espnow_role == 0 || strlen(sysConfig.espnow_peer_mac) == 0) return;

  uint8_t peerMac[6];
  if (sscanf(sysConfig.espnow_peer_mac, "%x:%x:%x:%x:%x:%x",
             &peerMac[0], &peerMac[1], &peerMac[2],
             &peerMac[3], &peerMac[4], &peerMac[5]) != 6) {
    return;
  }

  isSendingLogPacket = true;
  EspNowLogMessage logPacket;
  logPacket.pv = localProtocolVersion;
  logPacket.type = 3; // Log Line Stream
  strlcpy(logPacket.logText, logLine, sizeof(logPacket.logText));

  esp_now_send(peerMac, (uint8_t*)&logPacket, sizeof(EspNowLogMessage));
  isSendingLogPacket = false;
}

unsigned long lastEspNowRxTime = 0;
unsigned long lastEspNowTxSuccessTime = 0;
unsigned long connectedSince = 0;
unsigned long lastPurgeTimestamp = 0;
bool isPurgeActive = false;
unsigned long purgeStartMs = 0;
static unsigned long lastWifiAttemptTime = 0;
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

float calculateSVP(float temp) {
  if (isnan(temp))
    return 0.0f;
  return 0.61078f * exp((17.27f * temp) / (temp + 237.3f));
}

// Forward declarations
bool saveConfiguration();
void initEspNow();
void saveOdometer(bool force = false);

void loadOdometer() {
  Preferences prefs;
  float nvsMeters = 0.0f;
  if (prefs.begin("idry_odo", false)) { // Open Read-Write to create namespace if missing
    uint32_t magic = prefs.getUInt("magic", 0);
    if (magic == 0x49445259) { // "IDRY"
      nvsMeters = prefs.getFloat("meters", 0.0f);
    }
    prefs.end();
  }
  float lfsMeters = sysConfig.servo_total_meters;
  servoTotalMeters = max(lfsMeters, nvsMeters);
  lastSavedServoMeters = servoTotalMeters;
  if (fabs(lfsMeters - nvsMeters) > 0.05f) {
    saveOdometer(true);
  }
  Serial.printf("[Odometer] Loaded Servo Laufleistung: %.2f m (%.3f km) [NVS: %.2f m, LittleFS: %.2f m]\n", servoTotalMeters, servoTotalMeters / 1000.0f, nvsMeters, lfsMeters);
}

void saveOdometer(bool force) {
  if (!force && (fabs(servoTotalMeters - lastSavedServoMeters) < 0.05f)) {
    return; // No movement occurred, preserve Flash write cycles!
  }
  // 1. Mirror to ESP32 NVS Partition
  Preferences prefs;
  if (prefs.begin("idry_odo", false)) { // Read-Write
    prefs.putUInt("magic", 0x49445259);
    prefs.putFloat("meters", servoTotalMeters);
    prefs.end();
  }
  // 2. Mirror to LittleFS config.json
  sysConfig.servo_total_meters = servoTotalMeters;
  saveConfiguration();
  lastSavedServoMeters = servoTotalMeters;
  lastOdometerSaveTime = millis();
}

void performFactoryReset(const char* sourceTag) {
  addAppLogEx(1, "[System] FACTORY RESET TRIGGERED via %s! Clearing /config.json...", sourceTag);
  
  // Descending alert chime
  tone(BUZZER_PIN, 1047, 120); // C6
  delay(140);
  tone(BUZZER_PIN, 784, 120);  // G5
  delay(140);
  tone(BUZZER_PIN, 523, 200);  // C5
  delay(220);
  noTone(BUZZER_PIN);

  // Reset sysConfig struct to defaults and explicitly enforce 60/60 Mode (strategy = 0)
  sysConfig = Config();
  sysConfig.dry_strategy = 0;
  sysConfig.log_level = 3;

  LittleFS.remove("/config.json");
  saveConfiguration();

  delay(300);
  ESP.restart();
}

void onEspNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1],
          mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  if (status == ESP_NOW_SEND_SUCCESS && sysConfig.espnow_role == 1 &&
      strlen(sysConfig.espnow_peer_mac) > 0 &&
      strcasecmp(macStr, sysConfig.espnow_peer_mac) == 0) {
    lastEspNowTxSuccessTime = millis();
  }
}

void onEspNowDataRecv(const uint8_t *mac_addr, const uint8_t *data,
                      int data_len) {
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1],
          mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  if (data_len == (int)sizeof(EspNowLogMessage)) {
    EspNowLogMessage logMsg;
    memcpy(&logMsg, data, sizeof(EspNowLogMessage));
    if (logMsg.pv == localProtocolVersion && logMsg.type == 3) {
      addRemoteAppLog(logMsg.logText);
      bool isFromPeer = (strlen(sysConfig.espnow_peer_mac) > 0 && strcasecmp(macStr, sysConfig.espnow_peer_mac) == 0);
      if (isFromPeer || isPairingActive) {
        lastEspNowRxTime = millis();
      }
    }
    return;
  }

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
    addAppLogEx(1, "[Pairing] SUCCESS! Slave paired with Master MAC: %s, LMK: %s", sysConfig.espnow_peer_mac, sysConfig.espnow_lmk);
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
    addAppLogEx(1, "[Pairing] SUCCESS! Master paired with Slave MAC: %s, LMK: %s", macStr, proposedLmk);
  }

  // 3. Command/Data (Type 2)
  else if (msg.type == 2) {
    // Only accept commands from paired peer (case insensitive MAC check)
    if (strcasecmp(macStr, sysConfig.espnow_peer_mac) == 0) {
      lastEspNowRxTime = millis();
      if (msg.command == 1) {
        playWinnerMelody();
      } else if (msg.command == 2) {
        if (sysConfig.espnow_role == 2) {
          rotorPosition = msg.value;
          if (msg.dry_strategy <= 2) {
            remoteMasterDryStrategy = msg.dry_strategy;
          }
          addAppLogEx(3, "[ESP-NOW] RX Sync from Master: Rotor=%.0f%%, Mode=%d", msg.value, remoteMasterDryStrategy);
          static unsigned long lastSlaveSyncRecvTime = 0;
          if (lastSlaveSyncRecvTime == 0) {
            lastSlaveSyncRecvTime = millis();
          } else {
            unsigned long diff = millis() - lastSlaveSyncRecvTime;
            if (diff >= 750 && diff <= 4000) {
              avgEspNowIntervalMs = diff;
              lastSlaveSyncRecvTime = millis();
            }
          }

          // Construct and transmit Ping-Reply back to Master
          EspNowMessage replyMsg;
          memset(&replyMsg, 0, sizeof(EspNowMessage));
          replyMsg.pv = localProtocolVersion;
          replyMsg.type = 2; // Data/Command
          strlcpy(replyMsg.key, sysConfig.espnow_lmk, sizeof(replyMsg.key));
          replyMsg.command = 3; // Ping-Reply
          replyMsg.value = rotorPosition;
          replyMsg.dry_strategy = sysConfig.dry_strategy;

          uint8_t curChan = 1;
          wifi_second_chan_t secondChan;
          esp_wifi_get_channel(&curChan, &secondChan);

          esp_now_peer_info_t peerInfo;
          memset(&peerInfo, 0, sizeof(peerInfo));
          memcpy(peerInfo.peer_addr, mac_addr, 6);
          peerInfo.channel = (curChan > 0) ? curChan : 1;
          peerInfo.ifidx = WIFI_IF_STA;
          peerInfo.encrypt = false;
          if (esp_now_is_peer_exist(mac_addr)) {
            esp_now_mod_peer(&peerInfo);
          } else {
            esp_now_add_peer(&peerInfo);
          }

          esp_now_send(mac_addr, (uint8_t *)&replyMsg, sizeof(EspNowMessage));
        }
      } else if (msg.command == 3) {
        // Ping-Reply from Slave received by Master
        if (sysConfig.espnow_role == 1) {
          lastEspNowTxSuccessTime = millis();
          static unsigned long lastMasterSyncRecvTime = 0;
          if (lastMasterSyncRecvTime == 0) {
            lastMasterSyncRecvTime = millis();
          } else {
            unsigned long diff = millis() - lastMasterSyncRecvTime;
            if (diff >= 750 && diff <= 4000) {
              avgEspNowIntervalMs = diff;
              lastMasterSyncRecvTime = millis();
            }
          }
        }
      } else if (msg.command == 99) {
        addAppLogEx(1, "[System] Remote Reboot command received over ESP-NOW from %s! Rebooting in 300ms...", macStr);
        tone(BUZZER_PIN, 600, 150);
        delay(300);
        ESP.restart();
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
    isEspNowInitialized = false;
    esp_now_deinit();
    return;
  }

  // Safely reset driver before initializing
  isEspNowInitialized = false;
  esp_now_deinit();

  Serial.println("[ESP-NOW] Initializing ESP-NOW...");
  if (esp_now_init() != ESP_OK) {
    isEspNowInitialized = false;
    Serial.println("[ESP-NOW] Initialization failed!");
    return;
  }

  isEspNowInitialized = true;

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

    uint8_t activeChannel = (WiFi.status() == WL_CONNECTED)
                                ? WiFi.channel()
                                : sysConfig.espnow_channel;
    if (activeChannel == 0)
      activeChannel =
          (sysConfig.espnow_channel > 0) ? sysConfig.espnow_channel : 1;

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
      pingMsg.dry_strategy = (uint8_t)sysConfig.dry_strategy;
      esp_now_send(peerInfo.peer_addr, (uint8_t *)&pingMsg,
                   sizeof(EspNowMessage));
    }
  }
}

uint32_t calculateConfigCRC(const JsonDocument &doc) {
  String payload;
  serializeJson(doc, payload);

  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < payload.length(); i++) {
    uint8_t b = (uint8_t)payload[i];
    crc ^= b;
    for (int j = 0; j < 8; j++) {
      uint32_t mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
  }
  return ~crc;
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
    Serial.println("[LittleFS] CORRUPTED CONFIG DETECTED: JSON parse error! "
                   "Auto-purging bad config file...");
    LittleFS.remove("/config.json");
    return false;
  }

  // Verify embedded CRC32 checksum if present
  if (doc["crc"].is<uint32_t>()) {
    uint32_t savedCrc = doc["crc"].as<uint32_t>();
    doc.remove("crc");
    uint32_t computedCrc = calculateConfigCRC(doc);
    if (savedCrc != computedCrc) {
      Serial.printf(
          "[LittleFS] CORRUPTED CONFIG DETECTED: CRC32 mismatch! (Saved: "
          "0x%08X, Computed: 0x%08X). Auto-purging bad config file...\n",
          savedCrc, computedCrc);
      LittleFS.remove("/config.json");
      return false;
    }
    Serial.printf("[LittleFS] CRC32 verification SUCCESS! (0x%08X)\n",
                  savedCrc);
  } else {
    Serial.println("[LittleFS] Legacy configuration without CRC detected. "
                   "Upgrading on next save.");
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
  strlcpy(sysConfig.web_password, doc["web_password"] | "",
          sizeof(sysConfig.web_password));
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
  sysConfig.dry_strategy = doc["dry_strategy"] | 0;
  sysConfig.hygro_limit = doc["hygro_limit"] | 70;
  sysConfig.vpd_auto_day = doc["vpd_auto_day"] | 1;
  sysConfig.vpd_auto_start_time = doc["vpd_auto_start_time"] | 0;
  sysConfig.log_level = doc["log_level"] | 3;
  sysConfig.purge_interval_min = doc["purge_interval_min"] | 240;
  sysConfig.purge_duration_sec = doc["purge_duration_sec"] | 30;
  sysConfig.servo_total_meters = doc["servo_total_meters"] | 0.0f;

  loadOdometer();

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
  doc["web_password"] = sysConfig.web_password;
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
  doc["dry_strategy"] = sysConfig.dry_strategy;
  doc["hygro_limit"] = sysConfig.hygro_limit;
  doc["vpd_auto_day"] = sysConfig.vpd_auto_day;
  doc["vpd_auto_start_time"] = sysConfig.vpd_auto_start_time;
  doc["log_level"] = sysConfig.log_level;
  doc["purge_interval_min"] = sysConfig.purge_interval_min;
  doc["purge_duration_sec"] = sysConfig.purge_duration_sec;
  doc["servo_total_meters"] = sysConfig.servo_total_meters;

  // Compute and embed CRC32 checksum
  uint32_t crcVal = calculateConfigCRC(doc);
  doc["crc"] = crcVal;

  if (serializeJson(doc, file) == 0) {
    Serial.println("[LittleFS] Failed to serialize configuration JSON.");
    file.close();
    return false;
  }
  file.close();
  Serial.printf(
      "[LittleFS] Configuration successfully saved with CRC32: 0x%08X\n",
      crcVal);
  return true;
}

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

  long espnowLastSeen = -1;
  if (sysConfig.espnow_role == 1) {
    espnowLastSeen = (lastEspNowTxSuccessTime == 0)
                         ? -1
                         : (long)(millis() - lastEspNowTxSuccessTime);
  } else if (sysConfig.espnow_role == 2) {
    espnowLastSeen =
        (lastEspNowRxTime == 0) ? -1 : (long)(millis() - lastEspNowRxTime);
  }

  if (sysConfig.espnow_role > 0 &&
      (espnowLastSeen == -1 || espnowLastSeen > 3500)) {
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
          static unsigned long lastBmeLog[2] = {0, 0};
          if (millis() - lastBmeLog[i] >= 10000) {
            lastBmeLog[i] = millis();
            addAppLogEx(3, "[Sensor] BME280 (0x%02X): %.1f C, %.1f %%, %.1f hPa", tempSensors[i].address, t, h, p);
          }
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
          static unsigned long lastShtLog[2] = {0, 0};
          if (millis() - lastShtLog[i] >= 10000) {
            lastShtLog[i] = millis();
            addAppLogEx(3, "[Sensor] SHT3x (0x%02X): %.1f C, %.1f %%", tempSensors[i].address, t, h);
          }
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

int getVpdAutoCurrentDay() {
  if (sysConfig.vpd_auto_day < 1) sysConfig.vpd_auto_day = 1;
  if (sysConfig.vpd_auto_day > 14) sysConfig.vpd_auto_day = 14;

  if (sysConfig.vpd_auto_start_time == 0) {
    time_t now = time(NULL);
    if (now > 1700000000UL) {
      sysConfig.vpd_auto_start_time = (uint32_t)now;
    } else {
      sysConfig.vpd_auto_start_time = (uint32_t)(millis() / 1000UL);
    }
    saveConfiguration();
  }

  time_t now = time(NULL);
  int daysPassed = 0;
  if (now > 1700000000UL && sysConfig.vpd_auto_start_time > 1700000000UL) {
    time_t startTime = (time_t)sysConfig.vpd_auto_start_time;
    struct tm tmStart, tmNow;
    localtime_r(&startTime, &tmStart);
    localtime_r(&now, &tmNow);
    tmStart.tm_hour = 0; tmStart.tm_min = 0; tmStart.tm_sec = 0;
    tmNow.tm_hour = 0; tmNow.tm_min = 0; tmNow.tm_sec = 0;
    time_t startMidnight = mktime(&tmStart);
    time_t nowMidnight = mktime(&tmNow);

    if (nowMidnight > startMidnight) {
      daysPassed = (int)((nowMidnight - startMidnight) / 86400UL);
    }
  } else if (sysConfig.vpd_auto_start_time <= 1700000000UL) {
    unsigned long currentSec = millis() / 1000UL;
    if (currentSec >= sysConfig.vpd_auto_start_time) {
      daysPassed = (int)((currentSec - sysConfig.vpd_auto_start_time) / 86400UL);
    }
  }

  if (daysPassed > 0) {
    int newDay = sysConfig.vpd_auto_day + daysPassed;
    if (newDay > 14) newDay = 14;
    if (newDay != sysConfig.vpd_auto_day) {
      sysConfig.vpd_auto_day = newDay;
      if (now > 1700000000UL) {
        sysConfig.vpd_auto_start_time = (uint32_t)now;
      } else {
        sysConfig.vpd_auto_start_time = (uint32_t)(millis() / 1000UL);
      }
      saveConfiguration(); // PERSIST AUTOMATIC DAY ROLLOVER TO FLASH!
      addAppLogEx(1, "[VPD AUTO] Midnight Rollover! Auto-advanced & saved active Day %d to Flash.", sysConfig.vpd_auto_day);
    }
  }

  return sysConfig.vpd_auto_day;
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

  if (sysConfig.dry_strategy == 2) { // VPD AUTO Mode (Scientific 21x14 Temperature-Compensated Matrix)
    int currentDay = getVpdAutoCurrentDay();
    float indoorTemp = NAN;
    if (tempSensors[0].active && !isnan(tempSensors[0].temperature)) {
      indoorTemp = tempSensors[0].temperature;
    } else if (tempSensors[1].active && !isnan(tempSensors[1].temperature)) {
      indoorTemp = tempSensors[1].temperature;
    }
    if (isnan(indoorTemp)) indoorTemp = 20.0f; // Default fallback temperature

    int tempIdx = (int)round(indoorTemp) - 15;
    if (tempIdx < 0) tempIdx = 0;
    if (tempIdx > 20) tempIdx = 20;

    targetVpdVal = vpdTempMatrix[tempIdx][currentDay - 1];

    float svp = calculateSVP(indoorTemp);
    float avpTarget = svp - targetVpdVal;
    rawCalculatedRh = (svp > 0.001f) ? (avpTarget / svp) * 100.0f : 60.0f;
    float rhCalc = rawCalculatedRh;
    if (rhCalc < 30.0f) rhCalc = 30.0f;
    if (rhCalc > (float)sysConfig.hygro_limit) {
      rhCalc = (float)sysConfig.hygro_limit;
    }
    effectiveTargetRh = (float)round(rhCalc * 10.0f) / 10.0f;
    static unsigned long lastVpdAutoLog = 0;
    if (millis() - lastVpdAutoLog >= 15000) {
      lastVpdAutoLog = millis();
      addAppLogEx(3, "[VPD AUTO] Day %d @ %.1f C -> Matrix Target VPD: %.2f kPa, RH Target: %.1f %%", currentDay, indoorTemp, targetVpdVal, effectiveTargetRh);
    }
  } else if (sysConfig.dry_strategy == 1) { // VPD Mode
    targetVpdVal = 0.60f + (smoothedA / 4095.0f) * 0.80f;
    targetVpdVal = (float)round(targetVpdVal * 100.0f) / 100.0f;
    if (targetVpdVal < 0.60f)
      targetVpdVal = 0.60f;
    if (targetVpdVal > 1.40f)
      targetVpdVal = 1.40f;

    float indoorTemp = NAN;
    if (tempSensors[0].active && !isnan(tempSensors[0].temperature)) {
      indoorTemp = tempSensors[0].temperature;
    } else if (tempSensors[1].active && !isnan(tempSensors[1].temperature)) {
      indoorTemp = tempSensors[1].temperature;
    }

    if (!isnan(indoorTemp)) {
      float svp = calculateSVP(indoorTemp);
      float avpTarget = svp - targetVpdVal;
      rawCalculatedRh = (svp > 0.001f) ? (avpTarget / svp) * 100.0f : 60.0f;
      float rhCalc = rawCalculatedRh;
      if (rhCalc < 30.0f)
        rhCalc = 30.0f;
      if (rhCalc > (float)sysConfig.hygro_limit)
        rhCalc = (float)sysConfig.hygro_limit;
      effectiveTargetRh = (float)round(rhCalc * 10.0f) / 10.0f;
    } else {
      rawCalculatedRh = 60.0f;
      effectiveTargetRh = 60.0f;
    }
  } else {
    targetVpdVal = 0.0f;
    rawCalculatedRh = potiAVal;
    effectiveTargetRh = potiAVal;
  }
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
                            (millis() - lastEspNowRxTime <= 5000);
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
      if (sysConfig.dry_strategy == 0 && potiAVal <= 49.0f) {
        // Virtual switch at bottom end (60/60 mode only): Rigorously closed (0% opening)
        rotorPosition = 0.0f;
        bypassModeActive = false;
        isPurgeActive = false;
      } else if (sysConfig.dry_strategy == 0 && potiAVal >= 71.0f) {
        // Virtual switch at top end (60/60 mode only): Rigorously open (100% opening)
        rotorPosition = 100.0f;
        bypassModeActive = false;
        isPurgeActive = false;
      } else {
        // Normal closed-loop sensor-servo control algorithm
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

        // Stoßlüftungs-Timer (Purge Ventilation) State Machine
        if (sysConfig.purge_interval_min > 0) {
          unsigned long intervalMs = sysConfig.purge_interval_min * 60000UL;
          unsigned long durationMs = sysConfig.purge_duration_sec * 1000UL;
          if (lastPurgeTimestamp == 0) {
            lastPurgeTimestamp = millis();
          }

          if (isPurgeActive) {
            if (millis() - purgeStartMs >= durationMs) {
              isPurgeActive = false;
              lastPurgeTimestamp = millis();
              addAppLogEx(1, "[Purge] Stoßlüften BEENDET nach %ds.", sysConfig.purge_duration_sec);
            }
          } else {
            if (millis() - lastPurgeTimestamp >= intervalMs) {
              float activeTargetRh = (sysConfig.dry_strategy != 0) ? effectiveTargetRh : potiAVal;
              if (!isnan(hum_inside) && hum_inside < activeTargetRh) {
                lastPurgeTimestamp = millis();
                addAppLogEx(1, "[Purge] Stoßlüften ÜBERSPRUNGEN: Innen-Feuchte (%.1f%%) liegt unter Sollwert (%.1f%%).", hum_inside, activeTargetRh);
              } else {
                isPurgeActive = true;
                purgeStartMs = millis();
                addAppLogEx(1, "[Purge] Stoßlüften AKTIV! Rotor auf 100%% für %ds.", sysConfig.purge_duration_sec);
              }
            }
          }
        } else {
          isPurgeActive = false;
        }

        if (isPurgeActive) {
          rotorPosition = 100.0f;
          bypassModeActive = false;
        } else if (!isnan(hum_inside)) {
          float activeTargetRh =
              (sysConfig.dry_strategy != 0) ? effectiveTargetRh : potiAVal;

          // Thermodynamic bypass check: If outside humidity is higher than
          // inside humidity OR outside humidity is more than 2% above the
          // target humidity, keep the rotor closed!
          if (!isnan(hum_outside) && (hum_outside > hum_inside ||
                                      hum_outside > (activeTargetRh + 2.0f))) {
            rotorPosition =
                0.0f; // Moisture loading threat! Keep shutter fully closed.
            if (!bypassModeActive) {
              bypassModeActive = true;
              // Suppress warning chime if we are already dry (below target
              // humidity)
              if (isnan(hum_inside) || hum_inside >= activeTargetRh) {
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
            // If current inside humidity is higher than Target, we
            // open the rotor to dry the system
            float error = hum_inside - activeTargetRh;
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
              float diff = activeTargetRh - hum_outside;
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
      addAppLogEx(3, "[Servo] Shutter: %.0f%% -> Angle: %.1f deg (Move time: %.1fs)", rotorPosition, targetServoAngle, servoMoveDuration/1000.0f);
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

bool isWebAuthenticated() {
  if (strlen(sysConfig.web_password) == 0) return true;
  String pass = server.arg("pass");
  if (pass.length() == 0) {
    pass = server.header("X-Web-Pass");
  }
  if (pass.length() == 0 && server.hasHeader("Cookie")) {
    String cookie = server.header("Cookie");
    int idx = cookie.indexOf("idry_pass=");
    if (idx != -1) {
      int endIdx = cookie.indexOf(";", idx);
      if (endIdx == -1) endIdx = cookie.length();
      pass = cookie.substring(idx + 10, endIdx);
      pass.trim();
    }
  }
  return (strcmp(pass.c_str(), sysConfig.web_password) == 0);
}

void handleApiAuth() {
  String pass = server.arg("pass");
  if (strlen(sysConfig.web_password) == 0 || strcmp(pass.c_str(), sysConfig.web_password) == 0) {
    if (strlen(sysConfig.web_password) > 0) {
      server.sendHeader("Set-Cookie", "idry_pass=" + pass + "; Path=/; Max-Age=86400");
    }
    server.send(200, "application/json", "{\"status\":\"ok\",\"authenticated\":true}");
  } else {
    server.send(401, "application/json", "{\"status\":\"error\",\"message\":\"Passwort falsch\"}");
  }
}

void handlePurgeApi() {
  if (!isWebAuthenticated()) {
    server.send(401, "application/json", "{\"status\":\"error\",\"message\":\"Passwort erforderlich.\"}");
    return;
  }
  if (server.hasArg("interval")) {
    sysConfig.purge_interval_min = server.arg("interval").toInt();
  }
  if (server.hasArg("duration")) {
    sysConfig.purge_duration_sec = server.arg("duration").toInt();
  }
  saveConfiguration();
  lastPurgeTimestamp = millis();
  isPurgeActive = false;
  addAppLogEx(1, "[Config] Stoßlüften aktualisiert: Intervall=%d min, Dauer=%d sec.", sysConfig.purge_interval_min, sysConfig.purge_duration_sec);
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleOdometerApi() {
  if (!isWebAuthenticated()) {
    server.send(401, "application/json", "{\"status\":\"error\",\"message\":\"Nicht autorisiert\"}");
    return;
  }
  if (server.hasArg("meters")) {
    float newMeters = server.arg("meters").toFloat();
    if (newMeters < 0.0f) newMeters = 0.0f;
    servoTotalMeters = newMeters;
    saveOdometer(true);
    addAppLogEx(1, "[Odometer] Manually updated Servo total travel: %.2f m (%.3f km)", servoTotalMeters, servoTotalMeters / 1000.0f);
    server.send(200, "application/json", "{\"status\":\"ok\",\"meters\":" + String(servoTotalMeters, 2) + "}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Parameter 'meters' fehlt\"}");
  }
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
  doc["display_mode"] =
      isHeadless ? "Headless (Kein Display)" : (isTFTMode ? "TFT (ILI9488 / ILI9341)" : "e-Paper (Waveshare GxEPD2)");
  doc["mode"] = doc["display_mode"];
  doc["wifi_ssid"] = sysConfig.wifi_ssid; // Send SSID for client-side use

  bool authRequired = (strlen(sysConfig.web_password) > 0);
  bool isAuthenticated = isWebAuthenticated();
  doc["web_auth_required"] = authRequired;
  doc["web_authenticated"] = isAuthenticated;

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
      float vpd =
          calculateVPD(tempSensors[i].temperature, tempSensors[i].humidity);
      s["vpd"] = isnan(vpd) ? JsonVariant() : round(vpd * 100.0f) / 100.0f;
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
  potis["target_vpd"] = targetVpdVal;
  potis["raw_calculated_rh"] = (float)round(rawCalculatedRh * 10.0f) / 10.0f;
  potis["effective_target_rh"] = effectiveTargetRh;

  doc["poti_a"] = potiAVal;
  doc["poti_b"] = potiBVal;
  doc["poti_c"] = potiCVal;
  doc["target_vpd"] = targetVpdVal;
  doc["effective_target_rh"] = effectiveTargetRh;
  doc["rotor_position"] = rotorPosition;
  doc["servo_angle"] = currentServoAngle;
  doc["bypass_active"] = bypassModeActive;

  bool isSlaveConnected =
      (sysConfig.espnow_role == 2 && lastEspNowRxTime != 0 &&
       (millis() - lastEspNowRxTime <= 5000));
  doc["is_slave_connected"] = isSlaveConnected;
  doc["dry_strategy"] =
      isSlaveConnected ? remoteMasterDryStrategy : sysConfig.dry_strategy;
  doc["hygro_limit"] = sysConfig.hygro_limit;
  doc["vpd_auto_day"] = getVpdAutoCurrentDay();
  doc["purge_interval_min"] = sysConfig.purge_interval_min;
  doc["purge_duration_sec"] = sysConfig.purge_duration_sec;
  doc["purge_active"] = isPurgeActive;

  long purgeRemainingSec = 0;
  if (sysConfig.purge_interval_min > 0) {
    if (isPurgeActive) {
      long elapsed = (long)((millis() - purgeStartMs) / 1000UL);
      purgeRemainingSec = (long)sysConfig.purge_duration_sec - elapsed;
      if (purgeRemainingSec < 0) purgeRemainingSec = 0;
    } else {
      long intervalSec = (long)sysConfig.purge_interval_min * 60L;
      long elapsed = (long)((millis() - lastPurgeTimestamp) / 1000UL);
      purgeRemainingSec = intervalSec - elapsed;
      if (purgeRemainingSec < 0) purgeRemainingSec = 0;
    }
  }
  doc["purge_remaining_sec"] = purgeRemainingSec;

  doc["espnow_role"] = sysConfig.espnow_role;
  doc["espnow_channel"] = sysConfig.espnow_channel;
  doc["espnow_peer_mac"] = sysConfig.espnow_peer_mac;

  long lastSeenMs = -1;
  if (sysConfig.espnow_role == 1) {
    lastSeenMs = (lastEspNowTxSuccessTime == 0)
                     ? -1
                     : (long)(millis() - lastEspNowTxSuccessTime);
  } else if (sysConfig.espnow_role == 2) {
    lastSeenMs =
        (lastEspNowRxTime == 0) ? -1 : (long)(millis() - lastEspNowRxTime);
  }
  doc["espnow_last_seen_ms"] = lastSeenMs;
  doc["espnow_interval_ms"] = avgEspNowIntervalMs;
  bool isEspNowOnline = (sysConfig.espnow_role > 0) &&
                        (strlen(sysConfig.espnow_peer_mac) > 0) &&
                        (lastSeenMs != -1) && (lastSeenMs <= 5000);
  doc["espnow_pv_mismatch"] = isEspNowOnline && (remoteProtocolVersion > 0) &&
                              (remoteProtocolVersion != localProtocolVersion);
  doc["espnow_remote_pv"] = remoteProtocolVersion;
  doc["espnow_local_pv"] = localProtocolVersion;
  doc["espnow_pairing"] = isPairingActive;
  doc["espnow_failsafe_mode"] = sysConfig.espnow_failsafe_mode;
  doc["wifi_mac"] = WiFi.macAddress();
  doc["wifi_channel"] = WiFi.status() == WL_CONNECTED ? WiFi.channel() : 1;
  doc["rssi"] = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -100;
  doc["watchdog_reset_countdown"] = getWatchdogResetCountdown();
  doc["update_available"] = (cachedOnlineVersion > localFirmwareVersion);
  doc["online_version"] = cachedOnlineVersion;
  doc["fw_version"] = "1." + String(localFirmwareVersion);
  doc["loops_per_sec"] = loopsPerSecond;
  doc["free_heap"] = ESP.getFreeHeap();
  doc["servo_total_meters"] = (float)round(servoTotalMeters * 100.0f) / 100.0f;
  float lifetimePct = min(100.0f, (servoTotalMeters / 50000.0f) * 100.0f);
  doc["servo_lifetime_pct"] = (float)round(lifetimePct * 100.0f) / 100.0f;
  doc["max_alloc_heap"] = ESP.getMaxAllocHeap();
  doc["log_level"] = sysConfig.log_level;

  if (!authRequired || isAuthenticated) {
    JsonArray logsArr = doc["sys_logs"].to<JsonArray>();
    int logStart = (appLogCount < APP_LOG_BUFFER_SIZE) ? 0 : appLogHead;
    for (int i = 0; i < appLogCount; i++) {
      int idx = (logStart + i) % APP_LOG_BUFFER_SIZE;
      logsArr.add(appLogBuffer[idx].text);
    }

    JsonArray remoteLogsArr = doc["remote_sys_logs"].to<JsonArray>();
    int rLogStart = (remoteAppLogCount < APP_LOG_BUFFER_SIZE) ? 0 : remoteAppLogHead;
    for (int i = 0; i < remoteAppLogCount; i++) {
      int idx = (rLogStart + i) % APP_LOG_BUFFER_SIZE;
      remoteLogsArr.add(remoteAppLogBuffer[idx].text);
    }
  }

  String jsonResponse;
  serializeJson(doc, jsonResponse);
  server.sendHeader("Connection", "close");
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
  server.sendHeader("Connection", "close");
  server.send(200, "application/json", jsonResponse);
}

void handleSetLogLevel() {
  if (server.hasArg("level")) {
    int lvl = server.arg("level").toInt();
    if (lvl >= 1 && lvl <= 3) {
      sysConfig.log_level = lvl;
      saveConfiguration();
      addAppLogEx(1, "[Config] System Log Level changed to Level %d", lvl);
    }
  }
  server.send(200, "application/json", "{\"status\":\"ok\"}");
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
    const char* DASHBOARD_HTML_PART1 = R"rawhtml(
<!DOCTYPE html>
<html lang="de">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>)rawhtml";

    const char* DASHBOARD_HTML_PART2 = R"rawhtml(</title>
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
        .card-title { font-size: 13px; text-transform: uppercase; letter-spacing: 1px; color: #94a3b8; margin-bottom: 12px; font-weight: bold; border-bottom: 1px solid rgba(255,255,255,0.05); padding-bottom: 5px; display: flex; justify-content: space-between; align-items: center; position: relative; }
        .info-btn {
            display: inline-flex;
            align-items: center;
            justify-content: center;
            width: 15px;
            height: 15px;
            border-radius: 50%;
            background: rgba(56, 189, 248, 0.12);
            border: 1px solid rgba(56, 189, 248, 0.35);
            color: #38bdf8;
            font-size: 11px;
            font-family: serif;
            font-style: italic;
            font-weight: bold;
            cursor: pointer;
            user-select: none;
            line-height: 1;
            transition: all 0.2s ease;
            flex-shrink: 0;
            margin-left: auto;
        }
        .info-btn:hover, .info-btn.active {
            background: rgba(56, 189, 248, 0.3);
            border-color: #38bdf8;
            color: #ffffff;
            box-shadow: 0 0 8px rgba(56, 189, 248, 0.6);
        }
        .info-bubble {
            position: absolute;
            top: calc(100% + 6px);
            right: 0;
            width: 280px;
            max-width: 85vw;
            background: #090d16;
            border: 1px solid rgba(56, 189, 248, 0.6);
            border-radius: 12px;
            padding: 12px 14px;
            color: #e2e8f0;
            font-size: 12px;
            font-weight: normal;
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            text-transform: none;
            letter-spacing: normal;
            line-height: 1.5;
            box-shadow: 0 20px 45px -5px rgba(0, 0, 0, 0.95), 0 0 20px rgba(56, 189, 248, 0.3);
            z-index: 9999;
            pointer-events: auto;
            animation: info-fade-in 0.2s ease-out;
        }
        .info-bubble::before {
            content: '';
            position: absolute;
            top: -8.5px;
            right: 4px;
            width: 0;
            height: 0;
            border-left: 7px solid transparent;
            border-right: 7px solid transparent;
            border-bottom: 8.5px solid rgba(56, 189, 248, 0.6);
        }
        .info-bubble::after {
            content: '';
            position: absolute;
            top: -7px;
            right: 5px;
            width: 0;
            height: 0;
            border-left: 6px solid transparent;
            border-right: 6px solid transparent;
            border-bottom: 7px solid #090d16;
        }
        @keyframes info-fade-in {
            from { opacity: 0; transform: translateY(-4px); }
            to { opacity: 1; transform: translateY(0); }
        }
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
        @keyframes vpd-candle-pulse {
            0% { box-shadow: 0 0 3px rgba(56,189,248,0.4); border-color: #38bdf8; background-color: #38bdf8; }
            50% { box-shadow: 0 0 16px rgba(255,255,255,1), 0 0 8px rgba(56,189,248,1); border-color: #ffffff; background-color: #ffffff; }
            100% { box-shadow: 0 0 3px rgba(56,189,248,0.4); border-color: #38bdf8; background-color: #38bdf8; }
        }
        .vpd-candle-active { animation: vpd-candle-pulse 1.8s infinite ease-in-out; }
        @keyframes sand-stream-glow {
            0% { filter: drop-shadow(0 0 2.0px rgba(2, 132, 199, 0.75)); opacity: 0.90; }
            50% { filter: drop-shadow(0 0 3.6px rgba(14, 165, 233, 0.95)); opacity: 1.0; }
            100% { filter: drop-shadow(0 0 2.0px rgba(2, 132, 199, 0.75)); opacity: 0.90; }
        }
        @keyframes sand-stream-color {
            0% { stroke: #1ea2dc; stroke-width: 2.3px; }
            50% { stroke: #58c7f9; stroke-width: 2.5px; }
            100% { stroke: #1ea2dc; stroke-width: 2.3px; }
        }
        @keyframes sand-stream-glow-red {
            0% { filter: drop-shadow(0 0 2.0px rgba(185, 28, 28, 0.75)); opacity: 0.90; }
            50% { filter: drop-shadow(0 0 3.6px rgba(239, 68, 68, 0.95)); opacity: 1.0; }
            100% { filter: drop-shadow(0 0 2.0px rgba(185, 28, 28, 0.75)); opacity: 0.90; }
        }
        @keyframes sand-stream-color-red {
            0% { stroke: #e65c5c; stroke-width: 2.3px; }
            50% { stroke: #fa8282; stroke-width: 2.5px; }
            100% { stroke: #e65c5c; stroke-width: 2.3px; }
        }
        .sand-stream-flowing-blue {
            animation: sand-stream-glow 310ms infinite ease-in-out, sand-stream-color 190ms infinite ease-in-out;
        }
        .sand-stream-flowing-red {
            animation: sand-stream-glow-red 310ms infinite ease-in-out, sand-stream-color-red 190ms infinite ease-in-out;
        }
        .drum-picker-wrapper {
            position: relative;
            height: 84px;
            width: 100%;
            overflow: hidden;
            background: linear-gradient(180deg, #0e1526 0%, #172238 25%, #243452 50%, #172238 75%, #0e1526 100%);
            border-radius: 8px;
            border: 1px solid rgba(56, 189, 248, 0.35);
            box-shadow: inset 0 2px 6px rgba(0, 0, 0, 0.5), 0 4px 12px rgba(0, 0, 0, 0.35);
        }
        .drum-picker {
            height: 100%;
            overflow-y: scroll;
            scroll-snap-type: y mandatory;
            scrollbar-width: none;
            -ms-overflow-style: none;
            padding: 28px 0;
            box-sizing: border-box;
            cursor: grab;
        }
        .drum-picker:active { cursor: grabbing; }
        .drum-picker::-webkit-scrollbar { display: none; }
        .drum-item {
            height: 28px;
            display: flex;
            align-items: center;
            justify-content: center;
            scroll-snap-align: center;
            font-size: 11px;
            color: #cbd5e1;
            font-weight: 500;
            user-select: none;
            transition: all 0.15s ease;
            transform: scale(0.92, 0.65);
            opacity: 0.85;
        }
        .drum-item.top-neighbor, .drum-item.bottom-neighbor {
            transform: scale(0.92, 0.65);
            opacity: 0.85;
            color: #cbd5e1;
        }
        .drum-item.active {
            color: #38bdf8;
            font-weight: 800;
            font-size: 13.5px;
            transform: scale(1.0, 1.0);
            opacity: 1.0;
            text-shadow: 0 0 10px rgba(56, 189, 248, 0.7);
        }
        .drum-overlay {
            position: absolute;
            top: 26px;
            left: 0;
            right: 0;
            height: 32px;
            border-top: 1.5px solid #38bdf8;
            border-bottom: 1.5px solid #38bdf8;
            background: linear-gradient(180deg, rgba(56, 189, 248, 0.28) 0%, rgba(56, 189, 248, 0.10) 50%, rgba(56, 189, 248, 0.28) 100%);
            pointer-events: none;
            border-radius: 3px;
            box-shadow: 0 0 24px rgba(56, 189, 248, 0.42), inset 0 0 10px rgba(56, 189, 248, 0.25);
        }
        /* Smart Live-Advisor & Ticker Widget */
        .advisor-card {
            background: rgba(30, 41, 59, 0.55);
            backdrop-filter: blur(12px);
            border: 1px solid rgba(56, 189, 248, 0.25);
            border-radius: 16px;
            padding: 12px 16px;
            margin-bottom: 18px;
            box-shadow: 0 10px 25px -5px rgba(0, 0, 0, 0.5), 0 0 15px rgba(56, 189, 248, 0.08);
            position: relative;
            z-index: 100;
        }
        .advisor-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 8px;
            border-bottom: 1px solid rgba(255, 255, 255, 0.06);
            padding-bottom: 6px;
            position: relative;
        }
        .advisor-title {
            display: flex;
            align-items: center;
            gap: 8px;
            font-size: 13px;
            font-weight: 700;
            text-transform: uppercase;
            letter-spacing: 1px;
            color: #38bdf8;
        }
        .advisor-icon {
            font-size: 15px;
            animation: pulse-advisor 2.5s infinite ease-in-out;
        }
        @keyframes pulse-advisor {
            0%, 100% { transform: scale(1); filter: drop-shadow(0 0 2px rgba(56,189,248,0.5)); }
            50% { transform: scale(1.15); filter: drop-shadow(0 0 8px rgba(56,189,248,0.9)); }
        }
        .advisor-controls {
            display: flex;
            align-items: center;
            gap: 8px;
        }
        .advisor-nav-btn {
            background: rgba(15, 23, 42, 0.8);
            border: 1px solid rgba(56, 189, 248, 0.3);
            color: #38bdf8;
            border-radius: 6px;
            padding: 2px 8px;
            font-size: 11px;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.2s ease;
            user-select: none;
        }
        .advisor-nav-btn:hover {
            background: rgba(56, 189, 248, 0.25);
            color: #ffffff;
            border-color: #38bdf8;
        }
        .advisor-counter {
            font-family: monospace;
            font-size: 11px;
            color: #94a3b8;
            min-width: 45px;
            text-align: center;
            user-select: none;
        }
        .advisor-ticker-box {
            position: relative;
            overflow: hidden;
            width: 100%;
            height: 32px;
            display: flex;
            align-items: center;
            background: rgba(15, 23, 42, 0.65);
            border-radius: 8px;
            border: 1px solid rgba(255, 255, 255, 0.05);
            padding: 0 10px;
            cursor: grab;
            user-select: none;
            touch-action: pan-y;
        }
        .advisor-ticker-box:active {
            cursor: grabbing;
        }
        .advisor-ticker-track {
            display: inline-flex;
            align-items: center;
            white-space: nowrap;
            will-change: transform;
            position: absolute;
            left: 10px;
        }
        .advisor-badge {
            display: inline-flex;
            align-items: center;
            gap: 4px;
            padding: 2px 7px;
            border-radius: 12px;
            font-size: 10.5px;
            font-weight: 700;
            letter-spacing: 0.5px;
            text-transform: uppercase;
            margin-right: 8px;
            flex-shrink: 0;
            line-height: 1.2;
        }
        .badge-optimal {
            background: rgba(34, 197, 94, 0.18);
            border: 1px solid rgba(34, 197, 94, 0.5);
            color: #4ade80;
            box-shadow: 0 0 8px rgba(34, 197, 94, 0.3);
        }
        .badge-tip {
            background: rgba(234, 179, 8, 0.18);
            border: 1px solid rgba(234, 179, 8, 0.5);
            color: #facc15;
            box-shadow: 0 0 8px rgba(234, 179, 8, 0.3);
        }
        .badge-alert {
            background: rgba(239, 68, 68, 0.2);
            border: 1px solid rgba(239, 68, 68, 0.6);
            color: #f87171;
            box-shadow: 0 0 10px rgba(239, 68, 68, 0.4);
            animation: badge-alert-pulse 1.2s infinite ease-in-out;
        }
        @keyframes badge-alert-pulse {
            0%, 100% { transform: scale(1); }
            50% { transform: scale(1.04); }
        }
        .badge-weather {
            background: rgba(56, 189, 248, 0.18);
            border: 1px solid rgba(56, 189, 248, 0.5);
            color: #38bdf8;
            box-shadow: 0 0 8px rgba(56, 189, 248, 0.3);
        }
        .badge-system {
            background: rgba(168, 85, 247, 0.18);
            border: 1px solid rgba(168, 85, 247, 0.5);
            color: #c084fc;
            box-shadow: 0 0 8px rgba(168, 85, 247, 0.3);
        }
        .advisor-time {
            font-family: monospace;
            font-size: 11px;
            color: #64748b;
            margin-right: 8px;
            flex-shrink: 0;
        }
        .advisor-msg-text {
            font-size: 12.5px;
            color: #f1f5f9;
            font-weight: 500;
            display: inline;
        }
        .advisor-popup-bubble {
            position: absolute;
            top: calc(100% + 8px);
            left: 0;
            right: 0;
            background: #090d16;
            border: 1px solid rgba(56, 189, 248, 0.6);
            border-radius: 14px;
            padding: 14px 16px;
            box-shadow: 0 25px 60px -5px rgba(0, 0, 0, 0.98), 0 0 25px rgba(56, 189, 248, 0.35);
            z-index: 9999;
            animation: info-fade-in 0.2s ease-out;
        }
        .advisor-popup-bubble::before {
            content: '';
            position: absolute;
            top: -8.5px;
            left: 35px;
            width: 0;
            height: 0;
            border-left: 7px solid transparent;
            border-right: 7px solid transparent;
            border-bottom: 8.5px solid rgba(56, 189, 248, 0.6);
        }
        .advisor-popup-bubble::after {
            content: '';
            position: absolute;
            top: -7px;
            left: 36px;
            width: 0;
            height: 0;
            border-left: 6px solid transparent;
            border-right: 6px solid transparent;
            border-bottom: 7px solid #090d16;
        }
        .advisor-popup-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 10px;
            padding-bottom: 6px;
            border-bottom: 1px solid rgba(255, 255, 255, 0.08);
        }
        .advisor-popup-close {
            background: rgba(255, 255, 255, 0.08);
            border: 1px solid rgba(255, 255, 255, 0.15);
            color: #cbd5e1;
            border-radius: 6px;
            width: 22px;
            height: 22px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 12px;
            cursor: pointer;
            transition: all 0.2s ease;
        }
        .advisor-popup-close:hover {
            background: rgba(239, 68, 68, 0.3);
            border-color: #ef4444;
            color: #ffffff;
        }
        .advisor-popup-content {
            font-size: 13px;
            color: #f1f5f9;
            line-height: 1.55;
            margin-bottom: 12px;
            word-break: break-word;
        }
        .advisor-popup-footer {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding-top: 8px;
            border-top: 1px solid rgba(255, 255, 255, 0.08);
        }
    </style>
</head>
<body>
    <div class="container">
        <h1 id="device-title">IDRY-26 Loading...</h1>
        
        <!-- Smart Live-Advisor & Grow-Heuristic Ticker Widget -->
        <div class="advisor-card" id="advisor-widget">
            <div class="advisor-header">
                <div class="advisor-title">
                    <span class="advisor-icon">🧠</span>
                    <span>Grow Advisor &amp; Live Ticker</span>
                </div>
                <div class="advisor-controls">
                    <button type="button" id="advisor-prev-btn" class="advisor-nav-btn" onclick="prevAdvisorMsg()" title="Vorherige ältere Nachricht">◀</button>
                    <span id="advisor-counter" class="advisor-counter">1 / 1</span>
                    <button type="button" id="advisor-next-btn" class="advisor-nav-btn" onclick="nextAdvisorMsg()" title="Nächste neuere Nachricht">▶</button>
                    <span class="info-btn" onclick="toggleInfo(event, 20)" onmouseenter="showInfo(this, 20)" onmouseleave="hideInfo(this)">i</span>
                </div>
            </div>
            <div class="advisor-ticker-box" id="advisor-ticker-box" title="Klicken für Volltext-Ansicht & Historie" onmouseenter="pauseAdvisorTicker()" onmouseleave="resumeAdvisorTicker()" onpointerdown="startAdvisorDrag(event)">
                <div class="advisor-ticker-track" id="advisor-ticker-track">
                    <span class="advisor-badge badge-optimal">🟢 SYSTEMBEREIT</span>
                    <span class="advisor-time">[--:--:--]</span>
                    <span class="advisor-msg-text">Smart Live Advisor Engine bereit. Analysiere thermodynamische Klimadaten...</span>
                </div>
            </div>
            <!-- Interactive Full-Text Speech Bubble Modal -->
            <div id="advisor-popup-bubble" class="advisor-popup-bubble" style="display:none;" onclick="event.stopPropagation()">
                <div class="advisor-popup-header">
                    <div style="display: flex; align-items: center; gap: 8px;">
                        <span class="advisor-icon">🧠</span>
                        <span style="font-weight: bold; color: #38bdf8; font-size: 12.5px; letter-spacing: 0.5px;">ADVISOR VOLLTEXT</span>
                    </div>
                    <button type="button" class="advisor-popup-close" onclick="closeAdvisorPopup(event)" title="Schließen">✕</button>
                </div>
                <div id="advisor-popup-content" class="advisor-popup-content"></div>
                <div class="advisor-popup-footer">
                    <button type="button" id="advisor-popup-prev-btn" class="advisor-nav-btn" onclick="prevAdvisorPopup(event)" title="Vorherige ältere Nachricht">◀ Älter</button>
                    <span id="advisor-popup-counter" class="advisor-counter">1 / 1</span>
                    <button type="button" id="advisor-popup-next-btn" class="advisor-nav-btn" onclick="nextAdvisorPopup(event)" title="Nächste neuere Nachricht">Neuer ▶</button>
                </div>
            </div>
        </div>

        <div class="grid">
            <div class="card">
                <div id="strat-section" style="margin-bottom: 14px;">
                    <div class="card-title"><span>Dry Strategy</span><span class="info-btn" onclick="toggleInfo(event, 0)" onmouseenter="showInfo(this, 0)" onmouseleave="hideInfo(this)">i</span></div>
                    <div style="display: flex; gap: 6px;">
                        <button id="strat-btn-6060" onclick="setDryStrategy(0, currentHygroLimit)" style="flex: 1; padding: 10px 0; background: #22c55e; border: 1px solid rgba(255,255,255,0.2); color: white; font-weight: bold; border-radius: 8px; cursor: pointer; transition: all 0.2s;">60/60</button>
                        <button id="strat-btn-vpd" onclick="setDryStrategy(1, currentHygroLimit)" style="flex: 1; padding: 10px 0; background: #1e293b; border: 1px solid rgba(255,255,255,0.15); color: #94a3b8; font-weight: bold; border-radius: 8px; cursor: pointer; transition: all 0.2s;">VPD</button>
                        <button id="strat-btn-vpd-auto" onclick="setDryStrategy(2, currentHygroLimit)" style="flex: 1; padding: 10px 0; background: #1e293b; border: 1px solid rgba(255,255,255,0.15); color: #94a3b8; font-weight: bold; border-radius: 8px; cursor: pointer; transition: all 0.2s;">VPD AU</button>
                        <div id="strat-btn-remote" style="display: none; flex: 1; padding: 10px 0; background: rgba(15,23,42,0.6); border: 1px solid rgba(255,255,255,0.1); border-radius: 8px; text-align: center; font-size: 13px; font-weight: bold; user-select: none;">REMOTE 60/60</div>
                    </div>
                </div>
                <div id="vpd-auto-box" style="display: none; background: rgba(15,23,42,0.6); padding: 10px 12px; border-radius: 12px; border: 1px solid rgba(56,189,248,0.25); margin-bottom: 14px;">
                    <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; position: relative;">
                        <span style="font-size: 11px; text-transform: uppercase; letter-spacing: 0.5px; color: #38bdf8; font-weight: bold;">VPD AUTO</span>
                        <div style="display: flex; align-items: center; gap: 8px;">
                            <select id="vpd-auto-day-select" onchange="onVpdDaySelectChange(this.value)" style="background: rgba(15,23,42,0.9); color: #38bdf8; font-size: 11px; font-weight: bold; border: 1px solid rgba(56,189,248,0.4); border-radius: 6px; padding: 2px 6px; outline: none; cursor: pointer;">
                                <option value="1">Tag 1</option>
                                <option value="2">Tag 2</option>
                                <option value="3">Tag 3</option>
                                <option value="4">Tag 4</option>
                                <option value="5">Tag 5</option>
                                <option value="6">Tag 6</option>
                                <option value="7">Tag 7</option>
                                <option value="8">Tag 8</option>
                                <option value="9">Tag 9</option>
                                <option value="10">Tag 10</option>
                                <option value="11">Tag 11 (~Curing)</option>
                                <option value="12">Tag 12 (~Curing)</option>
                                <option value="13">Tag 13 (~Curing)</option>
                                <option value="14">Tag 14 (~Curing)</option>
                            </select>
                            <span class="info-btn" onclick="toggleInfo(event, 1)" onmouseenter="showInfo(this, 1)" onmouseleave="hideInfo(this)">i</span>
                        </div>
                    </div>
                    <div style="position: relative; width: 100%; height: 100px; background: rgba(15,23,42,0.8); border-radius: 8px; border: 1px solid rgba(255,255,255,0.08); overflow: hidden; margin-bottom: 6px;">
                        <canvas id="vpd-heatmap-canvas" onpointerdown="handleHeatmapPointer(event)" onpointermove="if(event.buttons) handleHeatmapPointer(event)" onpointerup="stopHeatmapInspection()" onpointerleave="stopHeatmapInspection()" style="width: 100%; height: 100px; display: block; cursor: crosshair; touch-action: none;"></canvas>
                    </div>
                    <div id="vpd-auto-timeline" style="display: flex; gap: 3px; align-items: flex-end; height: 28px; padding: 3px; background: rgba(15,23,42,0.8); border-radius: 8px; border: 1px solid rgba(255,255,255,0.05);">
                        <!-- 14 sleek vertical candle bars without text -->
                    </div>
                </div>
                <div id="hygro-limit-box" style="display: none; background: rgba(15,23,42,0.5); padding: 10px 12px; border-radius: 8px; border: 1px solid rgba(255,255,255,0.08); margin-bottom: 14px;">
                    <div style="font-size: 11px; color: #94a3b8; font-weight: 600; margin-bottom: 6px; display: flex; justify-content: space-between; align-items: center; position: relative;">
                        <span>Hygro Limit (Schimmelschutz):</span>
                        <span class="info-btn" onclick="toggleInfo(event, 2)" onmouseenter="showInfo(this, 2)" onmouseleave="hideInfo(this)">i</span>
                    </div>
                    <div style="display: flex; gap: 14px; font-size: 13px; font-weight: bold; margin-bottom: 8px;">
                        <label style="cursor: pointer; color: #22c55e; display: flex; align-items: center; gap: 4px;">
                            <input type="radio" name="hygro_limit_radio" value="70" onchange="setDryStrategy(currentDryStrategy, 70)" id="hl-70"> 70%
                        </label>
                        <label style="cursor: pointer; color: #f97316; display: flex; align-items: center; gap: 4px;">
                            <input type="radio" name="hygro_limit_radio" value="75" onchange="setDryStrategy(currentDryStrategy, 75)" id="hl-75"> 75%
                        </label>
                        <label style="cursor: pointer; color: #f87171; display: flex; align-items: center; gap: 4px;">
                            <input type="radio" name="hygro_limit_radio" value="80" onchange="setDryStrategy(currentDryStrategy, 80)" id="hl-80"> 80%
                        </label>
                    </div>
                    <div style="font-size: 11px; color: #94a3b8; border-top: 1px solid rgba(255,255,255,0.08); padding-top: 6px;">
                        <div style="display: flex; justify-content: space-between; align-items: center;">
                            <span>RH calculated soll:</span>
                            <span id="calc-soll-rh" style="font-weight: bold; color: #38bdf8; font-size: 12px;">--</span>
                        </div>
                        <div id="calc-limit-notice" style="display: none; text-align: right; font-weight: bold; color: #f87171; font-size: 11px; margin-top: 3px;"></div>
                    </div>
                </div>
                <div class="card-title" style="margin-top: 14px;"><span>Potentiometer</span><span class="info-btn" onclick="toggleInfo(event, 3)" onmouseenter="showInfo(this, 3)" onmouseleave="hideInfo(this)">i</span></div>
                <div class="value-row"><span id="poti-a-label">Sollwert Feuchte (A):</span><span class="val" id="poti-a">--</span></div>
                <div class="value-row"><span>Gain Faktor (B):</span><span class="val" id="poti-b">--</span></div>
                <div class="value-row"><span>Rotor-Offset (C):</span><span class="val" id="poti-c">--</span></div>
            </div>
            <div class="card">
                <div class="card-title"><span>Rotor & Servo</span><span class="info-btn" onclick="toggleInfo(event, 4)" onmouseenter="showInfo(this, 4)" onmouseleave="hideInfo(this)">i</span></div>
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
                <div id="purge-section" style="margin-top: 15px; border-top: 1px solid rgba(255,255,255,0.08); padding-top: 12px;">
                    <div style="font-size: 11px; font-weight: bold; text-transform: uppercase; color: #94a3b8; letter-spacing: 0.5px; margin-bottom: 8px; display: flex; justify-content: space-between; align-items: center; position: relative;">
                        <span>⏳ Stoßlüftungs-Timer</span>
                        <div style="display: flex; align-items: center; gap: 8px;">
                            <span id="purge-badge" style="font-size: 10.5px; font-family: monospace; font-weight: bold; padding: 2px 6px; border-radius: 4px; background: rgba(56, 189, 248, 0.15); color: #38bdf8; border: 1px solid rgba(56, 189, 248, 0.3);">Aus</span>
                            <span class="info-btn" onclick="toggleInfo(event, 5)" onmouseenter="showInfo(this, 5)" onmouseleave="hideInfo(this)">i</span>
                        </div>
                    </div>
                    
                    <!-- Enlarged & Centered Sanduhr SVG (+40% size, 95x142px) -->
                    <div id="hourglass-container" style="width: 95px; height: 142px; margin: 12px auto 16px auto; position: relative;">
                        <svg viewBox="0 0 60 100" width="100%" height="100%">
                            <path d="M 10 5 L 50 5 L 33 48 C 31 50, 31 50, 33 52 L 50 95 L 10 95 L 27 52 C 29 50, 29 50, 27 48 Z" fill="none" stroke="#64748b" stroke-width="3" stroke-linejoin="round"/>
                            <polygon id="sand-top" points="14,10 46,10 30,46" fill="#38bdf8" style="transform-origin: 30px 46px; transition: transform 0.5s ease;"/>
                            <line id="sand-stream" x1="30" y1="50" x2="30" y2="88" stroke="#38bdf8" stroke-width="2.5" stroke-linecap="round" style="opacity: 0; transition: opacity 0.3s;"/>
                            <polygon id="sand-bottom" points="30,58 46,92 14,92" fill="#38bdf8" style="transform-origin: 30px 92px; transition: transform 0.5s ease;"/>
                        </svg>
                    </div>

                    <!-- Dual 3D Selection Wheels (Drum Pickers) Below Sanduhr -->
                    <div style="display: flex; gap: 10px; margin-top: 6px;">
                        <div style="flex: 1;">
                            <label style="font-size: 10px; color: #94a3b8; display: block; margin-bottom: 4px; text-align: center; font-weight: 600;">Intervall:</label>
                            <div class="drum-picker-wrapper">
                                <div class="drum-picker" id="wheel-interval">
                                    <div class="drum-item" data-val="0">Aus</div>
                                    <div class="drum-item" data-val="10">10 min</div>
                                    <div class="drum-item" data-val="20">20 min</div>
                                    <div class="drum-item" data-val="30">30 min</div>
                                    <div class="drum-item" data-val="45">45 min</div>
                                    <div class="drum-item" data-val="60">1 h</div>
                                    <div class="drum-item" data-val="120">2 h</div>
                                    <div class="drum-item" data-val="180">3 h</div>
                                    <div class="drum-item" data-val="240">4 h</div>
                                    <div class="drum-item" data-val="300">5 h</div>
                                    <div class="drum-item" data-val="600">10 h</div>
                                    <div class="drum-item" data-val="720">12 h</div>
                                    <div class="drum-item" data-val="1440">24 h</div>
                                </div>
                                <div class="drum-overlay"></div>
                            </div>
                        </div>
                        <div style="flex: 1;">
                            <label style="font-size: 10px; color: #94a3b8; display: block; margin-bottom: 4px; text-align: center; font-weight: 600;">Dauer:</label>
                            <div class="drum-picker-wrapper">
                                <div class="drum-picker" id="wheel-duration">
                                    <div class="drum-item" data-val="10">10 sec</div>
                                    <div class="drum-item" data-val="20">20 sec</div>
                                    <div class="drum-item" data-val="30">30 sec</div>
                                    <div class="drum-item" data-val="45">45 sec</div>
                                    <div class="drum-item" data-val="60">1 min</div>
                                    <div class="drum-item" data-val="90">1.5 min</div>
                                    <div class="drum-item" data-val="120">2 min</div>
                                    <div class="drum-item" data-val="180">3 min</div>
                                    <div class="drum-item" data-val="240">4 min</div>
                                    <div class="drum-item" data-val="300">5 min</div>
                                    <div class="drum-item" data-val="450">7.5 min</div>
                                    <div class="drum-item" data-val="600">10 min</div>
                                </div>
                                <div class="drum-overlay"></div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            <div class="card" id="sensor-card-0" style="display:none;">
                <div class="card-title"><span id="sensor-title-0">Sensor 1</span><span class="info-btn" onclick="toggleInfo(event, 9)" onmouseenter="showInfo(this, 9)" onmouseleave="hideInfo(this)">i</span></div>
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
                <div class="card-title"><span id="sensor-title-1">Sensor 2</span><span class="info-btn" onclick="toggleInfo(event, 10)" onmouseenter="showInfo(this, 10)" onmouseleave="hideInfo(this)">i</span></div>
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
                <div class="card-title"><span id="light-title-0">TSL2561 (1)</span><span class="info-btn" onclick="toggleInfo(event, 11)" onmouseenter="showInfo(this, 11)" onmouseleave="hideInfo(this)">i</span></div>
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
                <div class="card-title"><span id="light-title-1">TSL2561 (2)</span><span class="info-btn" onclick="toggleInfo(event, 12)" onmouseenter="showInfo(this, 12)" onmouseleave="hideInfo(this)">i</span></div>
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
            <div class="card" id="vpd-card" style="display:none; grid-column: 1 / -1;">
                <div class="card-title"><span>VPD (Sättigungsdefizit)</span><span class="info-btn" onclick="toggleInfo(event, 8)" onmouseenter="showInfo(this, 8)" onmouseleave="hideInfo(this)">i</span></div>
                <div style="display: flex; gap: 15px; flex-wrap: wrap;">
                    <div id="vpd-row-0" style="display:none; flex: 1; min-width: 240px;">
                        <div class="value-row"><span>VPD Innen (BME280):</span><span class="val" id="vpd-val-0">--</span></div>
                        <details open class="hist-toggle" id="details-vpd-0" ontoggle="renderAllCharts()">
                            <summary>60m Verlauf (VPD Innen)</summary>
                            <div class="spark-box" onclick="openChartZoom('vpd_0', 'VPD Innen (BME280)')">
                                <canvas id="cv-vpd-0"></canvas>
                            </div>
                        </details>
                    </div>
                    <div id="vpd-row-1" style="display:none; flex: 1; min-width: 240px;">
                        <div class="value-row"><span>VPD Außen (SHT31):</span><span class="val" id="vpd-val-1">--</span></div>
                        <details open class="hist-toggle" id="details-vpd-1" ontoggle="renderAllCharts()">
                            <summary>60m Verlauf (VPD Außen)</summary>
                            <div class="spark-box" onclick="openChartZoom('vpd_1', 'VPD Außen (SHT31)')">
                                <canvas id="cv-vpd-1"></canvas>
                            </div>
                        </details>
                    </div>
                </div>
            </div>
            <div class="card" id="espnow-card" style="display:none;">
                <div class="card-title"><span>ESPNOW</span><span class="info-btn" onclick="toggleInfo(event, 6)" onmouseenter="showInfo(this, 6)" onmouseleave="hideInfo(this)">i</span></div>
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
                <div class="card-title"><span id="mqtt-title">MQTT Dashboard</span><span class="info-btn" onclick="toggleInfo(event, 7)" onmouseenter="showInfo(this, 7)" onmouseleave="hideInfo(this)">i</span></div>
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

        <!-- Protected UI Login Card -->
        <div id="login-card" class="card" style="display:none; border:1px solid rgba(129, 140, 248, 0.4); background:rgba(30, 41, 59, 0.7); text-align:center; padding:22px; margin-bottom:20px; box-shadow: 0 10px 25px -5px rgba(0,0,0,0.5);">
            <div style="font-size:28px; margin-bottom:6px;">🔒</div>
            <div style="font-size:15px; font-weight:bold; color:#818cf8; margin-bottom:6px;">Webinterface geschützt</div>
            <p style="font-size:12.5px; color:#cbd5e1; margin-bottom:16px;">Für erweiterte Log-Konsolen &amp; Einstellungen bitte Anmelden:</p>
            <div style="display:flex; gap:10px; justify-content:center; max-width:380px; margin:0 auto;">
                <input type="password" id="login-pass-input" placeholder="Passwort eingeben..." onkeypress="if(event.key==='Enter') performUiLogin()" style="flex:1; padding:9px 12px; background:rgba(15,23,42,0.8); border:1px solid rgba(255,255,255,0.2); border-radius:8px; color:white; font-size:13px; outline:none;">
                <button type="button" onclick="performUiLogin()" style="background:linear-gradient(135deg, #4f46e5 0%, #3b82f6 100%); border:none; color:white; padding:9px 18px; border-radius:8px; font-weight:bold; cursor:pointer; font-size:13px; box-shadow:0 4px 12px rgba(79,70,229,0.3); transition:all 0.2s;">Anmelden</button>
            </div>
            <div id="login-err-msg" style="display:none; color:#f87171; font-size:12px; margin-top:10px; font-weight:bold;">Passwort falsch!</div>
        </div>

        <div class="card">
            <div class="card-title"><span>System Status</span><span class="info-btn" onclick="toggleInfo(event, 13)" onmouseenter="showInfo(this, 13)" onmouseleave="hideInfo(this)">i</span></div>
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
            <details open class="hist-toggle" id="details-rssi" ontoggle="renderAllCharts()">
                <summary>4h Verlauf (WLAN RSSI Signal)</summary>
                <div class="spark-box" onclick="openChartZoom('rssi', 'WLAN Signalstärke (RSSI)')">
                    <canvas id="cv-rssi"></canvas>
                </div>
            </details>
            <details open class="hist-toggle" id="details-logs-local" style="margin-top: 12px; border-top: 1px solid rgba(255,255,255,0.08); padding-top: 8px;">
                <summary style="font-size: 11px; color: #38bdf8; cursor: pointer; user-select: none; font-weight: bold; outline: none; display: flex; justify-content: space-between; align-items: center;">
                    <span style="display: inline-flex; align-items: center; gap: 8px;">
                        <span id="label-log-local">▼ Local Terminal Console</span>
                        <button type="button" title="Local Log-Historie als TXT herunterladen" onclick="event.stopPropagation(); downloadLogHistory(webLogHistoryLocal, 'Local_Console');" style="background: rgba(56, 189, 248, 0.15); border: 1px solid rgba(56, 189, 248, 0.35); border-radius: 4px; color: #38bdf8; cursor: pointer; padding: 1px 6px; font-size: 11px; display: inline-flex; align-items: center; gap: 3px; transition: all 0.2s;" onmouseover="this.style.background='rgba(56, 189, 248, 0.35)'" onmouseout="this.style.background='rgba(56, 189, 248, 0.15)'">💾</button>
                    </span>
                    <div style="display: flex; gap: 8px; font-family: monospace; font-size: 10px; background: rgba(0,0,0,0.25); padding: 2px 6px; border-radius: 6px;" onclick="event.stopPropagation();">
                        <span style="color: #94a3b8; font-size: 9.5px; margin-right: 2px;">Filter:</span>
                        <label style="cursor: pointer; color: #38bdf8;"><input type="radio" name="loglvl_loc" value="1" onclick="setLocalFilter(1)" id="lvl-loc-1"> L1</label>
                        <label style="cursor: pointer; color: #eab308;"><input type="radio" name="loglvl_loc" value="2" onclick="setLocalFilter(2)" id="lvl-loc-2"> L2</label>
                        <label style="cursor: pointer; color: #a855f7;"><input type="radio" name="loglvl_loc" value="3" onclick="setLocalFilter(3)" id="lvl-loc-3" checked> L3</label>
                    </div>
                </summary>
                <div id="web-log-console" style="margin-top: 8px; background: #090d16; font-family: monospace; font-size: 10px; color: #4ade80; max-height: 150px; overflow-y: auto; padding: 8px 10px; border-radius: 8px; border: 1px solid rgba(56, 189, 248, 0.4); line-height: 1.45; white-space: pre-wrap; word-break: break-all;">
[00:00:00] Initializing Local System Console...
                </div>
            </details>
            <details open class="hist-toggle" id="details-logs-remote" style="margin-top: 10px; border-top: 1px solid rgba(255,255,255,0.08); padding-top: 8px;">
                <summary style="font-size: 11px; color: #c084fc; cursor: pointer; user-select: none; font-weight: bold; outline: none; display: flex; justify-content: space-between; align-items: center;">
                    <span style="display: inline-flex; align-items: center; gap: 8px;">
                        <span id="label-log-remote">▼ Remote Peer Terminal Console [ESP-NOW]</span>
                        <button type="button" title="Remote Log-Historie als TXT herunterladen" onclick="event.stopPropagation(); downloadLogHistory(webLogHistoryRemote, 'Remote_Console');" style="background: rgba(192, 132, 252, 0.15); border: 1px solid rgba(192, 132, 252, 0.35); border-radius: 4px; color: #c084fc; cursor: pointer; padding: 1px 6px; font-size: 11px; display: inline-flex; align-items: center; gap: 3px; transition: all 0.2s;" onmouseover="this.style.background='rgba(192, 132, 252, 0.35)'" onmouseout="this.style.background='rgba(192, 132, 252, 0.15)'">💾</button>
                    </span>
                    <div style="display: flex; gap: 8px; font-family: monospace; font-size: 10px; background: rgba(0,0,0,0.25); padding: 2px 6px; border-radius: 6px;" onclick="event.stopPropagation();">
                        <span style="color: #94a3b8; font-size: 9.5px; margin-right: 2px;">Filter:</span>
                        <label style="cursor: pointer; color: #38bdf8;"><input type="radio" name="loglvl_rem" value="1" onclick="setRemoteFilter(1)" id="lvl-rem-1"> L1</label>
                        <label style="cursor: pointer; color: #eab308;"><input type="radio" name="loglvl_rem" value="2" onclick="setRemoteFilter(2)" id="lvl-rem-2"> L2</label>
                        <label style="cursor: pointer; color: #a855f7;"><input type="radio" name="loglvl_rem" value="3" onclick="setRemoteFilter(3)" id="lvl-rem-3" checked> L3</label>
                    </div>
                </summary>
                <div id="web-log-console-remote" style="margin-top: 8px; background: #0c0916; font-family: monospace; font-size: 10px; color: #c084fc; max-height: 150px; overflow-y: auto; padding: 8px 10px; border-radius: 8px; border: 1px solid rgba(168, 85, 247, 0.5); line-height: 1.45; white-space: pre-wrap; word-break: break-all;">
[00:00:00] Waiting for Remote Peer ESP-NOW Log Stream...
                </div>
            </details>
        </div>
        <div class="footer" style="display: flex; justify-content: space-between; align-items: center; margin-top: 25px; padding-top: 15px; border-top: 1px solid rgba(255,255,255,0.05);">
            <span id="footer-text">iDRY26 - (bench: <span id="footer-bench" style="font-family: monospace; color: #38bdf8; font-weight: bold;">--</span> loops/s | heap: <span id="footer-heap" style="font-family: monospace; color: #38bdf8; font-weight: bold;">--</span> KB | alloc: <span id="footer-alloc" style="font-family: monospace; color: #38bdf8; font-weight: bold;">--</span> KB)</span>
            <a href="/settings" id="footer-settings-link" style="color: #818cf8; text-decoration: none; display: inline-flex; align-items: center; gap: 5px; font-weight: 600; padding: 6px 12px; background: rgba(129, 140, 248, 0.1); border-radius: 8px; border: 1px solid rgba(129, 140, 248, 0.2); transition: all 0.2s;">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="3"></circle><path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 1 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-4 0v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 1 1-2.83-2.83l-.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1 0-4h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 1 1 2.83-2.83l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 4 0v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 1 1 2.83 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1z"></path></svg>
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
            <div id="modal-tooltip" style="font-family:monospace; font-size:13px; color:#38bdf8; margin-top:12px; text-align:center; min-height:40px; display:flex; flex-direction:column; align-items:center; justify-content:center;">Tippe oder fahre über eine Kerze für Details...</div>
            <button onclick="closeChartModal()" style="margin-top:18px; width:100%; padding:12px; border-radius:8px; border:none; background:#3b82f6; color:white; font-weight:bold; cursor:pointer; font-size:14px;">Schließen</button>
        </div>
    </div>

    <div id="vpd-day-modal" style="display:none; position:fixed; top:0; left:0; width:100vw; height:100vh; background:rgba(15,23,42,0.88); backdrop-filter:blur(10px); z-index:1000; align-items:center; justify-content:center; padding:20px;">
        <div style="background:#1e293b; border:1px solid rgba(56,189,248,0.3); border-radius:16px; padding:24px; max-width:420px; width:100%; box-shadow:0 25px 50px -12px rgba(0,0,0,0.8); text-align:center;">
            <h3 id="vpd-day-modal-title" style="font-size:16px; color:#38bdf8; margin-bottom:10px;">Tag X aktivieren?</h3>
            <p id="vpd-day-modal-desc" style="font-size:13px; color:#cbd5e1; margin-bottom:20px; line-height:1.5;">Möchtest du den Trocknungs-Fortschritt manuell auf <b>Tag X</b> umstellen?</p>
            <div style="font-size:11px; color:#94a3b8; margin-bottom:15px; font-weight:bold;">Zum Bestätigen Button 2 Sekunden gedrückt halten:</div>
            <div style="display:flex; gap:10px;">
                <button onclick="closeVpdDayModal()" style="flex:1; padding:12px; background:#334155; border:none; color:#94a3b8; font-weight:bold; border-radius:8px; cursor:pointer;">Abbrechen</button>
                <button id="btn-confirm-hold" style="flex:1.5; padding:12px; background:#ef4444; border:none; color:white; font-weight:bold; border-radius:8px; cursor:pointer; position:relative; overflow:hidden; user-select:none;">
                    <span id="btn-confirm-text" style="position:relative; z-index:2;">Gedrückt halten...</span>
                    <div id="btn-confirm-progress" style="position:absolute; top:0; left:0; height:100%; width:0%; background:#22c55e; transition: width 0.05s linear; z-index:1;"></div>
                </button>
            </div>
        </div>
    </div>

    <!-- Remote Reboot Notification Modal -->
    <div id="remote-reboot-modal" style="display:none; position:fixed; top:0; left:0; width:100vw; height:100vh; background:rgba(15,23,42,0.85); backdrop-filter:blur(10px); z-index:9999; align-items:center; justify-content:center; padding:20px;">
        <div style="background:#1e293b; border:1px solid #f87171; border-radius:16px; padding:25px; max-width:420px; width:100%; text-align:center; box-shadow:0 25px 50px -12px rgba(0,0,0,0.8);">
            <div style="font-size:36px; margin-bottom:10px;">⚡</div>
            <h2 style="color:#f87171; font-size:18px; margin-bottom:10px; font-weight:600;">Remote Reboot ausgelöst!</h2>
            <p style="color:#cbd5e1; font-size:13px; line-height:1.5; margin-bottom:20px;">Dieses Gerät wurde aus der Ferne von deinem gekoppelten Partner-Gerät per ESP-NOW neugestartet.</p>
            <button type="button" onclick="document.getElementById('remote-reboot-modal').style.display='none';" style="background:rgba(248,113,113,0.15); border:1px solid #f87171; color:#f87171; padding:10px 24px; border-radius:8px; cursor:pointer; font-weight:600; font-size:13px; transition:all 0.2s;" onmouseover="this.style.background='rgba(248,113,113,0.3)'" onmouseout="this.style.background='rgba(248,113,113,0.15)'">Verstanden / Schließen</button>
        </div>
    </div>

    <!-- Factory Reset / Captive Portal Notification Modal -->
    <div id="factory-reset-modal" style="display:none; position:fixed; top:0; left:0; width:100vw; height:100vh; background:rgba(15,23,42,0.9); backdrop-filter:blur(12px); z-index:99999; align-items:center; justify-content:center; padding:20px;">
        <div style="background:#1e293b; border:1px solid #eab308; border-radius:18px; padding:28px; max-width:440px; width:100%; text-align:center; box-shadow:0 25px 50px -12px rgba(0,0,0,0.9);">
            <div style="font-size:42px; margin-bottom:12px;">⚙️</div>
            <h2 style="color:#eab308; font-size:20px; margin-bottom:12px; font-weight:700;">Werkseinstellungen geladen</h2>
            <p style="color:#e2e8f0; font-size:13.5px; line-height:1.6; margin-bottom:15px; text-align:left;">
                Das Gerät wurde zurückgesetzt und befindet sich im <b>Einrichtungs-Modus</b> (Access Point).
            </p>
            <div style="background:rgba(234,179,8,0.1); border:1px solid rgba(234,179,8,0.3); border-radius:10px; padding:12px 15px; margin-bottom:20px; text-align:left; font-size:12.5px; color:#fef08a; line-height:1.5;">
                <b>Schritte zur Wiederherstellung:</b><br>
                1. Verbinde dich mit dem WLAN <b>iDRY26-Setup</b><br>
                2. Öffne im Browser <b>http://192.168.4.1</b><br>
                3. WLAN/MQTT eintragen &amp; speichern.
            </div>
            <div style="font-size:11px; color:#94a3b8; margin-bottom:15px;">Hinweis: Standard-Modus ist auf <b>60/60 Blind-Betrieb</b> voreingestellt.</div>
            <button type="button" onclick="document.getElementById('factory-reset-modal').style.display='none'; document.getElementById('factory-reset-modal').dataset.closed='true';" style="background:rgba(234,179,8,0.2); border:1px solid #eab308; color:#eab308; padding:10px 24px; border-radius:8px; cursor:pointer; font-weight:600; font-size:13px; transition:all 0.2s;" onmouseover="this.style.background='rgba(234,179,8,0.35)'" onmouseout="this.style.background='rgba(234,179,8,0.2)'">Verstanden / Schließen</button>
        </div>
    </div>
    <script>
        const PANEL_INFOS = {
            // [0] Dry Strategy
            0: "<b>Dry Strategy</b><br>Wahl der Trocknungsstrategie: Klassischer 60/60 Modus (60°F / 60% rF), dynamischer VPD Modus oder automatisierter 14-Tage VPD AUTO Stufenplan.",
            
            // [1] VPD AUTO Matrix & Timeline
            1: "<b>VPD AUTO Modus</b><br>Wissenschaftlicher 21x14 Matrix-Stufenplan mit temperaturkompensiertem VPD-Sollwertverlauf über 14 Tage inklusive Schimmelschutz.",
            
            // [2] Hygro Limit Schimmelschutz
            2: "<b>Hygro-Limit Schimmelschutz</b><br>Sicherheits-Obergrenze für die relative Zielfeuchte (70%, 75% oder 80%), um Schimmelbildung in feuchten Umgebungen rigoros zu verhindern.",
            
            // [3] Potentiometer
            3: "<b>Potentiometer</b><br>Analoge Hardware-Regler für Sollwert-Feuchte A (inkl. Rigoros ZU/AUF Schalter), Regelverstärkung B (Gain 0-400%) und virtuellen 0°-Kalibrierungs-Offset C.",
            
            // [4] Rotor & Servo
            4: "<b>Rotor & Servo</b><br>Echtzeit-Stellungsanzeige des Lüftungsrotors (0-100%) mit animierter Mondphasen-Visualisierung und 60-Minuten-Verlaufshistorie.",
            
            // [5] Stoßlüftungs-Timer
            5: "<b>Stoßlüftungs-Timer</b><br>Periodische Zwangsbelüftung mit animierter Sanduhr und 3D-Walzenwählern für Intervall (10m-24h) und Öffnungsdauer (10s-10m).",
            
            // [6] ESPNOW
            6: "<b>ESP-NOW Funkverbindung</b><br>Drahtlose Echtzeit-Synchronisation zwischen Master und Slave-Geräten mit Link-Monitoring und Ausfall-Historie.",
            
            // [7] MQTT Dashboard
            7: "<b>MQTT Dashboard</b><br>Status der Anbindung an Home Assistant / MQTT-Broker mit Verbindungs-Historie und Telemetrie-Topics.",
            
            // [8] VPD (Sättigungsdefizit)
            8: "<b>VPD (Dampfdruckdefizit)</b><br>Berechnetes Sättigungsdefizit der Innen- und Außenluft in kPa zur präzisen Steuerung des Transpirationsdrucks.",
            
            // [9] Sensor 1
            9: "<b>Sensor 1 (Innen)</b><br>Primärer Klimasensor (BME280 / SHT3x) für Temperatur, relative Feuchte, Taupunkt und Luftdruck.",
            
            // [10] Sensor 2
            10: "<b>Sensor 2 (Außen)</b><br>Sekundärer Umgebungssensor für thermodynamischen Bypass-Schutz und Zuluft-Kompensation.",
            
            // [11] TSL2561 (1)
            11: "<b>Lichtsensor 1</b><br>Digitaler Helligkeitssensor (TSL2561) zur Erfassung von Lux, Breitband- und Infrarot-Lichtspektrum.",
            
            // [12] TSL2561 (2)
            12: "<b>Lichtsensor 2</b><br>Sekundärer digitaler Helligkeitssensor zur redundanten Lichtüberwachung.",
            
            // [13] System Status & Konsolen
            13: "<b>System Status &amp; Konsolen</b><br>Diagnose-Übersicht mit IP-Adresse, Display-Modus, RSSI-Signalstärke, 4h-Signalverlauf sowie Live-Terminal-Konsolen für lokale System-Logs und remote ESP-NOW Logs.",

            // [20] Grow Advisor & Live Ticker (Grow-Bro Disclaimer)
            20: "<b>Grow Advisor &amp; Live Ticker</b><br>Dies sind unverbindliche Tipps &amp; Denkanstöße – nimm sie bitte nicht zu bierernst! Die Automatik regelt so gut es geht, aber kein Algorithmus kann dein gärtnerisches Feingefühl ersetzen. Jeder Grow, jedes Zelt und jedes Raumklima ist anders. Sieh die Tipps nicht als Panik-Alarm, sondern als Anregung zum Mitdenken und selber Recherchieren. Keine Gewähr auf dynamische Tipps – Happy Growing! 🌿✌️"
        };

        let activeBubble = null;
        let activeBubbleBtn = null;

        function showInfo(btn, idx) {
            hideInfo();
            if (!PANEL_INFOS[idx]) return;
            const bubble = document.createElement('div');
            bubble.className = 'info-bubble';
            bubble.innerHTML = PANEL_INFOS[idx];
            btn.parentElement.appendChild(bubble);
            btn.classList.add('active');
            activeBubble = bubble;
            activeBubbleBtn = btn;
        }

        function hideInfo() {
            if (activeBubble) {
                if (activeBubble.parentElement) activeBubble.parentElement.removeChild(activeBubble);
                activeBubble = null;
            }
            if (activeBubbleBtn) {
                activeBubbleBtn.classList.remove('active');
                activeBubbleBtn = null;
            }
        }

        function toggleInfo(evt, idx) {
            evt.stopPropagation();
            if (activeBubbleBtn === evt.currentTarget) {
                hideInfo();
            } else {
                showInfo(evt.currentTarget, idx);
            }
        }

        document.addEventListener('click', function(e) {
            if (activeBubble && !activeBubble.contains(e.target) && e.target !== activeBubbleBtn) {
                hideInfo();
            }
            const advisorBubble = document.getElementById('advisor-popup-bubble');
            const tickerBox = document.getElementById('advisor-ticker-box');
            if (advisorBubble && advisorBubble.style.display !== 'none' && !advisorBubble.contains(e.target) && (!tickerBox || !tickerBox.contains(e.target))) {
                closeAdvisorPopup();
            }
        });

        // --- Smart Live-Advisor & Heuristic Ringbuffer Engine ---
        const ADVISOR_MAX_MSGS = 20;
        let advisorRingBuffer = [
            {
                type: 'optimal',
                badgeClass: 'badge-optimal',
                badgeText: '🟢 SYSTEMBEREIT',
                timeStr: '[00:00:00]',
                rawText: 'Smart Live Advisor Engine bereit. Analysiere thermodynamische Klimadaten...'
            }
        ];
        let advisorCurrentIdx = 0;
        let advisorPaused = false;
        let advisorScrollAnim = null;
        let advisorDwellTimer = null;
        let advisorCurrentX = 0;
        let advisorSpeed = 38; // pixels per second
        let advisorLastTimestamp = null;
        let lastAdvisorEvalTime = 0;
        let pressureHistory = []; // { time, press } for barometer 3h slope

        function pushAdvisorMsg(type, badgeClass, badgeText, msgText) {
            const now = new Date();
            const timeStr = "[" + String(now.getHours()).padStart(2,'0') + ":" + String(now.getMinutes()).padStart(2,'0') + ":" + String(now.getSeconds()).padStart(2,'0') + "]";
            
            // Deduplication: Only append if NOT identical to the latest report in ringbuffer
            if (advisorRingBuffer.length > 0 && advisorRingBuffer[0].rawText === msgText) {
                return;
            }

            const item = {
                type: type,
                badgeClass: badgeClass,
                badgeText: badgeText,
                timeStr: timeStr,
                rawText: msgText
            };

            advisorRingBuffer.unshift(item);
            if (advisorRingBuffer.length > ADVISOR_MAX_MSGS) {
                advisorRingBuffer.pop();
            }

            // If user is viewing the latest live message (idx 0), render and start continuous scroll
            if (advisorCurrentIdx === 0) {
                renderAdvisorMsg(0);
            } else {
                updateAdvisorCounter();
            }
        }

        function renderAdvisorMsg(idx) {
            if (idx < 0 || idx >= advisorRingBuffer.length) return;
            advisorCurrentIdx = idx;
            const item = advisorRingBuffer[idx];
            const track = document.getElementById('advisor-ticker-track');
            const box = document.getElementById('advisor-ticker-box');
            if (!track || !box) return;

            if (advisorScrollAnim) cancelAnimationFrame(advisorScrollAnim);
            if (advisorDwellTimer) clearTimeout(advisorDwellTimer);

            track.innerHTML = '<span class="advisor-badge ' + item.badgeClass + '">' + item.badgeText + '</span>' +
                              '<span class="advisor-time">' + item.timeStr + '</span>' +
                              '<span class="advisor-msg-text">' + item.rawText + '</span>';

            updateAdvisorCounter();

            // Start at x = 0 with a 2.0s initial reading pause, then scroll continuously non-stop!
            advisorCurrentX = 0;
            track.style.transform = 'translateX(0px)';
            advisorLastTimestamp = null;

            advisorDwellTimer = setTimeout(() => {
                advisorLastTimestamp = null;
                advisorScrollAnim = requestAnimationFrame(stepContinuousScroll);
            }, 2000);
        }

        function stepContinuousScroll(timestamp) {
            const track = document.getElementById('advisor-ticker-track');
            const box = document.getElementById('advisor-ticker-box');
            if (!track || !box) return;

            if (advisorPaused) {
                advisorLastTimestamp = timestamp;
                advisorScrollAnim = requestAnimationFrame(stepContinuousScroll);
                return;
            }

            if (!advisorLastTimestamp) advisorLastTimestamp = timestamp;
            const dt = (timestamp - advisorLastTimestamp) / 1000.0;
            advisorLastTimestamp = timestamp;

            const boxW = box.clientWidth || 300;
            const trackW = track.scrollWidth || 600;

            advisorCurrentX -= advisorSpeed * dt;

            // When message has completely scrolled out on left, wrap it back to right edge (seamless non-stop loop!)
            if (advisorCurrentX <= -trackW) {
                advisorCurrentX = boxW;
            }

            track.style.transform = 'translateX(' + advisorCurrentX + 'px)';
            advisorScrollAnim = requestAnimationFrame(stepContinuousScroll);
        }

        function prevAdvisorMsg() {
            if (advisorCurrentIdx < advisorRingBuffer.length - 1) {
                renderAdvisorMsg(advisorCurrentIdx + 1);
            }
        }

        function nextAdvisorMsg() {
            if (advisorCurrentIdx > 0) {
                renderAdvisorMsg(advisorCurrentIdx - 1);
            }
        }

        function updateAdvisorCounter() {
            const total = Math.max(1, advisorRingBuffer.length);
            const current = total === 0 ? 1 : (advisorCurrentIdx + 1);

            const counter = document.getElementById('advisor-counter');
            if (counter) counter.innerText = current + " / " + total;

            const popupCounter = document.getElementById('advisor-popup-counter');
            if (popupCounter) popupCounter.innerText = current + " / " + total;

            // Boundary Visibility:
            // "Neuer" (next) button: only visible if currentIdx > 0 (can go newer)
            const showNext = advisorCurrentIdx > 0;
            const nextBtn = document.getElementById('advisor-next-btn');
            if (nextBtn) nextBtn.style.visibility = showNext ? 'visible' : 'hidden';

            const popupNextBtn = document.getElementById('advisor-popup-next-btn');
            if (popupNextBtn) popupNextBtn.style.visibility = showNext ? 'visible' : 'hidden';

            // "Älter" (prev) button: only visible if currentIdx < total - 1 (can go older)
            const showPrev = advisorCurrentIdx < total - 1;
            const prevBtn = document.getElementById('advisor-prev-btn');
            if (prevBtn) prevBtn.style.visibility = showPrev ? 'visible' : 'hidden';

            const popupPrevBtn = document.getElementById('advisor-popup-prev-btn');
            if (popupPrevBtn) popupPrevBtn.style.visibility = showPrev ? 'visible' : 'hidden';
        }

        function pauseAdvisorTicker() {
            advisorPaused = true;
        }

        function resumeAdvisorTicker() {
            advisorPaused = false;
            advisorLastTimestamp = null;
        }

        // Gesture Drag, Tap & Full-Text Bubble Handling
        let dragStartX = 0;
        let dragStartY = 0;
        let isDraggingAdvisor = false;

        function startAdvisorDrag(e) {
            dragStartX = e.clientX || (e.touches && e.touches[0].clientX) || 0;
            dragStartY = e.clientY || (e.touches && e.touches[0].clientY) || 0;
            isDraggingAdvisor = true;
            pauseAdvisorTicker();
        }

        document.addEventListener('pointerup', function(e) {
            if (!isDraggingAdvisor) return;
            isDraggingAdvisor = false;
            const endX = e.clientX || 0;
            const endY = e.clientY || 0;
            const diffX = endX - dragStartX;
            const diffY = endY - dragStartY;

            if (Math.abs(diffX) <= 6 && Math.abs(diffY) <= 6) {
                // User clicked/tapped without moving -> Open Full-Text Speech Bubble!
                openAdvisorPopup(advisorCurrentIdx);
                return;
            }

            if (Math.abs(diffX) > 35) {
                if (diffX > 0) {
                    prevAdvisorMsg(); // Swiped right -> older
                } else {
                    nextAdvisorMsg(); // Swiped left -> newer
                }
            }
            setTimeout(resumeAdvisorTicker, 1500);
        });

        function openAdvisorPopup(idx) {
            if (idx < 0 || idx >= advisorRingBuffer.length) return;
            advisorCurrentIdx = idx;
            const item = advisorRingBuffer[idx];
            const popup = document.getElementById('advisor-popup-bubble');
            const content = document.getElementById('advisor-popup-content');
            if (!popup || !content) return;

            pauseAdvisorTicker();
            content.innerHTML = '<div style="display:flex; align-items:center; gap:8px; margin-bottom:10px;">' +
                                '<span class="advisor-badge ' + item.badgeClass + '">' + item.badgeText + '</span>' +
                                '<span class="advisor-time">' + item.timeStr + '</span>' +
                                '</div>' +
                                '<div style="font-size:13.5px; color:#f8fafc; font-weight:500; line-height:1.5;">' + item.rawText + '</div>';

            popup.style.display = 'block';
            updateAdvisorCounter();
        }

        function closeAdvisorPopup(e) {
            if (e) e.stopPropagation();
            const popup = document.getElementById('advisor-popup-bubble');
            if (popup) popup.style.display = 'none';
            resumeAdvisorTicker();
        }

        function prevAdvisorPopup(e) {
            if (e) e.stopPropagation();
            if (advisorCurrentIdx < advisorRingBuffer.length - 1) {
                let pIdx = advisorCurrentIdx + 1;
                renderAdvisorMsg(pIdx);
                openAdvisorPopup(pIdx);
            }
        }

        function nextAdvisorPopup(e) {
            if (e) e.stopPropagation();
            if (advisorCurrentIdx > 0) {
                let nIdx = advisorCurrentIdx - 1;
                renderAdvisorMsg(nIdx);
                openAdvisorPopup(nIdx);
            }
        }

        function evaluateGrowerHeuristics(data) {
            const now = Date.now();
            if (now - lastAdvisorEvalTime < 10000) return;
            lastAdvisorEvalTime = now;

            // Extract sensor & state variables
            let s0 = data.sensors && data.sensors[0] ? data.sensors[0] : null;
            let s1 = data.sensors && data.sensors[1] ? data.sensors[1] : null;

            const tempIn = s0 && s0.temperature !== undefined && s0.temperature !== null ? s0.temperature : null;
            const humIn = s0 && s0.humidity !== undefined && s0.humidity !== null ? s0.humidity : null;
            const dpIn = s0 && s0.dewpoint !== undefined && s0.dewpoint !== null ? s0.dewpoint : null;
            const vpdIn = s0 && s0.vpd !== undefined && s0.vpd !== null ? s0.vpd : (calculateVPD_JS(tempIn, humIn));
            const pressIn = s0 && s0.pressure !== undefined && s0.pressure !== null ? s0.pressure : null;

            const tempOut = s1 && s1.temperature !== undefined && s1.temperature !== null ? s1.temperature : null;
            const humOut = s1 && s1.humidity !== undefined && s1.humidity !== null ? s1.humidity : null;
            const vpdOut = s1 && s1.vpd !== undefined && s1.vpd !== null ? s1.vpd : (calculateVPD_JS(tempOut, humOut));
            const pressOut = s1 && s1.pressure !== undefined && s1.pressure !== null ? s1.pressure : null;

            const dryStrat = data.dry_strategy !== undefined ? data.dry_strategy : 0;
            const vpdTarget = (data.potentiometers && data.potentiometers.target_vpd) ? data.potentiometers.target_vpd : 0.85;
            const targetHumPoti = (data.potentiometers && data.potentiometers.poti_a_target_hum) ? data.potentiometers.poti_a_target_hum : 60;
            const vpdAutoDay = data.vpd_auto_day !== undefined ? data.vpd_auto_day : 1;
            const hygroLimit = data.hygro_limit !== undefined ? data.hygro_limit : 75;
            const purgeActive = data.purge_active || false;
            const rotorPos = data.rotor_position !== undefined ? Math.round(data.rotor_position) : 0;

            // Integer-quantized values for stable deduplication without jitter
            const rTempIn = tempIn !== null ? Math.round(tempIn) : null;
            const rHumIn = humIn !== null ? Math.round(humIn) : null;
            const rDpIn = dpIn !== null ? Math.round(dpIn) : null;
            const rVpdIn = vpdIn !== null ? (Math.round(vpdIn * 10) / 10).toFixed(1) : null;
            const rVpdTarget = (Math.round(vpdTarget * 10) / 10).toFixed(1);

            const rTempOut = tempOut !== null ? Math.round(tempOut) : null;
            const rHumOut = humOut !== null ? Math.round(humOut) : null;

            // 1. Weather Pressure Slope Tracking
            if (pressIn && pressIn > 300) {
                pressureHistory.push({ time: now, press: pressIn });
                pressureHistory = pressureHistory.filter(p => now - p.time <= 4 * 3600 * 1000);
            }

            let pressSlope = 0;
            if (pressureHistory.length >= 2) {
                const oldest = pressureHistory[0];
                const newest = pressureHistory[pressureHistory.length - 1];
                const dtHours = (newest.time - oldest.time) / (3600 * 1000);
                if (dtHours >= 0.5) {
                    pressSlope = Math.round(((newest.press - oldest.press) / dtHours) * 3.0); // rounded hPa/3h
                }
            }

            // --- Priority 1: Hardware & Environment Safety Events ---

            // A. Active Stoßlüftung Notification
            if (purgeActive) {
                pushAdvisorMsg('event', 'badge-system', '🟣 STOSSLÜFTUNG', 'Stoßlüftung aktiv: Klappe 100% geöffnet. Zelt wird intensiv mit Frischluft gespült.');
                return;
            }

            // B. Tent Wall Condensation Hazard (Outside colder than inside & high humidity)
            if (tempIn !== null && tempOut !== null && humIn !== null) {
                const deltaT = Math.round(tempIn - tempOut);
                if (deltaT >= 3 && rHumIn >= 65) {
                    pushAdvisorMsg('alert', 'badge-alert', '🔴 KONDENSATION', 'Kondensationsrisiko: Zeltwand kühlt von außen stark ab (ΔT ' + deltaT + '°C, Innenfeuchte ' + rHumIn + '% rF). Umluft auf Zeltwände/Boden richten oder Vorraum heizen!');
                    return;
                }
            }

            // C. Barometric Trend Alerts
            if (pressSlope <= -2) {
                pushAdvisorMsg('weather', 'badge-weather', '🔵 TIEFDRUCKFRONT', 'Barometer fällt (' + pressSlope + ' hPa/3h). Regen/steigende Außenfeuchte im Anmarsch – Entfeuchter im Vorraum bereithalten.');
            } else if (pressSlope >= 2) {
                pushAdvisorMsg('weather', 'badge-weather', '🔵 HOCHDRUCKWETTER', 'Barometer steigt (+' + pressSlope + ' hPa/3h). Trockene Witterung zieht auf – Übertrocknung im Auge behalten.');
            }

            // --- Priority 2: Strategy-Specific Climate Diagnostics & Tips ---

            if (dryStrat === 2) { // === VPD AUTO MODE ===
                if (rVpdIn !== null) {
                    let stageText = "";
                    if (vpdAutoDay <= 3) stageText = "Tag " + vpdAutoDay + "/14 (Frische Ernte): Hoher Feuchteabtrag. Klappe federt Spitzen ab.";
                    else if (vpdAutoDay <= 10) stageText = "Tag " + vpdAutoDay + "/14 (Curing-Phase): Gleichmäßiger Feuchteabbau im Zielkorridor.";
                    else stageText = "Tag " + vpdAutoDay + "/14 (Ziellandung): Stängel-Knicktest durchführen (knackt der Stängel, ist die Ernte perfekt trocken!).";

                    if (vpdIn < 0.55 || (humIn && humIn >= hygroLimit)) {
                        if (rHumOut !== null && rHumOut < 58) {
                            pushAdvisorMsg('alert', 'badge-alert', '🔴 SCHIMMELSCHUTZ', 'VPD AUTO ' + stageText + ' – Warnung: Ist-VPD ' + rVpdIn + ' kPa zu niedrig (Soll: ' + rVpdTarget + ' kPa, Feuchte: ' + (rHumIn !== null ? rHumIn : '--') + '% rF). Lüftung erhöhen, um trockenere Außenluft nachzuziehen.');
                        } else {
                            pushAdvisorMsg('alert', 'badge-alert', '🔴 SCHIMMELSCHUTZ', 'VPD AUTO ' + stageText + ' – Warnung: Ist-VPD ' + rVpdIn + ' kPa zu niedrig. Außenluft ebenfalls feucht: Vorraum um 1–2°C heizen oder Entfeuchter einschalten!');
                        }
                    } else if (vpdIn > 1.30) {
                        if (rHumOut !== null && rHumOut > (rHumIn || 50)) {
                            pushAdvisorMsg('tip', 'badge-tip', '🟡 HEUGERUCH-GEFAHR', 'VPD AUTO ' + stageText + ' – Tipp: Ist-VPD ' + rVpdIn + ' kPa zu hoch (Soll: ' + rVpdTarget + ' kPa). Trocknung zu aggressiv! Klappe weiter schließen, um Feuchte im Zelt zu halten.');
                        } else {
                            pushAdvisorMsg('tip', 'badge-tip', '🟡 TROCKENE ZULUFT', 'VPD AUTO ' + stageText + ' – Tipp: Ist-VPD ' + rVpdIn + ' kPa zu hoch. Trockene Zuluft: Nasses Handtuch oder Wasserschale vor Bodenventilator platzieren (Umluft NIE direkt auf Blüten!).');
                        }
                    } else {
                        pushAdvisorMsg('optimal', 'badge-optimal', '🟢 VPD AUTO OPTIMAL', 'VPD AUTO ' + stageText + ' – Perfekt: Ist-VPD ' + rVpdIn + ' kPa liegt exakt am Stufenplan-Zielwert (' + rVpdTarget + ' kPa, Klappe: ' + rotorPos + '%).');
                    }
                }
            } else if (dryStrat === 1) { // === MANUAL VPD MODE ===
                if (rVpdIn !== null) {
                    if (vpdIn < 0.55 || (humIn && humIn >= hygroLimit)) {
                        if (rHumOut !== null && rHumOut < 58) {
                            pushAdvisorMsg('alert', 'badge-alert', '🔴 SCHIMMELSCHUTZ', 'VPD Modus: Ist-VPD ' + rVpdIn + ' kPa zu niedrig (Soll: ' + rVpdTarget + ' kPa / ' + (rHumIn !== null ? rHumIn : '--') + '% rF). Schimmelrisiko: Klappe/Abluft erhöhen, um trockenere Außenluft einzusaugen.');
                        } else {
                            pushAdvisorMsg('alert', 'badge-alert', '🔴 RAUMKLIMA', 'VPD Modus: Ist-VPD ' + rVpdIn + ' kPa zu niedrig. Außenluft ebenfalls feucht: Vorraum um +1–2°C erwärmen (senkt rF) oder Raumentfeuchter zuschalten.');
                        }
                    } else if (vpdIn > 1.30) {
                        if (rHumOut !== null && rHumOut > (rHumIn || 50)) {
                            pushAdvisorMsg('tip', 'badge-tip', '🟡 HEUGERUCH-GEFAHR', 'VPD Modus: Ist-VPD ' + rVpdIn + ' kPa zu hoch (Soll: ' + rVpdTarget + ' kPa). Zu rasche Austrocknung zerstört Terpene! Klappe drosseln.');
                        } else {
                            pushAdvisorMsg('tip', 'badge-tip', '🟡 TROCKENE ZULUFT', 'VPD Modus: Ist-VPD ' + rVpdIn + ' kPa zu hoch. Sehr trockener Vorraum. DIY: Nasses Handtuch auf Zeltboden platzieren; Umluft nur indirekt strömen lassen.');
                        }
                    } else {
                        pushAdvisorMsg('optimal', 'badge-optimal', '🟢 VPD OPTIMAL', 'VPD Modus: Transpirationsdruck bei ' + rVpdIn + ' kPa (Soll: ' + rVpdTarget + ' kPa) perfekt ausbalanciert. Blüten reifen gleichmäßig.');
                    }
                }
            } else { // === 60/60 MODE ===
                if (rHumIn !== null) {
                    const targetRH = Math.round((targetHumPoti >= 50 && targetHumPoti <= 70) ? targetHumPoti : 60);
                    const deltaRH = Math.round(humIn - targetRH);

                    if (deltaRH >= 3 || rHumIn > 65) {
                        if (rHumOut !== null && rHumOut < (rHumIn - 3)) {
                            pushAdvisorMsg('tip', 'badge-tip', '🟡 ENTLÜFTEN', '60/60 Modus: Feuchte bei ' + rHumIn + '% rF (Soll: ' + targetRH + '%, Δ +' + deltaRH + '%). Außenluft ist mit ' + rHumOut + '% rF trockener – Klappe weiter öffnen und Abluft steigern.');
                        } else {
                            pushAdvisorMsg('tip', 'badge-tip', '🟡 RAUMKLIMA', '60/60 Modus: Feuchte bei ' + rHumIn + '% rF (Soll: ' + targetRH + '%, Δ +' + (deltaRH >= 0 ? '+' : '') + deltaRH + '%). Außenluft ebenfalls feucht (' + (rHumOut !== null ? rHumOut : '--') + '% rF)! Vorraum um 1–2°C heizen oder Entfeuchter starten.');
                        }
                    } else if (deltaRH <= -3 || rHumIn < 55) {
                        pushAdvisorMsg('tip', 'badge-tip', '🟡 ZU TROCKEN', '60/60 Modus: Feuchte bei ' + rHumIn + '% rF (Soll: ' + targetRH + '%, Δ ' + deltaRH + '%). Gefahr von Heugeruch! Klappe drosseln; DIY-Tipp: Nasses Tuch oder Wasserschale auf Zeltboden stellen.');
                    } else {
                        pushAdvisorMsg('optimal', 'badge-optimal', '🟢 60/60 OPTIMAL', '60/60 Modus: Feuchte bei ' + rHumIn + '% rF liegt perfekt im Zielkorridor (Soll: ' + targetRH + '% rF, Taupunkt: ' + (rDpIn !== null ? rDpIn : '--') + '°C). Reifung verläuft ideal.');
                    }
                }
            }
        }

        const wifiSSID = ")rawhtml";

    const char* DASHBOARD_HTML_PART3 = R"rawhtml(";
        let wasInPortalMode = false;
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
        let currentDryStrategy = 0;
        let currentHygroLimit = 70;
        let latestData = null;
        const vpdAutoProfileJS = [0.70, 0.72, 0.74, 0.76, 0.78, 0.80, 0.81, 0.82, 0.83, 0.84, 0.85, 0.85, 0.85, 0.85];

        function performUiLogin() {
            let passInput = document.getElementById('login-pass-input');
            let passVal = passInput ? passInput.value : '';
            fetch('/api/auth?pass=' + encodeURIComponent(passVal))
                .then(r => r.json())
                .then(d => {
                    if (d.status === 'ok') {
                        sessionStorage.setItem('idry_web_pass', passVal);
                        document.cookie = "idry_pass=" + encodeURIComponent(passVal) + "; path=/; max-age=86400";
                        let errEl = document.getElementById('login-err-msg');
                        if (errEl) errEl.style.display = 'none';
                        updateData();
                    } else {
                        let errEl = document.getElementById('login-err-msg');
                        if (errEl) errEl.style.display = 'block';
                    }
                })
                .catch(err => console.error(err));
        }

        function setDryStrategy(mode, limit, day) {
            if (latestData && latestData.web_auth_required && !latestData.web_authenticated) {
                alert("🔒 Webinterface ist geschützt. Bitte zuerst Passwort eingeben.");
                return;
            }
            currentDryStrategy = mode;
            if (limit) currentHygroLimit = limit;
            let savedPass = sessionStorage.getItem('idry_web_pass') || '';
            let url = '/api/settings/dry_strategy?mode=' + mode + '&limit=' + currentHygroLimit + '&pass=' + encodeURIComponent(savedPass);
            if (day) url += '&day=' + day;
            fetch(url, { method: 'POST' })
                .then(r => r.json())
                .then(data => {
                    if (data.status === 'ok') {
                        updateData();
                    }
                }).catch(e => console.error("Dry Strategy save error", e));
        }

        let isDrumUserScrolling = false;
        let drumScrollTimeout = null;

        function getDrumValue(pickerId) {
            let picker = document.getElementById(pickerId);
            if (!picker) return 0;
            let items = picker.querySelectorAll('.drum-item');
            let itemHeight = 28;
            let index = Math.round(picker.scrollTop / itemHeight);
            if (index < 0) index = 0;
            if (index >= items.length) index = items.length - 1;
            return parseInt(items[index].dataset.val || "0");
        }

        function setDrumValue(pickerId, val) {
            let picker = document.getElementById(pickerId);
            if (!picker || isDrumUserScrolling) return;
            let items = picker.querySelectorAll('.drum-item');
            let itemHeight = 28;
            for (let i = 0; i < items.length; i++) {
                if (parseInt(items[i].dataset.val) === parseInt(val)) {
                    picker.scrollTop = i * itemHeight;
                    updateDrumItemClasses(picker, i);
                    break;
                }
            }
        }

        function updateDrumItemClasses(picker, activeIdx) {
            let items = picker.querySelectorAll('.drum-item');
            items.forEach((item, idx) => {
                item.classList.remove('active', 'top-neighbor', 'bottom-neighbor');
                if (idx === activeIdx) {
                    item.classList.add('active');
                } else if (idx === activeIdx - 1) {
                    item.classList.add('top-neighbor');
                } else if (idx === activeIdx + 1) {
                    item.classList.add('bottom-neighbor');
                }
            });
        }

        function initDrumPickers() {
            ['wheel-interval', 'wheel-duration'].forEach(pickerId => {
                let picker = document.getElementById(pickerId);
                if (!picker) return;

                let handleScroll = () => {
                    isDrumUserScrolling = true;
                    if (drumScrollTimeout) clearTimeout(drumScrollTimeout);
                    let itemHeight = 28;
                    let activeIdx = Math.round(picker.scrollTop / itemHeight);
                    updateDrumItemClasses(picker, activeIdx);

                    drumScrollTimeout = setTimeout(() => {
                        isDrumUserScrolling = false;
                        onPurgeSettingChange();
                    }, 300);
                };

                picker.addEventListener('scroll', handleScroll, { passive: true });

                // PC Mouse Drag-to-Scroll Engine
                let isMouseDown = false;
                let startY = 0;
                let startScrollTop = 0;
                let hasDragged = false;

                picker.addEventListener('mousedown', (e) => {
                    isMouseDown = true;
                    hasDragged = false;
                    startY = e.clientY;
                    startScrollTop = picker.scrollTop;
                    picker.style.scrollSnapType = 'none';
                });

                window.addEventListener('mousemove', (e) => {
                    if (!isMouseDown) return;
                    let deltaY = e.clientY - startY;
                    if (Math.abs(deltaY) > 4) {
                        hasDragged = true;
                    }
                    picker.scrollTop = startScrollTop - deltaY;
                    let activeIdx = Math.round(picker.scrollTop / 28);
                    updateDrumItemClasses(picker, activeIdx);
                });

                let endDrag = () => {
                    if (!isMouseDown) return;
                    isMouseDown = false;
                    picker.style.scrollSnapType = 'y mandatory';
                    let targetIdx = Math.round(picker.scrollTop / 28);
                    picker.scrollTop = targetIdx * 28;
                    updateDrumItemClasses(picker, targetIdx);
                    if (hasDragged) {
                        onPurgeSettingChange();
                    }
                };

                window.addEventListener('mouseup', endDrag);

                let items = picker.querySelectorAll('.drum-item');
                items.forEach((item, idx) => {
                    item.addEventListener('click', () => {
                        if (hasDragged) return;
                        picker.scrollTop = idx * 28;
                        updateDrumItemClasses(picker, idx);
                        onPurgeSettingChange();
                    });
                });
            });
        }

        function onPurgeSettingChange() {
            if (latestData && latestData.web_auth_required && !latestData.web_authenticated) {
                alert("🔒 Webinterface ist geschützt. Bitte zuerst Passwort eingeben.");
                if (latestData) {
                    setDrumValue('wheel-interval', latestData.purge_interval_min !== undefined ? latestData.purge_interval_min : 240);
                    setDrumValue('wheel-duration', latestData.purge_duration_sec !== undefined ? latestData.purge_duration_sec : 30);
                }
                return;
            }
            let intVal = getDrumValue('wheel-interval');
            let durVal = getDrumValue('wheel-duration');
            let savedPass = sessionStorage.getItem('idry_web_pass') || '';
            fetch('/api/settings/purge?interval=' + intVal + '&duration=' + durVal + '&pass=' + encodeURIComponent(savedPass), { method: 'POST' })
                .then(r => r.json())
                .then(data => {
                    if (data.status === 'ok') {
                        updateData();
                    }
                }).catch(e => console.error("Purge setting save error", e));
        }

        let currentVpdAutoActiveDay = 1;

        const vpdMatrixJS = [
            [0.56, 0.58, 0.59, 0.61, 0.62, 0.64, 0.65, 0.66, 0.66, 0.67, 0.68, 0.68, 0.68, 0.68],
            [0.59, 0.60, 0.62, 0.64, 0.66, 0.67, 0.68, 0.69, 0.70, 0.71, 0.71, 0.71, 0.71, 0.71],
            [0.62, 0.63, 0.65, 0.67, 0.69, 0.70, 0.71, 0.72, 0.73, 0.74, 0.75, 0.75, 0.75, 0.75],
            [0.64, 0.66, 0.68, 0.70, 0.72, 0.74, 0.75, 0.75, 0.76, 0.77, 0.78, 0.78, 0.78, 0.78],
            [0.67, 0.69, 0.71, 0.73, 0.75, 0.77, 0.78, 0.79, 0.80, 0.81, 0.82, 0.82, 0.82, 0.82],
            [0.70, 0.72, 0.74, 0.76, 0.78, 0.80, 0.81, 0.82, 0.83, 0.84, 0.85, 0.85, 0.85, 0.85],
            [0.73, 0.75, 0.77, 0.79, 0.81, 0.83, 0.84, 0.85, 0.86, 0.87, 0.88, 0.88, 0.88, 0.88],
            [0.76, 0.78, 0.80, 0.82, 0.84, 0.86, 0.87, 0.89, 0.90, 0.91, 0.92, 0.92, 0.92, 0.92],
            [0.78, 0.81, 0.83, 0.85, 0.87, 0.90, 0.91, 0.92, 0.93, 0.94, 0.95, 0.95, 0.95, 0.95],
            [0.81, 0.84, 0.86, 0.88, 0.90, 0.93, 0.94, 0.95, 0.96, 0.97, 0.99, 0.99, 0.99, 0.99],
            [0.84, 0.86, 0.89, 0.91, 0.94, 0.96, 0.97, 0.98, 1.00, 1.01, 1.02, 1.02, 1.02, 1.02],
            [0.87, 0.89, 0.92, 0.94, 0.97, 0.99, 1.00, 1.02, 1.03, 1.04, 1.05, 1.05, 1.05, 1.05],
            [0.90, 0.92, 0.95, 0.97, 1.00, 1.02, 1.04, 1.05, 1.06, 1.07, 1.09, 1.09, 1.09, 1.09],
            [0.92, 0.95, 0.98, 1.00, 1.03, 1.06, 1.07, 1.08, 1.10, 1.11, 1.12, 1.12, 1.12, 1.12],
            [0.95, 0.98, 1.01, 1.03, 1.06, 1.09, 1.10, 1.12, 1.13, 1.14, 1.16, 1.16, 1.16, 1.16],
            [0.98, 1.01, 1.04, 1.06, 1.10, 1.12, 1.13, 1.15, 1.16, 1.18, 1.19, 1.19, 1.19, 1.19],
            [1.01, 1.04, 1.07, 1.10, 1.13, 1.16, 1.17, 1.19, 1.20, 1.21, 1.23, 1.23, 1.23, 1.23],
            [1.02, 1.05, 1.08, 1.11, 1.14, 1.17, 1.18, 1.20, 1.21, 1.22, 1.24, 1.24, 1.24, 1.24],
            [1.04, 1.07, 1.10, 1.13, 1.16, 1.19, 1.20, 1.22, 1.23, 1.25, 1.26, 1.26, 1.26, 1.26],
            [1.06, 1.09, 1.12, 1.15, 1.18, 1.21, 1.22, 1.24, 1.25, 1.27, 1.28, 1.28, 1.28, 1.28],
            [1.08, 1.11, 1.14, 1.17, 1.20, 1.23, 1.24, 1.26, 1.27, 1.29, 1.30, 1.30, 1.30, 1.30]
        ];

        function getVpdColor(val) {
            let t = Math.max(0, Math.min(1, (val - 0.50) / 0.85));
            if (t < 0.33) {
                let r = Math.round(56 + (34 - 56) * (t / 0.33));
                let g = Math.round(189 + (197 - 189) * (t / 0.33));
                let b = Math.round(248 + (94 - 248) * (t / 0.33));
                return `rgb(${r},${g},${b})`;
            } else if (t < 0.67) {
                let factor = (t - 0.33) / 0.34;
                let r = Math.round(34 + (250 - 34) * factor);
                let g = Math.round(197 + (204 - 197) * factor);
                let b = Math.round(94 + (21 - 94) * factor);
                return `rgb(${r},${g},${b})`;
            } else {
                let factor = (t - 0.67) / 0.33;
                let r = Math.round(250 + (239 - 250) * factor);
                let g = Math.round(204 + (68 - 204) * factor);
                let b = Math.round(21 + (68 - 21) * factor);
                return `rgb(${r},${g},${b})`;
            }
        }

        function renderVpdHeatmapCanvas(activeDay, indoorTemp) {
            const canvas = document.getElementById('vpd-heatmap-canvas');
            if (!canvas) return;
            const container = canvas.parentElement;
            const boxW = container.offsetWidth || 280;
            const boxH = 100;
            const dpr = window.devicePixelRatio || 1;
            const w = canvas.width = boxW * dpr;
            const h = canvas.height = boxH * dpr;
            const ctx = canvas.getContext('2d');

            ctx.clearRect(0, 0, w, h);

            const marginL = 20 * dpr;
            const marginR = 24 * dpr;
            const marginB = 12 * dpr;
            const marginT = 14 * dpr;

            const gridW = w - marginL - marginR;
            const gridH = h - marginT - marginB;

            const cellW = gridW / 14;
            const cellH = gridH / 21;

            for (let r = 0; r < 21; r++) {
                let tIdx = 20 - r;
                for (let c = 0; c < 14; c++) {
                    let val = vpdMatrixJS[tIdx][c];
                    ctx.fillStyle = getVpdColor(val);
                    ctx.fillRect(marginL + c * cellW, marginT + r * cellH, cellW - 0.4 * dpr, cellH - 0.4 * dpr);
                }
            }

            ctx.fillStyle = '#64748b';
            ctx.font = `${7 * dpr}px monospace`;
            ctx.textAlign = 'right';
            ctx.textBaseline = 'middle';
            ctx.fillText('35°', marginL - 2 * dpr, marginT + 0.5 * cellH);
            ctx.fillText('25°', marginL - 2 * dpr, marginT + 10.5 * cellH);
            ctx.fillText('15°', marginL - 2 * dpr, marginT + 20.5 * cellH);

            ctx.textAlign = 'center';
            ctx.textBaseline = 'top';
            ctx.fillText('1', marginL + 0.5 * cellW, h - marginB + 1 * dpr);
            ctx.fillText('7', marginL + 6.5 * cellW, h - marginB + 1 * dpr);
            ctx.fillText('14', marginL + 13.5 * cellW, h - marginB + 1 * dpr);

            const legX = w - marginR + 4 * dpr;
            const legW = 5 * dpr;
            const grad = ctx.createLinearGradient(0, marginT + gridH, 0, marginT);
            grad.addColorStop(0, '#38bdf8');
            grad.addColorStop(0.35, '#22c55e');
            grad.addColorStop(0.65, '#facc15');
            grad.addColorStop(1.0, '#ef4444');
            ctx.fillStyle = grad;
            ctx.fillRect(legX, marginT, legW, gridH);

            ctx.fillStyle = '#64748b';
            ctx.font = `${6.5 * dpr}px monospace`;
            ctx.textAlign = 'left';
            ctx.textBaseline = 'top';
            ctx.fillText('1.4', legX + legW + 2 * dpr, marginT);
            ctx.textBaseline = 'bottom';
            ctx.fillText('0.5', legX + legW + 2 * dpr, marginT + gridH);

            let col = activeDay - 1;
            let tempRounded = Math.round(indoorTemp || 20);
            let tIdx = tempRounded - 15;
            if (tIdx < 0) tIdx = 0;
            if (tIdx > 20) tIdx = 20;
            let row = 20 - tIdx;

            let cx = marginL + col * cellW + cellW / 2;
            let cy = marginT + row * cellH + cellH / 2;
            let activeVpd = vpdMatrixJS[tIdx][col];

            // 3D Neon Laser Crosshair with dark high-contrast outline
            ctx.setLineDash([3 * dpr, 3 * dpr]);
            
            // 1. Dark Outer Shadow/Outline Line (3.5px) for 100% contrast on any cell color
            ctx.strokeStyle = '#0f172a';
            ctx.lineWidth = 3.5 * dpr;

            ctx.beginPath();
            ctx.moveTo(cx, marginT);
            ctx.lineTo(cx, marginT + gridH);
            ctx.stroke();

            ctx.beginPath();
            ctx.moveTo(marginL, cy);
            ctx.lineTo(marginL + gridW, cy);
            ctx.stroke();

            // 2. Bright Inner Neon Laser Line (1.5px)
            ctx.strokeStyle = isInspectingHeatmap ? '#facc15' : '#38bdf8';
            ctx.lineWidth = 1.5 * dpr;

            ctx.beginPath();
            ctx.moveTo(cx, marginT);
            ctx.lineTo(cx, marginT + gridH);
            ctx.stroke();

            ctx.beginPath();
            ctx.moveTo(marginL, cy);
            ctx.lineTo(marginL + gridW, cy);
            ctx.stroke();
            ctx.setLineDash([]);

            // 3. Laser Center Dot with outer ring
            ctx.beginPath();
            ctx.arc(cx, cy, 4 * dpr, 0, Math.PI * 2);
            ctx.fillStyle = '#0f172a';
            ctx.fill();

            ctx.beginPath();
            ctx.arc(cx, cy, 2.5 * dpr, 0, Math.PI * 2);
            ctx.fillStyle = '#ffffff';
            ctx.fill();
            ctx.strokeStyle = isInspectingHeatmap ? '#facc15' : '#38bdf8';
            ctx.lineWidth = 1.5 * dpr;
            ctx.stroke();

            // 4. Elevated Floating Badge Tooltip (Shifted 22px higher above intersection)
            let inspectTag = isInspectingHeatmap ? "[INSPEKTION] " : "";
            let badgeText = `${inspectTag}Tag ${activeDay} @ ${tempRounded}°C: ${activeVpd.toFixed(2)} kPa`;
            ctx.font = `bold ${7.5 * dpr}px sans-serif`;
            let textW = ctx.measureText(badgeText).width;
            let padX = 5 * dpr, padY = 2 * dpr;
            let bx = cx - textW / 2 - padX;
            let by = cy - 22 * dpr; // Elevated 22px above cy so crosshair line is visible!
            if (bx < marginL) bx = marginL;
            if (bx + textW + padX * 2 > w - marginR) bx = w - marginR - textW - padX * 2;
            if (by < 1 * dpr) by = cy + 6 * dpr;

            ctx.fillStyle = 'rgba(15, 23, 42, 0.95)';
            ctx.strokeStyle = isInspectingHeatmap ? '#facc15' : '#38bdf8';
            ctx.lineWidth = 1 * dpr;
            ctx.beginPath();
            ctx.roundRect(bx, by, textW + padX * 2, 10 * dpr + padY * 2, 3 * dpr);
            ctx.fill();
            ctx.stroke();

            ctx.fillStyle = isInspectingHeatmap ? '#facc15' : '#38bdf8';
            ctx.textAlign = 'left';
            ctx.textBaseline = 'middle';
            ctx.fillText(badgeText, bx + padX, by + (10 * dpr + padY * 2) / 2);
        }

        let isInspectingHeatmap = false;
        let inspectDay = 1;
        let inspectTemp = 20;

        function handleHeatmapPointer(e) {
            const canvas = document.getElementById('vpd-heatmap-canvas');
            if (!canvas) return;
            const rect = canvas.getBoundingClientRect();
            const clientX = e.touches ? e.touches[0].clientX : e.clientX;
            const clientY = e.touches ? e.touches[0].clientY : e.clientY;
            const x = clientX - rect.left;
            const y = clientY - rect.top;

            const marginL = 20;
            const marginR = 24;
            const marginT = 14;
            const marginB = 12;

            const gridW = rect.width - marginL - marginR;
            const gridH = rect.height - marginT - marginB;

            const cellW = gridW / 14;
            const cellH = gridH / 21;

            let col = Math.floor((x - marginL) / cellW) + 1;
            let row = Math.floor((y - marginT) / cellH);

            if (col >= 1 && col <= 14 && row >= 0 && row < 21) {
                isInspectingHeatmap = true;
                inspectDay = col;
                inspectTemp = 35 - row;
                renderVpdAutoTimeline(inspectDay, inspectTemp);
            }
        }

        function stopHeatmapInspection() {
            if (isInspectingHeatmap) {
                isInspectingHeatmap = false;
                let tempR = (latestData && latestData.indoor_temp_rounded) ? latestData.indoor_temp_rounded : 20;
                renderVpdAutoTimeline(currentVpdAutoActiveDay, tempR);
            }
        }

        function renderVpdAutoTimeline(activeDay, activeTemp) {
            currentVpdAutoActiveDay = activeDay;
            let tempR = (activeTemp !== undefined) ? activeTemp : ((latestData && latestData.indoor_temp_rounded) ? latestData.indoor_temp_rounded : 20);
            let tIdx = Math.round(tempR) - 15;
            if (tIdx < 0) tIdx = 0;
            if (tIdx > 20) tIdx = 20;

            renderVpdHeatmapCanvas(activeDay, tempR);

            let container = document.getElementById('vpd-auto-timeline');
            if (!container) return;
            let html = "";
            for (let day = 1; day <= 14; day++) {
                let vpdVal = vpdMatrixJS[tIdx][day - 1];
                let heightPct = Math.round(((vpdVal - 0.50) / 0.90) * 100);
                if (heightPct < 25) heightPct = 25;
                if (heightPct > 100) heightPct = 100;

                let barBg = "";
                let borderStyle = "";
                let isCurrent = (day === activeDay);

                if (day < activeDay) {
                    barBg = "#1e293b";
                    borderStyle = "1px solid rgba(255,255,255,0.08)";
                } else if (isCurrent) {
                    barBg = "#ffffff";
                    borderStyle = "2px solid #38bdf8";
                } else {
                    barBg = "#0284c7";
                    borderStyle = "1px solid rgba(56,189,248,0.4)";
                }

                let activeClass = isCurrent ? "class='vpd-candle-active'" : "";

                html += `<div onclick="openVpdDayModal(${day}, ${vpdVal.toFixed(2)})" style="flex:1; height:100%; display:flex; align-items:flex-end; cursor:pointer; user-select:none; padding: 0 1px;">
                    <div ${activeClass} style="width:100%; height:${heightPct}%; background:${barBg}; border-radius:3px; border:${borderStyle}; transition:all 0.3s;" title="Tag ${day} @ ${Math.round(tempR)}°C: ${vpdVal.toFixed(2)} kPa"></div>
                </div>`;
            }
            container.innerHTML = html;

            let selectEl = document.getElementById('vpd-auto-day-select');
            if (selectEl && document.activeElement !== selectEl) {
                selectEl.value = activeDay;
            }
        }

        function onVpdDaySelectChange(newDayVal) {
            let day = parseInt(newDayVal);
            if (isNaN(day) || day < 1 || day > 14) return;
            let vpdVal = vpdAutoProfileJS[day - 1];
            openVpdDayModal(day, vpdVal.toFixed(2));
        }

        let pendingTargetDay = 1;
        let holdTimer = null;
        let holdStartTime = 0;
        let holdInterval = null;

        function openVpdDayModal(day, vpdVal) {
            pendingTargetDay = day;
            document.getElementById('vpd-day-modal-title').innerText = "Tag " + day + " (Ziel: " + vpdVal + " kPa) aktivieren?";
            document.getElementById('vpd-day-modal-desc').innerHTML = "Möchtest du den Trocknungs-Fortschritt manuell auf <b>Tag " + day + "</b> umstellen?";
            resetHoldButton();
            let modal = document.getElementById('vpd-day-modal');
            if (modal) modal.style.display = 'flex';
        }

        function closeVpdDayModal() {
            resetHoldButton();
            let modal = document.getElementById('vpd-day-modal');
            if (modal) modal.style.display = 'none';
            let selectEl = document.getElementById('vpd-auto-day-select');
            if (selectEl) selectEl.value = currentVpdAutoActiveDay;
        }

        function resetHoldButton() {
            if (holdTimer) clearTimeout(holdTimer);
            if (holdInterval) clearInterval(holdInterval);
            holdTimer = null;
            holdInterval = null;
            let progressEl = document.getElementById('btn-confirm-progress');
            let textEl = document.getElementById('btn-confirm-text');
            if (progressEl) progressEl.style.width = '0%';
            if (textEl) textEl.innerText = "Gedrückt halten...";
        }

        function initHoldButtonListeners() {
            let btn = document.getElementById('btn-confirm-hold');
            if (!btn) return;

            function startHold(e) {
                if (e.cancelable) e.preventDefault();
                resetHoldButton();
                holdStartTime = Date.now();
                let progressEl = document.getElementById('btn-confirm-progress');
                let textEl = document.getElementById('btn-confirm-text');
                
                holdInterval = setInterval(function() {
                    let elapsed = Date.now() - holdStartTime;
                    let pct = Math.min(100, Math.round((elapsed / 2000) * 100));
                    if (progressEl) progressEl.style.width = pct + "%";
                    if (pct < 100) {
                        if (textEl) textEl.innerText = "Halten... " + ((2000 - elapsed)/1000).toFixed(1) + "s";
                    }
                }, 30);

                holdTimer = setTimeout(function() {
                    resetHoldButton();
                    if (textEl) textEl.innerText = "Bestätigt! ✔";
                    setDryStrategy(2, currentHygroLimit, pendingTargetDay);
                    setTimeout(closeVpdDayModal, 400);
                }, 2000);
            }

            function endHold(e) {
                if (holdTimer) {
                    resetHoldButton();
                }
            }

            btn.addEventListener('mousedown', startHold);
            btn.addEventListener('mouseup', endHold);
            btn.addEventListener('mouseleave', endHold);
            btn.addEventListener('touchstart', startHold, {passive: false});
            btn.addEventListener('touchend', endHold);
            btn.addEventListener('touchcancel', endHold);
        }

        let webLogHistoryLocal = [];
        let webLogHistoryRemote = [];
        let localFilterLvl = 3;
        let remoteFilterLvl = 3;

        function setLocalFilter(lvl) {
            localFilterLvl = lvl;
            updateLogHistory('web-log-console', [], webLogHistoryLocal, "[00:00:00] Initializing Local System Console...", localFilterLvl);
        }

        function setRemoteFilter(lvl) {
            remoteFilterLvl = lvl;
            updateLogHistory('web-log-console-remote', [], webLogHistoryRemote, "[00:00:00] Waiting for Remote ESP-NOW Log Stream...", remoteFilterLvl);
        }

        function downloadLogHistory(historyArr, defaultFilename) {
            if (!historyArr || historyArr.length === 0) {
                alert("Keine Log-Einträge zum Exportieren vorhanden.");
                return;
            }
            let devName = (latestData && latestData.device_name) ? latestData.device_name : "IDRY26";
            let dateStr = new Date().toISOString().slice(0,10);
            let finalName = devName + "_" + defaultFilename + "_" + dateStr + ".txt";
            let content = historyArr.join('\r\n');
            let blob = new Blob([content], { type: 'text/plain;charset=utf-8' });
            let url = URL.createObjectURL(blob);
            let a = document.createElement('a');
            a.href = url;
            a.download = finalName;
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            setTimeout(function() { URL.revokeObjectURL(url); }, 1000);
        }

        function updateLogHistory(elementId, incomingLogs, historyArr, defaultMsg, filterLvl) {
            let el = document.getElementById(elementId);
            if (!el) return;

            if (incomingLogs && Array.isArray(incomingLogs)) {
                for (let i = 0; i < incomingLogs.length; i++) {
                    let line = incomingLogs[i];
                    if (line && !historyArr.includes(line)) {
                        historyArr.push(line);
                    }
                }
            }

            if (historyArr.length > 1000) {
                historyArr.splice(0, historyArr.length - 1000);
            }

            let filteredLines = historyArr.filter(line => {
                if (filterLvl >= 3) return true;
                if (line.includes('[DBG ]')) return false;
                return true;
            });

            let textToShow = filteredLines.length > 0 ? filteredLines.join('\n') : (defaultMsg || "");
            let isAtBottom = (el.scrollHeight - el.clientHeight - el.scrollTop) < 40;
            el.innerText = textToShow;
            if (isAtBottom) {
                el.scrollTop = el.scrollHeight;
            }
        }

        function updateData() {
            let savedPass = sessionStorage.getItem('idry_web_pass') || '';
            fetchWithTimeout('/api/data?pass=' + encodeURIComponent(savedPass), { timeout: 1000 })
                .then(response => {
                    if (!response.ok) throw new Error("Connection lost");
                    return response.json();
                })
                .then(data => {
                    latestData = data;

                    let loginCard = document.getElementById('login-card');
                    let settingsLink = document.getElementById('footer-settings-link');
                    let localLogs = document.getElementById('details-logs-local');
                    let remoteLogs = document.getElementById('details-logs-remote');

                    if (data.web_auth_required && !data.web_authenticated) {
                        if (loginCard) loginCard.style.display = 'block';
                        if (settingsLink) settingsLink.style.display = 'none';
                        if (localLogs) localLogs.style.display = 'none';
                        if (remoteLogs) remoteLogs.style.display = 'none';
                    } else {
                        if (loginCard) loginCard.style.display = 'none';
                        if (settingsLink) {
                            settingsLink.style.display = 'inline-flex';
                            settingsLink.href = "/settings?pass=" + encodeURIComponent(savedPass);
                        }
                        let otaBtn = document.getElementById('ota-update-btn');
                        if (otaBtn) {
                            otaBtn.href = "/firmware?pass=" + encodeURIComponent(savedPass);
                        }
                        if (localLogs) localLogs.style.display = 'block';
                        if (remoteLogs) remoteLogs.style.display = 'block';
                        if (savedPass) {
                            document.cookie = "idry_pass=" + encodeURIComponent(savedPass) + "; path=/; max-age=86400";
                        }
                    }

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

                    document.getElementById('sys-ip').innerText = data.ip_address || data.ip || "--";
                    document.getElementById('sys-ip').style.color = "#4ade80";

                    let displayModeText = data.display_mode || data.mode || "e-Paper / TFT Auto-Detect";
                    let modeEl = document.getElementById('sys-mode');
                    if (modeEl) {
                        modeEl.innerText = displayModeText;
                        modeEl.style.color = "#38bdf8";
                    }

                    let rssi = data.rssi !== undefined ? data.rssi : -100;
                    let rssiBar = document.getElementById('sys-rssi-bar');
                    let pct = Math.min(100, Math.max(0, (rssi + 100) * 2));
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

                    // Update potentiometers & Strategy
                    let dryStrat = data.dry_strategy || 0;
                    let hygroLim = data.hygro_limit || 70;
                    let vpdAutoDay = data.vpd_auto_day || 1;
                    currentDryStrategy = dryStrat;
                    currentHygroLimit = hygroLim;

                    let btn6060 = document.getElementById('strat-btn-6060');
                    let btnVpd = document.getElementById('strat-btn-vpd');
                    let btnVpdAuto = document.getElementById('strat-btn-vpd-auto');
                    let btnRemote = document.getElementById('strat-btn-remote');
                    let stratSection = document.getElementById('strat-section');
                    let hlBox = document.getElementById('hygro-limit-box');
                    let vpdAutoBox = document.getElementById('vpd-auto-box');
                    let potiALabel = document.getElementById('poti-a-label');
                    let hl70 = document.getElementById('hl-70');
                    let hl75 = document.getElementById('hl-75');
                    let hl80 = document.getElementById('hl-80');

                    // Check if any temperature/humidity sensor is connected & active
                    let hasAnyTempSensor = false;
                    if (data.sensors && data.sensors.length > 0) {
                        for (let i = 0; i < data.sensors.length; i++) {
                            if (data.sensors[i] && data.sensors[i].temperature !== undefined && data.sensors[i].temperature !== null) {
                                hasAnyTempSensor = true;
                                break;
                            }
                        }
                    }

                    if (data.espnow_role === 2) {
                        let isOnline = (data.is_slave_connected !== undefined) ? data.is_slave_connected : ((data.espnow_last_seen_ms !== -1) && (data.espnow_last_seen_ms <= 5000));
                        if (stratSection) stratSection.style.display = 'block';
                        if (btn6060) btn6060.style.display = 'none';
                        if (btnVpd) btnVpd.style.display = 'none';
                        if (btnVpdAuto) btnVpdAuto.style.display = 'none';
                        if (btnRemote) {
                            btnRemote.style.display = 'block';
                            if (isOnline) {
                                if (dryStrat === 2) {
                                    btnRemote.innerHTML = "<span style='color: #a855f7; font-weight: bold;'>REMOTE</span> VPD AU";
                                } else if (dryStrat === 1) {
                                    btnRemote.innerHTML = "<span style='color: #f87171; font-weight: bold;'>REMOTE</span> VPD";
                                } else {
                                    btnRemote.innerHTML = "<span style='color: #38bdf8; font-weight: bold;'>REMOTE</span> 60/60";
                                }
                            } else {
                                let localStratText = "";
                                if (data.espnow_failsafe_mode === 0) {
                                    localStratText = "50% OPEN";
                                } else {
                                    localStratText = (data.dry_strategy === 2) ? "VPD AU" : ((data.dry_strategy === 1) ? "VPD" : "60/60");
                                }
                                btnRemote.innerHTML = "<span style='color: #f87171; font-weight: bold;'>NOTFALL</span> " + localStratText;
                            }
                        }
                        if (hlBox) hlBox.style.display = 'none';
                        if (vpdAutoBox) vpdAutoBox.style.display = 'none';
                        if (potiALabel) potiALabel.innerText = 'Sollwert Feuchte (A):';

                        let potValA = data.potentiometers.poti_a_target_hum;
                        let displayA = potValA.toFixed(0) + " %";
                        if (potValA <= 49.5) {
                            displayA = "Rigoros ZU";
                        } else if (potValA >= 70.5) {
                            displayA = "Rigoros AUF";
                        }
                        document.getElementById('poti-a').innerText = displayA;
                    } else if (!hasAnyTempSensor) {
                        if (stratSection) stratSection.style.display = 'none';
                        if (hlBox) hlBox.style.display = 'none';
                        if (vpdAutoBox) vpdAutoBox.style.display = 'none';
                        if (potiALabel) potiALabel.innerText = 'Sollwert Feuchte (A):';

                        let potValA = data.potentiometers.poti_a_target_hum;
                        let displayA = potValA.toFixed(0) + " %";
                        if (potValA <= 49.5) {
                            displayA = "Rigoros ZU";
                        } else if (potValA >= 70.5) {
                            displayA = "Rigoros AUF";
                        }
                        document.getElementById('poti-a').innerText = displayA;
                    } else {
                        if (stratSection) stratSection.style.display = 'block';
                        if (btnRemote) btnRemote.style.display = 'none';
                        if (btnVpd) btnVpd.style.display = 'block';
                        if (btn6060) btn6060.style.display = 'block';
                        if (btnVpdAuto) btnVpdAuto.style.display = 'block';

                        if (dryStrat === 2) { // VPD AUTO Mode
                            if (btnVpdAuto) { btnVpdAuto.style.background = '#22c55e'; btnVpdAuto.style.color = '#ffffff'; }
                            if (btnVpd) { btnVpd.style.background = '#1e293b'; btnVpd.style.color = '#94a3b8'; }
                            if (btn6060) { btn6060.style.background = '#1e293b'; btn6060.style.color = '#94a3b8'; }
                            if (hlBox) hlBox.style.display = 'block';
                            if (vpdAutoBox) vpdAutoBox.style.display = 'block';
                            
                            let tempR = data.indoor_temp_rounded || 20;
                            if (potiALabel) potiALabel.innerText = 'AUTO VPD (Tag ' + vpdAutoDay + ' @ ' + tempR + '°C):';
                            if (hygroLim === 80) { if (hl80) hl80.checked = true; }
                            else if (hygroLim === 75) { if (hl75) hl75.checked = true; }
                            else { if (hl70) hl70.checked = true; }

                            let targetVpd = data.potentiometers.target_vpd || 1.00;
                            let effRh = data.potentiometers.effective_target_rh || 60.0;
                            let rawRh = data.potentiometers.raw_calculated_rh !== undefined ? data.potentiometers.raw_calculated_rh : effRh;
                            let displayA = targetVpd.toFixed(2) + " kPa";
                            document.getElementById('poti-a').innerText = displayA;

                            renderVpdAutoTimeline(vpdAutoDay);

                            let calcSollEl = document.getElementById('calc-soll-rh');
                            let calcNoticeEl = document.getElementById('calc-limit-notice');
                            if (calcSollEl) calcSollEl.innerText = rawRh.toFixed(1) + " %";
                            if (calcNoticeEl) {
                                if (rawRh > hygroLim) {
                                    calcNoticeEl.style.display = 'block';
                                    calcNoticeEl.innerText = "(limited to " + hygroLim + "%)";
                                } else {
                                    calcNoticeEl.style.display = 'none';
                                }
                            }
                        } else if (dryStrat === 1) { // VPD Mode
                            if (btnVpd) { btnVpd.style.background = '#22c55e'; btnVpd.style.color = '#ffffff'; }
                            if (btnVpdAuto) { btnVpdAuto.style.background = '#1e293b'; btnVpdAuto.style.color = '#94a3b8'; }
                            if (btn6060) { btn6060.style.background = '#1e293b'; btn6060.style.color = '#94a3b8'; }
                            if (hlBox) hlBox.style.display = 'block';
                            if (vpdAutoBox) vpdAutoBox.style.display = 'none';
                            if (potiALabel) potiALabel.innerText = 'Soll VPD:';
                            if (hygroLim === 80) { if (hl80) hl80.checked = true; }
                            else if (hygroLim === 75) { if (hl75) hl75.checked = true; }
                            else { if (hl70) hl70.checked = true; }

                            let targetVpd = data.potentiometers.target_vpd || 1.00;
                            let effRh = data.potentiometers.effective_target_rh || 60.0;
                            let rawRh = data.potentiometers.raw_calculated_rh !== undefined ? data.potentiometers.raw_calculated_rh : effRh;
                            let displayA = targetVpd.toFixed(2) + " kPa";
                            document.getElementById('poti-a').innerText = displayA;

                            let calcSollEl = document.getElementById('calc-soll-rh');
                            let calcNoticeEl = document.getElementById('calc-limit-notice');
                            if (calcSollEl) calcSollEl.innerText = rawRh.toFixed(1) + " %";
                            if (calcNoticeEl) {
                                if (rawRh > hygroLim) {
                                    calcNoticeEl.style.display = 'block';
                                    calcNoticeEl.innerText = "(limited to " + hygroLim + "%)";
                                } else {
                                    calcNoticeEl.style.display = 'none';
                                }
                            }
                        } else { // 60/60 Mode
                            if (btn6060) { btn6060.style.background = '#22c55e'; btn6060.style.color = '#ffffff'; }
                            if (btnVpd) { btnVpd.style.background = '#1e293b'; btnVpd.style.color = '#94a3b8'; }
                            if (btnVpdAuto) { btnVpdAuto.style.background = '#1e293b'; btnVpdAuto.style.color = '#94a3b8'; }
                            if (hlBox) hlBox.style.display = 'none';
                            if (vpdAutoBox) vpdAutoBox.style.display = 'none';
                            if (potiALabel) potiALabel.innerText = 'Sollwert Feuchte (A):';

                            let potValA = data.potentiometers.poti_a_target_hum;
                            let displayA = potValA.toFixed(0) + " %";
                            if (potValA <= 49.5) {
                                displayA = "Rigoros ZU";
                            } else if (potValA >= 70.5) {
                                displayA = "Rigoros AUF";
                            }
                            document.getElementById('poti-a').innerText = displayA;
                        }
                    }
                    document.getElementById('poti-b').innerText = data.potentiometers.poti_b_gain.toFixed(0) + " %";
                    document.getElementById('poti-c').innerText = data.potentiometers.poti_c_cal_offset.toFixed(0) + " °";

                    // Update Rotor & Servo card status dynamically
                    let rotorHTML = (data.espnow_role === 2 ? "<span style='color: #f87171; font-weight: bold; margin-right: 6px;'>[remote]</span>" : "") + data.rotor_position.toFixed(0) + " %";
                    document.getElementById('rotor-pos').innerHTML = rotorHTML;
                    const m = document.getElementById('luna');
                    if (m) m.style.backgroundColor = '#191b28';
                    setMoon(data.rotor_position, data.espnow_role === 2);

                    // Update Purge Timer & Sanduhr animation (Hidden in Slave role 2)
                    let purgeSection = document.getElementById('purge-section');
                    if (data.espnow_role === 2) {
                        if (purgeSection) purgeSection.style.display = 'none';
                    } else {
                        if (purgeSection) purgeSection.style.display = 'block';

                        if (!isDrumUserScrolling) {
                            setDrumValue('wheel-interval', data.purge_interval_min !== undefined ? data.purge_interval_min : 240);
                            setDrumValue('wheel-duration', data.purge_duration_sec !== undefined ? data.purge_duration_sec : 30);
                        }

                        let purgeBadge = document.getElementById('purge-badge');
                        let sandTop = document.getElementById('sand-top');
                        let sandStream = document.getElementById('sand-stream');
                        let sandBottom = document.getElementById('sand-bottom');

                        if (data.purge_interval_min === 0) {
                            if (purgeBadge) {
                                purgeBadge.innerText = "Aus";
                                purgeBadge.style.color = "#94a3b8";
                                purgeBadge.style.background = "rgba(255,255,255,0.05)";
                                purgeBadge.style.borderColor = "rgba(255,255,255,0.1)";
                            }
                            if (sandTop) sandTop.style.transform = "scale(0)";
                            if (sandBottom) sandBottom.style.transform = "scale(0)";
                            if (sandStream) {
                                sandStream.setAttribute("class", "");
                                sandStream.style.opacity = "0";
                            }
                        } else if (data.purge_active) {
                            // ACTIVE PURGE (100% AUF) -> RED WARN SAND!
                            let remaining = data.purge_remaining_sec !== undefined ? data.purge_remaining_sec : 0;
                            let totalDur = data.purge_duration_sec || 30;
                            let pct = Math.max(0, Math.min(1, remaining / totalDur));
                            
                            if (purgeBadge) {
                                purgeBadge.innerText = "🔥 100% AUF (" + remaining + "s)";
                                purgeBadge.style.color = "#f87171";
                                purgeBadge.style.background = "rgba(248, 113, 113, 0.2)";
                                purgeBadge.style.borderColor = "rgba(248, 113, 113, 0.4)";
                            }
                            if (sandTop) {
                                sandTop.setAttribute("fill", "#f87171");
                                sandTop.style.transform = "scale(" + pct + ")";
                            }
                            if (sandStream) {
                                sandStream.setAttribute("class", "sand-stream-flowing-red");
                                sandStream.style.opacity = "1";
                            }
                            if (sandBottom) {
                                sandBottom.setAttribute("fill", "#f87171");
                                sandBottom.style.transform = "scale(" + (1 - pct) + ")";
                            }
                        } else {
                            // NORMAL COUNTDOWN -> HELLBLAUER SAND!
                            let remaining = data.purge_remaining_sec !== undefined ? data.purge_remaining_sec : 0;
                            let totalIntSec = (data.purge_interval_min || 240) * 60;
                            let pct = Math.max(0, Math.min(1, remaining / totalIntSec));
                            
                            let h = Math.floor(remaining / 3600);
                            let m = Math.floor((remaining % 3600) / 60);
                            let s = remaining % 60;
                            let timeStr = "In " + (h > 0 ? (h + "h " + (m < 10 ? "0" : "") + m + "m") : (m + ":" + (s < 10 ? "0" : "") + s));

                            if (purgeBadge) {
                                purgeBadge.innerText = timeStr;
                                purgeBadge.style.color = "#38bdf8";
                                purgeBadge.style.background = "rgba(56, 189, 248, 0.15)";
                                purgeBadge.style.borderColor = "rgba(56, 189, 248, 0.3)";
                            }
                            if (sandTop) {
                                sandTop.setAttribute("fill", "#38bdf8");
                                sandTop.style.transform = "scale(" + pct + ")";
                            }
                            if (sandStream) {
                                sandStream.setAttribute("class", "sand-stream-flowing-blue");
                                sandStream.style.opacity = "1";
                            }
                            if (sandBottom) {
                                sandBottom.setAttribute("fill", "#38bdf8");
                                sandBottom.style.transform = "scale(" + (1 - pct) + ")";
                            }
                        }
                    }

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
                        } else if (lastSeenMs <= 5000) {
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
                    let showMqtt = data.mqtt_enabled || data.espnow_role > 0 || (data.espnow_peer_mac && data.espnow_peer_mac.length > 0);
                    if (showMqtt) {
                        mqttCard.style.display = 'block';
                        document.getElementById('mqtt-title').innerText = "MQTT " + (data.device_name || "IDRY26");
                        document.getElementById('mqtt-broker').innerText = (data.mqtt_server && data.mqtt_server.length > 0) ? (data.mqtt_server + ":" + data.mqtt_port) : "--";
                        
                        let statusEl = document.getElementById('mqtt-status');
                        if (statusEl) {
                            if (data.mqtt_connected) {
                                statusEl.innerText = "connected";
                                statusEl.style.color = "#4ade80"; // green
                            } else if (data.mqtt_server && data.mqtt_server.length > 0) {
                                statusEl.innerText = "try to connect";
                                statusEl.style.color = "#f87171"; // red
                            } else {
                                statusEl.innerText = "disconnected";
                                statusEl.style.color = "#f87171"; // red
                            }
                        }
                        let topicEl = document.getElementById('mqtt-topic');
                        if (topicEl) {
                            topicEl.innerText = data.mqtt_topic || ("idry/" + (data.device_name || "IDRY26") + "/state");
                        }
                    } else {
                        mqttCard.style.display = 'none';
                    }

                    // Update VPD card status dynamically
                    let vpdCard = document.getElementById('vpd-card');
                    let vpdRow0 = document.getElementById('vpd-row-0');
                    let vpdRow1 = document.getElementById('vpd-row-1');
                    let hasVpd0 = data.sensors && data.sensors[0] && data.sensors[0].vpd !== undefined && data.sensors[0].vpd !== null;
                    let hasVpd1 = data.sensors && data.sensors[1] && data.sensors[1].vpd !== undefined && data.sensors[1].vpd !== null;

                    if (hasVpd0 || hasVpd1) {
                        vpdCard.style.display = 'block';
                        if (hasVpd0) {
                            vpdRow0.style.display = 'block';
                            document.getElementById('vpd-val-0').innerText = data.sensors[0].vpd.toFixed(2) + " kPa";
                        } else { vpdRow0.style.display = 'none'; }
                        if (hasVpd1) {
                            vpdRow1.style.display = 'block';
                            document.getElementById('vpd-val-1').innerText = data.sensors[1].vpd.toFixed(2) + " kPa";
                        } else { vpdRow1.style.display = 'none'; }
                    } else {
                        vpdCard.style.display = 'none';
                    }

                    const benchEl = document.getElementById('footer-bench');
                    if (benchEl) {
                        benchEl.innerText = data.loops_per_sec || 0;
                    }
                    const heapEl = document.getElementById('footer-heap');
                    if (heapEl) {
                        heapEl.innerText = data.free_heap ? (data.free_heap / 1024).toFixed(1) : '--';
                    }
                    const allocEl = document.getElementById('footer-alloc');
                    if (allocEl) {
                        allocEl.innerText = data.max_alloc_heap ? (data.max_alloc_heap / 1024).toFixed(1) : '--';
                    }

                    if (data.sys_logs && Array.isArray(data.sys_logs)) {
                        let hasRemoteReboot = data.sys_logs.some(l => l && l.includes('Remote Reboot command received'));
                        if (hasRemoteReboot) {
                            let rebootModal = document.getElementById('remote-reboot-modal');
                            if (rebootModal && !rebootModal.dataset.shown) {
                                rebootModal.style.display = 'flex';
                                rebootModal.dataset.shown = "true";
                            }
                        }
                    }

                    if (data.is_portal || data.is_factory_reset) {
                        wasInPortalMode = true;
                        let resetModal = document.getElementById('factory-reset-modal');
                        if (resetModal && !resetModal.dataset.closed) {
                            resetModal.style.display = 'flex';
                        }
                    } else if (wasInPortalMode) {
                        wasInPortalMode = false;
                        window.location.reload();
                    }

                    let isSlave = (data.espnow_role === 2);
                    let isMaster = (data.espnow_role === 1);
                    let roleStr = isMaster ? "MASTER" : (isSlave ? "SLAVE" : "STANDALONE");
                    let remoteRoleStr = isMaster ? "SLAVE" : "MASTER";

                    // MASTER DATA = BLUE BOX (#38bdf8, #090d16)
                    // SLAVE DATA  = RED BOX (#f87171, #160909)
                    let localIsBlue = isMaster || !isSlave;
                    let localColor  = localIsBlue ? "#38bdf8" : "#f87171";
                    let localBorder = localIsBlue ? "1px solid rgba(56, 189, 248, 0.5)" : "1px solid rgba(248, 113, 113, 0.5)";
                    let localBg     = localIsBlue ? "#090d16" : "#160909";
                    let localText   = localIsBlue ? "#38bdf8" : "#fca5a5";

                    let lblLocal = document.getElementById('label-log-local');
                    let subLocal = document.querySelector('#details-logs-local summary span:last-child');
                    if (lblLocal) {
                        lblLocal.innerText = "▼ Local Terminal Console (" + roleStr + ")";
                        if (lblLocal.parentElement) lblLocal.parentElement.style.color = localColor;
                    }
                    if (subLocal) subLocal.style.color = localColor;

                    const logEl = document.getElementById('web-log-console');
                    if (logEl) {
                        logEl.style.border = localBorder;
                        logEl.style.background = localBg;
                        logEl.style.color = localText;
                    }

                    // Remote Console is opposite of Local
                    let remoteIsBlue = isSlave; // On Slave UI, Remote is Master -> BLUE BOX! On Master UI, Remote is Slave -> RED BOX!
                    let remoteColor  = remoteIsBlue ? "#38bdf8" : "#f87171";
                    let remoteBorder = remoteIsBlue ? "1px solid rgba(56, 189, 248, 0.5)" : "1px solid rgba(248, 113, 113, 0.5)";
                    let remoteBg     = remoteIsBlue ? "#090d16" : "#160909";
                    let remoteText   = remoteIsBlue ? "#38bdf8" : "#fca5a5";

                    let lblRemote = document.getElementById('label-log-remote');
                    let subRemote = document.querySelector('#details-logs-remote summary span:last-child');
                    let detRemote = document.getElementById('details-logs-remote');
                    if (data.espnow_role > 0) {
                        if (detRemote) detRemote.style.display = 'block';
                        if (lblRemote) {
                            lblRemote.innerText = "▼ Remote " + remoteRoleStr + " Terminal Console [ESP-NOW]";
                            if (lblRemote.parentElement) lblRemote.parentElement.style.color = remoteColor;
                        }
                        if (subRemote) subRemote.style.color = remoteColor;
                    } else {
                        if (detRemote) detRemote.style.display = 'none';
                    }

                    const logElRemote = document.getElementById('web-log-console-remote');
                    if (logElRemote) {
                        logElRemote.style.border = remoteBorder;
                        logElRemote.style.background = remoteBg;
                        logElRemote.style.color = remoteText;
                    }

                    updateLogHistory('web-log-console', data.sys_logs, webLogHistoryLocal, "[00:00:00] Initializing Local System Console...", localFilterLvl);
                    updateLogHistory('web-log-console-remote', data.remote_sys_logs, webLogHistoryRemote, "[00:00:00] Waiting for Remote " + remoteRoleStr + " ESP-NOW Log Stream...", remoteFilterLvl);

                    // Smart Live-Advisor & Heuristic Evaluation
                    evaluateGrowerHeuristics(data);
                })
                .catch(err => {
                    // Connection lost to ESP32
                    document.getElementById('sys-ip').innerText = "try to reconnect to: [" + wifiSSID + "]";
                    document.getElementById('sys-ip').style.color = "#f87171";
                    document.getElementById('sys-rssi').innerText = "OFFLINE";
                    document.getElementById('sys-rssi').style.color = "#f87171";
                    const rssiBar = document.getElementById('sys-rssi-bar');
                    if (rssiBar) {
                        rssiBar.style.width = "100%";
                        rssiBar.style.backgroundColor = "#f87171";
                    }
                    
                    let espnowConn = document.getElementById('espnow-val-conn');
                    if (espnowConn) {
                        espnowConn.innerText = "Offline (Reconnecting)";
                        espnowConn.style.color = "#f87171";
                    }

                    let btnRemote = document.getElementById('strat-btn-remote');
                    if (btnRemote) {
                        btnRemote.style.display = 'block';
                        btnRemote.innerHTML = "<span style='color: #f87171; font-weight: bold;'>NOTFALL</span> LINK LOST";
                    }

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

        function calculateVPD_JS(temp, hum) {
            if (temp === null || temp === undefined || isNaN(temp) || hum === null || hum === undefined || isNaN(hum) || hum <= 0) return null;
            let svp = 0.61078 * Math.exp((17.27 * temp) / (temp + 237.3));
            let avp = svp * (hum / 100);
            let vpd = svp - avp;
            return (vpd < 0) ? 0 : vpd;
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
            renderCardChart('details-vpd-0', 'cv-vpd-0', 'vpd', 0);
            renderCardChart('details-vpd-1', 'cv-vpd-1', 'vpd', 1);
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
            const isRssi = (type === 'rssi');
            const targetHist = (isRssi && history24h && history24h.length > 0) ? history24h : history120m;
            if (!targetHist || targetHist.length === 0) return;

            const count = isRssi ? 24 : 60;
            const data120m = targetHist.slice(-count);

            let minY = 0, maxY = 100, midY = 50;
            let labelMax = "100", labelMid = "50", labelMin = "0";
            let greenLineVal = null;
            let lineStyleColor = '#22c55e'; // Green default for Temp & RH

            if (type === 'temp') {
                maxY = 50; minY = 0; midY = 25;
                labelMax = "50"; labelMid = "25"; labelMin = "0";
                greenLineVal = 25;
            } else if (type === 'hum') {
                maxY = 100; minY = 0; midY = 50;
                labelMax = "100"; labelMid = "50"; labelMin = "0";
                greenLineVal = 50;
            } else if (type === 'vpd') {
                maxY = 3.0; minY = 0.0; midY = 1.5;
                labelMax = "3.0"; labelMid = "1.5"; labelMin = "0.0";
                lineStyleColor = '#facc15'; // Bright Yellow for VPD target baseline line!
                
                let strat = (latestData && latestData.dry_strategy !== undefined) ? latestData.dry_strategy : currentDryStrategy;
                if (strat === 2 || strat === 1) {
                    // VPD AUTO & VPD Modes: Baseline line dynamically tracks temperature-compensated target VPD
                    greenLineVal = (latestData && latestData.potentiometers && latestData.potentiometers.target_vpd !== undefined)
                        ? latestData.potentiometers.target_vpd
                        : ((latestData && latestData.vpd_auto_target_vpd !== undefined) ? latestData.vpd_auto_target_vpd : 0.85);
                } else {
                    // 60/60 Mode: Target VPD baseline not applicable (hidden)
                    greenLineVal = null;
                }
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

            let strat = (latestData && latestData.dry_strategy !== undefined) ? latestData.dry_strategy : currentDryStrategy;
            if (type === 'vpd' && strat === 2 && latestData && latestData.indoor_temp_rounded) {
                ctx.fillStyle = '#facc15';
                ctx.font = `bold ${9 * dpr}px sans-serif`;
                ctx.textAlign = 'right';
                ctx.textBaseline = 'top';
                ctx.fillText(`${latestData.indoor_temp_rounded}°C Profil`, w - 4 * dpr, 2 * dpr);
                ctx.textAlign = 'left'; // reset
            }

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
                } else if (type === 'vpd') {
                    let temp = (index === 0 ? d.t0 : d.t1);
                    let hum = (index === 0 ? d.h0 : d.h1);
                    valMax = calculateVPD_JS(temp, hum);
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
                    } else if (type === 'vpd') {
                        const vpdGrad = ctx.createLinearGradient(0, chartH, 0, 0);
                        vpdGrad.addColorStop(0.00, '#ef4444'); // Rot (0.0 kPa - Zu feucht / Schimmel)
                        vpdGrad.addColorStop(0.13, '#eab308'); // Gelb (0.4 kPa)
                        vpdGrad.addColorStop(0.20, '#22c55e'); // Grün Start (0.6 kPa)
                        vpdGrad.addColorStop(0.33, '#22c55e'); // Grün Ende (1.0 kPa - Perfect Curing)
                        vpdGrad.addColorStop(0.50, '#eab308'); // Gelb (1.5 kPa)
                        vpdGrad.addColorStop(0.67, '#ef4444'); // Rot (>= 2.0 kPa - Zu trocken / Übertrocknung)
                        ctx.fillStyle = vpdGrad;
                    } else {
                        ctx.fillStyle = '#38bdf8';
                    }
                    ctx.fillRect(x1, y, barW, valH);
                }
            }

            if (greenLineVal !== null) {
                const greenY = chartH - ((greenLineVal - minY) / (maxY - minY)) * chartH;
                ctx.strokeStyle = lineStyleColor;
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
            if (type.startsWith('vpd_')) { index = parseInt(type.split('_')[1]); type = 'vpd'; }
            if (type.startsWith('lux_')) { index = parseInt(type.split('_')[1]); type = 'lux'; }

            let minY = 0, maxY = 100, labelMax = "100", labelMid = "50", labelMin = "0", greenLineVal = null;
            let lineStyleColor = '#22c55e'; // Green default for Temp & RH

            if (type === 'temp') { maxY = 50; labelMax = "50"; labelMid = "25"; labelMin = "0"; greenLineVal = 25; }
            else if (type === 'hum') { maxY = 100; labelMax = "100"; labelMid = "50"; labelMin = "0"; greenLineVal = 50; }
            else if (type === 'vpd') {
                maxY = 3.0; labelMax = "3.0"; labelMid = "1.5"; labelMin = "0.0";
                lineStyleColor = '#facc15'; // Bright Yellow for VPD target baseline line!
                let strat = (latestData && latestData.dry_strategy !== undefined) ? latestData.dry_strategy : currentDryStrategy;
                if (strat === 2 || strat === 1) {
                    greenLineVal = (latestData && latestData.potentiometers && latestData.potentiometers.target_vpd !== undefined)
                        ? latestData.potentiometers.target_vpd
                        : ((latestData && latestData.vpd_auto_target_vpd !== undefined) ? latestData.vpd_auto_target_vpd : 0.85);
                } else {
                    greenLineVal = null;
                }
            }
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
                } else if (type === 'vpd') {
                    let temp = (index === 0 ? d.t0 : d.t1);
                    let hum = (index === 0 ? d.h0 : d.h1);
                    valMax = calculateVPD_JS(temp, hum);
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
                    } else if (type === 'vpd') {
                        const vpdGrad = ctx.createLinearGradient(0, chartH, 0, 0);
                        vpdGrad.addColorStop(0.00, '#ef4444'); // Rot (0.0 kPa - Zu feucht / Schimmel)
                        vpdGrad.addColorStop(0.13, '#eab308'); // Gelb (0.4 kPa)
                        vpdGrad.addColorStop(0.20, '#22c55e'); // Grün Start (0.6 kPa)
                        vpdGrad.addColorStop(0.33, '#22c55e'); // Grün Ende (1.0 kPa - Perfect Curing)
                        vpdGrad.addColorStop(0.50, '#eab308'); // Gelb (1.5 kPa)
                        vpdGrad.addColorStop(0.67, '#ef4444'); // Rot (>= 2.0 kPa - Zu trocken / Übertrocknung)
                        ctx.fillStyle = vpdGrad;
                    } else {
                        ctx.fillStyle = '#38bdf8';
                    }
                    ctx.fillRect(x1, y, barW, valH);
                }
            }

            if (greenLineVal !== null) {
                const greenY = chartH - ((greenLineVal - minY) / (maxY - minY)) * chartH;
                ctx.strokeStyle = lineStyleColor;
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
                    if (type.startsWith('vpd_')) { index = parseInt(type.split('_')[1]); type = 'vpd'; }
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
                    } else if (type === 'vpd') {
                        let tempMax = (index === 0 ? d.t0 : d.t1);
                        let humMin = (index === 0 ? d.h0_min : d.h1_min);
                        let tempMin = (index === 0 ? d.t0_min : d.t1_min);
                        let humMax = (index === 0 ? d.h0 : d.h1);
                        valMax = calculateVPD_JS(tempMax, humMin);
                        valMin = calculateVPD_JS(tempMin, humMax);
                        if (valMax === null && tempMax !== null && humMax !== null) {
                            valMax = calculateVPD_JS(tempMax, humMax);
                        }
                        if (valMin === null) valMin = valMax;
                        maxY = 3.0; unit = " kPa";
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

                    let badgeBorderColor = (type === 'espnow' || type === 'mqtt') ? "#f87171" : (type === 'rssi' ? "#22c55e" : (type === 'vpd' ? "#22c55e" : "#38bdf8"));
                    let popupTxt = "";

                    if (type === 'temp' || type === 'hum') {
                        popupTxt = `Min: ${valMin.toFixed(1)}${unit} (Max: ${valMax.toFixed(1)}${unit}) [${timeStr}]`;
                    } else if (type === 'vpd') {
                        popupTxt = `VPD: ${valMax.toFixed(2)}${unit} [${timeStr}]`;
                    } else {
                        popupTxt = `${valMax.toFixed(1)}${unit} [${timeStr}]`;
                    }

                    if (popup) {
                        const cssTargetX = (marginL + (candleIndex + 0.5) * candleW) / dpr;
                        const cssTargetY = (yTop / dpr) - 15;

                        popup.style.display = 'block';
                        popup.style.left = cssTargetX + 'px';
                        popup.style.top = cssTargetY + 'px';
                        popup.style.borderColor = badgeBorderColor;
                        popup.innerText = popupTxt;
                    }

                    if (tooltipEl) {
                        if (type === 'temp' || type === 'hum' || type === 'vpd') {
                            let delta = Math.abs(valMax - valMin).toFixed(type === 'vpd' ? 2 : 1);
                            let minStr = valMin !== null ? valMin.toFixed(type === 'vpd' ? 2 : 1) : "--";
                            let maxStr = valMax !== null ? valMax.toFixed(type === 'vpd' ? 2 : 1) : "--";
                            tooltipEl.innerHTML = `<div style="color:#38bdf8; font-weight:600;">${type === 'vpd' ? 'VPD Min' : 'Min'}: ${minStr}${unit} [${timeStr}]</div>` +
                                                 `<div style="color:#facc15; font-weight:600; margin-top:2px;">${type === 'vpd' ? 'VPD Max' : 'Max'}: ${maxStr}${unit} (Delta: +${delta}${unit})</div>`;
                        } else {
                            tooltipEl.innerHTML = `<div style="color:${badgeBorderColor}; font-weight:600;">${valMax.toFixed(1)}${unit} [${timeStr}]</div>`;
                        }
                    }
                }
            }

            modalCanvas.addEventListener('pointerdown', handleModalPointer);
            modalCanvas.addEventListener('pointermove', handleModalPointer);
        }

        setInterval(updateData, 1000);
        setInterval(fetchHistory, 1000);
        initHoldButtonListeners();
        initDrumPickers();
        renderAdvisorMsg(0);
        updateData();
        fetchHistory();
    </script>
</body>
</html>
)rawhtml";
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");
    server.sendContent(DASHBOARD_HTML_PART1);
    server.sendContent(pageTitle);
    server.sendContent(DASHBOARD_HTML_PART2);
    server.sendContent(String(sysConfig.wifi_ssid));
    server.sendContent(DASHBOARD_HTML_PART3);
    server.sendContent("");
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
  if (!isWebAuthenticated()) {
    server.send(401, "text/html",
                "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Zugriff geschützt</title>"
                "<style>body{background:#0f172a;color:white;text-align:center;padding-top:100px;font-family:sans-serif;}</style></head>"
                "<body><div style='background:#1e293b;padding:30px;border-radius:15px;display:inline-block;'>"
                "<h1 style='color:#f87171;margin-bottom:15px;'>🔒 Webinterface geschützt</h1>"
                "<p style='color:#cbd5e1;margin-bottom:20px;'>Für den Zugriff auf die Einstellungen ist eine Anmeldung im Dashboard erforderlich.</p>"
                "<a href='/' style='color:#38bdf8;'>Zurück zum Dashboard</a></div></body></html>");
    return;
  }
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
            justify-content: space-between;
            position: relative;
        }
        .info-btn {
            display: inline-flex;
            align-items: center;
            justify-content: center;
            width: 15px;
            height: 15px;
            border-radius: 50%;
            background: rgba(56, 189, 248, 0.12);
            border: 1px solid rgba(56, 189, 248, 0.35);
            color: #38bdf8;
            font-size: 11px;
            font-family: serif;
            font-style: italic;
            font-weight: bold;
            cursor: pointer;
            user-select: none;
            line-height: 1;
            transition: all 0.2s ease;
            flex-shrink: 0;
            margin-left: auto;
        }
        .info-btn:hover, .info-btn.active {
            background: rgba(56, 189, 248, 0.3);
            border-color: #38bdf8;
            color: #ffffff;
            box-shadow: 0 0 8px rgba(56, 189, 248, 0.6);
        }
        .info-bubble {
            position: absolute;
            top: calc(100% + 6px);
            right: 0;
            width: 280px;
            max-width: 85vw;
            background: #090d16;
            border: 1px solid rgba(56, 189, 248, 0.6);
            border-radius: 12px;
            padding: 12px 14px;
            color: #e2e8f0;
            font-size: 12px;
            font-weight: normal;
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            text-transform: none;
            letter-spacing: normal;
            line-height: 1.5;
            box-shadow: 0 20px 45px -5px rgba(0, 0, 0, 0.95), 0 0 20px rgba(56, 189, 248, 0.3);
            z-index: 9999;
            pointer-events: auto;
            animation: info-fade-in 0.2s ease-out;
        }
        .info-bubble::before {
            content: '';
            position: absolute;
            top: -8.5px;
            right: 4px;
            width: 0;
            height: 0;
            border-left: 7px solid transparent;
            border-right: 7px solid transparent;
            border-bottom: 8.5px solid rgba(56, 189, 248, 0.6);
        }
        .info-bubble::after {
            content: '';
            position: absolute;
            top: -7px;
            right: 5px;
            width: 0;
            height: 0;
            border-left: 6px solid transparent;
            border-right: 6px solid transparent;
            border-bottom: 7px solid #090d16;
        }
        @keyframes info-fade-in {
            from { opacity: 0; transform: translateY(-4px); }
            to { opacity: 1; transform: translateY(0); }
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
        @keyframes pulse-update-border {
            0%, 100% { border-color: rgba(129, 140, 248, 0.4); box-shadow: 0 0 0 0 rgba(239, 68, 68, 0); color: #cbd5e1; }
            50% { border-color: #ef4444 !important; color: #f87171 !important; box-shadow: 0 0 14px rgba(239, 68, 68, 0.9); }
        }
        .pulse-update {
            animation: pulse-update-border 1s infinite ease-in-out !important;
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
                    <span style="display: flex; align-items: center; gap: 8px;"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M5 12.55a11 11 0 0 1 14.08 0"></path><path d="M1.42 9a16 16 0 0 1 21.16 0"></path><path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path><circle cx="12" cy="20" r="1"></circle></svg> WLAN Verbindung</span>
                    <span class="info-btn" onclick="toggleInfo(event, 13)" onmouseenter="showInfo(this, 13)" onmouseleave="hideInfo(this)">i</span>
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
                <div class="form-group">
                    <label for="web_password">Webinterface Passwort (Optional)</label>
                    <input type="password" name="web_password" id="web_password" value=")rawhtml";
  if (strlen(sysConfig.web_password) > 0) {
    html += "********";
  }
  html += R"rawhtml(" placeholder="passwort eintragen">
                    <span class="hint-text">Freilassen für freien Lesezugriff. Sobald ein Passwort eingetragen ist, schützt es Konsolen &amp; Einstellungen.</span>
                </div>
            </div>

            <!-- MQTT Einstellungen Panel -->
            <div class="settings-card">
                <div class="section-title">
                    <span style="display: flex; align-items: center; gap: 8px;"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect x="3" y="3" width="18" height="18" rx="2" ry="2"></rect><line x1="9" y1="3" x2="9" y2="21"></line></svg> MQTT Konfiguration</span>
                    <span class="info-btn" onclick="toggleInfo(event, 14)" onmouseenter="showInfo(this, 14)" onmouseleave="hideInfo(this)">i</span>
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
                    <span style="display: flex; align-items: center; gap: 8px;"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M18 8A6 6 0 0 0 6 8c0 7-3 9-3 9h18s-3-2-3-9"></path><path d="M13.73 21a2 2 0 0 1-3.46 0"></path></svg> ESPNOW &nbsp;<span id="espnow-local-mac" style="font-family: monospace; text-transform: none; color: #94a3b8;">[Laden...]</span></span>
                    <span class="info-btn" onclick="toggleInfo(event, 15)" onmouseenter="showInfo(this, 15)" onmouseleave="hideInfo(this)">i</span>
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
                    <span style="display: flex; align-items: center; gap: 8px;"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M11 5L6 9H2v6h4l5 4V5z"></path><path d="M19.07 4.93a10 10 0 0 1 0 14.14M15.54 8.46a5 5 0 0 1 0 7.07"></path></svg> Buzzer Test</span>
                    <span class="info-btn" onclick="toggleInfo(event, 16)" onmouseenter="showInfo(this, 16)" onmouseleave="hideInfo(this)">i</span>
                </div>
                <div class="btn-row" style="margin-top: 5px;">
                    <button type="button" onclick="testBuzzer('local')" class="btn btn-secondary">Lokal abspielen</button>
                    <button type="button" id="remote-buzz-btn" onclick="testBuzzer('remote')" class="btn btn-secondary" style="display: none;">Remote abspielen</button>
                </div>
            </div>

            <!-- Servo Laufleistung & Odometer Panel -->
            <div class="settings-card">
                <div class="section-title">
                    <span style="display: flex; align-items: center; gap: 8px;">
                        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="10"></circle><polyline points="12 6 12 12 16 14"></polyline></svg>
                        Servo Laufleistung &amp; Odometer
                    </span>
                    <span class="info-btn" onclick="toggleInfo(event, 21)" onmouseenter="showInfo(this, 21)" onmouseleave="hideInfo(this)">i</span>
                </div>
                <div class="value-row" style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; font-size: 13px;">
                    <span>Gesamtweg:</span>
                    <div style="display: flex; align-items: center; gap: 8px;">
                        <button type="button" id="odo-save-btn" onclick="submitOdometerChange()" style="display:none; padding: 3px 8px; background: #0284c7; border: 1px solid #38bdf8; color: white; border-radius: 6px; font-size: 11px; font-weight: bold; cursor: pointer;">Ändern</button>
                        <input type="number" step="0.01" min="0" id="odo-input" onfocus="pauseOdoUpdate(true)" oninput="onOdoInputChanged()" onblur="setTimeout(() => pauseOdoUpdate(false), 5000)" style="width: 110px; padding: 4px 8px; background: rgba(15,23,42,0.8); border: 1px solid rgba(255,255,255,0.15); border-radius: 6px; color: #38bdf8; font-family: monospace; font-size: 13px; font-weight: 600; text-align: right; outline: none;">
                        <span id="odo-unit" style="font-family: monospace; color: #94a3b8; font-size: 12px;">m</span>
                    </div>
                </div>
                <div style="margin-bottom: 8px;">
                    <div style="display: flex; justify-content: space-between; font-size: 11px; color: #94a3b8; margin-bottom: 4px;">
                        <span>Lebensdauer-Status (50 km Basis):</span>
                        <span id="odo-pct-label" style="font-family: monospace; color: #38bdf8; font-weight: bold;">0.00 %</span>
                    </div>
                    <div style="width: 100%; height: 10px; background: rgba(15,23,42,0.8); border-radius: 5px; border: 1px solid rgba(255,255,255,0.08); overflow: hidden;">
                        <div id="odo-bar" style="width: 0%; height: 100%; background: linear-gradient(90deg, #0284c7 0%, #38bdf8 100%); transition: width 0.4s ease;"></div>
                    </div>
                </div>
                <span class="hint-text" style="font-size: 11px; color: #64748b; margin-top: 6px; display: block;">
                    Berechnung: Hebelarm r=27mm (0.471 mm/&deg;) &bull; Nenn-Lebensdauer: 50.000 m (Dual-Storage Flash &amp; NVS)
                </span>
            </div>

            <!-- System Status Panel -->
            <div class="settings-card">
                <div class="section-title">
                    <span style="display: flex; align-items: center; gap: 8px;"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect x="2" y="3" width="20" height="14" rx="2" ry="2"></rect><line x1="8" y1="21" x2="16" y2="21"></line><line x1="12" y1="17" x2="12" y2="21"></line></svg> System Status</span>
                    <span class="info-btn" onclick="toggleInfo(event, 18)" onmouseenter="showInfo(this, 18)" onmouseleave="hideInfo(this)">i</span>
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

            <!-- Systemeinstellungen Panel -->
            <div class="settings-card">
                <div class="section-title">
                    <span style="display: flex; align-items: center; gap: 8px;"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect x="2" y="3" width="20" height="14" rx="2" ry="2"></rect><line x1="8" y1="21" x2="16" y2="21"></line><line x1="12" y1="17" x2="12" y2="21"></line></svg> System & Anzeige</span>
                    <span class="info-btn" onclick="toggleInfo(event, 17)" onmouseenter="showInfo(this, 17)" onmouseleave="hideInfo(this)">i</span>
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

        <!-- Geräte-Management Panel -->
        <div class="settings-card" style="margin-top: 25px;">
            <div class="section-title" style="color: #f87171;">
                <span style="display: flex; align-items: center; gap: 8px;"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"></path><line x1="12" y1="9" x2="12" y2="13"></line><line x1="12" y1="17" x2="12.01" y2="17"></line></svg> Geräte-Management</span>
                <span class="info-btn" onclick="toggleInfo(event, 19)" onmouseenter="showInfo(this, 19)" onmouseleave="hideInfo(this)">i</span>
            </div>
            <form id="reset-form" action="/settings/reset" method="POST">
                <div class="btn-row" style="margin-top: 5px; flex-direction: column; gap: 12px;">
                    <a href="/firmware" id="ota-update-btn" class="btn btn-secondary" style="width:100%; border-color: rgba(129, 140, 248, 0.4); color: #cbd5e1; text-decoration: none; text-align: center; display: block; box-sizing: border-box;">Firmware & OTA Update</a>
                    <button type="submit" name="action" value="reboot" class="btn btn-secondary" style="width:100%; border-color: rgba(74, 222, 128, 0.4); color: #4ade80;">Reboot Device</button>
                    <button type="submit" name="action" value="reboot_linked" id="reboot-linked-btn" class="btn btn-secondary" style="width:100%; border-color: rgba(74, 222, 128, 0.4); color: #4ade80; display: )rawhtml";
  if (strlen(sysConfig.espnow_peer_mac) > 0) {
    html += "inline-flex;";
  } else {
    html += "none;";
  }
  html += R"rawhtml( align-items: center; justify-content: center; gap: 8px;">
                        Reboot linked Device
                        <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M10 13a5 5 0 0 0 7.54.54l3-3a5 5 0 0 0-7.07-7.07l-1.72 1.71"></path><path d="M14 11a5 5 0 0 0-7.54-.54l-3 3a5 5 0 0 0 7.07 7.07l1.71-1.71"></path></svg>
                    </button>
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
        const PANEL_INFOS = {
            // [13] WLAN Verbindung
            13: "<b>Wi-Fi Verbindung</b><br>Konfiguration des lokalen WLAN-Netzwerks (SSID, Passwort), Web-Passwortschutz und RF-Sendeleistung.",
            
            // [14] MQTT Konfiguration
            14: "<b>MQTT Konfiguration</b><br>Parameter für den MQTT-Broker (Server, Port, Anmeldedaten, Gerätename und Sendeintervall).",
            
            // [15] ESPNOW Einstellungen
            15: "<b>ESP-NOW Funknetzwerk</b><br>Rollenkonfiguration (Master/Slave), Funkkanal, Pairing-Steuerung und Verbindungsverlust-Verhalten.",
            
            // [16] Buzzer Test
            16: "<b>Buzzer Test</b><br>Akustischer Funktionstest des Onboard-Piezo-Lautsprechers (lokal und über Funk auf gekoppeltem Slave).",
            
            // [17] System & Anzeige
            17: "<b>System & Anzeige</b><br>Display-Helligkeit mit Gamma-2.2-Dimmung, Servo-Update-Intervall und WLAN-Verbindungswatchdog.",
            
            // [18] System Status
            18: "<b>System Status</b><br>Diagnoseübersicht mit aktueller IP-Adresse, Display-Modus, RSSI-Signalstärke und automatischem Wochen-Reboot.",
            
            // [19] Geräte-Management
            19: "<b>Geräte-Management</b><br>Firmware & OTA-Update, Geräte-Neustart, Werkseinstellungen und vollständiger System-Reset.",
            
            // [21] Servo Laufleistung & Odometer
            21: "<b>Servo Laufleistung &amp; Odometer</b><br>Präziser Wegstreckenzähler für den Lüftungsrotor (Hebelarm r=27mm). Berechnet kumulierte Fahrstrecke und Lebensdauer (100% = 50 km). Gesichert über Dual-Storage (LittleFS + NVS)."
        };

        let activeBubble = null;
        let activeBubbleBtn = null;

        function showInfo(btn, idx) {
            hideInfo();
            if (!PANEL_INFOS[idx]) return;
            const bubble = document.createElement('div');
            bubble.className = 'info-bubble';
            bubble.innerHTML = PANEL_INFOS[idx];
            btn.parentElement.appendChild(bubble);
            btn.classList.add('active');
            activeBubble = bubble;
            activeBubbleBtn = btn;
        }

        function hideInfo() {
            if (activeBubble) {
                if (activeBubble.parentElement) activeBubble.parentElement.removeChild(activeBubble);
                activeBubble = null;
            }
            if (activeBubbleBtn) {
                activeBubbleBtn.classList.remove('active');
                activeBubbleBtn = null;
            }
        }

        function toggleInfo(evt, idx) {
            evt.stopPropagation();
            if (activeBubbleBtn === evt.currentTarget) {
                hideInfo();
            } else {
                showInfo(evt.currentTarget, idx);
            }
        }

        document.addEventListener('click', function(e) {
            if (activeBubble && !activeBubble.contains(e.target) && e.target !== activeBubbleBtn) {
                hideInfo();
            }
        });

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
                    const rebootLinkedBtn = document.getElementById('reboot-linked-btn');
                    if (rebootLinkedBtn) {
                        if (peerMac.length > 0) {
                            rebootLinkedBtn.style.display = "inline-flex";
                        } else {
                            rebootLinkedBtn.style.display = "none";
                        }
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
                    const otaBtn = document.getElementById('ota-update-btn');
                    if (otaBtn) {
                        if (data.update_available) {
                            otaBtn.classList.add('pulse-update');
                        } else {
                            otaBtn.classList.remove('pulse-update');
                        }
                    }

                    // Update Servo Odometer card
                    const odoInput = document.getElementById('odo-input');
                    const odoBar = document.getElementById('odo-bar');
                    const odoPctLabel = document.getElementById('odo-pct-label');
                    if (odoInput && !isOdoEditing && document.activeElement !== odoInput) {
                        const meters = (data.servo_total_meters !== undefined) ? data.servo_total_meters : 0;
                        odoInput.value = Number(meters).toFixed(2);
                        const pct = (data.servo_lifetime_pct !== undefined) ? data.servo_lifetime_pct : ((meters / 50000.0) * 100.0);
                        if (odoBar) odoBar.style.width = Math.min(100, Math.max(0, pct)).toFixed(2) + "%";
                        if (odoPctLabel) odoPctLabel.innerText = Number(pct).toFixed(2) + " %";
                    }
                }).catch(err => console.error(err));
        }

        let isOdoEditing = false;
        let odoPauseTimeout = null;

        function pauseOdoUpdate(isFocus) {
            if (isFocus) {
                isOdoEditing = true;
                if (odoPauseTimeout) clearTimeout(odoPauseTimeout);
            } else {
                odoPauseTimeout = setTimeout(() => {
                    isOdoEditing = false;
                    const saveBtn = document.getElementById('odo-save-btn');
                    if (saveBtn) saveBtn.style.display = 'none';
                }, 3000);
            }
        }

        function onOdoInputChanged() {
            isOdoEditing = true;
            const saveBtn = document.getElementById('odo-save-btn');
            if (saveBtn) saveBtn.style.display = 'inline-block';

            const odoInput = document.getElementById('odo-input');
            const odoBar = document.getElementById('odo-bar');
            const odoPctLabel = document.getElementById('odo-pct-label');
            if (odoInput) {
                let val = parseFloat(odoInput.value) || 0;
                let pct = (val / 50000.0) * 100.0;
                if (pct < 0) pct = 0;
                if (pct > 100) pct = 100;
                if (odoBar) odoBar.style.width = pct.toFixed(2) + "%";
                if (odoPctLabel) odoPctLabel.innerText = pct.toFixed(2) + " %";
            }
        }

        function submitOdometerChange() {
            const odoInput = document.getElementById('odo-input');
            if (!odoInput) return;
            const newMeters = parseFloat(odoInput.value) || 0;
            
            fetch('/api/settings/odometer?meters=' + encodeURIComponent(newMeters), { method: 'POST' })
                .then(r => r.json())
                .then(res => {
                    if (res.status === 'ok') {
                        const saveBtn = document.getElementById('odo-save-btn');
                        if (saveBtn) saveBtn.style.display = 'none';
                        isOdoEditing = false;
                        odoInput.blur();

                        let m = (res.meters !== undefined) ? res.meters : newMeters;
                        odoInput.value = Number(m).toFixed(2);
                        let pct = (m / 50000.0) * 100.0;
                        if (pct < 0) pct = 0;
                        if (pct > 100) pct = 100;
                        const odoBar = document.getElementById('odo-bar');
                        const odoPctLabel = document.getElementById('odo-pct-label');
                        if (odoBar) odoBar.style.width = pct.toFixed(2) + "%";
                        if (odoPctLabel) odoPctLabel.innerText = pct.toFixed(2) + " %";

                        pollEspNowStatus();
                    } else {
                        alert("Fehler beim Speichern: " + (res.message || "Nicht autorisiert"));
                    }
                })
                .catch(err => alert("Netzwerkfehler: " + err));
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
  String web_pass = server.arg("web_password");
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

  // Handle web_password masked input
  String targetWebPass = String(sysConfig.web_password);
  if (web_pass != "********") {
    targetWebPass = web_pass;
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
       strcmp(sysConfig.web_password, targetWebPass.c_str()) != 0 ||
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
    strlcpy(sysConfig.web_password, targetWebPass.c_str(), sizeof(sysConfig.web_password));
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
    addAppLogEx(1, "[Config] Settings Saved: SSID='%s', Pass='%s', MQTT='%s:%d' (DevName: '%s', RptInt: %dmin), Brightness: %d%%, TXPower: %d, ServoUpdInt: %ds, WLANTimeTrap: %ds, ESP-NOW Role: %d, Channel: %d, PeerMAC: '%s', Failsafe: %d",
                sysConfig.wifi_ssid, sysConfig.wifi_pass, sysConfig.mqtt_server, sysConfig.mqtt_port,
                sysConfig.mqtt_device_name, sysConfig.mqtt_report_interval, sysConfig.display_brightness,
                sysConfig.wifi_tx_power, sysConfig.servo_update_interval, sysConfig.wlan_time_trap,
                sysConfig.espnow_role, sysConfig.espnow_channel, sysConfig.espnow_peer_mac, sysConfig.espnow_failsafe_mode);
  } else {
    addAppLogEx(3, "[Config] Save requested, but no changes detected.");
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
    addAppLogEx(1, "[Display] TFT Backlight brightness set to %d%%", sysConfig.display_brightness);
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
    addAppLogEx(1, "[System] Manual Reboot requested via Web UI! Rebooting...");
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
    delay(500);
    ESP.restart();
  } else if (action == "reboot_linked") {
    if (strlen(sysConfig.espnow_peer_mac) == 0) {
      server.send(400, "text/html",
                  "<html><body><h1>Kein gekoppeltes Geraet vorhanden!</h1><p><a href='/settings'>Zurueck</a></p></body></html>");
      return;
    }

    EspNowMessage msg;
    memset(&msg, 0, sizeof(EspNowMessage));
    msg.pv = localProtocolVersion;
    msg.type = 2; // Data/Command
    strlcpy(msg.key, sysConfig.espnow_lmk, sizeof(msg.key));
    msg.command = 99; // Remote Reboot Command
    msg.value = 0.0f;
    msg.dry_strategy = sysConfig.dry_strategy;

    uint8_t peerMac[6];
    sscanf(sysConfig.espnow_peer_mac, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
           &peerMac[0], &peerMac[1], &peerMac[2], &peerMac[3], &peerMac[4], &peerMac[5]);

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, peerMac, 6);
    peerInfo.channel = (sysConfig.espnow_channel > 0) ? sysConfig.espnow_channel : 1;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;
    if (esp_now_is_peer_exist(peerMac)) {
      esp_now_mod_peer(&peerInfo);
    } else {
      esp_now_add_peer(&peerInfo);
    }

    esp_now_send(peerMac, (uint8_t *)&msg, sizeof(EspNowMessage));
    addAppLogEx(1, "[System] Sent Remote Reboot command over ESP-NOW to linked peer: %s", sysConfig.espnow_peer_mac);

    String html = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Befehl gesendet</title>
    <style>
        body { background: #0f172a; color: white; text-align: center; padding-top: 100px; font-family: sans-serif; }
        .box { background: #1e293b; padding: 30px; border-radius: 15px; display: inline-block; border: 1px solid rgba(255,255,255,0.1); }
        h1 { color: #38bdf8; margin-bottom: 10px; font-size: 20px; }
        p { color: #cbd5e1; font-size: 14px; }
    </style>
</head>
<body>
    <div class="box">
        <h1>Fern-Neustart gesendet 🔗</h1>
        <p>Der Reboot-Befehl wurde per ESP-NOW an den Partner übertragen.</p>
    </div>
    <script>setTimeout(function(){ window.location.href = '/settings'; }, 1000);</script>
</body>
</html>
)rawhtml";
    server.send(200, "text/html", html);
  } else if (action == "defaults") {
    addAppLogEx(1, "[Config] Reset to default settings requested! (Brightness: 80%%, TX Power: 52, WLAN Trap: 120s)");
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
    delay(500);
    performFactoryReset("Web UI (Complete Reset Button)");
  } else if (action == "delete_espnow") {
    addAppLogEx(1, "[Pairing] UNPAIRED! Clearing Peer MAC '%s' and LMK from Flash...", sysConfig.espnow_peer_mac);
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
      addAppLogEx(1, "[Pairing] Master pairing STARTED! LMK generated: %s", proposedLmk);
    } else { // Slave
      currentPairingChannel = sysConfig.espnow_channel;
      lastChannelHopTime = millis();
      originalWifiChannel = WiFi.status() == WL_CONNECTED ? WiFi.channel() : 1;
      esp_wifi_set_channel(currentPairingChannel, WIFI_SECOND_CHAN_NONE);
      addAppLogEx(1, "[Pairing] Slave pairing STARTED on Channel %d", currentPairingChannel);
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
    addAppLogEx(1, "[BUZZER] Triggered Local Buzzer Test Chime!");
    server.send(200, "application/json", "{\"status\":\"ok\"}");
    playWinnerMelody();
  } else if (type == "remote") {
    if (strlen(sysConfig.espnow_peer_mac) == 0) {
      server.send(
          400, "application/json",
          "{\"status\":\"error\",\"message\":\"Kein Partner gekoppelt.\"}");
      return;
    }

    addAppLogEx(1, "[BUZZER] Triggered Remote Peer Buzzer Test over ESP-NOW!");
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

void handleDryStrategyApi() {
  time_t now = time(NULL);
  uint32_t currentTs = (now > 1700000000UL) ? (uint32_t)now : (uint32_t)(millis() / 1000UL);

  if (server.hasArg("mode")) {
    int mode = server.arg("mode").toInt();
    if (mode >= 0 && mode <= 2) {
      sysConfig.dry_strategy = mode;
      if (mode == 2 && !server.hasArg("day")) {
        if (sysConfig.vpd_auto_day < 1 || sysConfig.vpd_auto_day > 14) {
          sysConfig.vpd_auto_day = 1;
        }
        sysConfig.vpd_auto_start_time = currentTs;
      }
    }
  }
  if (server.hasArg("day")) {
    int day = server.arg("day").toInt();
    if (day >= 1 && day <= 14) {
      sysConfig.vpd_auto_day = day;
      sysConfig.vpd_auto_start_time = currentTs;
    }
  }
  if (server.hasArg("limit")) {
    sysConfig.hygro_limit = server.arg("limit").toInt();
  }
  saveConfiguration();
  updateServoRamping(true);
  addAppLogEx(1, "[Strategy] Web UI changed Dry Strategy: Mode=%d (%s), Day=%d, HygroLimit=%d%%",
              sysConfig.dry_strategy,
              (sysConfig.dry_strategy == 0 ? "60/60" : (sysConfig.dry_strategy == 1 ? "VPD" : "VPD AUTO")),
              sysConfig.vpd_auto_day, sysConfig.hygro_limit);
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleFavicon() { server.send(204, "image/x-icon", ""); }

// =====================================================================
// FIRMWARE & ONLINE / MANUAL OTA UPDATE HANDLERS
// =====================================================================

int cachedOnlineVersion = -1;
unsigned long lastGithubCheckTime = 0;

int fetchGithubFirmwareVersion() {
  if (WiFi.status() != WL_CONNECTED)
    return -1;
  WiFiClientSecure client;
  client.setInsecure(); // Disable SSL cert check for ESP32 GitHub requests
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(4000);
  String url = "https://raw.githubusercontent.com/VR-addicted/grow-zone-iDry/main/FIRMWARE/version.txt?nocache=" + String(millis());
  if (http.begin(client, url.c_str())) {
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

void checkGithubUpdateAsync(bool force) {
  if (WiFi.status() != WL_CONNECTED)
    return;
  // Allow 10 seconds post-connection buffer for initial automatic check after boot
  if (!force && connectedSince > 0 && (millis() - connectedSince < 10000)) {
    return;
  }
  if (force || lastGithubCheckTime == 0 ||
      millis() - lastGithubCheckTime >= 3600000UL) { // Conservative 60 minutes interval (1 Hour)
    int ver = fetchGithubFirmwareVersion();
    if (ver > 0) {
      lastGithubCheckTime = millis();
      cachedOnlineVersion = ver;
      if (cachedOnlineVersion > localFirmwareVersion) {
        addAppLogEx(1, "[OTA] GitHub Check: Online Firmware v1.%d AVAILABLE! (Local is v1.%d)", cachedOnlineVersion, localFirmwareVersion);
      } else {
        addAppLogEx(1, "[OTA] GitHub Check: Firmware is up to date (Local v1.%d == Online v1.%d)", localFirmwareVersion, cachedOnlineVersion);
      }
    } else {
      // If network check failed, retry in 5 minutes instead of locking out for 60 minutes
      lastGithubCheckTime = millis() - 3300000UL;
    }
  }
}

void handleFirmwarePage() {
  if (!isWebAuthenticated()) {
    server.send(401, "text/html",
                "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Zugriff geschützt</title>"
                "<style>body{background:#0f172a;color:white;text-align:center;padding-top:100px;font-family:sans-serif;}</style></head>"
                "<body><div style='background:#1e293b;padding:30px;border-radius:15px;display:inline-block;'>"
                "<h1 style='color:#f87171;margin-bottom:15px;'>🔒 Webinterface geschützt</h1>"
                "<p style='color:#cbd5e1;margin-bottom:20px;'>Für den Zugriff auf das Firmware Update ist eine Anmeldung im Dashboard erforderlich.</p>"
                "<a href='/' style='color:#38bdf8;'>Zurück zum Dashboard</a></div></body></html>");
    return;
  }
  checkGithubUpdateAsync(true);
  int onlineVersion = cachedOnlineVersion;
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
            <form method="POST" action="/firmware/upload" enctype="multipart/form-data" onsubmit="triggerFlashGlow()">
                <input type="file" name="update" accept=".bin" required>
                <button type="submit" id="btn-flash-bin" class="btn btn-nav" style="background: rgba(99, 102, 241, 0.2); border-color: rgba(99, 102, 241, 0.4); color: #a5b4fc; transition: all 0.3s ease;">
                    Firmware .bin Flashen
                </button>
            </form>
        </div>

        <div style="display: flex; gap: 10px;">
            <a href="/settings" class="btn btn-nav" style="flex: 1;">Zurueck zu Settings</a>
            <a href="/" class="btn btn-nav" style="flex: 1;">Zurueck zum Monitor</a>
        </div>
    </div>
    <script>
        function triggerFlashGlow() {
            const btn = document.getElementById('btn-flash-bin');
            if (btn) {
                btn.style.background = 'linear-gradient(135deg, #10b981 0%, #059669 100%)';
                btn.style.borderColor = '#22c55e';
                btn.style.color = '#ffffff';
                btn.style.boxShadow = '0 0 20px rgba(34, 197, 94, 0.85), 0 0 35px rgba(34, 197, 94, 0.45)';
                btn.innerText = '⚡ Flashen gestartet...';
                setTimeout(() => {
                    btn.style.background = 'rgba(99, 102, 241, 0.2)';
                    btn.style.borderColor = 'rgba(99, 102, 241, 0.4)';
                    btn.style.color = '#a5b4fc';
                    btn.style.boxShadow = 'none';
                    btn.innerText = 'Firmware .bin Flashen';
                }, 3000);
            }
        }
    </script>
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

  addAppLogEx(1, "[OTA] Online OTA Update SUCCESSFUL! Written %u bytes. Magic byte (0xE9) verified. Rebooting...", (unsigned int)writtenBytes);
  server.send(200, "application/json",
              "{\"status\":\"ok\",\"message\":\"Update erfolgreich! iDry 26 "
              "reboot...\",\"written\":" +
                  String(writtenBytes) + "}");

  delay(500);
  ESP.restart();
}

static bool g_manualUploadError = false;

void handleUploadProgress() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    addAppLogEx(1, "[OTA] Manual Firmware Upload STARTED: %s", upload.filename.c_str());
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
          addAppLogEx(1, "[OTA] Manual Upload ABORTED: Magic byte 0x%02X != 0xE9 (Invalid ESP32 binary)", upload.buf[0]);
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
        addAppLogEx(1, "[OTA] Manual Upload ABORTED: Size too small (%u bytes < 100KB)", (unsigned int)upload.totalSize);
        g_manualUploadError = true;
        Update.abort();
      } else if (Update.end(true)) {
        addAppLogEx(1, "[OTA] Manual Firmware Upload SUCCESSFUL! Written %u bytes.", (unsigned int)upload.totalSize);
      } else {
        Update.printError(Serial);
        g_manualUploadError = true;
      }
    }
  }
}

void handleUploadFinish() {
  if (g_manualUploadError || Update.hasError()) {
    addAppLogEx(1, "[OTA] Manual Firmware Upload REJECTED! (Invalid binary or size < 100KB)");
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
    addAppLogEx(1, "[OTA] Firmware Flash Complete! Magic Byte valid (0xE9). Rebooting into new firmware...");
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
    delay(500);
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

  const char* headerKeys[] = {"Cookie", "X-Web-Pass"};
  server.collectHeaders(headerKeys, 2);

  server.on("/", handlePortalRoot);
  server.on("/save", handlePortalSave);
  server.on("/api/auth", handleApiAuth);
  server.on("/api/data", handleGetData);
  server.on("/api/history", handleGetHistory);
  server.on("/settings", handleSettingsPage);
  server.on("/settings/save", handleSettingsSave);
  server.on("/settings/reset", handleSettingsReset);
  server.on("/api/espnow/pair", handleEspNowPairApi);
  server.on("/api/espnow/buzzer_test", handleBuzzerTestApi);
  server.on("/api/settings/purge", handlePurgeApi);
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
  sendHADiscoveryConfig("dry_strategy", "Dry Strategy", "", "mdi:tune", "");
  sendHADiscoveryConfig("vpd_auto_day", "VPD Auto Tag", "Tag", "mdi:calendar-range", "");
  sendHADiscoveryConfig("fw_version", "Firmware Version", "", "mdi:chip", "");
  sendHADiscoveryConfig("update_available", "Firmware Update Verfügbar", "", "mdi:update", "");
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

  uint8_t activeDryStrat =
      (sysConfig.espnow_role == 2 && lastEspNowRxTime != 0 &&
       (millis() - lastEspNowRxTime <= 5000))
          ? remoteMasterDryStrategy
          : sysConfig.dry_strategy;
  doc["dry_strategy"] = activeDryStrat;
  doc["vpd_auto_day"] = (activeDryStrat == 2) ? getVpdAutoCurrentDay() : -1;
  doc["fw_version"] = "1." + String(localFirmwareVersion);
  doc["update_available"] = (cachedOnlineVersion > localFirmwareVersion);
  doc["online_version"] = cachedOnlineVersion;

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

static bool isServerStarted = false;

// WiFi Event Handler for Instant Reconnection & Dynamic WebServer Socket
// Binding
void WiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    addAppLog("[WLAN] Connected. IP: %s (Ch: %d)", WiFi.localIP().toString().c_str(), WiFi.channel());
    server.begin();
    isServerStarted = true;
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    addAppLog("[WLAN] Connection lost. Reconnecting...");
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  addAppLog("=== iDRY26 v1.99 System Boot. FreeHeap: %u B, MaxAlloc: %u B ===", ESP.getFreeHeap(), ESP.getMaxAllocHeap());

  // Initialize Buzzer & Hardware Boot Button (GPIO 0)
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(0, INPUT_PULLUP); // GPIO 0 Hardware Boot / Reset Button (active LOW)

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
      server.begin();
      isServerStarted = true;
    } else {
      Serial.println(
          "[WLAN] Wi-Fi router not ready yet (power outage recovery). "
          "Background reconnect watchdog will connect when router boots.");
      updateBootScreen("WLAN Suche...", sysConfig.wifi_ssid);
    }

    // Always scan I2C, initialize WebServer routes, start ESP-NOW and MQTT
    // settings
    scanI2C();

    // Web Server Routes Init
    const char* headerKeys[] = {"Cookie", "X-Web-Pass"};
    server.collectHeaders(headerKeys, 2);

    server.on("/", handlePortalRoot);
    server.on("/api/auth", handleApiAuth);
    server.on("/api/data", handleGetData);
    server.on("/api/history", handleGetHistory);
    server.on("/settings", handleSettingsPage);
    server.on("/settings/save", handleSettingsSave);
    server.on("/settings/reset", handleSettingsReset);
    server.on("/api/espnow/pair", handleEspNowPairApi);
    server.on("/api/espnow/buzzer_test", handleBuzzerTestApi);
    server.on("/api/settings/dry_strategy", handleDryStrategyApi);
    server.on("/api/settings/purge", handlePurgeApi);
    server.on("/api/settings/odometer", handleOdometerApi);
    server.on("/api/loglevel", HTTP_POST, handleSetLogLevel);
    server.on("/api/loglevel", HTTP_GET, handleSetLogLevel);
    server.on("/firmware", handleFirmwarePage);
    server.on("/firmware/autoupdate", handleAutoUpdate);
    server.on("/api/firmware/autoupdate_start", handleAutoUpdateApi);
    server.on("/firmware/upload", HTTP_POST, handleUploadFinish,
              handleUploadProgress);
    server.on("/favicon.ico", handleFavicon);
    server.on("/favicon-32x32.png", handleFavicon);
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
  // HARDWARE FACTORY RESET BUTTON (GPIO 0 / BOOT BUTTON) - 3 SEC HOLD
  // =====================================================================
  static unsigned long gpio0PressStartTime = 0;
  static bool gpio0ResetTriggered = false;
  if (digitalRead(0) == LOW) { // Button pressed (active LOW)
    if (gpio0PressStartTime == 0) {
      gpio0PressStartTime = millis();
    } else if (!gpio0ResetTriggered && (millis() - gpio0PressStartTime >= 3000)) {
      gpio0ResetTriggered = true;
      performFactoryReset("Hardware Button (GPIO 0)");
    }
  } else {
    gpio0PressStartTime = 0;
    gpio0ResetTriggered = false;
  }

  // Periodic online firmware update check (once at boot, then every 10 min)
  checkGithubUpdateAsync();

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
      uint8_t masterChan =
          (WiFi.status() == WL_CONNECTED)
              ? WiFi.channel()
              : ((sysConfig.espnow_channel > 0) ? sysConfig.espnow_channel : 1);
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
        memset(&pingMsg, 0, sizeof(EspNowMessage));
        pingMsg.pv = localProtocolVersion;
        pingMsg.type = 2; // Data/Command
        strlcpy(pingMsg.key, sysConfig.espnow_lmk, sizeof(pingMsg.key));
        pingMsg.command = 2; // Ping-Request
        pingMsg.value = rotorPosition;
        pingMsg.dry_strategy = (uint8_t)sysConfig.dry_strategy;

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
  // ESP-NOW SLAVE PEER SYNC & CHANNEL HOPPING (Only when Wi-Fi Offline)
  // =====================================================================
  static unsigned long lastSlaveScanHopTime = 0;
  static uint8_t slaveScanChan = 1;
  if (!isPairingActive && sysConfig.espnow_role == 2 &&
      strlen(sysConfig.espnow_peer_mac) > 0) {

    // Continuously keep Slave's peer registration locked onto active Wi-Fi
    // channel
    static unsigned long lastSlavePeerSyncTime = 0;
    if (millis() - lastSlavePeerSyncTime >= 1000) {
      lastSlavePeerSyncTime = millis();
      uint8_t slaveChan =
          (WiFi.status() == WL_CONNECTED)
              ? WiFi.channel()
              : ((sysConfig.espnow_channel > 0) ? sysConfig.espnow_channel : 1);
      uint8_t peerMac[6];
      if (sscanf(sysConfig.espnow_peer_mac, "%x:%x:%x:%x:%x:%x", &peerMac[0],
                 &peerMac[1], &peerMac[2], &peerMac[3], &peerMac[4],
                 &peerMac[5]) == 6) {
        esp_now_peer_info_t peerInfo;
        memset(&peerInfo, 0, sizeof(peerInfo));
        memcpy(peerInfo.peer_addr, peerMac, 6);
        peerInfo.channel = slaveChan;
        peerInfo.ifidx = WIFI_IF_STA;
        peerInfo.encrypt = false;
        if (esp_now_is_peer_exist(peerMac)) {
          esp_now_mod_peer(&peerInfo);
        } else {
          esp_now_add_peer(&peerInfo);
        }
      }
    }

    // Only hop channels if NOT connected to Wi-Fi AP and NOT during an active WiFi.begin() reconnect scan
    if (WiFi.status() != WL_CONNECTED) {
      if (lastEspNowRxTime == 0 || (millis() - lastEspNowRxTime > 3000)) {
        if (millis() - lastWifiAttemptTime > 3000) {
          if (millis() - lastSlaveScanHopTime >= 350) {
            lastSlaveScanHopTime = millis();
            slaveScanChan = (slaveScanChan % 13) + 1;
            esp_wifi_set_channel(slaveScanChan, WIFI_SECOND_CHAN_NONE);
          }
        }
      }
    }
  }

  // =====================================================================
  // AUTOMATIC ESP-NOW RE-INITIALIZATION WATCHDOG (>15s Disconnect)
  // =====================================================================
  static unsigned long lastEspNowReinitWatchdogTime = 0;
  if (!isPairingActive && sysConfig.espnow_role > 0 &&
      strlen(sysConfig.espnow_peer_mac) > 0) {
    if (lastEspNowRxTime == 0 || (millis() - lastEspNowRxTime > 15000)) {
      if (millis() - lastEspNowReinitWatchdogTime >= 10000) {
        lastEspNowReinitWatchdogTime = millis();
        Serial.println("[ESP-NOW Watchdog] Re-initializing ESP-NOW stack to "
                       "restore link...");
        initEspNow();
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

      // Track Odometer distance: r = 27mm -> 0.0004712389 m/deg
      if (lastTrackedServoAngle < 0.0f) {
        lastTrackedServoAngle = currentServoAngle;
      } else {
        float angleDiff = fabs(currentServoAngle - lastTrackedServoAngle);
        if (angleDiff >= 0.05f) {
          servoTotalMeters += (angleDiff * 0.0004712389f);
          lastTrackedServoAngle = currentServoAngle;
        }
      }

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

  // Periodic 1-Hour Flush for Servo Odometer (Dual-Storage NVS & LittleFS, only if moved)
  static unsigned long lastHourlyOdoSave = 0;
  if (millis() - lastHourlyOdoSave >= 3600000UL) {
    lastHourlyOdoSave = millis();
    saveOdometer(false);
  }

  // Connect / Maintain Wi-Fi Connection & Active Link Watchdog
  static unsigned long lastWifiCheck = 0;
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
      lastWifiAttemptTime = millis();
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

    // Master: Broadcast continuous ESP-NOW sync update (rotorPosition +
    // dry_strategy) every 1 second
    if (sysConfig.espnow_role == 1 && strlen(sysConfig.espnow_peer_mac) > 0 &&
        !isPairingActive) {
      uint8_t peerMac[6];
      if (sscanf(sysConfig.espnow_peer_mac, "%x:%x:%x:%x:%x:%x", &peerMac[0],
                 &peerMac[1], &peerMac[2], &peerMac[3], &peerMac[4],
                 &peerMac[5]) == 6) {
        EspNowMessage syncMsg;
        memset(&syncMsg, 0, sizeof(EspNowMessage));
        syncMsg.pv = localProtocolVersion;
        syncMsg.type = 2; // Data/Command
        strlcpy(syncMsg.key, sysConfig.espnow_lmk, sizeof(syncMsg.key));
        syncMsg.command = 2; // Data Sync
        syncMsg.value = rotorPosition;
        syncMsg.dry_strategy = (uint8_t)sysConfig.dry_strategy;
        esp_now_send(peerMac, (uint8_t *)&syncMsg, sizeof(EspNowMessage));

        static unsigned long lastMasterSyncSendTime = 0;
        if (lastMasterSyncSendTime == 0) {
          lastMasterSyncSendTime = millis();
        } else {
          unsigned long diff = millis() - lastMasterSyncSendTime;
          if (diff >= 750 && diff <= 4000) {
            avgEspNowIntervalMs = diff;
            lastMasterSyncSendTime = millis();
          }
        }
      }
    }

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

    // 30-Second Periodic Diagnostics Log for Web Terminal & T-Pipe Logger
    static unsigned long lastLogHeartbeatTime = 0;
    if (millis() - lastLogHeartbeatTime >= 30000) {
      lastLogHeartbeatTime = millis();
      float freeKb = ESP.getFreeHeap() / 1024.0f;
      float allocKb = ESP.getMaxAllocHeap() / 1024.0f;
      addAppLog("[Status] Bench: %u l/s | Heap: %.1f KB | Alloc: %.1f KB | RSSI: %d dBm",
                loopsPerSecond, freeKb, allocKb, WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
    }

    if (isHeadless) {
      // Headless Mode: skip drawing to display to conserve power/speed
    } else if (isTFTMode) {
      // --- Ambient Light Sensor Display Backlight Control (3s Debounce @ 200 Lux) ---
      bool hasActiveLightSensor = false;
      bool ambientLightPresent = false;
      for (int i = 0; i < 2; i++) {
        if (lightSensors[i].active) {
          hasActiveLightSensor = true;
          if (!isnan(lightSensors[i].lux) && lightSensors[i].lux > 200.0f) {
            ambientLightPresent = true;
            break;
          }
        }
      }

      uint8_t targetUserBrightness = sysConfig.display_brightness;

      if (!hasActiveLightSensor) {
        // No light sensor -> follow user config directly
        darknessStartTime = 0;
        isDisplayDarkened = false;
        uint8_t rawBrightness = (uint8_t)round(pow(targetUserBrightness / 100.0, 2.2) * 255.0);
        tft.setBrightness(rawBrightness);
      } else {
        // Light sensor present
        if (ambientLightPresent) {
          darknessStartTime = 0;
          if (isDisplayDarkened) {
            isDisplayDarkened = false;
            addAppLogEx(3, "[Display] Ambient light detected (>200 Lux). Backlight ON (%d%%).", targetUserBrightness);
          }
          uint8_t rawBrightness = (uint8_t)round(pow(targetUserBrightness / 100.0, 2.2) * 255.0);
          tft.setBrightness(rawBrightness);
        } else {
          // Below 200 Lux -> start/check 3s debounce timer
          if (darknessStartTime == 0) {
            darknessStartTime = millis();
          } else if (millis() - darknessStartTime >= 3000) {
            if (!isDisplayDarkened) {
              isDisplayDarkened = true;
              addAppLogEx(3, "[Display] Darkness confirmed (<=200 Lux for 3s). Backlight OFF (0%%).");
              tft.setBrightness(0);
            }
          }
        }
      }

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
      addAppLogEx(1, "[ALARM] Low Humidity Warning Chime: Inside (%.1f%%) < Target (%.1f%%)", hum_inside, potiAVal);
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
        addAppLogEx(1, "[ALARM] Thermodynamic Bypass Warning Chime: Outside humidity too high!");
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
