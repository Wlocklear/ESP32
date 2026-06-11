/*
 ******************************************************************************
 *  ESP32-S3-N16R8  |  Plant Monitor Firmware  |  v3.2
 ******************************************************************************
 *
 *  TARGET BOARD : ESP32-S3-N16R8  (16 MB Flash, 8 MB PSRAM)
 *
 *  ARDUINO IDE SETTINGS:
 *    Board            : "ESP32S3 Dev Module"
 *    Partition Scheme : "16M Flash (3MB APP OTA / 9.9MB FATFS)"
 *    PSRAM            : "OPI PSRAM"
 *    USB CDC On Boot  : "Enabled"
 *    Flash Size       : "16MB"
 *    Flash Mode       : "QIO 80MHz"
 *
 *  WIRING:
 *    AHT30 SDA        → SDA_PIN      (default GPIO 8)
 *    AHT30 SCL        → SCL_PIN      (default GPIO 9)
 *    Capacitive soil  → MOISTURE_PIN (default GPIO 10, ADC1)
 *    Pump relay IN    → PUMP_PIN     (default GPIO 2)
 *    Relay: HIGH = ON, LOW = OFF by default.
 *    If your relay board is active-low, swap PUMP_ON / PUMP_OFF below.
 *
 *  HOW IT WORKS (v3.0):
 *
 *    The firmware runs a tight state machine:
 *
 *    ┌─────────┐  moisture < threshold   ┌──────────┐
 *    │  READ   │ ──────────────────────► │ PUMP_RUN │
 *    └─────────┘                         └──────────┘
 *         ▲                                   │ pumpDuration elapsed
 *         │  cooldown elapsed                 ▼
 *         │                            ┌──────────────┐
 *         └────────────────────────────│  PUMP_COOL   │
 *                                      └──────────────┘
 *         ┌─────────┐  moisture >= threshold
 *         │  READ   │ ──────────────────────────────► ┌───────┐
 *         └─────────┘                                  │ SLEEP │
 *                                                      └───────┘
 *                                                          │
 *                           ┌──────────────────────────────┤
 *                           │  wake: timer (SLEEP_DURATION) │
 *                           │  wake: HTTP request arrives   │
 *                           └──────────────────────────────►
 *                                        READ
 *
 *    SLEEP uses esp_light_sleep_start() with two wakeup sources:
 *      1. Timer  — fires after SLEEP_DURATION_SEC (default 1800 s / 30 min)
 *      2. WiFi   — fires instantly when an HTTP request arrives
 *
 *    During light sleep the WiFi association is maintained, the TCP/IP
 *    stack stays alive, and the radio wakes automatically on each DTIM
 *    beacon. Any incoming HTTP packet wakes the CPU immediately so the
 *    web server responds normally. The dashboard polls every 30 s while
 *    the device is sleeping and every 3 s while a pump cycle is active.
 *
 *  CHANGES FROM v2.2:
 *    - Redesigned: replaced polling loop with READ→PUMP_RUN→PUMP_COOL→SLEEP
 *                  state machine
 *    - Added: light sleep between read cycles (esp_light_sleep_start)
 *    - Added: WiFi wakeup source so web server responds during sleep
 *    - Added: timer wakeup source for scheduled sensor reads
 *    - Changed: pump cooldown is now the re-read interval (default 180 s)
 *    - Changed: device sleeps once moisture is above threshold
 *    - Changed: dashboard polls every 30 s (idle/sleep) or 3 s (pumping)
 *    - Changed: /status includes appState and nextReadIn fields
 *    - Removed: old runPumpAutomation() polling function
 *    - Removed: SENSOR_READ_INTERVAL (replaced by SLEEP_DURATION_SEC)
 *
 ******************************************************************************
 */

// ============================================================
//  SENSOR SELECTION  —  set exactly ONE to 1
// ============================================================
#define USE_AHT30  1
#define USE_DHT22  0

// ============================================================
//  FIRMWARE VERSION
// ============================================================
#define FIRMWARE_VERSION  "3.2"

// ============================================================
//  PIN DEFINITIONS
// ============================================================
#ifdef XIAO_C6
  // Seeed XIAO ESP32-C6
  #define SDA_PIN       22   // D4
  #define SCL_PIN       23   // D5
  #define MOISTURE_PIN  0    // A0 — ADC1
  #define PUMP_PIN      18   // D10
#else
  // ESP32-S3-N16R8 (default)
  #define SDA_PIN       8
  #define SCL_PIN       9
  #define MOISTURE_PIN  10   // ADC1
  #define PUMP_PIN      2
#endif

#define DHTPIN        4    // DHT22 data pin (unused with AHT30)

#define PUMP_ON   HIGH     // Change to LOW if relay is active-low
#define PUMP_OFF  LOW

// ============================================================
//  AP PORTAL SETTINGS
// ============================================================
#define AP_SSID  "ESP32-Setup"
#define AP_PASS  ""             // Empty = open AP

// ============================================================
//  OTA CREDENTIALS  —  change these!
// ============================================================
#define OTA_USER  "admin"
#define OTA_PASS  "changeme"

// ============================================================
//  TIMING
// ============================================================
#define SLEEP_DURATION_SEC  1800   // How long to sleep when soil is moist (30 min)
#define WIFI_RETRY_INTERVAL 30000UL

// ============================================================
//  CALIBRATION DEFAULTS
// ============================================================
#define DEFAULT_DEVICE_NAME    "outside-plants"
#define DEFAULT_DRY_CAL        3200
#define DEFAULT_WET_CAL        1400
#define MOISTURE_SAMPLES       16

// ============================================================
//  PUMP DEFAULTS  (all configurable from dashboard)
// ============================================================
#define DEFAULT_PUMP_AUTO       true   // Auto on by default in v3
#define DEFAULT_PUMP_THRESHOLD  30     // % — water if moisture falls below this
#define DEFAULT_PUMP_DURATION   30     // seconds to run pump per cycle
#define DEFAULT_PUMP_COOLDOWN   180    // seconds to wait after pump before re-reading

// ============================================================
//  SENSOR HISTORY
// ============================================================
#define HISTORY_SIZE  48   // 48 × 30 min = 24 h

// ============================================================
//  INCLUDES
// ============================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Wire.h>
#include <time.h>
#include <ArduinoJson.h>
#include <ElegantOTA.h>
#include <ArduinoOTA.h>
#include <esp_wifi.h>        // Modem sleep
#include <esp_pm.h>          // Automatic light sleep via power management
#include <esp_task_wdt.h>    // Hardware watchdog

#if USE_AHT30
  #include <Adafruit_AHTX0.h>
#endif
#if USE_DHT22
  #include <DHT.h>
#endif

// ============================================================
//  APPLICATION STATE MACHINE
// ============================================================
enum AppState {
  STATE_READ,       // Take sensor readings, then decide next state
  STATE_PUMP_RUN,   // Pump is on — waiting for pumpDuration to elapse
  STATE_PUMP_COOL,  // Pump off — waiting pumpCooldown seconds before re-reading
  STATE_SLEEP       // Light sleep until timer or HTTP wakeup
};

const char* stateNames[] = { "Reading", "Pump running", "Cooldown", "Sleeping" };

AppState appState     = STATE_READ;
unsigned long stateEnteredAt = 0;   // millis() when current state began
unsigned long nextReadAt     = 0;   // millis() when next scheduled read is due

// ============================================================
//  OBJECTS
// ============================================================
WebServer   server(80);
Preferences prefs;

#if USE_AHT30
  Adafruit_AHTX0 aht;
  bool ahtFound = false;
#endif
#if USE_DHT22
  DHT dht(DHTPIN, DHT22);
#endif

// ============================================================
//  RUNTIME STATE
// ============================================================
String deviceName;
int    dryCal;
int    wetCal;
bool   dryCalSet  = false;
bool   wetCalSet  = false;

// Pump manual state (persisted — only written on manual control)
bool   pumpManualState = false;

// Pump settings
bool   pumpAuto       = DEFAULT_PUMP_AUTO;
int    pumpThreshold  = DEFAULT_PUMP_THRESHOLD;
int    pumpDuration   = DEFAULT_PUMP_DURATION;
int    pumpCooldown   = DEFAULT_PUMP_COOLDOWN;

// Pump runtime tracking
bool          pumpRunning     = false;
unsigned long pumpStartedAt   = 0;
unsigned long pumpLastRan     = 0;
String        pumpLastRanTime = "Never";

// Sensors
float  ambientTempF   = NAN;
float  ambientHumPct  = NAN;
int    moistureRaw    = 0;
int    moisturePct    = 0;
bool   sensorReady    = false;
String lastSensorTime = "Never";

// Timing
unsigned long bootMillis    = 0;
unsigned long lastWiFiRetry = 0;

// WiFi / mDNS
bool mdnsRunning = false;
bool inAPMode    = false;
bool otaRunning  = false;

// Async WiFi scan
bool scanInProgress = false;

// CSRF token
String csrfToken;

// Sensor history ring buffer
struct HistoryPoint {
  int   moisture;
  float tempF;
  float humPct;
  unsigned long ts;   // millis() at capture
};
HistoryPoint history[HISTORY_SIZE];
int historyHead  = 0;
int historyCount = 0;

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
bool   connectWiFi();
void   startAPPortal();
void   startMDNS();
void   setupRoutes();
void   readSensors();
void   transitionTo(AppState next);
String sanitizeMDNS(const String &s);
String getTimeString();
String buildStatusJson();
bool   checkCsrf();
void   generateCsrfToken();
void   setupPowerManagement();

// ============================================================
//  NVS HELPERS
// ============================================================
void loadPrefs() {
  prefs.begin("system", true);
  deviceName      = prefs.getString("name",     DEFAULT_DEVICE_NAME);
  dryCal          = prefs.getInt("dry",          DEFAULT_DRY_CAL);
  wetCal          = prefs.getInt("wet",          DEFAULT_WET_CAL);
  dryCalSet       = prefs.getBool("drySet",      false);
  wetCalSet       = prefs.getBool("wetSet",      false);
  pumpManualState = prefs.getBool("pump",        false);
  pumpAuto        = prefs.getBool("pumpAuto",    DEFAULT_PUMP_AUTO);
  pumpThreshold   = prefs.getInt("pumpThr",      DEFAULT_PUMP_THRESHOLD);
  pumpDuration    = prefs.getInt("pumpDur",      DEFAULT_PUMP_DURATION);
  pumpCooldown    = prefs.getInt("pumpCool",     DEFAULT_PUMP_COOLDOWN);
  prefs.end();
}

void savePumpSettings() {
  prefs.begin("system", false);
  prefs.putBool("pumpAuto", pumpAuto);
  prefs.putInt("pumpThr",   pumpThreshold);
  prefs.putInt("pumpDur",   pumpDuration);
  prefs.putInt("pumpCool",  pumpCooldown);
  prefs.end();
}

void saveName(const String &name) {
  prefs.begin("system", false);
  prefs.putString("name", name);
  prefs.end();
}

void saveCalibration() {
  prefs.begin("system", false);
  prefs.putInt("dry",     dryCal);
  prefs.putInt("wet",     wetCal);
  prefs.putBool("drySet", dryCalSet);
  prefs.putBool("wetSet", wetCalSet);
  prefs.end();
}

void saveManualPump(bool state) {
  pumpManualState = state;
  prefs.begin("system", false);
  prefs.putBool("pump", state);
  prefs.end();
}

void saveWiFiCreds(const String &ssid, const String &pass) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}

void loadWiFiCreds(String &ssid, String &pass) {
  prefs.begin("wifi", true);
  ssid = prefs.getString("ssid", "");
  pass = prefs.getString("pass", "");
  prefs.end();
}

void clearWiFiCreds() {
  prefs.begin("wifi", false);
  prefs.remove("ssid");
  prefs.remove("pass");
  prefs.end();
}

// ============================================================
//  CSRF TOKEN
// ============================================================
void generateCsrfToken() {
  csrfToken = "";
  for (int i = 0; i < 16; i++) {
    csrfToken += String(esp_random() & 0xF, HEX);
  }
}

bool checkCsrf() {
  String tok = server.hasArg("csrf") ? server.arg("csrf") : "";
  if (tok.isEmpty()) tok = server.header("X-CSRF-Token");
  if (tok != csrfToken) {
    server.send(403, "application/json", "{\"error\":\"Invalid or missing CSRF token\"}");
    return false;
  }
  return true;
}

// ============================================================
//  mDNS
// ============================================================
String sanitizeMDNS(const String &input) {
  String out = "";
  for (int i = 0; i < (int)input.length(); i++) {
    char c = tolower((unsigned char)input[i]);
    if (isalnum((unsigned char)c)) out += c;
    else if (c == '-' || c == ' ') out += '-';
  }
  while (out.startsWith("-")) out = out.substring(1);
  while (out.endsWith("-"))   out = out.substring(0, out.length() - 1);
  if (out.isEmpty())          out = DEFAULT_DEVICE_NAME;
  return out;
}

void startMDNS() {
  if (mdnsRunning) MDNS.end();
  String host = sanitizeMDNS(deviceName);
  if (MDNS.begin(host.c_str())) {
    MDNS.addService("http", "tcp", 80);
    mdnsRunning = true;
    Serial.printf("[mDNS] http://%s.local\n", host.c_str());
  } else {
    mdnsRunning = false;
    Serial.println("[mDNS] Failed to start");
  }
}

// ============================================================
//  NTP
// ============================================================
void setupTime() {
  configTzTime("EST5EDT,M3.2.0,M11.1.0", "pool.ntp.org", "time.nist.gov");
  Serial.println("[NTP] Sync requested");
}

String getTimeString() {
  time_t now = time(nullptr);
  if (now < 100000) return "Syncing...";
  struct tm t;
  localtime_r(&now, &t);
  char buf[32];
  strftime(buf, sizeof(buf), "%m/%d/%Y %I:%M:%S %p", &t);
  return String(buf);
}

// ============================================================
//  POWER MANAGEMENT — AUTOMATIC LIGHT SLEEP
// ============================================================
/*
 *  Instead of manually calling esp_light_sleep_start() (which blocks
 *  the loop and breaks the web server), we enable automatic light sleep
 *  via the ESP-IDF power management API.
 *
 *  How it works:
 *    - esp_pm_configure() tells the CPU to enter light sleep automatically
 *      whenever the FreeRTOS idle task runs (i.e. nothing else needs the CPU)
 *    - delay() and vTaskDelay() yield to FreeRTOS, which triggers idle → sleep
 *    - The WiFi modem wakes the CPU instantly on any incoming TCP/IP packet
 *    - server.handleClient() is called normally every loop iteration
 *    - The web server stays fully responsive at all times
 *
 *  In STATE_SLEEP we call delay(50) each iteration. The CPU sleeps during
 *  that 50 ms gap automatically. With WIFI_PS_MIN_MODEM the radio sleeps
 *  between DTIM beacons. Together these save ~80-120 mA vs always-on.
 *
 *  Called once after WiFi connects. Safe to call again after reconnect.
 */
void setupPowerManagement() {
  // Enable modem sleep — radio powers down between DTIM beacons
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

  // Enable automatic CPU light sleep
  // max_freq_mhz / min_freq_mhz: CPU scales between these under load.
  // light_sleep_enable: CPU enters light sleep when FreeRTOS is idle.
#ifdef XIAO_C6
  esp_pm_config_t pm_cfg = {
    .max_freq_mhz       = 160,   // ESP32-C6 max is 160 MHz
    .min_freq_mhz       = 40,    // Scale down when idle (saves power, stays awake)
    .light_sleep_enable = true   // Auto light sleep when nothing is running
  };
#else
  esp_pm_config_t pm_cfg = {
    .max_freq_mhz       = 240,   // Full speed when active (ESP32-S3)
    .min_freq_mhz       = 40,    // Scale down when idle (saves power, stays awake)
    .light_sleep_enable = true   // Auto light sleep when nothing is running
  };
#endif
  esp_err_t err = esp_pm_configure(&pm_cfg);
  if (err == ESP_OK) {
    Serial.println("[Power] Automatic light sleep enabled (modem + CPU)");
  } else {
    // Fallback: modem sleep only (still saves significant power)
    Serial.printf("[Power] Auto sleep config failed (%s) — modem sleep only\n",
                  esp_err_to_name(err));
  }
}

// ============================================================
//  STATE MACHINE TRANSITION
// ============================================================
void transitionTo(AppState next) {
  Serial.printf("[State] %s → %s\n", stateNames[appState], stateNames[next]);
  appState      = next;
  stateEnteredAt = millis();
}

// ============================================================
//  AP SETUP PORTAL
// ============================================================
void startAPPortal() {
  inAPMode = true;
  Serial.printf("[WiFi] Starting AP: %s\n", AP_SSID);
  WiFi.mode(WIFI_AP);
  if (strlen(AP_PASS) >= 8) {
    WiFi.softAP(AP_SSID, AP_PASS);
  } else {
    WiFi.softAP(AP_SSID);
  }
  Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", R"HTML(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 WiFi Setup</title>
<style>
body{background:#111;color:#eee;font-family:Arial,sans-serif;display:flex;
  justify-content:center;align-items:center;min-height:100vh;margin:0;
  padding:1rem;box-sizing:border-box}
.box{background:#1e1e2e;padding:2rem;border-radius:1rem;width:100%;
  max-width:340px;text-align:center;box-shadow:0 4px 24px rgba(0,0,0,.5)}
h2{color:#4f8ef7;margin-bottom:1.5rem}
input{width:100%;padding:.6rem;margin:.4rem 0 1rem;border-radius:.5rem;
  border:1px solid #444;background:#2a2a3e;color:#fff;font-size:1rem;box-sizing:border-box}
button{width:100%;padding:.7rem;background:#4f8ef7;color:#fff;border:none;
  border-radius:.5rem;font-size:1rem;cursor:pointer;margin-bottom:.6rem}
button:hover{background:#3a7de0}
.scan-btn{background:#252840}.scan-btn:hover{background:#333658}
.note{font-size:.8rem;color:#888;margin-top:.8rem}
.net-item{display:flex;justify-content:space-between;align-items:center;
  padding:.4rem .6rem;border-radius:.4rem;cursor:pointer;margin-bottom:.3rem;
  background:#252840;text-align:left}
.net-item:hover{background:#333658}
.lock{font-size:.75rem;margin-left:.4rem;color:#f7b84f}
#netList{margin-bottom:.8rem;max-height:220px;overflow-y:auto}
#scanning{color:#8892a4;font-size:.85rem;margin:.5rem 0}
</style></head><body>
<div class="box">
  <h2>WiFi Setup</h2>
  <button class="scan-btn" onclick="doScan()">Scan for Networks</button>
  <div id="scanning" style="display:none">Scanning...</div>
  <div id="netList"></div>
  <input id="ssidInput" placeholder="Selected or type SSID" autocomplete="off">
  <input id="passInput" placeholder="Password" type="password" autocomplete="new-password">
  <button onclick="doConnect()">Connect and Save</button>
  <p class="note">Device will restart and connect to your network.</p>
</div>
<script>
function doScan(){
  var list=document.getElementById('netList'),sc=document.getElementById('scanning');
  list.innerHTML='';sc.style.display='block';
  fetch('/wifi/scan').then(function(r){return r.json();}).then(function(nets){
    sc.style.display='none';
    if(!nets.length){list.innerHTML='<p style="color:#8892a4;font-size:.85rem">No networks found</p>';return;}
    nets.forEach(function(n){
      var d=document.createElement('div');d.className='net-item';
      d.innerHTML='<span>'+n.ssid+(n.secure?'<span class="lock">&#x1F512;</span>':'')+'</span>'
        +'<span style="font-size:.75rem;color:#8892a4">'+n.rssi+' dBm</span>';
      d.onclick=function(){document.getElementById('ssidInput').value=n.ssid;};
      list.appendChild(d);
    });
  }).catch(function(){sc.textContent='Scan failed — try again';});
}
function doConnect(){
  var s=document.getElementById('ssidInput').value.trim();
  var p=document.getElementById('passInput').value;
  if(!s){alert('Enter or select an SSID first.');return;}
  var fd=new FormData();fd.append('ssid',s);fd.append('password',p);
  fetch('/save',{method:'POST',body:fd}).then(function(r){return r.text();})
    .then(function(t){document.body.innerHTML=t;});
}
doScan();
</script></body></html>
)HTML");
  });

  server.on("/save", HTTP_POST, []() {
    if (!server.hasArg("ssid") || server.arg("ssid").isEmpty()) {
      server.send(400, "text/plain", "SSID is required");
      return;
    }
    String ssid = server.arg("ssid");
    String pass = server.hasArg("password") ? server.arg("password") : "";
    ssid.trim();
    saveWiFiCreds(ssid, pass);
    String mdnsName = sanitizeMDNS(deviceName);
    server.send(200, "text/html",
      String("<!DOCTYPE html><html><head>")
      + "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      + "<style>body{background:#111;color:#eee;font-family:Arial;"
      + "text-align:center;padding-top:3rem}</style></head><body>"
      + "<h2 style='color:#3ecf8e'>Credentials Saved!</h2>"
      + "<p>Restarting...</p>"
      + "<p style='color:#888;font-size:.9rem'>Reconnect to your WiFi then visit<br>"
      + "<strong>http://" + mdnsName + ".local</strong></p></body></html>");
    delay(1500);
    ESP.restart();
  });

  server.on("/wifi/scan", HTTP_GET, []() {
    int n = WiFi.scanNetworks();
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < n; i++) {
      JsonObject net = arr.add<JsonObject>();
      net["ssid"]   = WiFi.SSID(i);
      net["rssi"]   = WiFi.RSSI(i);
      net["secure"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    }
    WiFi.scanDelete();
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  server.onNotFound([]() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
  });

  server.begin();
}

// ============================================================
//  WiFi — STATION CONNECT
// ============================================================
bool connectWiFi() {
  String ssid, pass;
  loadWiFiCreds(ssid, pass);
  if (ssid.isEmpty()) {
    Serial.println("[WiFi] No saved credentials");
    return false;
  }
  Serial.printf("[WiFi] Connecting to: %s\n", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(ssid.c_str(), pass.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Connected  IP: %s  RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
  }
  Serial.println("[WiFi] Connection failed");
  return false;
}

// ============================================================
//  AHT30
// ============================================================
#if USE_AHT30
bool initAHT30() {
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(50);
  ahtFound = aht.begin();
  Serial.println(ahtFound ? "[AHT30] OK" : "[AHT30] Not found");
  return ahtFound;
}
#endif

// ============================================================
//  SENSORS
// ============================================================
float chipTempF() {
  return (temperatureRead() * 9.0f / 5.0f) + 32.0f;
}

int readMoistureRaw() {
  long sum = 0;
  for (int i = 0; i < MOISTURE_SAMPLES; i++) {
    sum += analogRead(MOISTURE_PIN);
    delay(2);
  }
  int result = (int)(sum / MOISTURE_SAMPLES);
  Serial.printf("[Soil] Raw ADC: %d\n", result);
  return result;
}

int moistureToPercent(int raw) {
  if (dryCal == wetCal) return 0;
  return constrain(map(raw, dryCal, wetCal, 0, 100), 0, 100);
}

bool moistureRawPlausible(int raw) {
  return raw > 50 && raw < 4090;
}

void pushHistory(int moist, float t, float h) {
  history[historyHead].moisture = moist;
  history[historyHead].tempF    = t;
  history[historyHead].humPct   = h;
  history[historyHead].ts       = millis();
  historyHead = (historyHead + 1) % HISTORY_SIZE;
  if (historyCount < HISTORY_SIZE) historyCount++;
}

void readSensors() {
  Serial.println("[Sensors] Reading...");
  moistureRaw = readMoistureRaw();
  moisturePct = moistureRawPlausible(moistureRaw)
                ? moistureToPercent(moistureRaw) : 0;

#if USE_AHT30
  if (ahtFound) {
    sensors_event_t humEv, tempEv;
    if (aht.getEvent(&humEv, &tempEv)) {
      ambientTempF  = (tempEv.temperature * 9.0f / 5.0f) + 32.0f;
      ambientHumPct = humEv.relative_humidity;
    } else {
      Serial.println("[AHT30] Read failed — attempting re-init...");
      ahtFound = false;
      ambientTempF = ambientHumPct = NAN;
    }
  }
  if (!ahtFound) {
    if (initAHT30()) {
      sensors_event_t humEv, tempEv;
      if (aht.getEvent(&humEv, &tempEv)) {
        ambientTempF  = (tempEv.temperature * 9.0f / 5.0f) + 32.0f;
        ambientHumPct = humEv.relative_humidity;
      }
    }
  }
#endif

#if USE_DHT22
  float t = dht.readTemperature(true);
  float h = dht.readHumidity();
  if (!isnan(t)) ambientTempF  = t;
  if (!isnan(h)) ambientHumPct = h;
#endif

  sensorReady    = true;
  lastSensorTime = getTimeString();
  pushHistory(moisturePct, ambientTempF, ambientHumPct);

  Serial.printf("[Sensors] Moisture: %d%%  Temp: %.1f F  Hum: %.1f%%\n",
                moisturePct, ambientTempF, ambientHumPct);
}

// ============================================================
//  CALIBRATION
// ============================================================
int calStatus()    { return (dryCalSet ? 1 : 0) + (wetCalSet ? 1 : 0); }
bool isCalibrated(){ return dryCalSet && wetCalSet && (dryCal != wetCal); }

// ============================================================
//  WiFi SIGNAL
// ============================================================
String wifiQuality(int rssi) {
  if (!WiFi.isConnected()) return "Disconnected";
  if (rssi > -50) return "Excellent";
  if (rssi > -60) return "Good";
  if (rssi > -70) return "Fair";
  return "Weak";
}

// ============================================================
//  STATUS JSON
// ============================================================
String buildStatusJson() {
  int rssi = WiFi.isConnected() ? WiFi.RSSI() : 0;
  unsigned long uptimeSec = (millis() - bootMillis) / 1000UL;

  // Seconds until next scheduled read (only meaningful in SLEEP state)
  long nextReadIn = 0;
  if (appState == STATE_SLEEP && nextReadAt > millis()) {
    nextReadIn = (long)((nextReadAt - millis()) / 1000UL);
  }

  JsonDocument doc;
  doc["firmware"]    = FIRMWARE_VERSION;
  doc["device"]      = deviceName;
  doc["mdns"]        = sanitizeMDNS(deviceName);
  doc["time"]        = getTimeString();
  doc["uptime"]      = uptimeSec;
  doc["csrf"]        = csrfToken;
  doc["appState"]    = stateNames[appState];
  doc["nextReadIn"]  = nextReadIn;   // 0 when not sleeping

  // Moisture
  bool rawOK = moistureRawPlausible(moistureRaw);
  doc["moisture"]    = moisturePct;
  doc["raw"]         = moistureRaw;
  doc["rawOk"]       = rawOK;
  doc["dryCal"]      = dryCal;
  doc["wetCal"]      = wetCal;
  doc["calibrated"]  = isCalibrated();
  doc["calStatus"]   = calStatus();
  doc["dryCalSet"]   = dryCalSet;
  doc["wetCalSet"]   = wetCalSet;

  // Ambient sensor
  doc["sensorType"]   = USE_AHT30 ? "AHT30" : "DHT22";
  doc["sensorOnline"] = USE_AHT30 ? ahtFound : true;
  if (!isnan(ambientTempF))  doc["ambientTemp"] = String(ambientTempF,  1);
  else                       doc["ambientTemp"] = nullptr;
  if (!isnan(ambientHumPct)) doc["ambientHum"]  = String(ambientHumPct, 1);
  else                       doc["ambientHum"]  = nullptr;
  doc["chipTemp"]     = String(chipTempF(), 1);

  // Pump
  bool pumpPhysical = pumpRunning || pumpManualState;
  doc["pump"]          = pumpPhysical;
  doc["pumpRunning"]   = pumpRunning;
  doc["pumpManual"]    = pumpManualState;
  doc["pumpAuto"]      = pumpAuto;
  doc["pumpThreshold"] = pumpThreshold;
  doc["pumpDuration"]  = pumpDuration;
  doc["pumpCooldown"]  = pumpCooldown;
  doc["pumpLastRan"]   = pumpLastRanTime;

  // Cooldown remaining (seconds)
  long cooldownLeft = 0;
  if (appState == STATE_PUMP_COOL) {
    unsigned long elapsed = (millis() - stateEnteredAt) / 1000UL;
    cooldownLeft = (long)pumpCooldown - (long)elapsed;
    if (cooldownLeft < 0) cooldownLeft = 0;
  }
  doc["pumpCooldownLeft"] = cooldownLeft;
  doc["lastRead"]    = lastSensorTime;

  // Network
  doc["ssid"]    = WiFi.isConnected() ? WiFi.SSID()              : "Not connected";
  doc["ip"]      = WiFi.isConnected() ? WiFi.localIP().toString() : "--";
  doc["mac"]     = WiFi.macAddress();
  doc["rssi"]    = rssi;
  doc["quality"] = wifiQuality(rssi);

  // Sensor history with real timestamps
  time_t nowEpoch  = time(nullptr);
  unsigned long nowMs = millis();
  JsonArray hist = doc["history"].to<JsonArray>();
  int start = (historyCount < HISTORY_SIZE) ? 0 : historyHead;
  for (int i = 0; i < historyCount; i++) {
    int idx = (start + i) % HISTORY_SIZE;
    JsonObject pt = hist.add<JsonObject>();
    pt["m"] = history[idx].moisture;
    if (!isnan(history[idx].tempF))  pt["t"] = String(history[idx].tempF,  1);
    if (!isnan(history[idx].humPct)) pt["h"] = String(history[idx].humPct, 1);
    long ageSec = (long)((nowMs - history[idx].ts) / 1000UL);
    pt["ts"] = (nowEpoch > 100000) ? (long)nowEpoch - ageSec : 0;
  }

  String out;
  serializeJson(doc, out);
  return out;
}

// ============================================================
//  WEB ROUTE HANDLERS
// ============================================================
void handleStatus()  { server.send(200, "application/json", buildStatusJson()); }
void handleHealth()  { server.send(200, "text/plain", "OK"); }

void handleCalDry() {
  if (!checkCsrf()) return;
  dryCal = readMoistureRaw(); dryCalSet = true; saveCalibration();
  server.send(200, "application/json", "{\"ok\":true,\"dry\":" + String(dryCal) + "}");
}

void handleCalWet() {
  if (!checkCsrf()) return;
  wetCal = readMoistureRaw(); wetCalSet = true; saveCalibration();
  server.send(200, "application/json", "{\"ok\":true,\"wet\":" + String(wetCal) + "}");
}

void handleCalReset() {
  if (!checkCsrf()) return;
  dryCal = DEFAULT_DRY_CAL; wetCal = DEFAULT_WET_CAL;
  dryCalSet = wetCalSet = false; saveCalibration();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handlePumpOn() {
  if (!checkCsrf()) return;
  // Manual ON overrides the state machine — pump stays on until manual OFF
  pumpRunning = false;
  digitalWrite(PUMP_PIN, PUMP_ON);
  saveManualPump(true);
  // Keep state machine in current state; manual flag controls physical pin
  server.send(200, "application/json", "{\"pump\":true}");
}

void handlePumpOff() {
  if (!checkCsrf()) return;
  pumpRunning = false;
  digitalWrite(PUMP_PIN, PUMP_OFF);
  saveManualPump(false);
  server.send(200, "application/json", "{\"pump\":false}");
}

void handlePumpSettings() {
  if (!checkCsrf()) return;
  bool changed = false;
  if (server.hasArg("auto")) {
    pumpAuto = server.arg("auto") == "1"; changed = true;
  }
  if (server.hasArg("threshold")) {
    int v = server.arg("threshold").toInt();
    if (v >= 0 && v <= 100) { pumpThreshold = v; changed = true; }
  }
  if (server.hasArg("duration")) {
    int v = server.arg("duration").toInt();
    if (v >= 1 && v <= 3600) { pumpDuration = v; changed = true; }
  }
  if (server.hasArg("cooldown")) {
    int v = server.arg("cooldown").toInt();
    if (v >= 0 && v <= 86400) { pumpCooldown = v; changed = true; }
  }
  if (changed) savePumpSettings();
  String json = "{\"ok\":true,\"auto\":" + String(pumpAuto ? "true" : "false")
    + ",\"threshold\":" + String(pumpThreshold)
    + ",\"duration\":"  + String(pumpDuration)
    + ",\"cooldown\":"  + String(pumpCooldown) + "}";
  server.send(200, "application/json", json);
}

// GET /sensor/refresh — force an immediate read and return fresh status.
// Also wakes the device out of sleep so the user gets fresh data.
void handleSensorRefresh() {
  if (!checkCsrf()) return;
  readSensors();
  // If sleeping, transition back to read state so the watering logic
  // re-evaluates after the manual refresh
  if (appState == STATE_SLEEP) {
    transitionTo(STATE_READ);
  }
  server.send(200, "application/json", buildStatusJson());
}

void handleRename() {
  if (!checkCsrf()) return;
  if (!server.hasArg("name") || server.arg("name").isEmpty()) {
    server.send(400, "application/json", "{\"error\":\"Missing or empty name\"}");
    return;
  }
  String raw = server.arg("name"); raw.trim();
  deviceName = sanitizeMDNS(raw);
  saveName(deviceName);
  startMDNS();
  server.send(200, "application/json",
    "{\"ok\":true,\"name\":\"" + deviceName + "\"}");
}

void handleWiFiReset() {
  if (!checkCsrf()) return;
  server.send(200, "application/json",
    "{\"ok\":true,\"msg\":\"Rebooting into setup mode.\"}");
  clearWiFiCreds();
  delay(1000);
  ESP.restart();
}

void handleWiFiChange() {
  if (!server.hasArg("ssid") || server.arg("ssid").isEmpty()) {
    server.send(400, "application/json", "{\"error\":\"Missing SSID\"}");
    return;
  }
  String ssid = server.arg("ssid");
  String pass = server.hasArg("pass") ? server.arg("pass") : "";
  saveWiFiCreds(ssid, pass);
  server.send(200, "application/json", "{\"ok\":true}");
  delay(800);
  ESP.restart();
}

void handleWiFiScanStart() {
  WiFi.scanNetworks(true);
  server.send(200, "application/json", "{\"ok\":true,\"status\":\"scanning\"}");
}

void handleWiFiScanResult() {
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) {
    server.send(200, "application/json", "{\"status\":\"scanning\"}");
    return;
  }
  JsonDocument doc;
  if (n < 0) {
    doc["status"] = "error";
  } else {
    doc["status"] = "done";
    JsonArray arr = doc["networks"].to<JsonArray>();
    for (int i = 0; i < n; i++) {
      JsonObject net = arr.add<JsonObject>();
      net["ssid"]   = WiFi.SSID(i);
      net["rssi"]   = WiFi.RSSI(i);
      net["secure"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    }
  }
  WiFi.scanDelete();
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// ============================================================
//  DASHBOARD
// ============================================================
void handleRoot() {
  server.send(200, "text/html", R"DASH(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Dashboard</title>
<style>
:root{
  --bg:#0d0f1a;--card:#151724;--bdr:#252840;--acc:#4f8ef7;
  --grn:#3ecf8e;--red:#f76e6e;--warn:#f7b84f;--dim:#8892a4;--txt:#dde3f0;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font-family:'Segoe UI',Arial,sans-serif;
     padding:1rem;max-width:600px;margin:auto}
h1{text-align:center;color:var(--acc);font-size:1.3rem;margin:.8rem 0 .3rem}
.card{background:var(--card);border:1px solid var(--bdr);border-radius:14px;
      padding:1.2rem 1.4rem;margin-bottom:1rem}
.card-hdr{display:flex;justify-content:space-between;align-items:center;margin-bottom:.9rem}
.card-hdr h2,.card>h2{font-size:.78rem;text-transform:uppercase;letter-spacing:.1em;
  color:var(--dim);margin-bottom:.9rem}
.row{display:flex;justify-content:space-between;align-items:center;
     padding:.35rem 0;border-bottom:1px solid var(--bdr);font-size:.9rem}
.row:last-child{border-bottom:none}
.lbl{color:var(--dim);font-size:.88rem}.val{font-weight:600;font-size:.92rem}
.big{font-size:2.8rem;font-weight:700;color:var(--acc);text-align:center;padding:.5rem 0}
.badge{display:inline-block;padding:.2rem .75rem;border-radius:999px;
       font-size:.78rem;font-weight:600;background:#1e2535}
.badge.ok{color:var(--grn)}.badge.warn{color:var(--warn)}.badge.err{color:var(--red)}
.bar-wrap{background:#1e2535;border-radius:999px;height:10px;margin:.5rem 0}
.bar{height:10px;border-radius:999px;background:var(--acc);transition:width .5s;max-width:100%}
.btn-row{display:flex;gap:.6rem;flex-wrap:wrap;margin-top:.85rem}
button{padding:.5rem .95rem;border:none;border-radius:8px;font-size:.86rem;
       font-weight:600;cursor:pointer;transition:opacity .15s}
button:hover{opacity:.83}button:active{opacity:.65}
.b-blue{background:var(--acc);color:#fff}.b-grn{background:var(--grn);color:#111}
.b-red{background:var(--red);color:#fff}.b-dim{background:var(--bdr);color:var(--txt)}
.b-sm{padding:.3rem .7rem;font-size:.78rem}
.irow{display:flex;gap:.5rem;margin-top:.7rem}
input[type=text],input[type=password]{flex:1;padding:.5rem .8rem;background:#1e2535;
  border:1px solid var(--bdr);border-radius:8px;color:var(--txt);font-size:.9rem;outline:none}
input[type=text]:focus,input[type=password]:focus{border-color:var(--acc)}
.pump-big{font-size:1.15rem;font-weight:700;text-align:center;padding:.4rem 0}
.p-on{color:var(--grn)}.p-off{color:var(--dim)}
.ota-link{display:block;text-align:center;color:var(--acc);text-decoration:none;
          font-size:.92rem;font-weight:600;padding:.4rem 0}
.last-read{font-size:.72rem;color:var(--dim);text-align:right;margin-top:.5rem}
#upd{text-align:center;font-size:.73rem;color:var(--dim);margin-top:.4rem}
.net-list{margin-top:.7rem;max-height:180px;overflow-y:auto}
.net-item{display:flex;justify-content:space-between;align-items:center;
          padding:.4rem .6rem;border-radius:.4rem;cursor:pointer;
          margin-bottom:.3rem;background:#1e2535}
.net-item:hover{background:var(--bdr)}
.lock{font-size:.72rem;margin-left:.3rem;color:var(--warn)}
#scanStatus{font-size:.82rem;color:var(--dim);margin:.4rem 0;text-align:center}
canvas{width:100%;height:110px;display:block;margin-top:.2rem}
.warn-banner{background:#2a1e10;border:1px solid var(--warn);color:var(--warn);
  border-radius:8px;padding:.5rem .8rem;font-size:.82rem;margin-bottom:.6rem;display:none}
.uptime{font-size:.75rem;color:var(--dim);text-align:center;margin-bottom:.2rem}

/* Device state banner */
.state-banner{border-radius:8px;padding:.55rem 1rem;font-size:.84rem;font-weight:600;
  text-align:center;margin-bottom:.75rem}
.state-sleep{background:#0d1a2e;color:#4f8ef7;border:1px solid #1a3a6e}
.state-pump{background:#0a2a0a;color:#3ecf8e;border:1px solid #1a5c1a}
.state-cool{background:#2a1e00;color:#f7b84f;border:1px solid #5c3d00}
.state-read{background:#1e1e2e;color:#8892a4;border:1px solid #252840}

.dirty{color:var(--warn);font-size:.8rem;align-self:center;display:none}
</style>
</head>
<body>

<h1 id="hTitle">ESP32 Dashboard</h1>
<div class="uptime" id="uptimeEl"></div>

<!-- Device state banner -->
<div id="stateBanner" class="state-banner state-read">Initialising...</div>

<!-- Soil sensor disconnect warning -->
<div class="warn-banner" id="sensorWarn">
  ⚠ Soil sensor reading out of range — check wiring
</div>

<!-- SOIL MOISTURE -->
<div class="card">
  <div class="card-hdr">
    <h2>Soil Moisture</h2>
    <button class="b-dim b-sm" onclick="refreshSensors()">&#x21BB; Refresh</button>
  </div>
  <div class="big"><span id="moisture">--</span>%</div>
  <div class="bar-wrap"><div class="bar" id="bar" style="width:0%"></div></div>
  <div style="font-size:.72rem;color:var(--dim);margin-top:.8rem;margin-bottom:.1rem">Moisture (%)</div>
  <canvas id="moistChart"></canvas>
  <div class="row"><span class="lbl">Raw ADC</span><span class="val" id="raw">--</span></div>
  <div class="row"><span class="lbl">Dry cal</span><span class="val" id="dryCal">--</span></div>
  <div class="row"><span class="lbl">Wet cal</span><span class="val" id="wetCal">--</span></div>
  <div class="row"><span class="lbl">Status</span><span id="calBadge" class="badge">--</span></div>
  <div class="btn-row">
    <button id="calDryBtn" class="b-red" onclick="calDry()">Set DRY point</button>
    <button id="calWetBtn" class="b-red" onclick="calWet()">Set WET point</button>
    <button class="b-red" style="margin-left:auto" onclick="calReset()">Reset</button>
  </div>
  <div class="last-read">Last read: <span id="lastRead">--</span></div>
</div>

<!-- AMBIENT SENSOR -->
<div class="card">
  <div class="card-hdr">
    <h2 id="sensorLabel">Ambient Sensor</h2>
    <button class="b-dim b-sm" onclick="refreshSensors()">&#x21BB; Refresh</button>
  </div>
  <div class="row"><span class="lbl">Temperature</span>
    <span class="val"><span id="ambTemp">--</span> °F</span></div>
  <div class="row"><span class="lbl">Humidity</span>
    <span class="val"><span id="ambHum">--</span> %</span></div>
  <div style="font-size:.72rem;color:var(--dim);margin-top:.8rem;margin-bottom:.1rem">Temperature (°F)</div>
  <canvas id="tempChart"></canvas>
  <div style="font-size:.72rem;color:var(--dim);margin-top:.9rem;margin-bottom:.1rem">Humidity (%)</div>
  <canvas id="humChart"></canvas>
  <div class="last-read">Last read: <span id="lastRead2">--</span></div>
</div>

<!-- ESP32 INTERNAL -->
<div class="card">
  <div class="card-hdr">
    <h2>ESP32-S3 Internal</h2>
    <button class="b-dim b-sm" onclick="refreshSensors()">&#x21BB; Refresh</button>
  </div>
  <div class="row"><span class="lbl">Chip temperature</span>
    <span class="val"><span id="chipTemp">--</span> °F</span></div>
  <div class="row"><span class="lbl">Local time</span>
    <span class="val" id="devTime">--</span></div>
  <div class="row"><span class="lbl">Firmware</span>
    <span class="val" id="firmware">--</span></div>
  <div class="last-read">Last read: <span id="lastRead3">--</span></div>
</div>

<!-- PUMP -->
<div class="card">
  <h2>Pump Control</h2>
  <div class="pump-big" id="pumpStatus">--</div>
  <div class="row"><span class="lbl">Last auto run</span>
    <span class="val" id="pumpLastRan">--</span></div>
  <div class="row"><span class="lbl">Cooldown remaining</span>
    <span class="val" id="pumpCooldownLeft">--</span></div>
  <div class="btn-row">
    <button class="b-grn" onclick="pumpOn()">Turn ON</button>
    <button class="b-red" onclick="pumpOff()">Turn OFF</button>
  </div>

  <h2 style="margin-top:1.1rem">Auto Watering</h2>
  <div class="row">
    <span class="lbl">Auto mode</span>
    <span class="val">
      <label style="display:flex;align-items:center;gap:.5rem;cursor:pointer">
        <input type="checkbox" id="pumpAutoChk" onchange="markDirty()"
               style="width:1.1rem;height:1.1rem;cursor:pointer">
        <span id="pumpAutoLabel">Disabled</span>
      </label>
    </span>
  </div>
  <div class="row">
    <span class="lbl">Moisture threshold</span>
    <span class="val" style="display:flex;align-items:center;gap:.4rem">
      <input type="number" id="pumpThreshold" min="1" max="99" value="30"
             oninput="markDirty()"
             style="width:4rem;padding:.3rem .4rem;background:#1e2535;
                    border:1px solid var(--bdr);border-radius:6px;
                    color:var(--txt);font-size:.88rem;text-align:center">
      <span style="color:var(--dim);font-size:.85rem">%</span>
    </span>
  </div>
  <div class="row">
    <span class="lbl">Pump run time</span>
    <span class="val" style="display:flex;align-items:center;gap:.4rem">
      <input type="number" id="pumpDuration" min="1" max="3600" value="30"
             oninput="markDirty()"
             style="width:4.5rem;padding:.3rem .4rem;background:#1e2535;
                    border:1px solid var(--bdr);border-radius:6px;
                    color:var(--txt);font-size:.88rem;text-align:center">
      <span style="color:var(--dim);font-size:.85rem">sec</span>
    </span>
  </div>
  <div class="row">
    <span class="lbl">Re-check cooldown</span>
    <span class="val" style="display:flex;align-items:center;gap:.4rem">
      <input type="number" id="pumpCooldown" min="30" max="3600" value="180"
             oninput="markDirty()"
             style="width:5rem;padding:.3rem .4rem;background:#1e2535;
                    border:1px solid var(--bdr);border-radius:6px;
                    color:var(--txt);font-size:.88rem;text-align:center">
      <span style="color:var(--dim);font-size:.85rem">sec</span>
    </span>
  </div>
  <div class="btn-row">
    <button class="b-blue" onclick="savePumpSettings()">Save settings</button>
    <span id="dirtyNote" class="dirty">● Unsaved changes</span>
  </div>
  <p style="color:var(--dim);font-size:.76rem;margin-top:.6rem">
    When auto is on: if moisture is below the threshold the pump runs for
    <em>Pump run time</em> seconds. After each run the device waits
    <em>Re-check cooldown</em> seconds then reads the sensors again.
    Once moisture is above the threshold the device goes to sleep for
    30 minutes then checks again.
  </p>
</div>

<!-- NETWORK -->
<div class="card">
  <h2>Network</h2>
  <div class="row"><span class="lbl">SSID</span><span class="val" id="ssid">--</span></div>
  <div class="row"><span class="lbl">IP address</span><span class="val" id="ip">--</span></div>
  <div class="row"><span class="lbl">MAC address</span><span class="val" id="mac">--</span></div>
  <div class="row"><span class="lbl">RSSI</span>
    <span class="val"><span id="rssi">--</span> dBm</span></div>
  <div class="row"><span class="lbl">Signal</span><span class="val" id="quality">--</span></div>
  <div class="row"><span class="lbl">mDNS hostname</span>
    <span class="val" id="mdnsLabel">--</span></div>

  <h2 style="margin-top:1.1rem">Change WiFi Network</h2>
  <button class="b-dim" style="width:100%;margin-top:.5rem" onclick="doScan()">Scan for networks</button>
  <div id="scanStatus"></div>
  <div class="net-list" id="netList"></div>
  <div class="irow">
    <input type="text" id="wifiSSID" placeholder="Selected or type SSID" autocomplete="off">
    <input type="password" id="wifiPass" placeholder="Password" autocomplete="new-password"
           style="flex:1;padding:.5rem .8rem;background:#1e2535;border:1px solid var(--bdr);
                  border-radius:8px;color:var(--txt);font-size:.9rem;outline:none">
  </div>
  <div class="btn-row" style="margin-top:.7rem">
    <button class="b-blue" onclick="changeWiFi()">Connect to network</button>
  </div>

  <h2 style="margin-top:1.1rem">Change Device Name / mDNS</h2>
  <div class="irow">
    <input type="text" id="renameInput" placeholder="new-hostname">
    <button class="b-blue" onclick="renameDevice()">Save</button>
  </div>
  <p style="color:var(--dim);font-size:.76rem;margin-top:.5rem">
    Lowercase letters, numbers and hyphens only. mDNS updates immediately.
  </p>
  <div class="btn-row" style="margin-top:1rem">
    <button class="b-red" onclick="resetWiFi()">Reset WiFi credentials</button>
  </div>
</div>

<!-- OTA -->
<div class="card">
  <h2>Firmware Update</h2>
  <a class="ota-link" href="/update">Open OTA update page &#x2197;</a>
</div>

<div id="upd">Connecting...</div>

<script>
var $=function(id){return document.getElementById(id);};
var csrf='';

function toast(msg,ms){
  ms=ms||2500;
  var d=document.createElement('div');
  d.textContent=msg;
  d.style.cssText='position:fixed;bottom:1.4rem;left:50%;transform:translateX(-50%);'
    +'background:#252840;color:#dde3f0;padding:.55rem 1.1rem;border-radius:8px;'
    +'font-size:.86rem;z-index:9999;box-shadow:0 2px 12px rgba(0,0,0,.4);opacity:1;transition:opacity .4s';
  document.body.appendChild(d);
  setTimeout(function(){d.style.opacity='0';setTimeout(function(){d.parentNode&&d.parentNode.removeChild(d);},500);},ms);
}

function api(url){
  var sep=url.indexOf('?')<0?'?':'&';
  return fetch(url+sep+'csrf='+encodeURIComponent(csrf));
}

function uptimeStr(s){
  var d=Math.floor(s/86400),h=Math.floor((s%86400)/3600),m=Math.floor((s%3600)/60),sec=s%60;
  if(d>0) return d+'d '+h+'h '+m+'m';
  if(h>0) return h+'h '+m+'m '+sec+'s';
  if(m>0) return m+'m '+sec+'s';
  return sec+'s';
}

// ── Sparkline charts ────────────────────────────────────────
function fmtHHMM(unixSec){
  var dt=new Date(unixSec*1000);
  var h=dt.getHours(),m=dt.getMinutes();
  return (h<10?'0':'')+h+':'+(m<10?'0':'')+m;
}

// drawSparkline(canvasId, points, color, unit, minVal, maxVal)
//   points : [{v: number, ts: unix seconds}, ...]
//   color  : CSS rgb() string
//   unit   : label appended to Y axis values e.g. '°F' or '%'
//   minVal / maxVal : optional fixed axis bounds
//
// Layout (pixels):
//   LEFT_W  = 34 px  — Y axis labels
//   AXIS_H  = 18 px  — X axis labels (time)
//   plotW x plotH    — line area
//
// Y axis: max value at top, min at bottom, mid in centre.
// X axis: up to 6 evenly-spaced HH:MM labels.
function drawSparkline(canvasId, points, color, unit, minVal, maxVal){
  var cv=document.getElementById(canvasId);
  if(!cv||!points||points.length<2) return;
  var dpr=window.devicePixelRatio||1;
  var LEFT=34, AXIS=18, CH=110;
  cv.width=cv.offsetWidth*dpr; cv.height=CH*dpr;
  var ctx=cv.getContext('2d'); ctx.scale(dpr,dpr);
  var W=cv.offsetWidth;
  var plotW=W-LEFT, plotH=CH-AXIS;

  var vals=points.map(function(p){return p.v;});
  var mn=minVal!=null?minVal:Math.min.apply(null,vals);
  var mx=maxVal!=null?maxVal:Math.max.apply(null,vals);
  if(mn===mx){mn-=1;mx+=1;}

  ctx.clearRect(0,0,W,CH);

  // ── Faint vertical axis line ─────────────────────────────
  ctx.strokeStyle='rgba(255,255,255,0.06)';ctx.lineWidth=0.5;
  ctx.beginPath();ctx.moveTo(LEFT,0);ctx.lineTo(LEFT,plotH);ctx.stroke();

  // ── Y axis labels (max / mid / min) ──────────────────────
  ctx.fillStyle='rgba(136,146,164,0.9)';
  ctx.font='10px sans-serif';ctx.textAlign='right';
  var mid=((mx+mn)/2);
  function fmtY(v){
    // Round to 1 decimal only if the range is small, else integer
    return (mx-mn<20?v.toFixed(1):Math.round(v))+(unit||'');
  }
  ctx.fillText(fmtY(mx),  LEFT-3, 10);           // top
  ctx.fillText(fmtY(mid), LEFT-3, plotH/2+4);    // middle
  ctx.fillText(fmtY(mn),  LEFT-3, plotH-2);      // bottom

  // ── Horizontal guide lines ───────────────────────────────
  ctx.strokeStyle='rgba(255,255,255,0.04)';ctx.lineWidth=0.5;
  [0, 0.5, 1].forEach(function(frac){
    var y=plotH-(frac*(plotH-16))-4;
    ctx.beginPath();ctx.moveTo(LEFT,y);ctx.lineTo(W,y);ctx.stroke();
  });

  // ── Filled line ──────────────────────────────────────────
  ctx.beginPath();
  points.forEach(function(p,i){
    var x=LEFT+i/(points.length-1)*plotW;
    var y=plotH-(p.v-mn)/(mx-mn)*(plotH-16)-4;
    i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
  });
  ctx.strokeStyle=color;ctx.lineWidth=2;ctx.stroke();
  ctx.lineTo(LEFT+plotW,plotH);ctx.lineTo(LEFT,plotH);ctx.closePath();
  ctx.fillStyle=color.replace(')',',0.12)').replace('rgb','rgba');ctx.fill();

  // ── Baseline ─────────────────────────────────────────────
  ctx.strokeStyle='rgba(255,255,255,0.07)';ctx.lineWidth=0.5;
  ctx.beginPath();ctx.moveTo(LEFT,plotH);ctx.lineTo(W,plotH);ctx.stroke();

  // ── X axis time labels ───────────────────────────────────
  if(!points[0].ts) return;
  ctx.fillStyle='rgba(136,146,164,0.9)';ctx.font='10px sans-serif';
  var numLabels=Math.min(6,points.length);
  for(var li=0;li<numLabels;li++){
    var idx=Math.round(li/(numLabels-1)*(points.length-1));
    var x=LEFT+idx/(points.length-1)*plotW;
    ctx.textAlign=li===0?'left':li===numLabels-1?'right':'center';
    ctx.fillText(fmtHHMM(points[idx].ts),x,CH-3);
  }
}

// ── Dirty flag ────────────────────────────────────────────────
var settingsDirty=false;
function markDirty(){settingsDirty=true;$('dirtyNote').style.display='inline';}
function clearDirty(){settingsDirty=false;$('dirtyNote').style.display='none';}

// ── State banner ──────────────────────────────────────────────
function updateStateBanner(state, nextReadIn){
  var b=$('stateBanner');
  b.className='state-banner';
  if(state==='Sleeping'){
    var mins=Math.floor(nextReadIn/60), secs=nextReadIn%60;
    var countdown=nextReadIn>0?(mins>0?mins+'m ':'')+(secs+'s'):'soon';
    b.textContent='Sleeping — next check in '+countdown;
    b.classList.add('state-sleep');
  } else if(state==='Pump running'){
    b.textContent='⚡ Pump is running';
    b.classList.add('state-pump');
  } else if(state==='Cooldown'){
    b.textContent='Cooldown — re-checking moisture after wait';
    b.classList.add('state-cool');
  } else {
    b.textContent='Reading sensors...';
    b.classList.add('state-read');
  }
}

// ── applyStatus ───────────────────────────────────────────────
function applyStatus(d){
  csrf=d.csrf||csrf;
  $('hTitle').textContent=d.device+' Dashboard';
  $('devTime').textContent=d.time;
  $('firmware').textContent='v'+d.firmware;
  $('uptimeEl').textContent='Uptime: '+uptimeStr(d.uptime||0);

  updateStateBanner(d.appState||'', d.nextReadIn||0);

  $('sensorWarn').style.display=d.rawOk===false?'block':'none';

  $('moisture').textContent=d.moisture;
  $('bar').style.width=d.moisture+'%';
  $('raw').textContent=d.raw+(d.rawOk===false?' ⚠':'');
  $('dryCal').textContent=d.dryCal;
  $('wetCal').textContent=d.wetCal;

  var b=$('calBadge');
  if(d.calStatus===2){b.textContent='Calibrated';b.className='badge ok';}
  else if(d.calStatus===1){b.textContent=d.dryCalSet?'Dry set — need wet':'Wet set — need dry';b.className='badge warn';}
  else{b.textContent='Needs calibration';b.className='badge warn';}

  $('calDryBtn').className=d.dryCalSet?'b-grn':'b-red';
  $('calWetBtn').className=d.wetCalSet?'b-grn':'b-red';

  $('sensorLabel').textContent=d.sensorType+(d.sensorOnline?'':' (Offline)');
  $('ambTemp').textContent=d.sensorOnline?(d.ambientTemp||'--'):'No sensor';
  $('ambHum').textContent=d.sensorOnline?(d.ambientHum||'--'):'No sensor';

  var ct=parseFloat(d.chipTemp);
  var chipEl=$('chipTemp');
  chipEl.textContent=d.chipTemp;
  chipEl.style.color=ct>=176?'#f76e6e':ct>=150?'#f7b84f':'#3ecf8e';

  var lr=d.lastRead||'--';
  $('lastRead').textContent=$('lastRead2').textContent=$('lastRead3').textContent=lr;

  var ps=$('pumpStatus');
  if(d.pumpRunning){ps.textContent='AUTO RUNNING ⚡';ps.className='pump-big p-on';}
  else if(d.pumpManual){ps.textContent='RUNNING (Manual)';ps.className='pump-big p-on';}
  else{ps.textContent='OFF';ps.className='pump-big p-off';}

  $('pumpLastRan').textContent=d.pumpLastRan||'Never';
  var cl=d.pumpCooldownLeft||0;
  if(d.pumpRunning)$('pumpCooldownLeft').textContent='Pump running...';
  else if(cl>0){var m=Math.floor(cl/60),s=cl%60;$('pumpCooldownLeft').textContent=(m>0?m+'m ':'')+s+'s remaining';}
  else $('pumpCooldownLeft').textContent='--';

  if(!settingsDirty){
    $('pumpThreshold').value=d.pumpThreshold;
    $('pumpDuration').value=d.pumpDuration;
    $('pumpCooldown').value=d.pumpCooldown;
    $('pumpAutoChk').checked=d.pumpAuto;
    $('pumpAutoLabel').textContent=d.pumpAuto?'Enabled':'Disabled';
    $('pumpAutoLabel').style.color=d.pumpAuto?'var(--grn)':'var(--dim)';
  }

  $('ssid').textContent=d.ssid;
  $('ip').textContent=d.ip;
  $('mac').textContent=d.mac;
  $('rssi').textContent=d.rssi;
  $('quality').textContent=d.quality;
  $('mdnsLabel').textContent=d.mdns+'.local';

  if(d.history&&d.history.length>1){
    var moistPts=d.history.map(function(p){return{v:p.m,ts:p.ts||0};});
    var tempPts=d.history.filter(function(p){return p.t!=null;})
                          .map(function(p){return{v:parseFloat(p.t),ts:p.ts||0};});
    var humPts=d.history.filter(function(p){return p.h!=null;})
                         .map(function(p){return{v:parseFloat(p.h),ts:p.ts||0};});
    drawSparkline('moistChart', moistPts, 'rgb(79,142,247)',  '%',  0, 100);
    if(tempPts.length>1) drawSparkline('tempChart', tempPts, 'rgb(247,184,79)',  '°F');
    if(humPts.length>1)  drawSparkline('humChart',  humPts,  'rgb(62,207,142)', '%',  0, 100);
  }

  $('upd').textContent='Updated: '+new Date().toLocaleTimeString();
}

// ── Adaptive poll interval ─────────────────────────────────────
// 3 s while pump is running or in cooldown; 30 s while sleeping or reading.
// The 30 s poll is frequent enough to get fresh data soon after the
// device wakes from its 30-minute sleep, without hammering it during rest.
var pollInterval = 30000;
var pollTimer = null;

function setPollInterval(ms){
  if(ms===pollInterval) return;
  pollInterval=ms;
  clearInterval(pollTimer);
  pollTimer=setInterval(update,pollInterval);
}

function update(){
  fetch('/status')
    .then(function(r){return r.json();})
    .then(function(d){
      applyStatus(d);
      var active=(d.appState==='Pump running'||d.appState==='Cooldown');
      setPollInterval(active?3000:30000);
    })
    .catch(function(){$('upd').textContent='Connection lost — retrying...';});
}

// ── Manual refresh ─────────────────────────────────────────────
function refreshSensors(){
  $('upd').textContent='Refreshing...';
  api('/sensor/refresh')
    .then(function(r){return r.json();})
    .then(function(d){applyStatus(d);$('upd').textContent='Manually refreshed: '+new Date().toLocaleTimeString();})
    .catch(function(){toast('Refresh failed');});
}

// ── Async WiFi scan ────────────────────────────────────────────
var scanTimer=null;
function signalBars(r){return r>-50?'||||':r>-60?'||| ':r>-70?'||  ':'|   ';}

function doScan(){
  $('scanStatus').textContent='Starting scan...';$('netList').innerHTML='';
  fetch('/wifi/scan/start').then(function(){
    $('scanStatus').textContent='Scanning... (5-10 s)';
    clearInterval(scanTimer);
    scanTimer=setInterval(pollScan,2000);
  }).catch(function(){$('scanStatus').textContent='Scan request failed';});
}

function pollScan(){
  fetch('/wifi/scan/result').then(function(r){return r.json();}).then(function(d){
    if(d.status==='scanning') return;
    clearInterval(scanTimer);
    if(d.status==='error'||!d.networks){$('scanStatus').textContent='Scan failed';return;}
    $('scanStatus').textContent=d.networks.length+' network'+(d.networks.length!==1?'s':'')+' found';
    d.networks.forEach(function(n){
      var div=document.createElement('div');div.className='net-item';
      div.innerHTML='<span>'+n.ssid+(n.secure?'<span class="lock">&#x1F512;</span>':'')+'</span>'
        +'<span style="font-size:.75rem;color:var(--dim)">'+signalBars(n.rssi)+' '+n.rssi+' dBm</span>';
      div.onclick=function(){$('wifiSSID').value=n.ssid;$('wifiPass').focus();};
      $('netList').appendChild(div);
    });
  }).catch(function(){clearInterval(scanTimer);$('scanStatus').textContent='Poll failed';});
}

function changeWiFi(){
  var ssid=$('wifiSSID').value.trim(),pass=$('wifiPass').value;
  if(!ssid){toast('Enter or select a network first');return;}
  if(!confirm('Connect to "'+ssid+'"? Device will reboot.')) return;
  var fd=new FormData();fd.append('ssid',ssid);fd.append('pass',pass);
  fetch('/wifi/change',{method:'POST',body:fd})
    .then(function(){toast('Saved. Rebooting...',4000);});
}

function calDry(){api('/cal/dry').then(function(r){return r.json();}).then(function(d){toast('Dry point saved: '+d.dry);update();});}
function calWet(){api('/cal/wet').then(function(r){return r.json();}).then(function(d){toast('Wet point saved: '+d.wet);update();});}
function calReset(){if(!confirm('Reset calibration to factory defaults?')) return;api('/cal/reset').then(function(){toast('Calibration reset');update();});}

function pumpOn() {api('/pump/on').then(update);}
function pumpOff(){api('/pump/off').then(update);}

function savePumpSettings(){
  api('/pump/settings?auto='+($('pumpAutoChk').checked?'1':'0')
    +'&threshold='+$('pumpThreshold').value
    +'&duration='+$('pumpDuration').value
    +'&cooldown='+$('pumpCooldown').value)
    .then(function(r){return r.json();})
    .then(function(d){if(d.ok){toast('Settings saved');clearDirty();}else toast('Save failed');update();});
}

function renameDevice(){
  var name=$('renameInput').value.trim();
  if(!name){toast('Enter a name first');return;}
  api('/device/rename?name='+encodeURIComponent(name))
    .then(function(r){return r.json();})
    .then(function(d){if(d.ok){toast('Renamed to: '+d.name);$('renameInput').value='';update();}else toast('Error: '+(d.error||'Unknown'));});
}

function resetWiFi(){
  if(!confirm('Clear WiFi credentials and reboot into setup mode?')) return;
  api('/wifi/reset').then(function(){toast('Rebooting into setup mode...',4000);});
}

// ── Start polling ──────────────────────────────────────────────
pollTimer=setInterval(update,pollInterval);
update();
</script>
</body>
</html>
)DASH");
}

// ============================================================
//  ROUTE REGISTRATION
// ============================================================
void setupRoutes() {
  server.on("/",                  HTTP_GET,  handleRoot);
  server.on("/status",            HTTP_GET,  handleStatus);
  server.on("/health",            HTTP_GET,  handleHealth);
  server.on("/sensor/refresh",    HTTP_GET,  handleSensorRefresh);
  server.on("/wifi/scan/start",   HTTP_GET,  handleWiFiScanStart);
  server.on("/wifi/scan/result",  HTTP_GET,  handleWiFiScanResult);
  server.on("/wifi/change",       HTTP_POST, handleWiFiChange);
  server.on("/wifi/reset",        HTTP_GET,  handleWiFiReset);
  server.on("/cal/dry",           HTTP_GET,  handleCalDry);
  server.on("/cal/wet",           HTTP_GET,  handleCalWet);
  server.on("/cal/reset",         HTTP_GET,  handleCalReset);
  server.on("/pump/on",           HTTP_GET,  handlePumpOn);
  server.on("/pump/off",          HTTP_GET,  handlePumpOff);
  server.on("/pump/settings",     HTTP_GET,  handlePumpSettings);
  server.on("/device/rename",     HTTP_GET,  handleRename);
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });
}

// ============================================================
//  ArduinoOTA
// ============================================================
void setupArduinoOTA() {
  ArduinoOTA.setHostname(sanitizeMDNS(deviceName).c_str());
  // ArduinoOTA.setPassword("your-ota-password");

  ArduinoOTA.onStart([]() {
    // WDT already removed before sleeping; remove if somehow still subscribed
    esp_task_wdt_delete(NULL);
    Serial.println("[ArduinoOTA] Start");
  });
  ArduinoOTA.onEnd([]()  { Serial.println("\n[ArduinoOTA] Done"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    Serial.printf("[ArduinoOTA] %u%%\r", p * 100 / t);
  });
  ArduinoOTA.onError([](ota_error_t err) {
    const char* msgs[] = {"","Auth failed","Begin failed","Connect failed","Receive failed","End failed"};
    Serial.printf("[ArduinoOTA] Error[%u]: %s\n", err, err <= 5 ? msgs[err] : "Unknown");
  });

  ArduinoOTA.begin();
  otaRunning = true;
  Serial.println("[ArduinoOTA] Ready on port 3232");
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] ESP32-S3-N16R8 Plant Monitor v" FIRMWARE_VERSION);
  bootMillis = millis();

  // ── Hardware watchdog (30 s) ───────────────────────────────
  // Note: WDT task is removed before light sleep and re-added on wakeup.
  esp_task_wdt_config_t wdt_cfg = {
    .timeout_ms   = 30000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_task_wdt_init(&wdt_cfg);
  esp_task_wdt_add(NULL);
  Serial.println("[WDT] Enabled — 30 s timeout");

  // ── GPIO ──────────────────────────────────────────────────
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, PUMP_OFF);

  // ── ADC ───────────────────────────────────────────────────
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // ── NVS ───────────────────────────────────────────────────
  loadPrefs();

  // Restore manual pump state only
  if (pumpManualState) {
    digitalWrite(PUMP_PIN, PUMP_ON);
    Serial.println("[Pump] Restored manual ON state");
  }

  // ── CSRF ──────────────────────────────────────────────────
  generateCsrfToken();

  // ── Sensors ───────────────────────────────────────────────
#if USE_AHT30
  initAHT30();
#endif
#if USE_DHT22
  dht.begin();
  Serial.println("[DHT22] Init done");
#endif

  // ── WiFi ──────────────────────────────────────────────────
  String ssid, pass;
  loadWiFiCreds(ssid, pass);
  bool connected = !ssid.isEmpty() && connectWiFi();

  if (!connected) {
    Serial.println(ssid.isEmpty()
      ? "[WiFi] No credentials — starting AP portal"
      : "[WiFi] Could not connect — starting AP portal");
    startAPPortal();
    return;
  }

  // ── NTP ───────────────────────────────────────────────────
  setupTime();

  // ── Power management (modem + auto CPU light sleep) ───────
  setupPowerManagement();

  // ── mDNS ──────────────────────────────────────────────────
  startMDNS();

  // ── ArduinoOTA ────────────────────────────────────────────
  setupArduinoOTA();

  // ── Web server ────────────────────────────────────────────
  setupRoutes();
  ElegantOTA.begin(&server, OTA_USER, OTA_PASS);
  server.begin();
  Serial.println("[HTTP] Server started");

  // ── Enter state machine at READ ───────────────────────────
  transitionTo(STATE_READ);
}

// ============================================================
//  LOOP  —  state machine
// ============================================================
void loop() {
  // ── Always service the web server ─────────────────────────
  // In SLEEP state esp_light_sleep_start() returns immediately on any
  // WiFi packet, so handleClient() is called within one loop iteration
  // of the HTTP request arriving.
  server.handleClient();
  ElegantOTA.loop();
  if (otaRunning) ArduinoOTA.handle();
  esp_task_wdt_reset();

  // ── WiFi reconnect watchdog ────────────────────────────────
  if (!inAPMode
      && WiFi.getMode() == WIFI_STA
      && WiFi.status() != WL_CONNECTED
      && millis() - lastWiFiRetry >= WIFI_RETRY_INTERVAL)
  {
    lastWiFiRetry = millis();
    Serial.println("[WiFi] Reconnecting...");
    String ssid, pass;
    loadWiFiCreds(ssid, pass);
    WiFi.begin(ssid.c_str(), pass.c_str());
    setupPowerManagement();
  }

  // ── State machine ─────────────────────────────────────────
  switch (appState) {

    // ── READ ────────────────────────────────────────────────
    case STATE_READ: {
      readSensors();

      if (pumpAuto && moistureRawPlausible(moistureRaw) && moisturePct < pumpThreshold) {
        // Soil is too dry — start the pump
        pumpRunning   = true;
        pumpStartedAt = millis();
        digitalWrite(PUMP_PIN, PUMP_ON);
        Serial.printf("[Pump] Starting — moisture %d%% < threshold %d%%\n",
                      moisturePct, pumpThreshold);
        transitionTo(STATE_PUMP_RUN);
      } else {
        // Soil is wet enough (or auto is off) — go to sleep
        Serial.printf("[Pump] Moisture OK (%d%%) or auto off — sleeping\n", moisturePct);
        nextReadAt = millis() + (unsigned long)SLEEP_DURATION_SEC * 1000UL;
        transitionTo(STATE_SLEEP);
      }
      break;
    }

    // ── PUMP_RUN ────────────────────────────────────────────
    case STATE_PUMP_RUN: {
      unsigned long elapsed = (millis() - pumpStartedAt) / 1000UL;
      if ((int)elapsed >= pumpDuration) {
        // Pump duration complete — stop pump, start cooldown
        pumpRunning     = false;
        pumpLastRan     = millis();
        pumpLastRanTime = getTimeString();
        digitalWrite(PUMP_PIN, PUMP_OFF);
        Serial.printf("[Pump] Done after %d s — cooling down for %d s\n",
                      pumpDuration, pumpCooldown);
        transitionTo(STATE_PUMP_COOL);
      }
      // Keep web server alive while waiting
      delay(50);
      break;
    }

    // ── PUMP_COOL ───────────────────────────────────────────
    case STATE_PUMP_COOL: {
      unsigned long elapsed = (millis() - stateEnteredAt) / 1000UL;
      if ((int)elapsed >= pumpCooldown) {
        // Cooldown done — read sensors again to decide if another pump cycle is needed
        Serial.println("[State] Cooldown done — re-reading sensors");
        transitionTo(STATE_READ);
      }
      // Keep web server alive while waiting
      delay(50);
      break;
    }

    // ── SLEEP ───────────────────────────────────────────────
    case STATE_SLEEP: {
      // Has the scheduled read time arrived?
      if (millis() >= nextReadAt) {
        Serial.println("[Sleep] Wake time reached — reading sensors");
        transitionTo(STATE_READ);
        break;
      }

      // Yield for 50 ms. With automatic light sleep enabled, the CPU
      // sleeps during this gap whenever FreeRTOS has nothing else to do.
      // The WiFi modem wakes the CPU instantly on any incoming packet,
      // so the web server (handled at the top of loop()) stays responsive.
      // No blocking, no missed HTTP requests.
      delay(50);
      break;
    }
  }
}
