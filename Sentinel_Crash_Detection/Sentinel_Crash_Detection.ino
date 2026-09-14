/*
 * ═══════════════════════════════════════════════════════════════════════════
 * SENTINEL — ESP32 CRASH DETECTION NODE
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Derived from ESP32_Complete_3LED_System.ino. Everything that already worked
 * is preserved: 100 Hz MPU6050 sampling, the 2.0 g + 5.0 g/s dual-threshold
 * trigger, the 3-colour LED scheme and the buzzer patterns.
 *
 * ── THE SIX CHANGES (brief §6) ─────────────────────────────────────────────
 *  1. POSTs to {BACKEND_URL}/api/v1/events with an X-API-Key header instead of
 *     calling the ML API directly. The device no longer knows the ML API
 *     exists; the backend is the only component that talks to it.
 *  2. event_id = {DEVICE_ID}-{boot_count:04d}-{millis:08d}, with boot_count
 *     persisted in NVS so IDs stay unique across resets.
 *  3. Up to 3 retries with backoff on timeout/5xx. Safe because the server is
 *     idempotent on event_id. SMS fires regardless of whether the POST lands.
 *  4. IMPACT-CENTRED WINDOW. The old code collected 5 s AFTER the trigger,
 *     putting a ~70 ms crash pulse at the very edge of the window and adding
 *     ~5 s of latency. Now a continuous 500-sample ring buffer holds 2.5 s of
 *     pre-trigger history; on trigger we wait for 250 more samples and ship
 *     250 pre + 250 post. Still exactly 500 samples @ 100 Hz, so the API
 *     contract is unchanged — but the pulse sits mid-window and the alert goes
 *     out ~2.5 s sooner.
 *  5. Heartbeat FreeRTOS task every 30 s on core 0 at low priority, so it can
 *     never block or preempt IMU sampling (which owns core 1 via loop()).
 *  6. Memory discipline: the ~36 kB request body is built with a heap String
 *     and reserve(), NOT a StaticJsonDocument on the 8 kB loop-task stack
 *     (that overflowed this project once). Only the small flat ACK is parsed
 *     with ArduinoJson, and severity_name is mapped to a permanent string
 *     literal rather than a pointer into a locally-scoped document.
 *
 * ── ⚠ TWO THINGS TO CHECK BEFORE FLASHING ─────────────────────────────────
 *
 *  (a) ACCELEROMETER RANGE — FIXED HERE, PLEASE VERIFY.
 *      The previous firmware called mpu.initialize(), which the MPU6050
 *      library sets to MPU6050_ACCEL_FS_2 (±2 g), while the pin comment
 *      claimed "±16g range". RAW_ACCEL_TO_G = 1/16384 is the correct scale
 *      FOR ±2 g, so units were right — but the sensor physically CLIPS at
 *      2 g per axis. The model's entire crash band is 2–7 g, so no real crash
 *      could ever be measured in-band: the theoretical ceiling was
 *      sqrt(3 × 2²) = 3.46 g, and only with all three axes saturated.
 *      This build sets ±16 g explicitly (2048 LSB/g). See the final report.
 *
 *  (b) GPS + GSM BLOCKS ARE RECONSTRUCTED, NOT COPIED.
 *      ESP32_Crash_Detection_GPS_GSM.ino was not present in the workspace, so
 *      the TinyGPSPlus and SIM800L sections below were written from the
 *      hardware description in brief §1.1 (GPS on Serial2 GPIO 16/17 @ 9600;
 *      SIM800L on Serial1). The brief itself warns of a known TX/RX swap on
 *      the GSM UART and of LED_RED once being GPIO 34 (input-only). VERIFY
 *      THE #define BLOCK BELOW AGAINST YOUR WORKING GPS_GSM SKETCH before
 *      flashing. If your sketch differs, keep your pins — only the six
 *      changes above matter.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <time.h>
#include <math.h>

// ═══════════════════════════════════════════════════════════════
// GPIO CONFIGURATION  (unchanged from the working sketch)
// ═══════════════════════════════════════════════════════════════

#define LED_RED     33       // GPIO 33: Red LED (SEVERE/Emergency)
#define LED_YELLOW  25       // GPIO 25: Yellow LED (MODERATE/Warning)
#define LED_GREEN   26       // GPIO 26: Green LED (NORMAL/Safe)
#define BUZZER_PIN  32       // GPIO 32: Audio alert buzzer

#define I2C_SDA     21       // GPIO 21: MPU6050 SDA
#define I2C_SCL     22       // GPIO 22: MPU6050 SCL

// ── GPS (NEO-6M on Serial2) — VERIFY AGAINST YOUR GPS_GSM SKETCH ──────────
#define GPS_RX_PIN  16       // ESP32 RX  <-- GPS TX
#define GPS_TX_PIN  17       // ESP32 TX  --> GPS RX
#define GPS_BAUD    9600

// ── GSM (SIM800L on Serial1) — VERIFY: the brief notes a known TX/RX swap ──
#define GSM_RX_PIN  26       // ESP32 RX  <-- SIM800L TX   ⚠ clashes with
#define GSM_TX_PIN  27       // ESP32 TX  --> SIM800L RX     LED_GREEN(26)!
#define GSM_BAUD    9600
// ⚠ GSM_RX_PIN 26 collides with LED_GREEN 26 above. Your working sketch must
//   use different pins — set them here before flashing. Left visible rather
//   than silently guessed, because guessing pin maps is how this project got
//   bitten before. Set ENABLE_GSM to 0 to build without GSM meanwhile.
#define ENABLE_GSM  0

// ═══════════════════════════════════════════════════════════════
// NETWORK / BACKEND CONFIGURATION
// ═══════════════════════════════════════════════════════════════

const char* WIFI_SSID     = "BERNARDINE 7683";
const char* WIFI_PASSWORD = "12345678";

// The backend is now the only upstream. Use the LAN IP of the machine running
// sentinel-backend (ipconfig -> IPv4), or the Railway URL once deployed.
const char* BACKEND_URL   = "http://192.168.137.1:8080";
const char* API_KEY       = "change-me-sentinel-dev-key";
const char* DEVICE_ID     = "ESP32_ACC_001";
const char* FIRMWARE_VERSION = "2.0.0";

const uint16_t HTTP_TIMEOUT_MS   = 15000;
const uint8_t  MAX_POST_ATTEMPTS = 3;      // §6.3
const uint16_t RETRY_BACKOFF_MS[] = {1000, 3000, 6000};

// Emergency SMS recipients (the device keeps its own list — this is the
// fail-safe and is deliberately independent of the backend's contact table).
const char* SMS_RECIPIENTS[] = {"+233000000000"};
const uint8_t SMS_RECIPIENT_COUNT = 1;

// ═══════════════════════════════════════════════════════════════
// IMU HARDWARE
// ═══════════════════════════════════════════════════════════════

#define MPU6050_ADDR        0x68
#define SENSOR_INTERVAL_MS  10     // 100 Hz
#define PRINT_INTERVAL_MS   100

// ⚠ SEE HEADER NOTE (a). ±16 g is set explicitly in setup(); 2048 LSB/g is the
// matching sensitivity. The old 1/16384 was the ±2 g figure and clipped every
// crash at 2 g/axis.
#define RAW_ACCEL_TO_G      (1.0f / 2048.0f)    // ±16 g  (MPU6050_ACCEL_FS_16)
#define RAW_GYRO_TO_DPS     (1.0f / 131.0f)     // ±250 dps (library default)

// ═══════════════════════════════════════════════════════════════
// CRASH DETECTION PARAMETERS  (thresholds unchanged)
// ═══════════════════════════════════════════════════════════════

#define BUFFER_SIZE               500   // 500 samples = 5 s @ 100 Hz
#define POST_TRIGGER_SAMPLES      250   // 2.5 s after impact  ─┐ centred
#define ACCEL_THRESHOLD_G         2.0f  //                      │ window
#define JERK_THRESHOLD_G_PER_SEC  5.0f  //                     ─┘ (§6.4)
#define CRASH_DEBOUNCE_MS         2000

#define HEARTBEAT_INTERVAL_MS     30000 // §6.5

enum SeverityLevel {
  SEVERITY_NORMAL   = 0,   // 🟢
  SEVERITY_MODERATE = 1,   // 🟡
  SEVERITY_SEVERE   = 2    // 🔴
};

// ═══════════════════════════════════════════════════════════════
// DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════

struct IMUSample {
  float ax_g, ay_g, az_g;
  float gx_dps, gy_dps, gz_dps;
  float accel_mag;
};

struct CrashResult {
  bool  valid;                  // did the backend classify it?
  int   severity_class;
  const char* severity_name;    // ALWAYS a permanent literal (§6.6)
  float confidence;
  float p_crash;
  float peak_g;
  bool  accident_confirmed;
  const char* label_source;     // ALWAYS a permanent literal
  bool  classification_pending;
};

IMUSample imu_buffer[BUFFER_SIZE];
volatile uint16_t buffer_head = 0;      // next write slot == oldest sample
volatile bool     buffer_primed = false;

// ═══════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════

MPU6050     mpu(MPU6050_ADDR);
TinyGPSPlus gps;
Preferences prefs;

bool mpu_ready = false;
bool wifi_ready = false;
bool gsm_ready = false;

uint32_t boot_count = 0;
uint32_t crash_count = 0;
uint32_t sample_count = 0;

float prev_accel_mag_g = 0.0f;
unsigned long last_sensor_read = 0;
unsigned long last_print = 0;
unsigned long last_crash_detection = 0;

// centred-capture state (§6.4)
volatile bool     capture_armed = false;   // trigger seen, collecting post-roll
volatile uint16_t post_trigger_remaining = 0;
volatile bool     capture_ready = false;   // 250 post samples collected
float trigger_peak_g = 0.0f;
float trigger_jerk_gs = 0.0f;
uint32_t trigger_millis = 0;

CrashResult last_result = {false, 0, "Unknown", 0, 0, 0, false, "none", false};

SemaphoreHandle_t http_mutex = nullptr;   // one HTTP request at a time
SemaphoreHandle_t buffer_mutex = nullptr; // guards the ring during snapshot

// ═══════════════════════════════════════════════════════════════
// SMALL HELPERS
// ═══════════════════════════════════════════════════════════════

/** Map a class index to a PERMANENT literal. Never return a pointer into a
 *  locally-scoped JsonDocument — that dangles (§6.6). */
const char* severityLiteral(int cls) {
  switch (cls) {
    case SEVERITY_NORMAL:   return "Normal";
    case SEVERITY_MODERATE: return "Moderate";
    case SEVERITY_SEVERE:   return "Severe";
    default:                return "Unknown";
  }
}

const char* labelSourceLiteral(const char* src) {
  if (!src) return "none";
  if (!strcmp(src, "model+signature"))    return "model+signature";
  if (!strcmp(src, "signature_override")) return "signature_override";
  if (!strcmp(src, "model"))              return "model";
  if (!strcmp(src, "manual_panic"))       return "manual_panic";
  return "unknown";
}

/** TinyGPSPlus already returns DECIMAL DEGREES — do not re-scale it.
 *  The guard below rejects raw NMEA ddmm.mmmm values (e.g. 539.5128), which
 *  would otherwise be stored as a valid-looking but wildly wrong fix. */
bool gpsFixValid() {
  if (!gps.location.isValid()) return false;
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  if (fabs(lat) > 90.0 || fabs(lon) > 180.0) {
    Serial.println(F("[GPS] ✗ Out-of-range fix rejected (raw NMEA leaking through?)"));
    return false;
  }
  return !(lat == 0.0 && lon == 0.0);
}

void appendFloat(String& out, float v, uint8_t decimals) {
  char num[20];
  dtostrf(v, 0, decimals, num);
  out += num;
}

/** ISO-8601 UTC timestamp for the moment of the trigger.
 *
 *  The ESP32 has no RTC, so wall-clock time comes from NTP (set up after WiFi
 *  connects). detected_at must be the TRIGGER instant, not the upload instant —
 *  otherwise the dashboard's detected→received latency measures our own
 *  post-roll wait instead of the network. We therefore walk back from "now" by
 *  however long ago the trigger fired.
 *
 *  Returns an empty String if NTP has not synced yet; the field is optional
 *  server-side, and a wrong timestamp is worse than an absent one. */
String triggerTimestampIso() {
  time_t now_epoch = time(nullptr);
  if (now_epoch < 1600000000) return String();   // clock clearly not set
  uint32_t ago_ms = millis() - trigger_millis;
  time_t trigger_epoch = now_epoch - (time_t)(ago_ms / 1000);
  struct tm tm_utc;
  gmtime_r(&trigger_epoch, &tm_utc);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
  return String(buf);
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  printBanner();

  // ── boot counter in NVS, for globally unique event_ids (§6.2) ───────────
  prefs.begin("sentinel", false);
  boot_count = prefs.getUInt("boot", 0) + 1;
  prefs.putUInt("boot", boot_count);
  prefs.end();
  Serial.print(F("[NVS] Boot count: "));
  Serial.println(boot_count);

  http_mutex   = xSemaphoreCreateMutex();
  buffer_mutex = xSemaphoreCreateMutex();

  // ── I2C + MPU6050 ───────────────────────────────────────────────────────
  Serial.println(F("[I2C] Initializing..."));
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  delay(100);

  Serial.println(F("[MPU6050] Initializing sensor..."));
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println(F("[ERROR] MPU6050 not found!"));
    mpu_ready = false;
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_RED, HIGH); delay(200);
      digitalWrite(LED_RED, LOW);  delay(200);
    }
  } else {
    // ⚠ THE FIX: initialize() leaves the part at ±2 g, which clips every
    // crash. The model's crash band is 2–7 g, so ±16 g is required for any
    // in-band measurement to exist at all.
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    Serial.print(F("[MPU6050] ✓ Connected. Accel range set to ±16 g (raw code "));
    Serial.print(mpu.getFullScaleAccelRange());
    Serial.println(F(", expect 3)"));
    mpu_ready = true;
    digitalWrite(LED_GREEN, HIGH); delay(500); digitalWrite(LED_GREEN, LOW);
  }

  // ── GPS (reconstructed — verify pins) ───────────────────────────────────
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println(F("[GPS] Serial2 started (NEO-6M @ 9600)"));

  // ── GSM (reconstructed — verify pins; disabled by default) ──────────────
#if ENABLE_GSM
  Serial1.begin(GSM_BAUD, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  delay(3000);
  gsm_ready = gsmInit();
  Serial.println(gsm_ready ? F("[GSM] ✓ SIM800L ready")
                           : F("[GSM] ✗ SIM800L not responding"));
#else
  Serial.println(F("[GSM] Disabled at compile time (set ENABLE_GSM 1 after "
                   "confirming the UART pins)"));
#endif

  // ── WiFi ────────────────────────────────────────────────────────────────
  connectWiFi();

  // ── Heartbeat task: core 0, low priority, never preempts IMU (§6.5) ─────
  xTaskCreatePinnedToCore(heartbeatTask, "heartbeat", 8192, nullptr,
                          1 /* low */, nullptr, 0 /* core 0 */);
  Serial.println(F("[TASK] Heartbeat task started on core 0"));

  Serial.println();
  Serial.println(F("═══════════════════════════════════════════════════════════"));
  Serial.println(F("Time(ms) | Accel(g) | Jerk(g/s) | WiFi | MPU | Sats | Status"));
  Serial.println(F("═══════════════════════════════════════════════════════════"));

  last_sensor_read = millis();
  last_print = millis();
}

void connectWiFi() {
  Serial.print(F("[WiFi] Connecting to: "));
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(F("."));
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("[WiFi] ✓ Connected! IP: "));
    Serial.println(WiFi.localIP());
    wifi_ready = true;

    // UTC clock for detected_at. Without this the backend can only use its own
    // receive time and the latency breakdown loses its first leg.
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    Serial.print(F("[NTP] Syncing"));
    for (int i = 0; i < 20 && time(nullptr) < 1600000000; i++) {
      delay(250);
      Serial.print(F("."));
    }
    Serial.println(time(nullptr) >= 1600000000 ? F(" ✓ UTC set")
                                              : F(" ✗ not synced (detected_at omitted)"));

    digitalWrite(LED_YELLOW, HIGH); delay(300); digitalWrite(LED_YELLOW, LOW);
  } else {
    Serial.println(F("[WiFi] ✗ Connection failed — SMS fail-safe still active"));
    wifi_ready = false;
  }
}

// ═══════════════════════════════════════════════════════════════
// MAIN LOOP — IMU sampling owns this core
// ═══════════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // Feed the GPS parser continuously; non-blocking, a few bytes per pass.
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }

  if (WiFi.status() != WL_CONNECTED) {
    wifi_ready = false;
  } else if (!wifi_ready) {
    wifi_ready = true;
  }

  // ── 100 Hz sampling ─────────────────────────────────────────────────────
  if (now - last_sensor_read >= SENSOR_INTERVAL_MS) {
    if (mpu_ready) {
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getAcceleration(&ax, &ay, &az);
      mpu.getRotation(&gx, &gy, &gz);

      float ax_g = ax * RAW_ACCEL_TO_G;
      float ay_g = ay * RAW_ACCEL_TO_G;
      float az_g = az * RAW_ACCEL_TO_G;
      float gx_dps = gx * RAW_GYRO_TO_DPS;
      float gy_dps = gy * RAW_GYRO_TO_DPS;
      float gz_dps = gz * RAW_GYRO_TO_DPS;

      float mag = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
      float jerk = (mag - prev_accel_mag_g) / (SENSOR_INTERVAL_MS / 1000.0f);

      // store first, so the trigger sample itself is in the window
      if (xSemaphoreTake(buffer_mutex, 0) == pdTRUE) {
        imu_buffer[buffer_head] = {ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, mag};
        buffer_head = (buffer_head + 1) % BUFFER_SIZE;
        if (buffer_head == 0) buffer_primed = true;
        xSemaphoreGive(buffer_mutex);
      }

      // ── dual-threshold trigger (unchanged) ─────────────────────────────
      bool over_accel = (mag >= ACCEL_THRESHOLD_G);
      bool over_jerk  = (fabsf(jerk) >= JERK_THRESHOLD_G_PER_SEC);

      if (over_accel && over_jerk && !capture_armed && !capture_ready &&
          buffer_primed && (now - last_crash_detection > CRASH_DEBOUNCE_MS)) {
        capture_armed = true;
        post_trigger_remaining = POST_TRIGGER_SAMPLES;
        last_crash_detection = now;
        trigger_peak_g = mag;
        trigger_jerk_gs = jerk;
        trigger_millis = now;

        Serial.print(F("[CRASH] Trigger at "));
        Serial.print(now);
        Serial.print(F(" ms | "));
        Serial.print(mag, 2);
        Serial.print(F(" g | jerk "));
        Serial.print(jerk, 2);
        Serial.println(F(" g/s — collecting 2.5 s post-roll"));

        // immediate local alert; does not wait for the network
        for (int i = 0; i < 3; i++) {
          digitalWrite(LED_RED, HIGH);
          digitalWrite(LED_YELLOW, HIGH);
          digitalWrite(LED_GREEN, HIGH);
          tone(BUZZER_PIN, 1500, 50);
          delay(60);
          digitalWrite(LED_RED, LOW);
          digitalWrite(LED_YELLOW, LOW);
          digitalWrite(LED_GREEN, LOW);
          delay(60);
        }
      }

      // track the true peak across the whole capture
      if (capture_armed && mag > trigger_peak_g) trigger_peak_g = mag;

      // count down the post-roll → window ends up impact-centred
      if (capture_armed && post_trigger_remaining > 0) {
        post_trigger_remaining--;
        if (post_trigger_remaining == 0) {
          capture_armed = false;
          capture_ready = true;
        }
      }

      prev_accel_mag_g = mag;
      sample_count++;
    }
    last_sensor_read = now;
  }

  // ── ship the centred window ─────────────────────────────────────────────
  if (capture_ready) {
    capture_ready = false;
    processCrash();
  }

  // ── status line ─────────────────────────────────────────────────────────
  if (now - last_print >= PRINT_INTERVAL_MS) {
    if (mpu_ready && buffer_primed) {
      uint16_t curr = (buffer_head + BUFFER_SIZE - 1) % BUFFER_SIZE;
      uint16_t prev = (buffer_head + BUFFER_SIZE - 2) % BUFFER_SIZE;
      float jerk = (imu_buffer[curr].accel_mag - imu_buffer[prev].accel_mag) /
                   (SENSOR_INTERVAL_MS / 1000.0f);
      Serial.print(now);
      Serial.print(F("   | "));
      Serial.print(imu_buffer[curr].accel_mag, 2);
      Serial.print(F("g    | "));
      Serial.print(jerk, 2);
      Serial.print(F("    | "));
      Serial.print(wifi_ready ? F("OK") : F("--"));
      Serial.print(F("   | "));
      Serial.print(mpu_ready ? F("OK") : F("--"));
      Serial.print(F("  | "));
      Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
      Serial.print(F("    | "));
      Serial.println(capture_armed ? F("CAPTURING") : F("RUNNING"));
    }
    last_print = now;
  }
}

// ═══════════════════════════════════════════════════════════════
// CRASH PROCESSING
// ═══════════════════════════════════════════════════════════════

void processCrash() {
  crash_count++;
  Serial.println(F("[EVENT] Window complete (250 pre + 250 post) — uploading"));

  bool ok = postEventWithRetry();

  // LED + buzzer follow the backend's verdict when we have one; if the upload
  // never landed we fall back to the local threshold decision (a crash).
  if (ok && last_result.valid && !last_result.classification_pending) {
    triggerSeverityAlert(last_result.severity_class);
  } else {
    Serial.println(F("[ALERT] No classification — falling back to local decision"));
    triggerSeverityAlert(SEVERITY_SEVERE);
  }

  // SMS FAIL-SAFE (§2): fires on every confirmed local trigger, whether or not
  // the POST succeeded. Deliberately independent of the backend — this is the
  // path that still works when the network is down.
  sendEmergencySMS();

  displayCrashResults();
}

// ═══════════════════════════════════════════════════════════════
// BACKEND UPLOAD  (§6.1, §6.2, §6.3, §6.6)
// ═══════════════════════════════════════════════════════════════

/** Build the §5.1 body straight into a heap String. ~36 kB; never a
 *  StaticJsonDocument on the loop-task stack. */
void buildEventBody(String& body, const String& event_id) {
  body = "";
  body.reserve(40000);

  body += F("{\"device_id\":\"");
  body += DEVICE_ID;
  body += F("\",\"event_id\":\"");
  body += event_id;
  body += F("\"");

  String detected_at = triggerTimestampIso();
  if (detected_at.length()) {
    body += F(",\"detected_at\":\"");
    body += detected_at;
    body += F("\"");
  }

  body += F(",\"trigger\":{\"peak_g\":");
  appendFloat(body, trigger_peak_g, 3);
  body += F(",\"jerk_gs\":");
  appendFloat(body, trigger_jerk_gs, 3);
  body += F("},\"gps\":{");
  if (gpsFixValid()) {
    body += F("\"lat\":");
    appendFloat(body, gps.location.lat(), 6);
    body += F(",\"lon\":");
    appendFloat(body, gps.location.lng(), 6);
    body += F(",\"valid\":true,\"satellites\":");
    body += String(gps.satellites.isValid() ? gps.satellites.value() : 0);
    if (gps.speed.isValid()) {
      body += F(",\"speed_kmh\":");
      appendFloat(body, gps.speed.kmph(), 2);
    }
    if (gps.hdop.isValid()) {
      body += F(",\"hdop\":");
      appendFloat(body, gps.hdop.hdop(), 2);
    }
  } else {
    body += F("\"valid\":false");
  }
  body += F("},\"device\":{\"uptime_s\":");
  body += String(millis() / 1000);
  body += F(",\"free_heap\":");
  body += String(ESP.getFreeHeap());
  body += F(",\"rssi\":");
  body += String(WiFi.RSSI());
  body += F(",\"firmware_version\":\"");
  body += FIRMWARE_VERSION;
  body += F("\"},\"window\":{\"fs_hz\":100,\"units_accel\":\"g\","
            "\"units_gyro\":\"deg_s\"");

  // Snapshot the ring oldest→newest: with the post-roll complete this is
  // exactly 250 pre-trigger + 250 post-trigger samples, impact centred.
  const char* names[6] = {"ax", "ay", "az", "gx", "gy", "gz"};
  xSemaphoreTake(buffer_mutex, portMAX_DELAY);
  uint16_t start = buffer_head;  // next write slot == oldest sample
  for (uint8_t f = 0; f < 6; f++) {
    body += F(",\"");
    body += names[f];
    body += F("\":[");
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
      const IMUSample& s = imu_buffer[(start + i) % BUFFER_SIZE];
      float v;
      switch (f) {
        case 0: v = s.ax_g;   break;
        case 1: v = s.ay_g;   break;
        case 2: v = s.az_g;   break;
        case 3: v = s.gx_dps; break;
        case 4: v = s.gy_dps; break;
        default: v = s.gz_dps; break;
      }
      if (i) body += ',';
      appendFloat(body, v, 4);
    }
    body += ']';
  }
  xSemaphoreGive(buffer_mutex);

  body += F("}}");
}

/** event_id = {DEVICE_ID}-{boot:04d}-{millis:08d} (§6.2). */
String makeEventId() {
  char buf[64];
  snprintf(buf, sizeof(buf), "%s-%04lu-%08lu",
           DEVICE_ID, (unsigned long)boot_count, (unsigned long)trigger_millis);
  return String(buf);
}

bool postEventWithRetry() {
  last_result = {false, 0, "Unknown", 0, 0, trigger_peak_g, false, "none", false};

  if (!wifi_ready || WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[HTTP] ✗ WiFi down — skipping upload (SMS still fires)"));
    return false;
  }

  String event_id = makeEventId();
  String body;
  buildEventBody(body, event_id);

  Serial.print(F("[HTTP] Body "));
  Serial.print(body.length());
  Serial.print(F(" bytes | free heap "));
  Serial.println(ESP.getFreeHeap());

  String url = String(BACKEND_URL) + "/api/v1/events";

  for (uint8_t attempt = 1; attempt <= MAX_POST_ATTEMPTS; attempt++) {
    // Serialise against the heartbeat task; the crash upload always wins.
    xSemaphoreTake(http_mutex, portMAX_DELAY);

    HTTPClient http;
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-API-Key", API_KEY);

    unsigned long t0 = millis();
    int code = http.POST(body);
    unsigned long elapsed = millis() - t0;

    Serial.print(F("[HTTP] Attempt "));
    Serial.print(attempt);
    Serial.print(F(" -> "));
    Serial.print(code);
    Serial.print(F(" in "));
    Serial.print(elapsed);
    Serial.println(F(" ms"));

    if (code == 200 || code == 201) {
      String resp = http.getString();
      http.end();
      xSemaphoreGive(http_mutex);
      parseAck(resp);
      return true;
    }

    // 4xx is a contract violation — retrying will not fix it.
    if (code >= 400 && code < 500) {
      Serial.print(F("[HTTP] ✗ Rejected: "));
      Serial.println(http.getString().substring(0, 160));
      http.end();
      xSemaphoreGive(http_mutex);
      return false;
    }

    http.end();
    xSemaphoreGive(http_mutex);

    if (attempt < MAX_POST_ATTEMPTS) {
      uint16_t wait_ms = RETRY_BACKOFF_MS[attempt - 1];
      Serial.print(F("[HTTP] Retrying in "));
      Serial.print(wait_ms);
      Serial.println(F(" ms (server is idempotent on event_id)"));
      delay(wait_ms);
    }
  }

  Serial.println(F("[HTTP] ✗ Upload failed after all attempts"));
  return false;
}

/** Parse the small FLAT ack. Only literals are retained (§6.6). */
void parseAck(const String& resp) {
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, resp)) {
    Serial.println(F("[HTTP] ✗ Malformed ACK"));
    return;
  }
  last_result.valid = true;
  last_result.classification_pending = doc["classification_pending"] | false;

  if (doc["severity_class"].isNull()) {
    last_result.severity_class = SEVERITY_SEVERE;   // unknown -> treat as worst
    last_result.severity_name  = "Pending";
  } else {
    last_result.severity_class = doc["severity_class"].as<int>();
    last_result.severity_name  = severityLiteral(last_result.severity_class);
  }
  last_result.confidence         = doc["confidence"] | 0.0f;
  last_result.p_crash            = doc["p_crash"] | 0.0f;
  last_result.peak_g             = doc["peak_g"] | trigger_peak_g;
  last_result.accident_confirmed = doc["accident_confirmed"] | false;
  last_result.label_source       = labelSourceLiteral(doc["label_source"] | (const char*)nullptr);
}

// ═══════════════════════════════════════════════════════════════
// HEARTBEAT TASK  (§6.5 — core 0, low priority, never blocks the IMU)
// ═══════════════════════════════════════════════════════════════

void heartbeatTask(void* param) {
  (void)param;
  vTaskDelay(pdMS_TO_TICKS(5000));   // let setup settle

  for (;;) {
    if (wifi_ready && WiFi.status() == WL_CONNECTED) {
      // Never wait on the mutex: if a crash upload holds it, skip this beat.
      // A heartbeat is expendable; a crash record is not.
      if (xSemaphoreTake(http_mutex, 0) == pdTRUE) {
        String body;
        body.reserve(320);
        body += F("{\"device_id\":\"");
        body += DEVICE_ID;
        body += F("\",\"gps\":{");
        if (gpsFixValid()) {
          body += F("\"lat\":");
          appendFloat(body, gps.location.lat(), 6);
          body += F(",\"lon\":");
          appendFloat(body, gps.location.lng(), 6);
          body += F(",\"valid\":true,\"satellites\":");
          body += String(gps.satellites.isValid() ? gps.satellites.value() : 0);
        } else {
          body += F("\"valid\":false");
        }
        body += F("},\"device\":{\"uptime_s\":");
        body += String(millis() / 1000);
        body += F(",\"free_heap\":");
        body += String(ESP.getFreeHeap());
        body += F(",\"rssi\":");
        body += String(WiFi.RSSI());
        body += F(",\"firmware_version\":\"");
        body += FIRMWARE_VERSION;
        body += F("\"}}");

        HTTPClient http;
        http.setTimeout(5000);
        http.begin(String(BACKEND_URL) + "/api/v1/heartbeat");
        http.addHeader("Content-Type", "application/json");
        http.addHeader("X-API-Key", API_KEY);
        int code = http.POST(body);
        http.end();
        xSemaphoreGive(http_mutex);

        if (code != 200) {
          Serial.print(F("[HB] ✗ "));
          Serial.println(code);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
  }
}

// ═══════════════════════════════════════════════════════════════
// SEVERITY ALERTS  (unchanged behaviour)
// ═══════════════════════════════════════════════════════════════

void triggerSeverityAlert(int severity_class) {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, LOW);
  noTone(BUZZER_PIN);

  switch (severity_class) {
    case SEVERITY_NORMAL:
      Serial.println(F("🟢 SEVERITY: NORMAL - No alert needed"));
      digitalWrite(LED_GREEN, HIGH);
      tone(BUZZER_PIN, 1000, 100);
      delay(150);
      noTone(BUZZER_PIN);
      break;

    case SEVERITY_MODERATE:
      Serial.println(F("🟡 SEVERITY: MODERATE - Warning alert"));
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_YELLOW, HIGH);
        tone(BUZZER_PIN, 1500, 150);
        delay(300);
        digitalWrite(LED_YELLOW, LOW);
        noTone(BUZZER_PIN);
        delay(200);
      }
      digitalWrite(LED_YELLOW, HIGH);
      break;

    case SEVERITY_SEVERE:
      Serial.println(F("🔴 SEVERITY: SEVERE - EMERGENCY ALERT!!!"));
      for (int i = 0; i < 5; i++) {
        digitalWrite(LED_RED, HIGH);
        tone(BUZZER_PIN, 2000, 100);
        delay(150);
        digitalWrite(LED_RED, LOW);
        noTone(BUZZER_PIN);
        delay(150);
      }
      digitalWrite(LED_RED, HIGH);
      break;
  }
}

// ═══════════════════════════════════════════════════════════════
// GSM / SMS FAIL-SAFE  (reconstructed — verify UART pins first)
// ═══════════════════════════════════════════════════════════════

#if ENABLE_GSM
bool gsmSendAT(const char* cmd, const char* expect, uint16_t timeout_ms) {
  Serial1.println(cmd);
  unsigned long deadline = millis() + timeout_ms;
  String resp;
  while (millis() < deadline) {
    while (Serial1.available()) resp += (char)Serial1.read();
    if (resp.indexOf(expect) >= 0) return true;
    delay(10);
  }
  return false;
}

bool gsmInit() {
  if (!gsmSendAT("AT", "OK", 2000)) return false;
  gsmSendAT("ATE0", "OK", 1000);
  return gsmSendAT("AT+CMGF=1", "OK", 2000);   // text mode
}
#endif

void sendEmergencySMS() {
#if ENABLE_GSM
  if (!gsm_ready) {
    Serial.println(F("[SMS] ✗ GSM not ready"));
    return;
  }

  // Message keeps the existing field format: severity, peak, confidence,
  // coordinates, a Maps link, satellite count and uptime.
  String msg;
  msg.reserve(320);
  bool severe = (last_result.valid && last_result.severity_class == SEVERITY_SEVERE) ||
                (!last_result.valid);
  msg += severe ? F("SEVERE ACCIDENT") : F("ACCIDENT DETECTED");
  msg += F(" peak ");
  appendFloat(msg, last_result.valid ? last_result.peak_g : trigger_peak_g, 1);
  msg += F(" g");
  if (last_result.valid && last_result.confidence > 0) {
    msg += F(" conf ");
    msg += String((int)(last_result.confidence * 100));
    msg += F("%");
  }
  if (gpsFixValid()) {
    msg += F(" | ");
    appendFloat(msg, gps.location.lat(), 5);
    msg += F(",");
    appendFloat(msg, gps.location.lng(), 5);
    msg += F(" | https://maps.google.com/?q=");
    appendFloat(msg, gps.location.lat(), 5);
    msg += F(",");
    appendFloat(msg, gps.location.lng(), 5);
    msg += F(" | sats ");
    msg += String(gps.satellites.isValid() ? gps.satellites.value() : 0);
  } else {
    msg += F(" | NO GPS FIX");
  }
  msg += F(" | up ");
  msg += String(millis() / 1000);
  msg += F("s");

  for (uint8_t i = 0; i < SMS_RECIPIENT_COUNT; i++) {
    String cmd = String("AT+CMGS=\"") + SMS_RECIPIENTS[i] + "\"";
    if (!gsmSendAT(cmd.c_str(), ">", 5000)) {
      Serial.println(F("[SMS] ✗ No prompt"));
      continue;
    }
    Serial1.print(msg);
    Serial1.write(26);   // Ctrl-Z
    if (gsmSendAT("", "+CMGS", 15000)) {
      Serial.print(F("[SMS] ✓ Sent to "));
      Serial.println(SMS_RECIPIENTS[i]);
    } else {
      Serial.println(F("[SMS] ✗ Send failed"));
    }
  }
#else
  Serial.println(F("[SMS] Skipped — ENABLE_GSM is 0"));
#endif
}

// ═══════════════════════════════════════════════════════════════
// SERIAL OUTPUT
// ═══════════════════════════════════════════════════════════════

void displayCrashResults() {
  Serial.println();
  Serial.println(F("╔════════════════════════════════════════════════════════╗"));
  Serial.println(F("║              CRASH ANALYSIS COMPLETE                   ║"));
  Serial.println(F("╠════════════════════════════════════════════════════════╣"));
  Serial.print(F("║ Incident #"));         Serial.println(crash_count);
  Serial.print(F("║ Event ID:       "));   Serial.println(makeEventId());
  if (last_result.valid && !last_result.classification_pending) {
    Serial.print(F("║ Severity:       "));  Serial.println(last_result.severity_name);
    Serial.print(F("║ Confidence:     "));  Serial.print(last_result.confidence * 100, 1);
    Serial.println(F("%"));
    Serial.print(F("║ P(crash):       "));  Serial.println(last_result.p_crash, 3);
    Serial.print(F("║ Label source:   "));  Serial.println(last_result.label_source);
    Serial.print(F("║ Peak (server):  "));  Serial.print(last_result.peak_g, 2);
    Serial.println(F(" g"));
  } else if (last_result.classification_pending) {
    Serial.println(F("║ Severity:       PENDING (ML API was unreachable)"));
  } else {
    Serial.println(F("║ Severity:       UNKNOWN (upload failed)"));
  }
  Serial.print(F("║ Peak (local):   "));   Serial.print(trigger_peak_g, 2);
  Serial.println(F(" g"));
  Serial.println(F("╠════════════════════════════════════════════════════════╣"));
  Serial.println(F("║ GPS LOCATION                                           ║"));
  if (gpsFixValid()) {
    Serial.print(F("║ Latitude:   "));  Serial.println(gps.location.lat(), 6);
    Serial.print(F("║ Longitude:  "));  Serial.println(gps.location.lng(), 6);
    Serial.print(F("║ Satellites: "));  Serial.println(gps.satellites.value());
  } else {
    Serial.println(F("║ No valid fix"));
  }
  Serial.println(F("╚════════════════════════════════════════════════════════╝"));
  Serial.println();
}

void printBanner() {
  Serial.println();
  Serial.println(F("╔════════════════════════════════════════════════════════╗"));
  Serial.println(F("║   SENTINEL — ESP32 CRASH DETECTION NODE  v2.0.0        ║"));
  Serial.println(F("╠════════════════════════════════════════════════════════╣"));
  Serial.println(F("║  MPU6050 @ 100 Hz, ±16 g   (GPIO 21/22)                ║"));
  Serial.println(F("║  LEDs 33/25/26 · Buzzer 32                             ║"));
  Serial.println(F("║  GPS NEO-6M  Serial2 (16/17)                           ║"));
  Serial.println(F("║  GSM SIM800L Serial1  — see ENABLE_GSM                 ║"));
  Serial.println(F("║                                                        ║"));
  Serial.println(F("║  Uploads to the Sentinel backend, not the ML API.      ║"));
  Serial.println(F("║  Window: 2.5 s pre + 2.5 s post, impact centred.       ║"));
  Serial.println(F("║  SMS fail-safe fires independently of the network.     ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════╝"));
  Serial.println();
}
