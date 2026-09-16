// /*
//  * ═══════════════════════════════════════════════════════════════
//  * ESP32 CRASH DETECTION SYSTEM  —  MPU6050 + WiFi/ML + GPS + GSM
//  * RGB LED REVISION — DIRECT WIRING ON FREE PINS
//  * ═══════════════════════════════════════════════════════════════
//  *
//  * WHAT CHANGED IN THIS VERSION:
//  *   1. The three discrete LEDs (red / yellow / green) are replaced
//  *      by ONE common-cathode RGB LED.
//  *   2. The RGB LED is wired DIRECTLY to the ESP32 on three free
//  *      pins — GPIO 4, 27 and 23. It does NOT reuse the old LED
//  *      branches on the perfboard.
//  *   3. The old LED pins (25, 26, 33) are now completely unused.
//  *      No pinMode() is called on them, so they sit as high-impedance
//  *      inputs and the old LEDs stay dark. They remain soldered to
//  *      the perfboard; nothing needs to be removed.
//  *   4. [CRITICAL BUGFIX, now moot] LED_RED was previously defined as
//  *      GPIO 34. GPIO 34 is INPUT-ONLY on the ESP32 and cannot drive
//  *      an output, so the red LED never lit. Red now lives on GPIO 4.
//  *   5. There is no separate "yellow" channel. Amber is produced by
//  *      lighting RED and GREEN together.
//  *   6. Blue is now available and is used as a WiFi boot indicator
//  *      and as part of the white crash-trigger flash.
//  *   7. All LED writes go through rgbWrite() / the named colour
//  *      helpers. Common-anode LEDs are supported by one flag.
//  *
//  * NOTHING ELSE WAS ALTERED. Crash detection, buffering, the ML API
//  * call, local fallback classification, GPS parsing, GSM and SMS
//  * logic are the same behaviour as the previous version.
//  *
//  * ─── PIN ALLOCATION MAP (verify before soldering) ──────────────
//  * OCCUPIED:
//  *    1,  3   USB serial (Serial, 115200)
//  *   16, 17   GPS NEO-6M      (Serial2)
//  *   18, 19   SIM800L GSM     (Serial1)
//  *   21, 22   MPU6050 I2C     (SDA / SCL)
//  *       32   Buzzer
//  *   25, 26, 33  old LED branches - still soldered, now UNUSED
//  *
//  * NEVER USE:
//  *    6 - 11  SPI flash (using these bricks the board)
//  *   34, 35, 36, 39  input-only, no output driver
//  *    0, 2, 12, 15   strapping pins, interfere with boot/flash
//  *
//  * FREE:
//  *    4, 13, 23, 27     <- RGB uses 4, 27, 23. GPIO 13 is the spare.
//  *
//  * VERIFY FIRST: with the board powered off, put a multimeter in
//  * continuity mode and probe GPIO 4, 27 and 23 against the ground
//  * rail and any nearby component leg. All three must read OPEN. If
//  * one beeps, something is already wired to it — move that channel
//  * to GPIO 13 and change the matching #define below.
//  * ───────────────────────────────────────────────────────────────
//  *
//  * ─── RGB LED WIRING (common cathode, direct to ESP32) ─────────
//  * A standard 5mm common-cathode RGB LED has four legs, in order:
//  *     Red  |  Common (longest leg)  |  Green  |  Blue
//  *
//  *   RGB Red    --[ 330R ]-->  ESP32 GPIO 4
//  *   RGB Green  --[ 100R ]-->  ESP32 GPIO 27
//  *   RGB Blue   --[ 100R ]-->  ESP32 GPIO 23
//  *   RGB Common ------------>  ESP32 GND  (any GND pin)
//  *
//  * Each colour leg gets its OWN resistor. A single resistor on the
//  * common leg makes brightness depend on how many channels are lit,
//  * which ruins the amber mix.
//  *
//  * The resistor values are intentionally unequal. Red has a forward
//  * voltage near 2.0 V while green and blue sit near 3.0-3.2 V, so
//  * from a 3.3 V GPIO the red channel draws far more current at the
//  * same resistance. If amber still reads as orange-red on the bench,
//  * increase the red resistor to 470R or 680R.
//  *
//  * NEVER connect the LED without resistors. Red in particular would
//  * pull 30-40 mA, at or beyond the ESP32 pin's 40 mA absolute max.
//  *
//  * COMMON ANODE LED: set RGB_COMMON_ANODE to true below and wire the
//  * long leg to 3.3 V instead of GND. Everything else is identical.
//  * ───────────────────────────────────────────────────────────────
//  *
//  * ─── POWER WARNING (UNCHANGED, STILL APPLIES) ─────────────────
//  * The SIM800L draws up to 2 A in bursts when it transmits, at
//  * 3.7-4.2 V. It CANNOT be powered from the ESP32 3.3 V pin.
//  *   • Power SIM800L from a separate 4 V source rated >= 2 A
//  *   • Add a 1000 uF capacitor across SIM800L VCC/GND
//  *   • Tie ALL grounds together (ESP32 GND <-> SIM800L GND <-> supply GND)
//  * ───────────────────────────────────────────────────────────────
//  *
//  * GSM WIRING (UART lines cross over — TX always meets RX):
//  *   SIM800L VCC  ->  4.0 V external supply (NOT the ESP32!)
//  *   SIM800L GND  ->  common ground (shared with ESP32 GND)
//  *   ESP32 GPIO 19 (TXD)  ------>  SIM800L RXD
//  *   ESP32 GPIO 18 (RXD)  <------  SIM800L TXD
//  *   Antenna attached before powering on.
//  */

// #include <Wire.h>
// #include <MPU6050.h>
// #include <WiFi.h>
// #include <HTTPClient.h>
// #include <ArduinoJson.h>
// #include <Preferences.h>     // NVS-backed boot counter for unique event ids
// #include <time.h>            // NTP clock, so detected_at is a real timestamp
// #include <math.h>

// // ═══════════════════════════════════════════════════════════════
// // GPIO CONFIGURATION
// // ═══════════════════════════════════════════════════════════════

// // ─── RGB LED — direct to ESP32 on free pins ───────────────────
// // These three pins are NOT used by anything else in this sketch.
// // The old LED pins (25, 26, 33) are deliberately left out entirely.
// #define RGB_R 4              // GPIO 4:  RGB red   channel (via 330R)
// #define RGB_G 27             // GPIO 27: RGB green channel (via 100R)
// #define RGB_B 23             // GPIO 23: RGB blue  channel (via 100R)

// // Spare free pin if one of the above turns out to be occupied: GPIO 13

// // false = common cathode (long leg to GND)   <-- default
// // true  = common anode   (long leg to 3.3V)
// #define RGB_COMMON_ANODE false

// #define BUZZER_PIN 32        // GPIO 32: Audio alert buzzer

// // I2C for MPU6050
// #define I2C_SDA 21           // GPIO 21: MPU6050 SDA
// #define I2C_SCL 22           // GPIO 22: MPU6050 SCL

// // GPS (NEO-6M) on Hardware UART2
// #define GPS_RX 16            // GPIO 16: GPS TX (data FROM GPS)
// #define GPS_TX 17            // GPIO 17: GPS RX (data TO GPS)

// // GSM (SIM800L) on Hardware UART1
// // Names are from the ESP32's point of view (UART pins are crossed):
// //   GSM_RX = the ESP32's RX pin  <- wire to the modem's TXD
// //   GSM_TX = the ESP32's TX pin  -> wire to the modem's RXD
// #define GSM_RX 18            // GPIO 18: ESP32 RXD  <- SIM800L TXD
// #define GSM_TX 19            // GPIO 19: ESP32 TXD  -> SIM800L RXD
// #define GSM_BAUD 9600

// // ═══════════════════════════════════════════════════════════════
// // WIFI CONFIGURATION
// // ═══════════════════════════════════════════════════════════════

// const char* WIFI_SSID = "BERNARDINE 7683";
// const char* WIFI_PASSWORD = "12345678";

// // ═══════════════════════════════════════════════════════════════
// // BACKEND CONFIGURATION
// // ═══════════════════════════════════════════════════════════════
// //
// // The device no longer calls the ML API directly. It reports to the Sentinel
// // backend, which classifies, stores the raw window, and pushes the incident to
// // the dispatch console. The backend is the only component that talks to the
// // model, and it is the system of record — a classification that lived only in
// // this serial log used to vanish when the buffer scrolled.
// //
// // Swap BACKEND_URL for http://<laptop-LAN-IP>:8080 to run against a local
// // backend; the scheme decides whether TLS is used.

// const char* BACKEND_URL = "https://sentinel-backend-production-c650.up.railway.app";

// // A DEVICE-scoped key, deliberately NOT the responder key used by the
// // dashboard. Firmware flash can be read back over USB, so what is stored here
// // must only be able to report incidents — never dispatch or resolve them.
// // Register it on the backend as:  ROLE_KEYS={"<this value>":"device"}
// const char* API_KEY   = "dev_mgxPoy9qrM4nZOVlvOy7aoVicMpvbGmq";

// const char* DEVICE_ID = "ESP32_ACC_001";
// const char* FIRMWARE_VERSION = "3.0.0";

// const uint16_t HTTP_TIMEOUT_MS   = 15000;
// const uint8_t  MAX_POST_ATTEMPTS = 3;
// const uint16_t RETRY_BACKOFF_MS[] = {1000, 3000, 6000};
// #define HEARTBEAT_INTERVAL_MS 30000

// // ═══════════════════════════════════════════════════════════════
// // GSM / EMERGENCY CONTACT CONFIGURATION
// // ═══════════════════════════════════════════════════════════════

// /*
//  * GSM_OWN_NUMBER is the SIM sitting INSIDE the module (the sender).
//  * It is not messaged — it is recorded here for reference and can be
//  * quoted in the SMS body so recipients know which unit is reporting.
//  *
//  * EMERGENCY_CONTACTS are the recipients alerted on a crash. Every
//  * number must be in full international format (+233...), because the
//  * modem is given no default country code.
//  */
// const char* GSM_OWN_NUMBER = "+233537562836";   // SIM in the module

// const char* EMERGENCY_CONTACTS[] = {
//   "+233594282165",     // [1] primary emergency contact
//   "+233534898588",     // [2] secondary emergency contact
// };
// const uint8_t NUM_EMERGENCY_CONTACTS =
//     sizeof(EMERGENCY_CONTACTS) / sizeof(EMERGENCY_CONTACTS[0]);

// // Send SMS for these severities (NORMAL never triggers an SMS)
// #define SMS_ON_MODERATE true
// #define SMS_ON_SEVERE   true

// // Place an emergency voice call on SEVERE (in addition to SMS)
// #define CALL_ON_SEVERE  true
// #define CALL_DURATION_MS 25000    // ring for 25 s then hang up

// #define GSM_AT_TIMEOUT_MS   5000
// #define GSM_SMS_TIMEOUT_MS  20000
// #define GSM_BOOT_WAIT_MS    3000

// // ═══════════════════════════════════════════════════════════════
// // IMU HARDWARE
// // ═══════════════════════════════════════════════════════════════

// #define MPU6050_ADDR 0x68
// #define SENSOR_INTERVAL_MS 10
// #define PRINT_INTERVAL_MS 100

// // ═══════════════════════════════════════════════════════════════
// // CRASH DETECTION PARAMETERS
// // ═══════════════════════════════════════════════════════════════

// #define BUFFER_SIZE 500           // 500 samples = 5 seconds @ 100Hz
// #define ACCEL_THRESHOLD_G 2.0f    // 2.0g acceleration magnitude
// #define JERK_THRESHOLD_G_PER_SEC 5.0f  // 5.0g/s jerk rate
// // Keep sampling this many samples AFTER the trigger before sending, so the
// // 500-sample window is CENTRED on the impact (250 before + 250 after).
// // Sending at the trigger instant leaves only ~10 ms of the crash pulse at the
// // window edge; the API needs the whole 40-250 ms pulse and returns Normal.
// #define POST_TRIGGER_SAMPLES 250

// #define RAW_ACCEL_TO_G (1.0f / 2048.0f)   // MPU6050 ±16g range
// // ±500 dps: 2.3% of verified crashes exceed 250 dps (0.6% exceed 500).
// #define RAW_GYRO_TO_DPS (1.0f / 65.5f)     // MPU6050 ±500 dps range

// // Offline fallback classification (used only when the ML API is
// // unreachable). Mirrors the deployed signature logic: a crash is a
// // 2–7 g event of bounded duration; sustained high-g is a manoeuvre.
// #define SIG_PEAK_MIN_G      2.0f
// #define SIG_PEAK_MAX_G      7.0f
// #define SIG_DUR_MIN_MS        40
// #define SIG_DUR_MAX_MS       250
// #define SIG_SEVERE_IMPULSE  0.959f   // g·s, ≈ Δv 34 km/h

// // ═══════════════════════════════════════════════════════════════
// // SEVERITY LEVELS
// // ═══════════════════════════════════════════════════════════════

// enum SeverityLevel {
//   SEVERITY_NORMAL = 0,      // Safe
//   SEVERITY_MODERATE = 1,    // Warning
//   SEVERITY_SEVERE = 2       // Emergency
// };

// // ═══════════════════════════════════════════════════════════════
// // DATA STRUCTURES
// // ═══════════════════════════════════════════════════════════════

// struct IMUSample {
//   float ax_g;      // Accelerometer X in g-units
//   float ay_g;      // Accelerometer Y in g-units
//   float az_g;      // Accelerometer Z in g-units
//   float gx_dps;    // Gyroscope X in degrees/sec
//   float gy_dps;    // Gyroscope Y in degrees/sec
//   float gz_dps;    // Gyroscope Z in degrees/sec
//   float accel_mag; // Magnitude for crash detection
//   uint32_t timestamp_ms;
// };

// struct GPSData {
//   bool valid;
//   bool has_lock;
//   float latitude;      // DECIMAL DEGREES (signed, S/W negative)
//   float longitude;     // DECIMAL DEGREES (signed, S/W negative)
//   float altitude;
//   int satellites;
//   unsigned long last_fix_time;
// };

// struct CrashResult {
//   bool detected;
//   int severity_class;
//   const char* severity_name;
//   float confidence;
//   float peak_magnitude_g;
// };

// // GSM module state
// struct GSMData {
//   bool module_found;      // responded to AT
//   bool sim_ready;         // AT+CPIN? == READY
//   bool network_reg;       // AT+CREG? registered
//   int  signal_quality;    // AT+CSQ (0-31, >=10 usable, 99 = unknown)
//   uint32_t sms_sent;
//   uint32_t sms_failed;
//   unsigned long last_check;
// };

// IMUSample imu_buffer[BUFFER_SIZE];
// uint16_t buffer_index = 0;
// bool buffer_full = false;

// GPSData gps_data = {false, false, 0.0, 0.0, 0.0, 0, 0};
// GSMData gsm_data = {false, false, false, 0, 0, 0, 0};

// // ═══════════════════════════════════════════════════════════════
// // GLOBAL VARIABLES
// // ═══════════════════════════════════════════════════════════════

// MPU6050 mpu(MPU6050_ADDR);
// bool mpu_ready = false;
// bool wifi_ready = false;
// bool gps_ready = false;
// bool gsm_ready = false;

// int16_t accelX, accelY, accelZ;
// int16_t gyroX, gyroY, gyroZ;
// float accel_x_g = 0, accel_y_g = 0, accel_z_g = 0;
// float prev_accel_mag_g = 0;

// unsigned long last_sensor_read = 0;
// unsigned long last_print = 0;
// unsigned long last_crash_detection = 0;
// unsigned long last_gsm_health = 0;
// uint32_t sample_count = 0;
// uint32_t crash_count = 0;

// bool crash_detected_flag = false;
// uint16_t post_trigger_remaining = 0;
// bool api_request_in_progress = false;

// // What the device itself measured at the moment of the trigger. Sent alongside
// // the window so the dashboard can show the device's own reading next to the
// // server's filtered figure.
// float    trigger_peak_g  = 0.0f;
// float    trigger_jerk_gs = 0.0f;
// uint32_t trigger_millis  = 0;

// Preferences prefs;
// uint32_t boot_count = 0;

// // One HTTP request at a time. The heartbeat task takes this with zero timeout
// // and skips its turn rather than delaying a crash upload.
// SemaphoreHandle_t http_mutex = nullptr;
// CrashResult last_crash_result = {false, 0, "None", 0.0, 0.0};

// // Forward declarations
// bool  sendEventToBackend();
// void  heartbeatTask(void* param);
// bool  gsmInit();
// String gsmSendAT(const char* cmd, uint16_t timeout_ms = GSM_AT_TIMEOUT_MS);
// bool  gsmExpect(const char* cmd, const char* expect, uint16_t timeout_ms = GSM_AT_TIMEOUT_MS);
// bool  gsmCheckHealth(bool verbose = false);
// bool  sendEmergencySMS(int severity_class, float confidence, float peak_g);
// bool  gsmSendSMSTo(const char* number, const String& message);
// void  gsmPlaceEmergencyCall(const char* number);
// String buildAlertMessage(int severity_class, float confidence, float peak_g);
// void  classifyLocally(CrashResult& result);

// // ═══════════════════════════════════════════════════════════════
// // RGB LED CONTROL LAYER
// // ═══════════════════════════════════════════════════════════════

// /*
//  * Single point of control for the RGB LED. Every LED write in the
//  * whole sketch goes through here, so a common-anode part only needs
//  * the RGB_COMMON_ANODE flag flipped — nothing else changes.
//  *
//  * Plain digitalWrite is used rather than PWM on purpose. On the ESP32
//  * the Arduino tone() function is built on the LEDC peripheral, the
//  * same peripheral ledcWrite() uses. Driving the LED with PWM risks a
//  * channel collision with the buzzer, and a buzzer that stops working
//  * mid-demo is a worse outcome than a slightly imperfect amber. Colour
//  * balance is handled with resistor values instead (see header).
//  */
// inline void rgbWrite(bool r, bool g, bool b) {
//   if (RGB_COMMON_ANODE) {
//     digitalWrite(RGB_R, r ? LOW : HIGH);
//     digitalWrite(RGB_G, g ? LOW : HIGH);
//     digitalWrite(RGB_B, b ? LOW : HIGH);
//   } else {
//     digitalWrite(RGB_R, r ? HIGH : LOW);
//     digitalWrite(RGB_G, g ? HIGH : LOW);
//     digitalWrite(RGB_B, b ? HIGH : LOW);
//   }
// }

// // Named colours — these replace the old per-LED digitalWrite calls
// inline void rgbOff()    { rgbWrite(false, false, false); }
// inline void rgbRed()    { rgbWrite(true,  false, false); }  // SEVERE
// inline void rgbGreen()  { rgbWrite(false, true,  false); }  // NORMAL
// inline void rgbBlue()   { rgbWrite(false, false, true ); }  // WiFi / info
// inline void rgbAmber()  { rgbWrite(true,  true,  false); }  // MODERATE (was yellow)
// inline void rgbWhite()  { rgbWrite(true,  true,  true ); }  // crash trigger flash

// // ═══════════════════════════════════════════════════════════════
// // MOUNT ORIENTATION CHECK
// // ═══════════════════════════════════════════════════════════════
// // The model uses per-axis features (ax/ay/az) learned from data with gravity
// // on +Z (training median az = 1.0 g). A board mounted on its side puts gravity
// // on X or Y and silently skews every prediction. Checked at boot, vehicle still.

// void checkMountOrientation() {
//   const int N = 100;
//   float sx = 0, sy = 0, sz = 0;
//   for (int i = 0; i < N; i++) {
//     mpu.getAcceleration(&accelX, &accelY, &accelZ);
//     sx += accelX * RAW_ACCEL_TO_G;
//     sy += accelY * RAW_ACCEL_TO_G;
//     sz += accelZ * RAW_ACCEL_TO_G;
//     delay(SENSOR_INTERVAL_MS);
//   }
//   sx /= N; sy /= N; sz /= N;
//   Serial.printf("[MOUNT] At rest: ax=%.2fg ay=%.2fg az=%.2fg\n", sx, sy, sz);
//   if (sz < 0.8f || fabs(sx) > 0.35f || fabs(sy) > 0.35f) {
//     Serial.println(F("[MOUNT] WARNING: gravity is not on +Z. Mount the MPU6050 flat, Z axis up,"));
//     Serial.println(F("[MOUNT]          or severity predictions will be unreliable."));
//     for (int i = 0; i < 5; i++) {
//       rgbAmber();
//       delay(150);
//       rgbOff();
//       delay(150);
//     }
//   } else {
//     Serial.println(F("[MOUNT] OK - orientation correct (Z axis up)"));
//   }
// }

// // ═══════════════════════════════════════════════════════════════
// // SETUP
// // ═══════════════════════════════════════════════════════════════

// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   // Initialize RGB LED channels + buzzer.
//   // GPIO 25, 26 and 33 are intentionally NOT configured here — the
//   // old LEDs stay soldered to the perfboard but are never driven.
//   pinMode(RGB_R, OUTPUT);
//   pinMode(RGB_G, OUTPUT);
//   pinMode(RGB_B, OUTPUT);
//   pinMode(BUZZER_PIN, OUTPUT);

//   // Start with the LED off
//   rgbOff();
//   digitalWrite(BUZZER_PIN, LOW);

//   printBanner();

//   // ─────────────────────────────────────────────────────────────
//   // RGB self-test: confirms all three channels and the common leg
//   // are wired correctly before anything else runs. If a colour is
//   // missing here, it is a wiring fault, not a logic fault.
//   // ─────────────────────────────────────────────────────────────
//   Serial.println(F("[RGB] Self-test: RED -> GREEN -> BLUE -> AMBER -> WHITE"));
//   rgbRed();   delay(400);
//   rgbGreen(); delay(400);
//   rgbBlue();  delay(400);
//   rgbAmber(); delay(400);
//   rgbWhite(); delay(400);
//   rgbOff();
//   Serial.println(F("[RGB] Self-test complete"));

//   // ─────────────────────────────────────────────────────────────
//   // Initialize I2C & MPU6050
//   // ─────────────────────────────────────────────────────────────
//   Serial.println(F("[I2C] Initializing..."));
//   Wire.begin(I2C_SDA, I2C_SCL, 400000);
//   delay(100);

//   Serial.println(F("[MPU6050] Initializing sensor..."));
//   mpu.initialize();
//   mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);  // measure up to ±16g
//   mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);   // must match RAW_GYRO_TO_DPS

//   if (!mpu.testConnection()) {
//     Serial.println(F("[ERROR] MPU6050 not found!"));
//     mpu_ready = false;
//     for (int i = 0; i < 3; i++) {
//       rgbRed();
//       delay(200);
//       rgbOff();
//       delay(200);
//     }
//   } else {
//     Serial.println(F("[MPU6050] OK - Connected! (Accel + Gyro)"));
//     mpu_ready = true;
//     checkMountOrientation();
//     rgbGreen();
//     delay(500);
//     rgbOff();
//   }

//   // ─────────────────────────────────────────────────────────────
//   // Initialize GPS (Hardware UART2)
//   // ─────────────────────────────────────────────────────────────
//   Serial.println(F("[GPS] Initializing NEO-6M on Serial2..."));
//   Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
//   delay(500);
//   Serial.println(F("[GPS] OK - Initialized on GPIO 16/17 (9600 baud)"));
//   Serial.println(F("[GPS] Waiting for satellite lock (go outdoors)..."));
//   gps_ready = true;

//   // ─────────────────────────────────────────────────────────────
//   // Initialize GSM (Hardware UART1)
//   // ─────────────────────────────────────────────────────────────
//   Serial.println();
//   Serial.println(F("[GSM] Initializing SIM800L on Serial1..."));
//   Serial1.begin(GSM_BAUD, SERIAL_8N1, GSM_RX, GSM_TX);
//   delay(GSM_BOOT_WAIT_MS);          // modem needs time after power-up

//   gsm_ready = gsmInit();

//   if (gsm_ready) {
//     Serial.println(F("[GSM] OK - Ready to send emergency SMS"));
//     Serial.print(F("[GSM] Signal quality (CSQ): "));
//     Serial.print(gsm_data.signal_quality);
//     Serial.println(gsm_data.signal_quality >= 10 ? F(" (good)") : F(" (WEAK - check antenna)"));
//     Serial.print(F("[GSM] Emergency contacts configured: "));
//     Serial.println(NUM_EMERGENCY_CONTACTS);
//     // Amber double-blink = GSM up
//     for (int i = 0; i < 2; i++) {
//       rgbAmber(); delay(120);
//       rgbOff();   delay(120);
//     }
//   } else {
//     Serial.println(F("[GSM] FAILED - SMS alerts will be unavailable"));
//     Serial.println(F("[GSM] Check: separate 4V supply, common ground, antenna, SIM inserted"));
//   }

//   // ─────────────────────────────────────────────────────────────
//   // Initialize WiFi
//   // ─────────────────────────────────────────────────────────────
//   Serial.println();
//   Serial.print(F("[WiFi] Connecting to: "));
//   Serial.println(WIFI_SSID);

//   WiFi.mode(WIFI_STA);
//   // Boot counter makes event ids unique across resets, so a device that
//   // reboots mid-shift cannot reuse an id the server has already stored.
//   prefs.begin("sentinel", false);
//   boot_count = prefs.getUInt("boot", 0) + 1;
//   prefs.putUInt("boot", boot_count);
//   prefs.end();
//   Serial.print(F("[NVS] Boot count: "));
//   Serial.println(boot_count);

//   http_mutex = xSemaphoreCreateMutex();

//   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

//   int wifi_attempts = 0;
//   while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
//     delay(500);
//     Serial.print(F("."));
//     wifi_attempts++;
//   }

//   Serial.println();

//   if (WiFi.status() == WL_CONNECTED) {
//     Serial.println(F("[WiFi] OK - Connected!"));
//     Serial.print(F("[WiFi] IP: "));
//     Serial.println(WiFi.localIP());
//     wifi_ready = true;

//     // The ESP32 has no RTC, so detected_at comes from NTP. Without it the
//     // backend can only use its own receive time and the dashboard's latency
//     // breakdown loses its first leg.
//     configTime(0, 0, "pool.ntp.org", "time.nist.gov");
//     Serial.print(F("[NTP] Syncing"));
//     for (int i = 0; i < 20 && time(nullptr) < 1600000000; i++) {
//       delay(250);
//       Serial.print(F("."));
//     }
//     Serial.println(time(nullptr) >= 1600000000 ? F(" OK - UTC set")
//                                                : F(" FAILED - detected_at omitted"));

//     // Heartbeat runs on core 0 at low priority so it can never preempt the
//     // 100 Hz sampling loop, which owns core 1.
//     xTaskCreatePinnedToCore(heartbeatTask, "heartbeat", 8192, nullptr, 1, nullptr, 0);
//     Serial.println(F("[TASK] Heartbeat started on core 0"));

//     // Blue flash = WiFi up. Blue is free now that yellow is gone,
//     // so WiFi and GSM are distinguishable at boot.
//     rgbBlue();
//     delay(300);
//     rgbOff();
//   } else {
//     Serial.println(F("[WiFi] FAILED - will rely on GSM fallback"));
//     wifi_ready = false;
//   }

//   Serial.println();
//   Serial.println(F("══════════════════════════════════════════════════════════════════════════════════════════"));
//   Serial.println(F("Time(ms)  | Accel(g) | Jerk(g/s) | WiFi | MPU | GPS Sats | GSM | Status"));
//   Serial.println(F("══════════════════════════════════════════════════════════════════════════════════════════"));

//   last_sensor_read = millis();
//   last_print = millis();
//   last_gsm_health = millis();

//   // Normal-driving indication: green stays on once initialization is complete.
//   rgbGreen();
// }

// // ═══════════════════════════════════════════════════════════════
// // GPS PARSING (NON-BLOCKING)
// // ═══════════════════════════════════════════════════════════════

// void updateGPS() {
//   static String sentence = "";

//   while (Serial2.available()) {
//     char c = Serial2.read();

//     if (c == '\n') {
//       if (sentence.length() > 0 && sentence.startsWith("$GPGGA")) {
//         parseGPGGA(sentence);
//       }
//       sentence = "";
//     } else if (c != '\r') {
//       sentence += c;
//     }
//   }
// }

// /*
//  * Convert NMEA ddmm.mmmm / dddmm.mmmm to decimal degrees.
//  *
//  * NMEA reports latitude as 4807.038 meaning 48 deg 07.038 min, NOT
//  * 4807.038 degrees. Feeding the raw value into a maps link puts the
//  * pin in the wrong hemisphere/continent. This matters because the
//  * coordinates are transmitted by SMS.
//  */
// float nmeaToDecimalDegrees(String field, String hemisphere) {
//   if (field.length() < 3) return 0.0;

//   float raw = field.toFloat();               // e.g. 4807.038
//   int degrees = (int)(raw / 100.0);          // 48
//   float minutes = raw - (degrees * 100.0);   // 7.038
//   float decimal = degrees + (minutes / 60.0);

//   hemisphere.trim();
//   if (hemisphere == "S" || hemisphere == "W") decimal = -decimal;

//   return decimal;
// }

// void parseGPGGA(String gga_sentence) {
//   // $GPGGA,time,lat,N/S,lon,E/W,fix_quality,num_sats,hdop,altitude,M,...
//   int commas[15];
//   int comma_count = 0;

//   for (int i = 0; i < gga_sentence.length() && comma_count < 15; i++) {
//     if (gga_sentence[i] == ',') {
//       commas[comma_count++] = i;
//     }
//   }

//   if (comma_count < 9) return;

//   int fix_quality = gga_sentence.substring(commas[5] + 1, commas[6]).toInt();

//   if (fix_quality > 0) {
//     gps_data.valid = true;
//     gps_data.has_lock = true;

//     // proper decimal-degree conversion with hemisphere sign
//     String lat_field = gga_sentence.substring(commas[1] + 1, commas[2]);
//     String lat_hemi  = gga_sentence.substring(commas[2] + 1, commas[3]);
//     String lon_field = gga_sentence.substring(commas[3] + 1, commas[4]);
//     String lon_hemi  = gga_sentence.substring(commas[4] + 1, commas[5]);

//     if (lat_field.length() > 0) gps_data.latitude  = nmeaToDecimalDegrees(lat_field, lat_hemi);
//     if (lon_field.length() > 0) gps_data.longitude = nmeaToDecimalDegrees(lon_field, lon_hemi);

//     gps_data.satellites = gga_sentence.substring(commas[6] + 1, commas[7]).toInt();

//     if (comma_count > 8) {
//       gps_data.altitude = gga_sentence.substring(commas[8] + 1, commas[9]).toFloat();
//     }

//     gps_data.last_fix_time = millis();

//     static bool first_lock = false;
//     if (!first_lock && gps_data.satellites >= 4) {
//       first_lock = true;
//       Serial.print(F("[GPS] FIRST FIX ACQUIRED! "));
//       Serial.print(gps_data.latitude, 6);
//       Serial.print(F(", "));
//       Serial.println(gps_data.longitude, 6);
//     }
//   }
// }

// // ═══════════════════════════════════════════════════════════════
// // GSM LAYER
// // ═══════════════════════════════════════════════════════════════

// /*
//  * Send a raw AT command and collect the modem's reply until either
//  * the timeout expires or the reply terminates. Returns everything
//  * received so the caller can inspect it.
//  */
// String gsmSendAT(const char* cmd, uint16_t timeout_ms) {
//   // Flush anything stale first
//   while (Serial1.available()) Serial1.read();

//   Serial1.println(cmd);

//   String response = "";
//   unsigned long start = millis();

//   while (millis() - start < timeout_ms) {
//     while (Serial1.available()) {
//       response += (char)Serial1.read();
//     }
//     // Terminate early on a definitive result code
//     if (response.indexOf("OK") != -1 ||
//         response.indexOf("ERROR") != -1 ||
//         response.indexOf(">") != -1) {
//       break;
//     }
//     delay(5);
//   }
//   return response;
// }

// /* Convenience: run a command and test whether the reply contains `expect`. */
// bool gsmExpect(const char* cmd, const char* expect, uint16_t timeout_ms) {
//   String r = gsmSendAT(cmd, timeout_ms);
//   return (r.indexOf(expect) != -1);
// }

// /*
//  * Bring the modem up: verify it responds, SIM is unlocked, it has
//  * registered on the network, and put it in text-mode SMS.
//  */
// bool gsmInit() {
//   // 1) Is the modem alive? Try a few times — it may still be booting.
//   gsm_data.module_found = false;
//   for (int attempt = 0; attempt < 5; attempt++) {
//     if (gsmExpect("AT", "OK", 2000)) {
//       gsm_data.module_found = true;
//       break;
//     }
//     Serial.println(F("[GSM] No response to AT, retrying..."));
//     delay(1000);
//   }
//   if (!gsm_data.module_found) {
//     Serial.println(F("[GSM] ERROR: modem not responding on GPIO 18/19"));
//     return false;
//   }
//   Serial.println(F("[GSM] Modem responding"));

//   gsmSendAT("ATE0", 2000);              // echo off (cleaner parsing)

//   // 2) SIM present and unlocked?
//   String cpin = gsmSendAT("AT+CPIN?", 5000);
//   gsm_data.sim_ready = (cpin.indexOf("READY") != -1);
//   if (!gsm_data.sim_ready) {
//     Serial.println(F("[GSM] ERROR: SIM not ready (missing, or PIN-locked)"));
//     return false;
//   }
//   Serial.println(F("[GSM] SIM ready"));

//   // 3) Registered on the network? Allow time — this can take a while.
//   gsm_data.network_reg = false;
//   for (int attempt = 0; attempt < 15; attempt++) {
//     String creg = gsmSendAT("AT+CREG?", 3000);
//     // ",1" = registered home, ",5" = registered roaming
//     if (creg.indexOf(",1") != -1 || creg.indexOf(",5") != -1) {
//       gsm_data.network_reg = true;
//       break;
//     }
//     Serial.print(F("."));
//     delay(1000);
//   }
//   Serial.println();
//   if (!gsm_data.network_reg) {
//     Serial.println(F("[GSM] ERROR: could not register on network"));
//     Serial.println(F("[GSM] Check antenna, signal, and that the SIM has credit"));
//     return false;
//   }
//   Serial.println(F("[GSM] Registered on network"));

//   // 4) Signal quality
//   String csq = gsmSendAT("AT+CSQ", 3000);
//   int idx = csq.indexOf("+CSQ:");
//   if (idx != -1) {
//     gsm_data.signal_quality = csq.substring(idx + 6, csq.indexOf(',', idx)).toInt();
//   }

//   // 5) SMS text mode + GSM character set
//   gsmSendAT("AT+CMGF=1", 3000);         // text mode (not PDU)
//   gsmSendAT("AT+CSCS=\"GSM\"", 3000);   // standard character set
//   gsmSendAT("AT+CNMI=2,1,0,0,0", 3000); // new-SMS indications

//   gsm_data.last_check = millis();
//   return true;
// }

// /* Periodic lightweight health check (does not block the sensor loop). */
// bool gsmCheckHealth(bool verbose) {
//   if (!gsm_data.module_found) return false;

//   String csq = gsmSendAT("AT+CSQ", 1500);
//   int idx = csq.indexOf("+CSQ:");
//   if (idx != -1) {
//     gsm_data.signal_quality = csq.substring(idx + 6, csq.indexOf(',', idx)).toInt();
//   }

//   String creg = gsmSendAT("AT+CREG?", 1500);
//   gsm_data.network_reg = (creg.indexOf(",1") != -1 || creg.indexOf(",5") != -1);

//   if (verbose) {
//     Serial.print(F("[GSM] CSQ="));
//     Serial.print(gsm_data.signal_quality);
//     Serial.print(F("  registered="));
//     Serial.println(gsm_data.network_reg ? F("yes") : F("no"));
//   }
//   return gsm_data.network_reg;
// }

// /*
//  * Compose the emergency message. Kept compact so it fits comfortably
//  * in one or two SMS segments, and leads with the most important
//  * information in case only the notification preview is seen.
//  */
// String buildAlertMessage(int severity_class, float confidence, float peak_g) {
//   String msg = "";

//   if (severity_class == SEVERITY_SEVERE) {
//     msg += "*** SEVERE ACCIDENT ***\n";
//   } else if (severity_class == SEVERITY_MODERATE) {
//     msg += "** ACCIDENT DETECTED **\n";
//   } else {
//     msg += "Impact event logged\n";
//   }

//   msg += "Sentinel unit alert\n";
//   msg += "Peak: " + String(peak_g, 1) + "g\n";
//   msg += "Confidence: " + String((int)(confidence * 100)) + "%\n";

//   if (gps_data.has_lock && gps_data.valid) {
//     msg += "Loc: " + String(gps_data.latitude, 6) + "," + String(gps_data.longitude, 6) + "\n";
//     msg += "https://maps.google.com/?q=";
//     msg += String(gps_data.latitude, 6);
//     msg += ",";
//     msg += String(gps_data.longitude, 6);
//     msg += "\n";
//     msg += "Sats: " + String(gps_data.satellites) + "\n";
//   } else {
//     msg += "Loc: GPS FIX UNAVAILABLE\n";
//   }

//   msg += "Uptime: " + String(millis() / 1000) + "s";
//   return msg;
// }

// /*
//  * Send one SMS. Text mode: issue AT+CMGS="number", wait for the ">"
//  * prompt, write the body, then terminate with Ctrl+Z (0x1A).
//  */
// bool gsmSendSMSTo(const char* number, const String& message) {
//   if (!gsm_data.module_found) return false;

//   Serial.print(F("[GSM] Sending SMS to "));
//   Serial.println(number);

//   // Flush stale data
//   while (Serial1.available()) Serial1.read();

//   // Ensure text mode (cheap, and guards against a modem reset)
//   Serial1.println("AT+CMGF=1");
//   delay(300);
//   while (Serial1.available()) Serial1.read();

//   // Address
//   Serial1.print("AT+CMGS=\"");
//   Serial1.print(number);
//   Serial1.println("\"");

//   // Wait for the '>' prompt
//   unsigned long start = millis();
//   bool prompt = false;
//   String pre = "";
//   while (millis() - start < 5000) {
//     while (Serial1.available()) {
//       char c = (char)Serial1.read();
//       pre += c;
//       if (c == '>') { prompt = true; break; }
//     }
//     if (prompt) break;
//     delay(5);
//   }

//   if (!prompt) {
//     Serial.println(F("[GSM] ERROR: no '>' prompt from modem"));
//     Serial1.write(27);            // ESC — abort the command
//     delay(200);
//     gsm_data.sms_failed++;
//     return false;
//   }

//   // Body then Ctrl+Z
//   Serial1.print(message);
//   delay(100);
//   Serial1.write(26);              // Ctrl+Z terminates the message

//   // Wait for +CMGS confirmation
//   String response = "";
//   start = millis();
//   while (millis() - start < GSM_SMS_TIMEOUT_MS) {
//     while (Serial1.available()) {
//       response += (char)Serial1.read();
//     }
//     if (response.indexOf("+CMGS") != -1 || response.indexOf("ERROR") != -1) break;
//     delay(10);
//   }

//   if (response.indexOf("+CMGS") != -1) {
//     Serial.println(F("[GSM] SMS SENT OK"));
//     gsm_data.sms_sent++;
//     return true;
//   }

//   Serial.println(F("[GSM] ERROR: SMS send failed"));
//   Serial.print(F("[GSM] Modem said: "));
//   Serial.println(response);
//   gsm_data.sms_failed++;
//   return false;
// }

// /* Broadcast the alert to every configured emergency contact. */
// bool sendEmergencySMS(int severity_class, float confidence, float peak_g) {
//   if (!gsm_data.module_found) {
//     Serial.println(F("[GSM] Cannot send SMS - modem unavailable"));
//     return false;
//   }

//   String message = buildAlertMessage(severity_class, confidence, peak_g);

//   Serial.println(F("[GSM] ---- Emergency message ----"));
//   Serial.println(message);
//   Serial.println(F("[GSM] ---------------------------"));

//   bool any_sent = false;
//   for (uint8_t i = 0; i < NUM_EMERGENCY_CONTACTS; i++) {
//     // One retry per contact — SMS is the last line of defence
//     if (gsmSendSMSTo(EMERGENCY_CONTACTS[i], message)) {
//       any_sent = true;
//     } else {
//       Serial.println(F("[GSM] Retrying once..."));
//       delay(2000);
//       if (gsmSendSMSTo(EMERGENCY_CONTACTS[i], message)) any_sent = true;
//     }
//     delay(1000);   // small gap between recipients
//   }
//   return any_sent;
// }

// /* Optional voice call for SEVERE events. */
// void gsmPlaceEmergencyCall(const char* number) {
//   if (!gsm_data.module_found) return;

//   Serial.print(F("[GSM] Placing emergency call to "));
//   Serial.println(number);

//   String cmd = "ATD";
//   cmd += number;
//   cmd += ";";
//   Serial1.println(cmd);

//   unsigned long start = millis();
//   while (millis() - start < CALL_DURATION_MS) {
//     // Keep the red channel pulsing during the call
//     rgbRed();
//     delay(400);
//     rgbOff();
//     delay(400);
//   }

//   Serial1.println("ATH");        // hang up
//   delay(500);
//   rgbRed();                      // back to steady emergency state
//   Serial.println(F("[GSM] Call ended"));
// }

// // ═══════════════════════════════════════════════════════════════
// // OFFLINE FALLBACK CLASSIFICATION
// // ═══════════════════════════════════════════════════════════════

// /*
//  * When the ML API is unreachable, classify locally from the buffer
//  * so the SMS can still carry a meaningful severity.
//  *
//  * Uses the same signature logic as the deployed API: a crash is a
//  * 2–7 g event lasting 40–250 ms. Sustained high-g is treated as an
//  * aggressive manoeuvre, not a crash. Severity within the crash band
//  * is graded by impulse (a delta-v proxy).
//  */
// void classifyLocally(CrashResult& result) {
//   float peak = 0.0f;
//   float impulse = 0.0f;         // g·s above the 1 g baseline
//   int   samples_above = 0;
//   int   longest_run = 0;
//   int   current_run = 0;

//   // Walk oldest-first, exactly like the payload sent to the API. Walking the
//   // raw array order would split a pulse that spans the wrap point into two
//   // shorter runs and could under-measure the duration.
//   for (int i = 0; i < BUFFER_SIZE; i++) {
//     float m = imu_buffer[(buffer_index + i) % BUFFER_SIZE].accel_mag;
//     if (m > peak) peak = m;

//     if (m >= SIG_PEAK_MIN_G) {
//       samples_above++;
//       current_run++;
//       if (current_run > longest_run) longest_run = current_run;
//       impulse += (m - 1.0f) * (SENSOR_INTERVAL_MS / 1000.0f);
//     } else {
//       current_run = 0;
//     }
//   }

//   int longest_ms = longest_run * SENSOR_INTERVAL_MS;

//   bool signature_match = (peak >= SIG_PEAK_MIN_G && peak < SIG_PEAK_MAX_G &&
//                           longest_ms >= SIG_DUR_MIN_MS && longest_ms <= SIG_DUR_MAX_MS);

//   result.detected = true;
//   result.peak_magnitude_g = peak;
//   result.confidence = 0.0f;      // locally derived, not a model probability

//   if (!signature_match) {
//     result.severity_class = SEVERITY_NORMAL;
//     result.severity_name = "Normal";
//   } else if (impulse >= SIG_SEVERE_IMPULSE) {
//     result.severity_class = SEVERITY_SEVERE;
//     result.severity_name = "Severe";
//   } else {
//     result.severity_class = SEVERITY_MODERATE;
//     result.severity_name = "Moderate";
//   }

//   Serial.println(F("[LOCAL] ML API unreachable - classified on-device"));
//   Serial.print(F("[LOCAL] peak="));       Serial.print(peak, 2);
//   Serial.print(F("g  longest_run="));     Serial.print(longest_ms);
//   Serial.print(F("ms  impulse="));        Serial.print(impulse, 3);
//   Serial.print(F("g.s  -> "));            Serial.println(result.severity_name);
// }

// // ═══════════════════════════════════════════════════════════════
// // MAIN LOOP
// // ═══════════════════════════════════════════════════════════════

// void loop() {
//   unsigned long now = millis();

//   // Check WiFi status
//   if (WiFi.status() != WL_CONNECTED && wifi_ready) {
//     wifi_ready = false;
//   }

//   // ─────────────────────────────────────────────────────────────
//   // Update GPS (non-blocking)
//   // ─────────────────────────────────────────────────────────────
//   updateGPS();

//   // ─────────────────────────────────────────────────────────────
//   // Read IMU Sensors at 100Hz (every 10ms)
//   // ─────────────────────────────────────────────────────────────
//   if (now - last_sensor_read >= SENSOR_INTERVAL_MS) {
//     if (mpu_ready) {
//       mpu.getAcceleration(&accelX, &accelY, &accelZ);
//       mpu.getRotation(&gyroX, &gyroY, &gyroZ);

//       accel_x_g = accelX * RAW_ACCEL_TO_G;
//       accel_y_g = accelY * RAW_ACCEL_TO_G;
//       accel_z_g = accelZ * RAW_ACCEL_TO_G;

//       float gyro_x_dps = gyroX * RAW_GYRO_TO_DPS;
//       float gyro_y_dps = gyroY * RAW_GYRO_TO_DPS;
//       float gyro_z_dps = gyroZ * RAW_GYRO_TO_DPS;

//       float mag = sqrt(accel_x_g * accel_x_g +
//                        accel_y_g * accel_y_g +
//                        accel_z_g * accel_z_g);

//       float jerk_g_per_sec = (mag - prev_accel_mag_g) / (SENSOR_INTERVAL_MS / 1000.0f);

//       // ─────────────────────────────────────────────────────────
//       // CRASH DETECTION: Dual Threshold
//       // ─────────────────────────────────────────────────────────
//       bool accel_threshold = (mag >= ACCEL_THRESHOLD_G);
//       bool jerk_threshold = (fabs(jerk_g_per_sec) >= JERK_THRESHOLD_G_PER_SEC);

//       if (accel_threshold && jerk_threshold && !crash_detected_flag) {
//         if (now - last_crash_detection > 2000) {
//           crash_detected_flag = true;
//           last_crash_detection = now;

//           // Snapshot what the device measured, before the post-roll runs.
//           trigger_peak_g  = mag;
//           trigger_jerk_gs = jerk_g_per_sec;
//           trigger_millis  = now;

//           rgbOff();

//           Serial.print(F("[CRASH] Detected at "));
//           Serial.print(now);
//           Serial.print(F(" ms | Accel="));
//           Serial.print(mag, 2);
//           Serial.print(F("g, Jerk="));
//           Serial.print(jerk_g_per_sec, 2);
//           Serial.println(F("g/s"));

//           // White = trigger fired, post-impact window being captured.
//           // No delay() here: blocking would drop the samples right after the
//           // impact, which are exactly the ones the API needs.
//           rgbWhite();
//           tone(BUZZER_PIN, 1500, 50);
//           post_trigger_remaining = POST_TRIGGER_SAMPLES;
//           Serial.println(F("[CRASH] Capturing 2.5 s post-impact data..."));
//         }
//       }

//       // Store sample in circular buffer
//       imu_buffer[buffer_index].ax_g = accel_x_g;
//       imu_buffer[buffer_index].ay_g = accel_y_g;
//       imu_buffer[buffer_index].az_g = accel_z_g;
//       imu_buffer[buffer_index].gx_dps = gyro_x_dps;
//       imu_buffer[buffer_index].gy_dps = gyro_y_dps;
//       imu_buffer[buffer_index].gz_dps = gyro_z_dps;
//       imu_buffer[buffer_index].accel_mag = mag;
//       imu_buffer[buffer_index].timestamp_ms = now;

//       buffer_index++;
//       if (buffer_index >= BUFFER_SIZE) {
//         buffer_index = 0;
//         buffer_full = true;
//       }

//       if (crash_detected_flag && post_trigger_remaining > 0) {
//         post_trigger_remaining--;
//       }

//       prev_accel_mag_g = mag;
//       sample_count++;
//     }
//     last_sensor_read = now;
//   }

//   // ─────────────────────────────────────────────────────────────
//   // Process crash once the buffer holds the full 5 s window.
//   //
//   // The WiFi requirement is deliberately NOT part of this condition.
//   // The crash is always processed; processCrash() decides whether to
//   // use the ML API or fall back to on-device classification, and the
//   // SMS goes out either way.
//   // ─────────────────────────────────────────────────────────────
//   if (crash_detected_flag && post_trigger_remaining == 0 && buffer_full && !api_request_in_progress) {
//     api_request_in_progress = true;
//     Serial.println(F("[CRASH] Processing incident..."));
//     processCrash();
//     crash_detected_flag = false;
//     api_request_in_progress = false;
//   }

//   // ─────────────────────────────────────────────────────────────
//   // Periodic GSM health check (every 60 s, cheap)
//   // ─────────────────────────────────────────────────────────────
//   if (gsm_ready && (now - last_gsm_health >= 60000)) {
//     gsmCheckHealth(false);
//     last_gsm_health = now;
//   }

//   // ─────────────────────────────────────────────────────────────
//   // Print status every 100ms
//   // ─────────────────────────────────────────────────────────────
//   if (now - last_print >= PRINT_INTERVAL_MS) {
//     if (mpu_ready && buffer_full && sample_count > 1) {
//       uint16_t curr_idx = (buffer_index - 1 + BUFFER_SIZE) % BUFFER_SIZE;
//       uint16_t prev_idx = (buffer_index - 2 + BUFFER_SIZE) % BUFFER_SIZE;

//       float time_delta = (imu_buffer[curr_idx].timestamp_ms -
//                           imu_buffer[prev_idx].timestamp_ms) / 1000.0f;
//       float jerk = 0;
//       if (time_delta > 0) {
//         jerk = (imu_buffer[curr_idx].accel_mag -
//                 imu_buffer[prev_idx].accel_mag) / time_delta;
//       }

//       Serial.print(now);
//       Serial.print(F("  | "));
//       Serial.print(imu_buffer[curr_idx].accel_mag, 2);
//       Serial.print(F("g    | "));
//       Serial.print(jerk, 2);
//       Serial.print(F("    | "));
//       Serial.print(wifi_ready ? F("OK") : F("--"));
//       Serial.print(F("   | "));
//       Serial.print(mpu_ready ? F("OK") : F("--"));
//       Serial.print(F("  | "));
//       Serial.print(gps_data.has_lock ? gps_data.satellites : 0);
//       Serial.print(F("        | "));
//       if (gsm_data.network_reg)      Serial.print(F("OK "));
//       else if (gsm_data.module_found) Serial.print(F("NR "));   // no registration
//       else                            Serial.print(F("-- "));
//       Serial.print(F("| "));
//       Serial.println(F("RUNNING"));
//     }
//     last_print = now;
//   }
// }

// // ═══════════════════════════════════════════════════════════════
// // CRASH PROCESSING
// // ═══════════════════════════════════════════════════════════════

// void processCrash() {
//   bool api_success = false;

//   // ─── Path 1: WiFi available -> ask the ML API ───────────────
//   if (wifi_ready && WiFi.isConnected()) {
//     Serial.println(F("[BACKEND] Uploading crash window..."));
//     api_success = sendEventToBackend();
//   } else {
//     Serial.println(F("[BACKEND] Skipped - no WiFi connection"));
//   }

//   // ─── Path 2: API unavailable -> classify on-device ───────────
//   if (!api_success) {
//     classifyLocally(last_crash_result);
//     crash_count++;
//     displayCrashResults();
//   }

//   int severity = last_crash_result.severity_class;

//   // ─── Local alerting (RGB + buzzer) ───────────────────────────
//   triggerSeverityAlert(severity);

//   // ─── GSM emergency notification ──────────────────────────────
//   bool should_sms = (severity == SEVERITY_SEVERE   && SMS_ON_SEVERE) ||
//                     (severity == SEVERITY_MODERATE && SMS_ON_MODERATE);

//   if (should_sms) {
//     if (gsm_data.module_found) {
//       Serial.println(F("[GSM] Dispatching emergency SMS..."));
//       bool sent = sendEmergencySMS(severity,
//                                    last_crash_result.confidence,
//                                    last_crash_result.peak_magnitude_g);

//       if (sent) {
//         // Two short confirmation chirps: contacts have been notified
//         for (int i = 0; i < 2; i++) {
//           tone(BUZZER_PIN, 2500, 80);
//           delay(160);
//           noTone(BUZZER_PIN);
//         }
//         Serial.println(F("[GSM] Emergency contacts notified"));
//       } else {
//         Serial.println(F("[GSM] WARNING: could not notify any contact"));
//       }

//       // Escalate to a voice call on SEVERE
//       if (severity == SEVERITY_SEVERE && CALL_ON_SEVERE &&
//           NUM_EMERGENCY_CONTACTS > 0) {
//         gsmPlaceEmergencyCall(EMERGENCY_CONTACTS[0]);
//       }

//     } else {
//       Serial.println(F("[GSM] WARNING: modem unavailable - no SMS sent"));
//     }
//   } else {
//     Serial.println(F("[GSM] Normal severity - no SMS required"));
//   }
// }

// // ═══════════════════════════════════════════════════════════════
// // SEVERITY-BASED RGB & BUZZER ALERTS
// // ═══════════════════════════════════════════════════════════════

// void triggerSeverityAlert(int severity_class) {
//   rgbOff();
//   noTone(BUZZER_PIN);

//   switch (severity_class) {
//     case SEVERITY_NORMAL:
//       Serial.println(F("SEVERITY: NORMAL - No alert needed"));
//       rgbOff();
//       delay(150);
//       rgbGreen();
//       delay(150);
//       rgbOff();
//       delay(150);
//       tone(BUZZER_PIN, 1000, 100);
//       delay(150);
//       noTone(BUZZER_PIN);
//       rgbGreen();
//       break;

//     case SEVERITY_MODERATE:
//       Serial.println(F("SEVERITY: MODERATE - Warning alert"));
//       for (int i = 0; i < 3; i++) {
//         rgbAmber();
//         tone(BUZZER_PIN, 1500, 150);
//         delay(300);
//         rgbOff();
//         noTone(BUZZER_PIN);
//         delay(200);
//       }
//       rgbAmber();
//       break;

//     case SEVERITY_SEVERE:
//       Serial.println(F("SEVERITY: SEVERE - EMERGENCY ALERT!!!"));
//       for (int i = 0; i < 5; i++) {
//         rgbRed();
//         tone(BUZZER_PIN, 2000, 100);
//         delay(150);
//         rgbOff();
//         noTone(BUZZER_PIN);
//         delay(150);
//       }
//       rgbRed();
//       break;
//   }
// }

// // ═══════════════════════════════════════════════════════════════
// // ML API COMMUNICATION
// // ═══════════════════════════════════════════════════════════════

// /** Append a float without dragging printf's float formatting into the build. */
// static void appendFloat(String& out, float v, uint8_t decimals) {
//   char num[20];
//   dtostrf(v, 0, decimals, num);
//   out += num;
// }

// /** event_id = {DEVICE_ID}-{boot:04d}-{millis:08d}
//  *
//  *  The server is idempotent on this, which is what makes the retries below
//  *  safe: one shake yields exactly one incident however many times we send it.
//  */
// static String makeEventId() {
//   char buf[64];
//   snprintf(buf, sizeof(buf), "%s-%04lu-%08lu",
//            DEVICE_ID, (unsigned long)boot_count, (unsigned long)trigger_millis);
//   return String(buf);
// }

// /** ISO-8601 UTC for the TRIGGER instant, not the upload instant.
//  *
//  *  Walks back from now by the post-roll just spent, so the dashboard's
//  *  detected -> received figure measures the network rather than our own wait.
//  *  Returns empty when NTP has not synced: an absent timestamp beats a wrong one.
//  */
// static String triggerTimestampIso() {
//   time_t now_epoch = time(nullptr);
//   if (now_epoch < 1600000000) return String();
//   uint32_t ago_ms = millis() - trigger_millis;
//   time_t trigger_epoch = now_epoch - (time_t)(ago_ms / 1000);
//   struct tm tm_utc;
//   gmtime_r(&trigger_epoch, &tm_utc);
//   char buf[32];
//   strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
//   return String(buf);
// }

// /** Build the /api/v1/events body straight into a heap String.
//  *
//  *  Deliberately NOT a StaticJsonDocument: 500x6 floats needs far more than the
//  *  8 kB loop-task stack, and an oversized document on that stack has already
//  *  overflowed this project once. reserve() takes one allocation up front rather
//  *  than repeatedly reallocating while appending ~28 kB.
//  */
// static void buildEventBody(String& body, const String& event_id) {
//   body = "";
//   body.reserve(40000);

//   body += F("{\"device_id\":\"");
//   body += DEVICE_ID;
//   body += F("\",\"event_id\":\"");
//   body += event_id;
//   body += F("\"");

//   String detected_at = triggerTimestampIso();
//   if (detected_at.length()) {
//     body += F(",\"detected_at\":\"");
//     body += detected_at;
//     body += F("\"");
//   }

//   body += F(",\"trigger\":{\"peak_g\":");
//   appendFloat(body, trigger_peak_g, 3);
//   body += F(",\"jerk_gs\":");
//   appendFloat(body, trigger_jerk_gs, 3);

//   body += F("},\"gps\":{");
//   if (gps_data.valid && gps_data.has_lock) {
//     body += F("\"lat\":");
//     appendFloat(body, gps_data.latitude, 6);
//     body += F(",\"lon\":");
//     appendFloat(body, gps_data.longitude, 6);
//     body += F(",\"valid\":true,\"satellites\":");
//     body += String(gps_data.satellites);
//   } else {
//     body += F("\"valid\":false");
//   }

//   body += F("},\"device\":{\"uptime_s\":");
//   body += String(millis() / 1000);
//   body += F(",\"free_heap\":");
//   body += String(ESP.getFreeHeap());
//   body += F(",\"rssi\":");
//   body += String(WiFi.RSSI());
//   body += F(",\"firmware_version\":\"");
//   body += FIRMWARE_VERSION;
//   body += F("\"},\"window\":{\"fs_hz\":100,\"units_accel\":\"g\",\"units_gyro\":\"deg_s\"");

//   // Oldest -> newest. With the post-roll complete this is exactly 250
//   // pre-trigger + 250 post-trigger samples, so the impact sits mid-window.
//   const char* names[6] = {"ax", "ay", "az", "gx", "gy", "gz"};
//   for (uint8_t f = 0; f < 6; f++) {
//     body += F(",\"");
//     body += names[f];
//     body += F("\":[");
//     for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
//       const IMUSample& smp = imu_buffer[(buffer_index + i) % BUFFER_SIZE];
//       float v;
//       switch (f) {
//         case 0:  v = smp.ax_g;   break;
//         case 1:  v = smp.ay_g;   break;
//         case 2:  v = smp.az_g;   break;
//         case 3:  v = smp.gx_dps; break;
//         case 4:  v = smp.gy_dps; break;
//         default: v = smp.gz_dps; break;
//       }
//       if (i) body += ',';
//       appendFloat(body, v, 4);
//     }
//     body += ']';
//   }
//   body += F("}}");
// }

// /** Parse the backend's deliberately small, flat ACK. */
// static void parseAck(const String& resp) {
//   JsonDocument doc;
//   if (deserializeJson(doc, resp)) {
//     Serial.println(F("[BACKEND] Malformed ACK"));
//     return;
//   }
//   crash_count++;
//   last_crash_result.detected = true;

//   if (doc["severity_class"].isNull()) {
//     // Stored, but the model was unreachable. Treat as worst case locally so
//     // the alert still fires; the backend will classify it on retry.
//     last_crash_result.severity_class = SEVERITY_SEVERE;
//     last_crash_result.severity_name  = "Pending";
//     Serial.println(F("[BACKEND] Stored - classification pending"));
//   } else {
//     last_crash_result.severity_class = doc["severity_class"].as<int>();
//     switch (last_crash_result.severity_class) {
//       case SEVERITY_NORMAL:   last_crash_result.severity_name = "Normal";   break;
//       case SEVERITY_MODERATE: last_crash_result.severity_name = "Moderate"; break;
//       case SEVERITY_SEVERE:   last_crash_result.severity_name = "Severe";   break;
//       default:                last_crash_result.severity_name = "Unknown";  break;
//     }
//   }
//   last_crash_result.confidence       = doc["confidence"] | 0.0f;
//   last_crash_result.peak_magnitude_g = doc["peak_g"] | trigger_peak_g;

//   displayCrashResults();
//   Serial.print(F("[BACKEND] P(crash)="));
//   Serial.print((float)(doc["p_crash"] | 0.0f), 3);
//   Serial.print(F("  source="));
//   Serial.println((const char*)(doc["label_source"] | "unknown"));
// }

// /** POST the impact-centred window to the backend, retrying transport faults. */
// bool sendEventToBackend() {
//   if (!WiFi.isConnected()) {
//     Serial.println(F("[ERROR] WiFi not connected!"));
//     return false;
//   }

//   String event_id = makeEventId();
//   String body;
//   buildEventBody(body, event_id);

//   Serial.print(F("[BACKEND] Body "));
//   Serial.print(body.length());
//   Serial.print(F(" bytes | free heap "));
//   Serial.println(ESP.getFreeHeap());

//   String url = String(BACKEND_URL) + "/api/v1/events";

//   for (uint8_t attempt = 1; attempt <= MAX_POST_ATTEMPTS; attempt++) {
//     xSemaphoreTake(http_mutex, portMAX_DELAY);

//     HTTPClient http;
//     http.setTimeout(HTTP_TIMEOUT_MS);
//     http.begin(url);
//     http.addHeader("Content-Type", "application/json");
//     http.addHeader("X-API-Key", API_KEY);

//     unsigned long t0 = millis();
//     int code = http.POST(body);
//     unsigned long elapsed = millis() - t0;

//     Serial.print(F("[BACKEND] Attempt "));
//     Serial.print(attempt);
//     Serial.print(F(" -> "));
//     Serial.print(code);
//     Serial.print(F(" in "));
//     Serial.print(elapsed);
//     Serial.println(F(" ms"));

//     if (code == 200 || code == 201) {
//       String resp = http.getString();
//       http.end();
//       xSemaphoreGive(http_mutex);
//       parseAck(resp);
//       return true;
//     }

//     // 4xx is a contract violation; retrying cannot fix it.
//     if (code >= 400 && code < 500) {
//       Serial.print(F("[BACKEND] Rejected: "));
//       Serial.println(http.getString().substring(0, 160));
//       http.end();
//       xSemaphoreGive(http_mutex);
//       return false;
//     }

//     http.end();
//     xSemaphoreGive(http_mutex);

//     if (attempt < MAX_POST_ATTEMPTS) {
//       Serial.print(F("[BACKEND] Retrying in "));
//       Serial.print(RETRY_BACKOFF_MS[attempt - 1]);
//       Serial.println(F(" ms (server is idempotent on event_id)"));
//       delay(RETRY_BACKOFF_MS[attempt - 1]);
//     }
//   }

//   Serial.println(F("[BACKEND] Upload failed after all attempts"));
//   return false;
// }

// /** Liveness beacon. This is what puts the node on the dashboard as "online"
//  *  before any crash has ever happened. */
// void heartbeatTask(void* param) {
//   (void)param;
//   vTaskDelay(pdMS_TO_TICKS(5000));

//   for (;;) {
//     if (wifi_ready && WiFi.isConnected()) {
//       // Never wait on the lock: a heartbeat is expendable, a crash upload is not.
//       if (xSemaphoreTake(http_mutex, 0) == pdTRUE) {
//         String body;
//         body.reserve(320);
//         body += F("{\"device_id\":\"");
//         body += DEVICE_ID;
//         body += F("\",\"gps\":{");
//         if (gps_data.valid && gps_data.has_lock) {
//           body += F("\"lat\":");
//           appendFloat(body, gps_data.latitude, 6);
//           body += F(",\"lon\":");
//           appendFloat(body, gps_data.longitude, 6);
//           body += F(",\"valid\":true,\"satellites\":");
//           body += String(gps_data.satellites);
//         } else {
//           body += F("\"valid\":false");
//         }
//         body += F("},\"device\":{\"uptime_s\":");
//         body += String(millis() / 1000);
//         body += F(",\"free_heap\":");
//         body += String(ESP.getFreeHeap());
//         body += F(",\"rssi\":");
//         body += String(WiFi.RSSI());
//         body += F(",\"firmware_version\":\"");
//         body += FIRMWARE_VERSION;
//         body += F("\"}}");

//         HTTPClient http;
//         http.setTimeout(5000);
//         http.begin(String(BACKEND_URL) + "/api/v1/heartbeat");
//         http.addHeader("Content-Type", "application/json");
//         http.addHeader("X-API-Key", API_KEY);
//         int code = http.POST(body);
//         http.end();
//         xSemaphoreGive(http_mutex);

//         if (code != 200) {
//           Serial.print(F("[HB] "));
//           Serial.println(code);
//         }
//       }
//     }
//     vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
//   }
// }

// // ═══════════════════════════════════════════════════════════════

// void displayCrashResults() {
//   Serial.println();
//   Serial.println(F("+========================================================+"));
//   Serial.println(F("|              CRASH ANALYSIS COMPLETE                   |"));
//   Serial.println(F("+========================================================+"));
//   Serial.print(F("| Incident #"));
//   Serial.println(crash_count);
//   Serial.print(F("| Severity Class: "));
//   Serial.println(last_crash_result.severity_class);
//   Serial.print(F("| Severity Name: "));
//   Serial.println(last_crash_result.severity_name);
//   Serial.print(F("| Confidence: "));
//   Serial.print(last_crash_result.confidence * 100, 1);
//   Serial.println(F("%"));
//   Serial.print(F("| Peak Acceleration: "));
//   Serial.print(last_crash_result.peak_magnitude_g, 2);
//   Serial.println(F("g"));

//   if (gps_data.has_lock && gps_data.valid) {
//     Serial.println(F("+--------------------------------------------------------+"));
//     Serial.println(F("| GPS LOCATION                                           |"));
//     Serial.print(F("| Latitude:  "));
//     Serial.println(gps_data.latitude, 6);
//     Serial.print(F("| Longitude: "));
//     Serial.println(gps_data.longitude, 6);
//     Serial.print(F("| Altitude:  "));
//     Serial.print(gps_data.altitude, 1);
//     Serial.println(F(" m"));
//     Serial.print(F("| Satellites: "));
//     Serial.println(gps_data.satellites);
//     Serial.print(F("| Map: https://maps.google.com/?q="));
//     Serial.print(gps_data.latitude, 6);
//     Serial.print(F(","));
//     Serial.println(gps_data.longitude, 6);
//   } else {
//     Serial.println(F("+--------------------------------------------------------+"));
//     Serial.println(F("| GPS STATUS: No lock (awaiting satellites)              |"));
//   }

//   // GSM status block
//   Serial.println(F("+--------------------------------------------------------+"));
//   Serial.println(F("| GSM STATUS                                             |"));
//   Serial.print(F("| Modem: "));
//   Serial.println(gsm_data.module_found ? F("present") : F("NOT FOUND"));
//   Serial.print(F("| Network: "));
//   Serial.println(gsm_data.network_reg ? F("registered") : F("not registered"));
//   Serial.print(F("| Signal (CSQ): "));
//   Serial.println(gsm_data.signal_quality);
//   Serial.print(F("| SMS sent / failed: "));
//   Serial.print(gsm_data.sms_sent);
//   Serial.print(F(" / "));
//   Serial.println(gsm_data.sms_failed);

//   Serial.println(F("+========================================================+"));
//   Serial.println();
// }

// // ═══════════════════════════════════════════════════════════════
// // BANNER
// // ═══════════════════════════════════════════════════════════════

// void printBanner() {
//   Serial.println();
//   Serial.println(F("+========================================================+"));
//   Serial.println(F("|   ESP32 COMPLETE CRASH DETECTION SYSTEM                |"));
//   Serial.println(F("|   RGB LED + GPS + GSM EMERGENCY NOTIFICATION           |"));
//   Serial.println(F("|                                                        |"));
//   Serial.println(F("|   Components:                                          |"));
//   Serial.println(F("|   - MPU6050  (Crash Detection)      GPIO 21/22         |"));
//   Serial.println(F("|   - WiFi     (ML API severity)                         |"));
//   Serial.println(F("|   - GPS      NEO-6M  Serial2        GPIO 16/17         |"));
//   Serial.println(F("|   - GSM      SIM800L Serial1                           |"));
//   Serial.println(F("|       ESP32 GPIO19 TXD --> SIM800L RXD                 |"));
//   Serial.println(F("|       ESP32 GPIO18 RXD <-- SIM800L TXD                 |"));
//   Serial.println(F("|   - RGB LED  (common cathode, direct to ESP32)         |"));
//   Serial.println(F("|       R = GPIO 4  via 330R                             |"));
//   Serial.println(F("|       G = GPIO 27 via 100R                             |"));
//   Serial.println(F("|       B = GPIO 23 via 100R                             |"));
//   Serial.println(F("|       Common (long leg) -> GND                         |"));
//   Serial.println(F("|   - Buzzer                          GPIO 32            |"));
//   Serial.println(F("|                                                        |"));
//   Serial.println(F("|   Old LED pins 25 / 26 / 33 are UNUSED in this build.  |"));
//   Serial.println(F("|   They stay soldered but are never driven.             |"));
//   Serial.println(F("|                                                        |"));
//   Serial.println(F("|   Alert Colours:                                       |"));
//   Serial.println(F("|   GREEN  = NORMAL   : no alert, no SMS                 |"));
//   Serial.println(F("|   AMBER  = MODERATE : 3 pulses + SMS to contacts       |"));
//   Serial.println(F("|   RED    = SEVERE   : 5 flashes + SMS + voice call     |"));
//   Serial.println(F("|   WHITE  = crash trigger fired, capturing window       |"));
//   Serial.println(F("|   BLUE   = WiFi connected (boot indicator)             |"));
//   Serial.println(F("|                                                        |"));
//   Serial.println(F("|   Resilience:                                          |"));
//   Serial.println(F("|   If WiFi/ML API is down, the crash is classified      |"));
//   Serial.println(F("|   on-device and the SMS is still sent over GSM.        |"));
//   Serial.println(F("|                                                        |"));
//   Serial.println(F("|   *** SIM800L NEEDS ITS OWN 4V / 2A SUPPLY ***         |"));
//   Serial.println(F("|   *** DO NOT POWER IT FROM THE ESP32 3V3 PIN ***       |"));
//   Serial.println(F("+========================================================+"));
//   Serial.println();
// }




/*
 * ═══════════════════════════════════════════════════════════════
 * ESP32 CRASH DETECTION SYSTEM  —  MPU6050 + WiFi/ML + GPS + GSM
 * RGB LED REVISION — DIRECT WIRING ON FREE PINS
 * ═══════════════════════════════════════════════════════════════
 *
 * WHAT CHANGED IN THIS VERSION:
 *   1. The three discrete LEDs (red / yellow / green) are replaced
 *      by ONE common-cathode RGB LED.
 *   2. The RGB LED is wired DIRECTLY to the ESP32 on three free
 *      pins — GPIO 4, 27 and 23. It does NOT reuse the old LED
 *      branches on the perfboard.
 *   3. The old LED pins (25, 26, 33) are now completely unused.
 *      No pinMode() is called on them, so they sit as high-impedance
 *      inputs and the old LEDs stay dark. They remain soldered to
 *      the perfboard; nothing needs to be removed.
 *   4. [CRITICAL BUGFIX, now moot] LED_RED was previously defined as
 *      GPIO 34. GPIO 34 is INPUT-ONLY on the ESP32 and cannot drive
 *      an output, so the red LED never lit. Red now lives on GPIO 4.
 *   5. There is no separate "yellow" channel. Amber is produced by
 *      lighting RED and GREEN together.
 *   6. Blue is now available and is used as a WiFi boot indicator
 *      and as part of the white crash-trigger flash.
 *   7. All LED writes go through rgbWrite() / the named colour
 *      helpers. Common-anode LEDs are supported by one flag.
 *
 * NOTHING ELSE WAS ALTERED. Crash detection, buffering, the ML API
 * call, local fallback classification, GPS parsing, GSM and SMS
 * logic are the same behaviour as the previous version.
 *
 * ─── PIN ALLOCATION MAP (verify before soldering) ──────────────
 * OCCUPIED:
 *    1,  3   USB serial (Serial, 115200)
 *   16, 17   GPS NEO-6M      (Serial2)
 *   18, 19   SIM800L GSM     (Serial1)
 *   21, 22   MPU6050 I2C     (SDA / SCL)
 *       32   Buzzer
 *   25, 26, 33  old LED branches - still soldered, now UNUSED
 *
 * NEVER USE:
 *    6 - 11  SPI flash (using these bricks the board)
 *   34, 35, 36, 39  input-only, no output driver
 *    0, 2, 12, 15   strapping pins, interfere with boot/flash
 *
 * FREE:
 *    4, 13, 23, 27     <- RGB uses 4, 27, 23. GPIO 13 is the spare.
 *
 * VERIFY FIRST: with the board powered off, put a multimeter in
 * continuity mode and probe GPIO 4, 27 and 23 against the ground
 * rail and any nearby component leg. All three must read OPEN. If
 * one beeps, something is already wired to it — move that channel
 * to GPIO 13 and change the matching #define below.
 * ───────────────────────────────────────────────────────────────
 *
 * ─── RGB LED WIRING (common cathode, direct to ESP32) ─────────
 * A standard 5mm common-cathode RGB LED has four legs, in order:
 *     Red  |  Common (longest leg)  |  Green  |  Blue
 *
 *   RGB Red    --[ 330R ]-->  ESP32 GPIO 4
 *   RGB Green  --[ 100R ]-->  ESP32 GPIO 27
 *   RGB Blue   --[ 100R ]-->  ESP32 GPIO 23
 *   RGB Common ------------>  ESP32 GND  (any GND pin)
 *
 * Each colour leg gets its OWN resistor. A single resistor on the
 * common leg makes brightness depend on how many channels are lit,
 * which ruins the amber mix.
 *
 * The resistor values are intentionally unequal. Red has a forward
 * voltage near 2.0 V while green and blue sit near 3.0-3.2 V, so
 * from a 3.3 V GPIO the red channel draws far more current at the
 * same resistance. If amber still reads as orange-red on the bench,
 * increase the red resistor to 470R or 680R.
 *
 * NEVER connect the LED without resistors. Red in particular would
 * pull 30-40 mA, at or beyond the ESP32 pin's 40 mA absolute max.
 *
 * COMMON ANODE LED: set RGB_COMMON_ANODE to true below and wire the
 * long leg to 3.3 V instead of GND. Everything else is identical.
 * ───────────────────────────────────────────────────────────────
 *
 * ─── POWER WARNING (UNCHANGED, STILL APPLIES) ─────────────────
 * The SIM800L draws up to 2 A in bursts when it transmits, at
 * 3.7-4.2 V. It CANNOT be powered from the ESP32 3.3 V pin.
 *   • Power SIM800L from a separate 4 V source rated >= 2 A
 *   • Add a 1000 uF capacitor across SIM800L VCC/GND
 *   • Tie ALL grounds together (ESP32 GND <-> SIM800L GND <-> supply GND)
 * ───────────────────────────────────────────────────────────────
 *
 * GSM WIRING (UART lines cross over — TX always meets RX):
 *   SIM800L VCC  ->  4.0 V external supply (NOT the ESP32!)
 *   SIM800L GND  ->  common ground (shared with ESP32 GND)
 *   ESP32 GPIO 19 (TXD)  ------>  SIM800L RXD
 *   ESP32 GPIO 18 (RXD)  <------  SIM800L TXD
 *   Antenna attached before powering on.
 */

#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>     // NVS-backed boot counter for unique event ids
#include <time.h>            // NTP clock, so detected_at is a real timestamp
#include <math.h>

// ═══════════════════════════════════════════════════════════════
// GPIO CONFIGURATION
// ═══════════════════════════════════════════════════════════════

// ─── RGB LED — direct to ESP32 on free pins ───────────────────
// These three pins are NOT used by anything else in this sketch.
// The old LED pins (25, 26, 33) are deliberately left out entirely.
#define RGB_R 4              // GPIO 4:  RGB red   channel (via 330R)
#define RGB_G 27             // GPIO 27: RGB green channel (via 100R)
#define RGB_B 23             // GPIO 23: RGB blue  channel (via 100R)

// Spare free pin if one of the above turns out to be occupied: GPIO 13

// false = common cathode (long leg to GND)   <-- default
// true  = common anode   (long leg to 3.3V)
#define RGB_COMMON_ANODE false

#define BUZZER_PIN 32        // GPIO 32: Audio alert buzzer

// I2C for MPU6050
#define I2C_SDA 21           // GPIO 21: MPU6050 SDA
#define I2C_SCL 22           // GPIO 22: MPU6050 SCL

// GPS (NEO-6M) on Hardware UART2
#define GPS_RX 16            // GPIO 16: GPS TX (data FROM GPS)
#define GPS_TX 17            // GPIO 17: GPS RX (data TO GPS)

// GSM (SIM800L) on Hardware UART1
// Names are from the ESP32's point of view (UART pins are crossed):
//   GSM_RX = the ESP32's RX pin  <- wire to the modem's TXD
//   GSM_TX = the ESP32's TX pin  -> wire to the modem's RXD
#define GSM_RX 18            // GPIO 18: ESP32 RXD  <- SIM800L TXD
#define GSM_TX 19            // GPIO 19: ESP32 TXD  -> SIM800L RXD
#define GSM_BAUD 9600

// ═══════════════════════════════════════════════════════════════
// WIFI CONFIGURATION
// ═══════════════════════════════════════════════════════════════

const char* WIFI_SSID = "BERNARDINE 7683";
const char* WIFI_PASSWORD = "12345678";

// ═══════════════════════════════════════════════════════════════
// BACKEND CONFIGURATION
// ═══════════════════════════════════════════════════════════════
//
// The device no longer calls the ML API directly. It reports to the Sentinel
// backend, which classifies, stores the raw window, and pushes the incident to
// the dispatch console. The backend is the only component that talks to the
// model, and it is the system of record — a classification that lived only in
// this serial log used to vanish when the buffer scrolled.
//
// Swap BACKEND_URL for http://<laptop-LAN-IP>:8080 to run against a local
// backend; the scheme decides whether TLS is used.

const char* BACKEND_URL = "https://sentinel-backend-production-c650.up.railway.app";

// A DEVICE-scoped key, deliberately NOT the responder key used by the
// dashboard. Firmware flash can be read back over USB, so what is stored here
// must only be able to report incidents — never dispatch or resolve them.
// Register it on the backend as:  ROLE_KEYS={"<this value>":"device"}
const char* API_KEY   = "dev_mgxPoy9qrM4nZOVlvOy7aoVicMpvbGmq";

const char* DEVICE_ID = "ESP32_ACC_001";
const char* FIRMWARE_VERSION = "3.1.0-rcdemo";

const uint16_t HTTP_TIMEOUT_MS   = 15000;
const uint8_t  MAX_POST_ATTEMPTS = 3;
const uint16_t RETRY_BACKOFF_MS[] = {1000, 3000, 6000};
#define HEARTBEAT_INTERVAL_MS 30000

// ═══════════════════════════════════════════════════════════════
// GSM / EMERGENCY CONTACT CONFIGURATION
// ═══════════════════════════════════════════════════════════════

/*
 * GSM_OWN_NUMBER is the SIM sitting INSIDE the module (the sender).
 * It is not messaged — it is recorded here for reference and can be
 * quoted in the SMS body so recipients know which unit is reporting.
 *
 * EMERGENCY_CONTACTS are the recipients alerted on a crash. Every
 * number must be in full international format (+233...), because the
 * modem is given no default country code.
 */
const char* GSM_OWN_NUMBER = "+233537562836";   // SIM in the module

const char* EMERGENCY_CONTACTS[] = {
  "+233594282165",     // [1] primary emergency contact
  "+233534898588",     // [2] secondary emergency contact
};
const uint8_t NUM_EMERGENCY_CONTACTS =
    sizeof(EMERGENCY_CONTACTS) / sizeof(EMERGENCY_CONTACTS[0]);

// Send SMS for these severities (NORMAL never triggers an SMS)
#define SMS_ON_MODERATE true
#define SMS_ON_SEVERE   true

// Place an emergency voice call on SEVERE (in addition to SMS)
#define CALL_ON_SEVERE  true
#define CALL_DURATION_MS 25000    // ring for 25 s then hang up

#define GSM_AT_TIMEOUT_MS   5000
#define GSM_SMS_TIMEOUT_MS  20000
#define GSM_BOOT_WAIT_MS    3000

// ═══════════════════════════════════════════════════════════════
// IMU HARDWARE
// ═══════════════════════════════════════════════════════════════

#define MPU6050_ADDR 0x68
#define SENSOR_INTERVAL_MS 10
#define PRINT_INTERVAL_MS 100

// ═══════════════════════════════════════════════════════════════
// CRASH DETECTION PARAMETERS
// ═══════════════════════════════════════════════════════════════

#define BUFFER_SIZE 500           // 500 samples = 5 seconds @ 100Hz
#define ACCEL_THRESHOLD_G 2.0f    // 2.0g acceleration magnitude
#define JERK_THRESHOLD_G_PER_SEC 5.0f  // 5.0g/s jerk rate
// Keep sampling this many samples AFTER the trigger before sending, so the
// 500-sample window is CENTRED on the impact (250 before + 250 after).
// Sending at the trigger instant leaves only ~10 ms of the crash pulse at the
// window edge; the API needs the whole 40-250 ms pulse and returns Normal.
#define POST_TRIGGER_SAMPLES 250

#define RAW_ACCEL_TO_G (1.0f / 2048.0f)   // MPU6050 ±16g range
// ±500 dps: 2.3% of verified crashes exceed 250 dps (0.6% exceed 500).
#define RAW_GYRO_TO_DPS (1.0f / 65.5f)     // MPU6050 ±500 dps range

// ─── THRESHOLD PROFILE ────────────────────────────────────────
// RC_DEMO_PROFILE ON  = RC-car demonstration (this build).
// Comment it out for a real vehicle (full-scale VZCrash thresholds).
// The backend must match: set SIGNATURE_PROFILE=rc_demo on the backend,
// otherwise the ACK comes back under full-scale rules and the serial log
// prints a PROFILE MISMATCH line.
#define RC_DEMO_PROFILE

// Offline fallback classification (used only when the backend is
// unreachable). It works on the RAW 100 Hz magnitude, whereas the backend
// gates the 20 Hz-filtered signal, so the RC numbers here are the raw-signal
// equivalents of the backend's rc_demo profile (filtered 2.0 g floor ~= a
// 4.5 g raw single-sample spike; filtered 0.10 g·s ~= 0.12 g·s raw).
#ifdef RC_DEMO_PROFILE
#define PROFILE_NAME        "rc_demo"
#define PROFILE_LABEL_SOURCE "rc_demo_threshold"
#define SIG_PEAK_MIN_G      4.5f     // raw; ignores taps / put-downs
#define SIG_PEAK_MAX_G     28.0f     // no ceiling (±16 g per axis -> 27.7 g resultant)
#define SIG_DUR_MIN_MS        10     // one 100 Hz sample
#define SIG_DUR_MAX_MS       250
#define SIG_SEVERE_IMPULSE  0.12f    // g·s raw, full-throttle / sensor-clipping hit
#else
#define PROFILE_NAME        "full_scale"
#define PROFILE_LABEL_SOURCE ""
#define SIG_PEAK_MIN_G      2.0f
#define SIG_PEAK_MAX_G      7.0f
#define SIG_DUR_MIN_MS        40
#define SIG_DUR_MAX_MS       250
#define SIG_SEVERE_IMPULSE  0.959f   // g·s, ≈ Δv 34 km/h
#endif

// ═══════════════════════════════════════════════════════════════
// SEVERITY LEVELS
// ═══════════════════════════════════════════════════════════════

enum SeverityLevel {
  SEVERITY_NORMAL = 0,      // Safe
  SEVERITY_MODERATE = 1,    // Warning
  SEVERITY_SEVERE = 2       // Emergency
};

// ═══════════════════════════════════════════════════════════════
// DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════

struct IMUSample {
  float ax_g;      // Accelerometer X in g-units
  float ay_g;      // Accelerometer Y in g-units
  float az_g;      // Accelerometer Z in g-units
  float gx_dps;    // Gyroscope X in degrees/sec
  float gy_dps;    // Gyroscope Y in degrees/sec
  float gz_dps;    // Gyroscope Z in degrees/sec
  float accel_mag; // Magnitude for crash detection
  uint32_t timestamp_ms;
};

struct GPSData {
  bool valid;
  bool has_lock;
  float latitude;      // DECIMAL DEGREES (signed, S/W negative)
  float longitude;     // DECIMAL DEGREES (signed, S/W negative)
  float altitude;
  int satellites;
  unsigned long last_fix_time;
};

struct CrashResult {
  bool detected;
  int severity_class;
  const char* severity_name;
  float confidence;
  float peak_magnitude_g;
};

// GSM module state
struct GSMData {
  bool module_found;      // responded to AT
  bool sim_ready;         // AT+CPIN? == READY
  bool network_reg;       // AT+CREG? registered
  int  signal_quality;    // AT+CSQ (0-31, >=10 usable, 99 = unknown)
  uint32_t sms_sent;
  uint32_t sms_failed;
  unsigned long last_check;
};

IMUSample imu_buffer[BUFFER_SIZE];
uint16_t buffer_index = 0;
bool buffer_full = false;

GPSData gps_data = {false, false, 0.0, 0.0, 0.0, 0, 0};
GSMData gsm_data = {false, false, false, 0, 0, 0, 0};

// ═══════════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ═══════════════════════════════════════════════════════════════

MPU6050 mpu(MPU6050_ADDR);
bool mpu_ready = false;
bool wifi_ready = false;
bool gps_ready = false;
bool gsm_ready = false;

int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
float accel_x_g = 0, accel_y_g = 0, accel_z_g = 0;
float prev_accel_mag_g = 0;

unsigned long last_sensor_read = 0;
unsigned long last_print = 0;
unsigned long last_crash_detection = 0;
unsigned long last_gsm_health = 0;
uint32_t sample_count = 0;
uint32_t crash_count = 0;

bool crash_detected_flag = false;
uint16_t post_trigger_remaining = 0;
bool api_request_in_progress = false;

// What the device itself measured at the moment of the trigger. Sent alongside
// the window so the dashboard can show the device's own reading next to the
// server's filtered figure.
float    trigger_peak_g  = 0.0f;
float    trigger_jerk_gs = 0.0f;
uint32_t trigger_millis  = 0;

Preferences prefs;
uint32_t boot_count = 0;

// One HTTP request at a time. The heartbeat task takes this with zero timeout
// and skips its turn rather than delaying a crash upload.
SemaphoreHandle_t http_mutex = nullptr;
CrashResult last_crash_result = {false, 0, "None", 0.0, 0.0};

// Forward declarations
bool  sendEventToBackend();
void  heartbeatTask(void* param);
bool  gsmInit();
String gsmSendAT(const char* cmd, uint16_t timeout_ms = GSM_AT_TIMEOUT_MS);
bool  gsmExpect(const char* cmd, const char* expect, uint16_t timeout_ms = GSM_AT_TIMEOUT_MS);
bool  gsmCheckHealth(bool verbose = false);
bool  sendEmergencySMS(int severity_class, float confidence, float peak_g);
bool  gsmSendSMSTo(const char* number, const String& message);
void  gsmPlaceEmergencyCall(const char* number);
String buildAlertMessage(int severity_class, float confidence, float peak_g);
void  classifyLocally(CrashResult& result);

// ═══════════════════════════════════════════════════════════════
// RGB LED CONTROL LAYER
// ═══════════════════════════════════════════════════════════════

/*
 * Single point of control for the RGB LED. Every LED write in the
 * whole sketch goes through here, so a common-anode part only needs
 * the RGB_COMMON_ANODE flag flipped — nothing else changes.
 *
 * Plain digitalWrite is used rather than PWM on purpose. On the ESP32
 * the Arduino tone() function is built on the LEDC peripheral, the
 * same peripheral ledcWrite() uses. Driving the LED with PWM risks a
 * channel collision with the buzzer, and a buzzer that stops working
 * mid-demo is a worse outcome than a slightly imperfect amber. Colour
 * balance is handled with resistor values instead (see header).
 */
inline void rgbWrite(bool r, bool g, bool b) {
  if (RGB_COMMON_ANODE) {
    digitalWrite(RGB_R, r ? LOW : HIGH);
    digitalWrite(RGB_G, g ? LOW : HIGH);
    digitalWrite(RGB_B, b ? LOW : HIGH);
  } else {
    digitalWrite(RGB_R, r ? HIGH : LOW);
    digitalWrite(RGB_G, g ? HIGH : LOW);
    digitalWrite(RGB_B, b ? HIGH : LOW);
  }
}

// Named colours — these replace the old per-LED digitalWrite calls
inline void rgbOff()    { rgbWrite(false, false, false); }
inline void rgbRed()    { rgbWrite(true,  false, false); }  // SEVERE
inline void rgbGreen()  { rgbWrite(false, true,  false); }  // NORMAL
inline void rgbBlue()   { rgbWrite(false, false, true ); }  // WiFi / info
inline void rgbAmber()  { rgbWrite(true,  true,  false); }  // MODERATE (was yellow)
inline void rgbWhite()  { rgbWrite(true,  true,  true ); }  // crash trigger flash

// ═══════════════════════════════════════════════════════════════
// MOUNT ORIENTATION CHECK
// ═══════════════════════════════════════════════════════════════
// The model uses per-axis features (ax/ay/az) learned from data with gravity
// on +Z (training median az = 1.0 g). A board mounted on its side puts gravity
// on X or Y and silently skews every prediction. Checked at boot, vehicle still.

void checkMountOrientation() {
  const int N = 100;
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < N; i++) {
    mpu.getAcceleration(&accelX, &accelY, &accelZ);
    sx += accelX * RAW_ACCEL_TO_G;
    sy += accelY * RAW_ACCEL_TO_G;
    sz += accelZ * RAW_ACCEL_TO_G;
    delay(SENSOR_INTERVAL_MS);
  }
  sx /= N; sy /= N; sz /= N;
  Serial.printf("[MOUNT] At rest: ax=%.2fg ay=%.2fg az=%.2fg\n", sx, sy, sz);
  if (sz < 0.8f || fabs(sx) > 0.35f || fabs(sy) > 0.35f) {
    Serial.println(F("[MOUNT] WARNING: gravity is not on +Z. Mount the MPU6050 flat, Z axis up,"));
    Serial.println(F("[MOUNT]          or severity predictions will be unreliable."));
    for (int i = 0; i < 5; i++) {
      rgbAmber();
      delay(150);
      rgbOff();
      delay(150);
    }
  } else {
    Serial.println(F("[MOUNT] OK - orientation correct (Z axis up)"));
  }
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize RGB LED channels + buzzer.
  // GPIO 25, 26 and 33 are intentionally NOT configured here — the
  // old LEDs stay soldered to the perfboard but are never driven.
  pinMode(RGB_R, OUTPUT);
  pinMode(RGB_G, OUTPUT);
  pinMode(RGB_B, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Start with the LED off
  rgbOff();
  digitalWrite(BUZZER_PIN, LOW);

  printBanner();

  // ─────────────────────────────────────────────────────────────
  // RGB self-test: confirms all three channels and the common leg
  // are wired correctly before anything else runs. If a colour is
  // missing here, it is a wiring fault, not a logic fault.
  // ─────────────────────────────────────────────────────────────
  Serial.println(F("[RGB] Self-test: RED -> GREEN -> BLUE -> AMBER -> WHITE"));
  rgbRed();   delay(400);
  rgbGreen(); delay(400);
  rgbBlue();  delay(400);
  rgbAmber(); delay(400);
  rgbWhite(); delay(400);
  rgbOff();
  Serial.println(F("[RGB] Self-test complete"));

  // ─────────────────────────────────────────────────────────────
  // Initialize I2C & MPU6050
  // ─────────────────────────────────────────────────────────────
  Serial.println(F("[I2C] Initializing..."));
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  delay(100);

  Serial.println(F("[MPU6050] Initializing sensor..."));
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);  // measure up to ±16g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);   // must match RAW_GYRO_TO_DPS

  if (!mpu.testConnection()) {
    Serial.println(F("[ERROR] MPU6050 not found!"));
    mpu_ready = false;
    for (int i = 0; i < 3; i++) {
      rgbRed();
      delay(200);
      rgbOff();
      delay(200);
    }
  } else {
    Serial.println(F("[MPU6050] OK - Connected! (Accel + Gyro)"));
    mpu_ready = true;
    checkMountOrientation();
    rgbGreen();
    delay(500);
    rgbOff();
  }

  // ─────────────────────────────────────────────────────────────
  // Initialize GPS (Hardware UART2)
  // ─────────────────────────────────────────────────────────────
  Serial.println(F("[GPS] Initializing NEO-6M on Serial2..."));
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(500);
  Serial.println(F("[GPS] OK - Initialized on GPIO 16/17 (9600 baud)"));
  Serial.println(F("[GPS] Waiting for satellite lock (go outdoors)..."));
  gps_ready = true;

  // ─────────────────────────────────────────────────────────────
  // Initialize GSM (Hardware UART1)
  // ─────────────────────────────────────────────────────────────
  Serial.println();
  Serial.println(F("[GSM] Initializing SIM800L on Serial1..."));
  Serial1.begin(GSM_BAUD, SERIAL_8N1, GSM_RX, GSM_TX);
  delay(GSM_BOOT_WAIT_MS);          // modem needs time after power-up

  gsm_ready = gsmInit();

  if (gsm_ready) {
    Serial.println(F("[GSM] OK - Ready to send emergency SMS"));
    Serial.print(F("[GSM] Signal quality (CSQ): "));
    Serial.print(gsm_data.signal_quality);
    Serial.println(gsm_data.signal_quality >= 10 ? F(" (good)") : F(" (WEAK - check antenna)"));
    Serial.print(F("[GSM] Emergency contacts configured: "));
    Serial.println(NUM_EMERGENCY_CONTACTS);
    // Amber double-blink = GSM up
    for (int i = 0; i < 2; i++) {
      rgbAmber(); delay(120);
      rgbOff();   delay(120);
    }
  } else {
    Serial.println(F("[GSM] FAILED - SMS alerts will be unavailable"));
    Serial.println(F("[GSM] Check: separate 4V supply, common ground, antenna, SIM inserted"));
  }

  // ─────────────────────────────────────────────────────────────
  // Initialize WiFi
  // ─────────────────────────────────────────────────────────────
  Serial.println();
  Serial.print(F("[WiFi] Connecting to: "));
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  // Boot counter makes event ids unique across resets, so a device that
  // reboots mid-shift cannot reuse an id the server has already stored.
  prefs.begin("sentinel", false);
  boot_count = prefs.getUInt("boot", 0) + 1;
  prefs.putUInt("boot", boot_count);
  prefs.end();
  Serial.print(F("[NVS] Boot count: "));
  Serial.println(boot_count);

  http_mutex = xSemaphoreCreateMutex();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
    delay(500);
    Serial.print(F("."));
    wifi_attempts++;
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("[WiFi] OK - Connected!"));
    Serial.print(F("[WiFi] IP: "));
    Serial.println(WiFi.localIP());
    wifi_ready = true;

    // The ESP32 has no RTC, so detected_at comes from NTP. Without it the
    // backend can only use its own receive time and the dashboard's latency
    // breakdown loses its first leg.
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    Serial.print(F("[NTP] Syncing"));
    for (int i = 0; i < 20 && time(nullptr) < 1600000000; i++) {
      delay(250);
      Serial.print(F("."));
    }
    Serial.println(time(nullptr) >= 1600000000 ? F(" OK - UTC set")
                                               : F(" FAILED - detected_at omitted"));

    // Heartbeat runs on core 0 at low priority so it can never preempt the
    // 100 Hz sampling loop, which owns core 1.
    xTaskCreatePinnedToCore(heartbeatTask, "heartbeat", 8192, nullptr, 1, nullptr, 0);
    Serial.println(F("[TASK] Heartbeat started on core 0"));

    // Blue flash = WiFi up. Blue is free now that yellow is gone,
    // so WiFi and GSM are distinguishable at boot.
    rgbBlue();
    delay(300);
    rgbOff();
  } else {
    Serial.println(F("[WiFi] FAILED - will rely on GSM fallback"));
    wifi_ready = false;
  }

  Serial.println();
  Serial.println(F("══════════════════════════════════════════════════════════════════════════════════════════"));
  Serial.println(F("Time(ms)  | Accel(g) | Jerk(g/s) | WiFi | MPU | GPS Sats | GSM | Status"));
  Serial.println(F("══════════════════════════════════════════════════════════════════════════════════════════"));

  last_sensor_read = millis();
  last_print = millis();
  last_gsm_health = millis();

  // Normal-driving indication: green stays on once initialization is complete.
  rgbGreen();
}

// ═══════════════════════════════════════════════════════════════
// GPS PARSING (NON-BLOCKING)
// ═══════════════════════════════════════════════════════════════

void updateGPS() {
  static String sentence = "";

  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '\n') {
      if (sentence.length() > 0 && sentence.startsWith("$GPGGA")) {
        parseGPGGA(sentence);
      }
      sentence = "";
    } else if (c != '\r') {
      sentence += c;
    }
  }
}

/*
 * Convert NMEA ddmm.mmmm / dddmm.mmmm to decimal degrees.
 *
 * NMEA reports latitude as 4807.038 meaning 48 deg 07.038 min, NOT
 * 4807.038 degrees. Feeding the raw value into a maps link puts the
 * pin in the wrong hemisphere/continent. This matters because the
 * coordinates are transmitted by SMS.
 */
float nmeaToDecimalDegrees(String field, String hemisphere) {
  if (field.length() < 3) return 0.0;

  float raw = field.toFloat();               // e.g. 4807.038
  int degrees = (int)(raw / 100.0);          // 48
  float minutes = raw - (degrees * 100.0);   // 7.038
  float decimal = degrees + (minutes / 60.0);

  hemisphere.trim();
  if (hemisphere == "S" || hemisphere == "W") decimal = -decimal;

  return decimal;
}

void parseGPGGA(String gga_sentence) {
  // $GPGGA,time,lat,N/S,lon,E/W,fix_quality,num_sats,hdop,altitude,M,...
  int commas[15];
  int comma_count = 0;

  for (int i = 0; i < gga_sentence.length() && comma_count < 15; i++) {
    if (gga_sentence[i] == ',') {
      commas[comma_count++] = i;
    }
  }

  if (comma_count < 9) return;

  int fix_quality = gga_sentence.substring(commas[5] + 1, commas[6]).toInt();

  if (fix_quality > 0) {
    gps_data.valid = true;
    gps_data.has_lock = true;

    // proper decimal-degree conversion with hemisphere sign
    String lat_field = gga_sentence.substring(commas[1] + 1, commas[2]);
    String lat_hemi  = gga_sentence.substring(commas[2] + 1, commas[3]);
    String lon_field = gga_sentence.substring(commas[3] + 1, commas[4]);
    String lon_hemi  = gga_sentence.substring(commas[4] + 1, commas[5]);

    if (lat_field.length() > 0) gps_data.latitude  = nmeaToDecimalDegrees(lat_field, lat_hemi);
    if (lon_field.length() > 0) gps_data.longitude = nmeaToDecimalDegrees(lon_field, lon_hemi);

    gps_data.satellites = gga_sentence.substring(commas[6] + 1, commas[7]).toInt();

    if (comma_count > 8) {
      gps_data.altitude = gga_sentence.substring(commas[8] + 1, commas[9]).toFloat();
    }

    gps_data.last_fix_time = millis();

    static bool first_lock = false;
    if (!first_lock && gps_data.satellites >= 4) {
      first_lock = true;
      Serial.print(F("[GPS] FIRST FIX ACQUIRED! "));
      Serial.print(gps_data.latitude, 6);
      Serial.print(F(", "));
      Serial.println(gps_data.longitude, 6);
    }
  }
}

// ═══════════════════════════════════════════════════════════════
// GSM LAYER
// ═══════════════════════════════════════════════════════════════

/*
 * Send a raw AT command and collect the modem's reply until either
 * the timeout expires or the reply terminates. Returns everything
 * received so the caller can inspect it.
 */
String gsmSendAT(const char* cmd, uint16_t timeout_ms) {
  // Flush anything stale first
  while (Serial1.available()) Serial1.read();

  Serial1.println(cmd);

  String response = "";
  unsigned long start = millis();

  while (millis() - start < timeout_ms) {
    while (Serial1.available()) {
      response += (char)Serial1.read();
    }
    // Terminate early on a definitive result code
    if (response.indexOf("OK") != -1 ||
        response.indexOf("ERROR") != -1 ||
        response.indexOf(">") != -1) {
      break;
    }
    delay(5);
  }
  return response;
}

/* Convenience: run a command and test whether the reply contains `expect`. */
bool gsmExpect(const char* cmd, const char* expect, uint16_t timeout_ms) {
  String r = gsmSendAT(cmd, timeout_ms);
  return (r.indexOf(expect) != -1);
}

/*
 * Bring the modem up: verify it responds, SIM is unlocked, it has
 * registered on the network, and put it in text-mode SMS.
 */
bool gsmInit() {
  // 1) Is the modem alive? Try a few times — it may still be booting.
  gsm_data.module_found = false;
  for (int attempt = 0; attempt < 5; attempt++) {
    if (gsmExpect("AT", "OK", 2000)) {
      gsm_data.module_found = true;
      break;
    }
    Serial.println(F("[GSM] No response to AT, retrying..."));
    delay(1000);
  }
  if (!gsm_data.module_found) {
    Serial.println(F("[GSM] ERROR: modem not responding on GPIO 18/19"));
    return false;
  }
  Serial.println(F("[GSM] Modem responding"));

  gsmSendAT("ATE0", 2000);              // echo off (cleaner parsing)

  // 2) SIM present and unlocked?
  String cpin = gsmSendAT("AT+CPIN?", 5000);
  gsm_data.sim_ready = (cpin.indexOf("READY") != -1);
  if (!gsm_data.sim_ready) {
    Serial.println(F("[GSM] ERROR: SIM not ready (missing, or PIN-locked)"));
    return false;
  }
  Serial.println(F("[GSM] SIM ready"));

  // 3) Registered on the network? Allow time — this can take a while.
  gsm_data.network_reg = false;
  for (int attempt = 0; attempt < 15; attempt++) {
    String creg = gsmSendAT("AT+CREG?", 3000);
    // ",1" = registered home, ",5" = registered roaming
    if (creg.indexOf(",1") != -1 || creg.indexOf(",5") != -1) {
      gsm_data.network_reg = true;
      break;
    }
    Serial.print(F("."));
    delay(1000);
  }
  Serial.println();
  if (!gsm_data.network_reg) {
    Serial.println(F("[GSM] ERROR: could not register on network"));
    Serial.println(F("[GSM] Check antenna, signal, and that the SIM has credit"));
    return false;
  }
  Serial.println(F("[GSM] Registered on network"));

  // 4) Signal quality
  String csq = gsmSendAT("AT+CSQ", 3000);
  int idx = csq.indexOf("+CSQ:");
  if (idx != -1) {
    gsm_data.signal_quality = csq.substring(idx + 6, csq.indexOf(',', idx)).toInt();
  }

  // 5) SMS text mode + GSM character set
  gsmSendAT("AT+CMGF=1", 3000);         // text mode (not PDU)
  gsmSendAT("AT+CSCS=\"GSM\"", 3000);   // standard character set
  gsmSendAT("AT+CNMI=2,1,0,0,0", 3000); // new-SMS indications

  gsm_data.last_check = millis();
  return true;
}

/* Periodic lightweight health check (does not block the sensor loop). */
bool gsmCheckHealth(bool verbose) {
  if (!gsm_data.module_found) return false;

  String csq = gsmSendAT("AT+CSQ", 1500);
  int idx = csq.indexOf("+CSQ:");
  if (idx != -1) {
    gsm_data.signal_quality = csq.substring(idx + 6, csq.indexOf(',', idx)).toInt();
  }

  String creg = gsmSendAT("AT+CREG?", 1500);
  gsm_data.network_reg = (creg.indexOf(",1") != -1 || creg.indexOf(",5") != -1);

  if (verbose) {
    Serial.print(F("[GSM] CSQ="));
    Serial.print(gsm_data.signal_quality);
    Serial.print(F("  registered="));
    Serial.println(gsm_data.network_reg ? F("yes") : F("no"));
  }
  return gsm_data.network_reg;
}

/*
 * Compose the emergency message. Kept compact so it fits comfortably
 * in one or two SMS segments, and leads with the most important
 * information in case only the notification preview is seen.
 */
String buildAlertMessage(int severity_class, float confidence, float peak_g) {
  String msg = "";

  if (severity_class == SEVERITY_SEVERE) {
    msg += "*** SEVERE ACCIDENT ***\n";
  } else if (severity_class == SEVERITY_MODERATE) {
    msg += "** ACCIDENT DETECTED **\n";
  } else {
    msg += "Impact event logged\n";
  }

  msg += "Sentinel unit alert\n";
  msg += "Peak: " + String(peak_g, 1) + "g\n";
  msg += "Confidence: " + String((int)(confidence * 100)) + "%\n";

  if (gps_data.has_lock && gps_data.valid) {
    msg += "Loc: " + String(gps_data.latitude, 6) + "," + String(gps_data.longitude, 6) + "\n";
    msg += "https://maps.google.com/?q=";
    msg += String(gps_data.latitude, 6);
    msg += ",";
    msg += String(gps_data.longitude, 6);
    msg += "\n";
    msg += "Sats: " + String(gps_data.satellites) + "\n";
  } else {
    msg += "Loc: GPS FIX UNAVAILABLE\n";
  }

  msg += "Uptime: " + String(millis() / 1000) + "s";
  return msg;
}

/*
 * Send one SMS. Text mode: issue AT+CMGS="number", wait for the ">"
 * prompt, write the body, then terminate with Ctrl+Z (0x1A).
 */
bool gsmSendSMSTo(const char* number, const String& message) {
  if (!gsm_data.module_found) return false;

  Serial.print(F("[GSM] Sending SMS to "));
  Serial.println(number);

  // Flush stale data
  while (Serial1.available()) Serial1.read();

  // Ensure text mode (cheap, and guards against a modem reset)
  Serial1.println("AT+CMGF=1");
  delay(300);
  while (Serial1.available()) Serial1.read();

  // Address
  Serial1.print("AT+CMGS=\"");
  Serial1.print(number);
  Serial1.println("\"");

  // Wait for the '>' prompt
  unsigned long start = millis();
  bool prompt = false;
  String pre = "";
  while (millis() - start < 5000) {
    while (Serial1.available()) {
      char c = (char)Serial1.read();
      pre += c;
      if (c == '>') { prompt = true; break; }
    }
    if (prompt) break;
    delay(5);
  }

  if (!prompt) {
    Serial.println(F("[GSM] ERROR: no '>' prompt from modem"));
    Serial1.write(27);            // ESC — abort the command
    delay(200);
    gsm_data.sms_failed++;
    return false;
  }

  // Body then Ctrl+Z
  Serial1.print(message);
  delay(100);
  Serial1.write(26);              // Ctrl+Z terminates the message

  // Wait for +CMGS confirmation
  String response = "";
  start = millis();
  while (millis() - start < GSM_SMS_TIMEOUT_MS) {
    while (Serial1.available()) {
      response += (char)Serial1.read();
    }
    if (response.indexOf("+CMGS") != -1 || response.indexOf("ERROR") != -1) break;
    delay(10);
  }

  if (response.indexOf("+CMGS") != -1) {
    Serial.println(F("[GSM] SMS SENT OK"));
    gsm_data.sms_sent++;
    return true;
  }

  Serial.println(F("[GSM] ERROR: SMS send failed"));
  Serial.print(F("[GSM] Modem said: "));
  Serial.println(response);
  gsm_data.sms_failed++;
  return false;
}

/* Broadcast the alert to every configured emergency contact. */
bool sendEmergencySMS(int severity_class, float confidence, float peak_g) {
  if (!gsm_data.module_found) {
    Serial.println(F("[GSM] Cannot send SMS - modem unavailable"));
    return false;
  }

  String message = buildAlertMessage(severity_class, confidence, peak_g);

  Serial.println(F("[GSM] ---- Emergency message ----"));
  Serial.println(message);
  Serial.println(F("[GSM] ---------------------------"));

  bool any_sent = false;
  for (uint8_t i = 0; i < NUM_EMERGENCY_CONTACTS; i++) {
    // One retry per contact — SMS is the last line of defence
    if (gsmSendSMSTo(EMERGENCY_CONTACTS[i], message)) {
      any_sent = true;
    } else {
      Serial.println(F("[GSM] Retrying once..."));
      delay(2000);
      if (gsmSendSMSTo(EMERGENCY_CONTACTS[i], message)) any_sent = true;
    }
    delay(1000);   // small gap between recipients
  }
  return any_sent;
}

/* Optional voice call for SEVERE events. */
void gsmPlaceEmergencyCall(const char* number) {
  if (!gsm_data.module_found) return;

  Serial.print(F("[GSM] Placing emergency call to "));
  Serial.println(number);

  String cmd = "ATD";
  cmd += number;
  cmd += ";";
  Serial1.println(cmd);

  unsigned long start = millis();
  while (millis() - start < CALL_DURATION_MS) {
    // Keep the red channel pulsing during the call
    rgbRed();
    delay(400);
    rgbOff();
    delay(400);
  }

  Serial1.println("ATH");        // hang up
  delay(500);
  rgbRed();                      // back to steady emergency state
  Serial.println(F("[GSM] Call ended"));
}

// ═══════════════════════════════════════════════════════════════
// OFFLINE FALLBACK CLASSIFICATION
// ═══════════════════════════════════════════════════════════════

/*
 * When the ML API is unreachable, classify locally from the buffer
 * so the SMS can still carry a meaningful severity.
 *
 * Uses the same signature logic as the deployed API: a crash is a
 * 2–7 g event lasting 40–250 ms. Sustained high-g is treated as an
 * aggressive manoeuvre, not a crash. Severity within the crash band
 * is graded by impulse (a delta-v proxy).
 */
void classifyLocally(CrashResult& result) {
  float peak = 0.0f;
  float impulse = 0.0f;         // g·s above the 1 g baseline
  int   samples_above = 0;
  int   longest_run = 0;
  int   current_run = 0;

  // Walk oldest-first, exactly like the payload sent to the API. Walking the
  // raw array order would split a pulse that spans the wrap point into two
  // shorter runs and could under-measure the duration.
  for (int i = 0; i < BUFFER_SIZE; i++) {
    float m = imu_buffer[(buffer_index + i) % BUFFER_SIZE].accel_mag;
    if (m > peak) peak = m;

    if (m >= SIG_PEAK_MIN_G) {
      samples_above++;
      current_run++;
      if (current_run > longest_run) longest_run = current_run;
      impulse += (m - 1.0f) * (SENSOR_INTERVAL_MS / 1000.0f);
    } else {
      current_run = 0;
    }
  }

  int longest_ms = longest_run * SENSOR_INTERVAL_MS;

  bool signature_match = (peak >= SIG_PEAK_MIN_G && peak < SIG_PEAK_MAX_G &&
                          longest_ms >= SIG_DUR_MIN_MS && longest_ms <= SIG_DUR_MAX_MS);

  result.detected = true;
  result.peak_magnitude_g = peak;
  result.confidence = 0.0f;      // locally derived, not a model probability

  if (!signature_match) {
    result.severity_class = SEVERITY_NORMAL;
    result.severity_name = "Normal";
  } else if (impulse >= SIG_SEVERE_IMPULSE) {
    result.severity_class = SEVERITY_SEVERE;
    result.severity_name = "Severe";
  } else {
    result.severity_class = SEVERITY_MODERATE;
    result.severity_name = "Moderate";
  }

  Serial.println(F("[LOCAL] ML API unreachable - classified on-device"));
  Serial.print(F("[LOCAL] profile=" PROFILE_NAME "  peak="));  Serial.print(peak, 2);
  Serial.print(F("g  longest_run="));     Serial.print(longest_ms);
  Serial.print(F("ms  impulse="));        Serial.print(impulse, 3);
  Serial.print(F("g.s  -> "));            Serial.println(result.severity_name);
}

// ═══════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // Check WiFi status
  if (WiFi.status() != WL_CONNECTED && wifi_ready) {
    wifi_ready = false;
  }

  // ─────────────────────────────────────────────────────────────
  // Update GPS (non-blocking)
  // ─────────────────────────────────────────────────────────────
  updateGPS();

  // ─────────────────────────────────────────────────────────────
  // Read IMU Sensors at 100Hz (every 10ms)
  // ─────────────────────────────────────────────────────────────
  if (now - last_sensor_read >= SENSOR_INTERVAL_MS) {
    if (mpu_ready) {
      mpu.getAcceleration(&accelX, &accelY, &accelZ);
      mpu.getRotation(&gyroX, &gyroY, &gyroZ);

      accel_x_g = accelX * RAW_ACCEL_TO_G;
      accel_y_g = accelY * RAW_ACCEL_TO_G;
      accel_z_g = accelZ * RAW_ACCEL_TO_G;

      float gyro_x_dps = gyroX * RAW_GYRO_TO_DPS;
      float gyro_y_dps = gyroY * RAW_GYRO_TO_DPS;
      float gyro_z_dps = gyroZ * RAW_GYRO_TO_DPS;

      float mag = sqrt(accel_x_g * accel_x_g +
                       accel_y_g * accel_y_g +
                       accel_z_g * accel_z_g);

      float jerk_g_per_sec = (mag - prev_accel_mag_g) / (SENSOR_INTERVAL_MS / 1000.0f);

      // ─────────────────────────────────────────────────────────
      // CRASH DETECTION: Dual Threshold
      // ─────────────────────────────────────────────────────────
      bool accel_threshold = (mag >= ACCEL_THRESHOLD_G);
      bool jerk_threshold = (fabs(jerk_g_per_sec) >= JERK_THRESHOLD_G_PER_SEC);

      if (accel_threshold && jerk_threshold && !crash_detected_flag) {
        if (now - last_crash_detection > 2000) {
          crash_detected_flag = true;
          last_crash_detection = now;

          // Snapshot what the device measured, before the post-roll runs.
          trigger_peak_g  = mag;
          trigger_jerk_gs = jerk_g_per_sec;
          trigger_millis  = now;

          rgbOff();

          Serial.print(F("[CRASH] Detected at "));
          Serial.print(now);
          Serial.print(F(" ms | Accel="));
          Serial.print(mag, 2);
          Serial.print(F("g, Jerk="));
          Serial.print(jerk_g_per_sec, 2);
          Serial.println(F("g/s"));

          // White = trigger fired, post-impact window being captured.
          // No delay() here: blocking would drop the samples right after the
          // impact, which are exactly the ones the API needs.
          rgbWhite();
          tone(BUZZER_PIN, 1500, 50);
          post_trigger_remaining = POST_TRIGGER_SAMPLES;
          Serial.println(F("[CRASH] Capturing 2.5 s post-impact data..."));
        }
      }

      // Store sample in circular buffer
      imu_buffer[buffer_index].ax_g = accel_x_g;
      imu_buffer[buffer_index].ay_g = accel_y_g;
      imu_buffer[buffer_index].az_g = accel_z_g;
      imu_buffer[buffer_index].gx_dps = gyro_x_dps;
      imu_buffer[buffer_index].gy_dps = gyro_y_dps;
      imu_buffer[buffer_index].gz_dps = gyro_z_dps;
      imu_buffer[buffer_index].accel_mag = mag;
      imu_buffer[buffer_index].timestamp_ms = now;

      buffer_index++;
      if (buffer_index >= BUFFER_SIZE) {
        buffer_index = 0;
        buffer_full = true;
      }

      if (crash_detected_flag && post_trigger_remaining > 0) {
        post_trigger_remaining--;
      }

      prev_accel_mag_g = mag;
      sample_count++;
    }
    last_sensor_read = now;
  }

  // ─────────────────────────────────────────────────────────────
  // Process crash once the buffer holds the full 5 s window.
  //
  // The WiFi requirement is deliberately NOT part of this condition.
  // The crash is always processed; processCrash() decides whether to
  // use the ML API or fall back to on-device classification, and the
  // SMS goes out either way.
  // ─────────────────────────────────────────────────────────────
  if (crash_detected_flag && post_trigger_remaining == 0 && buffer_full && !api_request_in_progress) {
    api_request_in_progress = true;
    Serial.println(F("[CRASH] Processing incident..."));
    processCrash();
    crash_detected_flag = false;
    api_request_in_progress = false;
  }

  // ─────────────────────────────────────────────────────────────
  // Periodic GSM health check (every 60 s, cheap)
  // ─────────────────────────────────────────────────────────────
  if (gsm_ready && (now - last_gsm_health >= 60000)) {
    gsmCheckHealth(false);
    last_gsm_health = now;
  }

  // ─────────────────────────────────────────────────────────────
  // Print status every 100ms
  // ─────────────────────────────────────────────────────────────
  if (now - last_print >= PRINT_INTERVAL_MS) {
    if (mpu_ready && buffer_full && sample_count > 1) {
      uint16_t curr_idx = (buffer_index - 1 + BUFFER_SIZE) % BUFFER_SIZE;
      uint16_t prev_idx = (buffer_index - 2 + BUFFER_SIZE) % BUFFER_SIZE;

      float time_delta = (imu_buffer[curr_idx].timestamp_ms -
                          imu_buffer[prev_idx].timestamp_ms) / 1000.0f;
      float jerk = 0;
      if (time_delta > 0) {
        jerk = (imu_buffer[curr_idx].accel_mag -
                imu_buffer[prev_idx].accel_mag) / time_delta;
      }

      Serial.print(now);
      Serial.print(F("  | "));
      Serial.print(imu_buffer[curr_idx].accel_mag, 2);
      Serial.print(F("g    | "));
      Serial.print(jerk, 2);
      Serial.print(F("    | "));
      Serial.print(wifi_ready ? F("OK") : F("--"));
      Serial.print(F("   | "));
      Serial.print(mpu_ready ? F("OK") : F("--"));
      Serial.print(F("  | "));
      Serial.print(gps_data.has_lock ? gps_data.satellites : 0);
      Serial.print(F("        | "));
      if (gsm_data.network_reg)      Serial.print(F("OK "));
      else if (gsm_data.module_found) Serial.print(F("NR "));   // no registration
      else                            Serial.print(F("-- "));
      Serial.print(F("| "));
      Serial.println(F("RUNNING"));
    }
    last_print = now;
  }
}

// ═══════════════════════════════════════════════════════════════
// CRASH PROCESSING
// ═══════════════════════════════════════════════════════════════

void processCrash() {
  bool api_success = false;

  // ─── Path 1: WiFi available -> ask the ML API ───────────────
  if (wifi_ready && WiFi.isConnected()) {
    Serial.println(F("[BACKEND] Uploading crash window..."));
    api_success = sendEventToBackend();
  } else {
    Serial.println(F("[BACKEND] Skipped - no WiFi connection"));
  }

  // ─── Path 2: API unavailable -> classify on-device ───────────
  if (!api_success) {
    classifyLocally(last_crash_result);
    crash_count++;
    displayCrashResults();
  }

  int severity = last_crash_result.severity_class;

  // ─── Local alerting (RGB + buzzer) ───────────────────────────
  triggerSeverityAlert(severity);

  // ─── GSM emergency notification ──────────────────────────────
  bool should_sms = (severity == SEVERITY_SEVERE   && SMS_ON_SEVERE) ||
                    (severity == SEVERITY_MODERATE && SMS_ON_MODERATE);

  if (should_sms) {
    if (gsm_data.module_found) {
      Serial.println(F("[GSM] Dispatching emergency SMS..."));
      bool sent = sendEmergencySMS(severity,
                                   last_crash_result.confidence,
                                   last_crash_result.peak_magnitude_g);

      if (sent) {
        // Two short confirmation chirps: contacts have been notified
        for (int i = 0; i < 2; i++) {
          tone(BUZZER_PIN, 2500, 80);
          delay(160);
          noTone(BUZZER_PIN);
        }
        Serial.println(F("[GSM] Emergency contacts notified"));
      } else {
        Serial.println(F("[GSM] WARNING: could not notify any contact"));
      }

      // Escalate to a voice call on SEVERE
      if (severity == SEVERITY_SEVERE && CALL_ON_SEVERE &&
          NUM_EMERGENCY_CONTACTS > 0) {
        gsmPlaceEmergencyCall(EMERGENCY_CONTACTS[0]);
      }

    } else {
      Serial.println(F("[GSM] WARNING: modem unavailable - no SMS sent"));
    }
  } else {
    Serial.println(F("[GSM] Normal severity - no SMS required"));
  }
}

// ═══════════════════════════════════════════════════════════════
// SEVERITY-BASED RGB & BUZZER ALERTS
// ═══════════════════════════════════════════════════════════════

void triggerSeverityAlert(int severity_class) {
  rgbOff();
  noTone(BUZZER_PIN);

  switch (severity_class) {
    case SEVERITY_NORMAL:
      Serial.println(F("SEVERITY: NORMAL - No alert needed"));
      rgbOff();
      delay(150);
      rgbGreen();
      delay(150);
      rgbOff();
      delay(150);
      tone(BUZZER_PIN, 1000, 100);
      delay(150);
      noTone(BUZZER_PIN);
      rgbGreen();
      break;

    case SEVERITY_MODERATE:
      Serial.println(F("SEVERITY: MODERATE - Warning alert"));
      for (int i = 0; i < 3; i++) {
        rgbAmber();
        tone(BUZZER_PIN, 1500, 150);
        delay(300);
        rgbOff();
        noTone(BUZZER_PIN);
        delay(200);
      }
      rgbAmber();
      break;

    case SEVERITY_SEVERE:
      Serial.println(F("SEVERITY: SEVERE - EMERGENCY ALERT!!!"));
      for (int i = 0; i < 5; i++) {
        rgbRed();
        tone(BUZZER_PIN, 2000, 100);
        delay(150);
        rgbOff();
        noTone(BUZZER_PIN);
        delay(150);
      }
      rgbRed();
      break;
  }
}

// ═══════════════════════════════════════════════════════════════
// ML API COMMUNICATION
// ═══════════════════════════════════════════════════════════════

/** Append a float without dragging printf's float formatting into the build. */
static void appendFloat(String& out, float v, uint8_t decimals) {
  char num[20];
  dtostrf(v, 0, decimals, num);
  out += num;
}

/** event_id = {DEVICE_ID}-{boot:04d}-{millis:08d}
 *
 *  The server is idempotent on this, which is what makes the retries below
 *  safe: one shake yields exactly one incident however many times we send it.
 */
static String makeEventId() {
  char buf[64];
  snprintf(buf, sizeof(buf), "%s-%04lu-%08lu",
           DEVICE_ID, (unsigned long)boot_count, (unsigned long)trigger_millis);
  return String(buf);
}

/** ISO-8601 UTC for the TRIGGER instant, not the upload instant.
 *
 *  Walks back from now by the post-roll just spent, so the dashboard's
 *  detected -> received figure measures the network rather than our own wait.
 *  Returns empty when NTP has not synced: an absent timestamp beats a wrong one.
 */
static String triggerTimestampIso() {
  time_t now_epoch = time(nullptr);
  if (now_epoch < 1600000000) return String();
  uint32_t ago_ms = millis() - trigger_millis;
  time_t trigger_epoch = now_epoch - (time_t)(ago_ms / 1000);
  struct tm tm_utc;
  gmtime_r(&trigger_epoch, &tm_utc);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
  return String(buf);
}

/** Build the /api/v1/events body straight into a heap String.
 *
 *  Deliberately NOT a StaticJsonDocument: 500x6 floats needs far more than the
 *  8 kB loop-task stack, and an oversized document on that stack has already
 *  overflowed this project once. reserve() takes one allocation up front rather
 *  than repeatedly reallocating while appending ~28 kB.
 */
static void buildEventBody(String& body, const String& event_id) {
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
  if (gps_data.valid && gps_data.has_lock) {
    body += F("\"lat\":");
    appendFloat(body, gps_data.latitude, 6);
    body += F(",\"lon\":");
    appendFloat(body, gps_data.longitude, 6);
    body += F(",\"valid\":true,\"satellites\":");
    body += String(gps_data.satellites);
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
  body += F("\"},\"window\":{\"fs_hz\":100,\"units_accel\":\"g\",\"units_gyro\":\"deg_s\"");

  // Oldest -> newest. With the post-roll complete this is exactly 250
  // pre-trigger + 250 post-trigger samples, so the impact sits mid-window.
  const char* names[6] = {"ax", "ay", "az", "gx", "gy", "gz"};
  for (uint8_t f = 0; f < 6; f++) {
    body += F(",\"");
    body += names[f];
    body += F("\":[");
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
      const IMUSample& smp = imu_buffer[(buffer_index + i) % BUFFER_SIZE];
      float v;
      switch (f) {
        case 0:  v = smp.ax_g;   break;
        case 1:  v = smp.ay_g;   break;
        case 2:  v = smp.az_g;   break;
        case 3:  v = smp.gx_dps; break;
        case 4:  v = smp.gy_dps; break;
        default: v = smp.gz_dps; break;
      }
      if (i) body += ',';
      appendFloat(body, v, 4);
    }
    body += ']';
  }
  body += F("}}");
}

/** Parse the backend's deliberately small, flat ACK. */
static void parseAck(const String& resp) {
  JsonDocument doc;
  if (deserializeJson(doc, resp)) {
    Serial.println(F("[BACKEND] Malformed ACK"));
    return;
  }
  crash_count++;
  last_crash_result.detected = true;

  if (doc["severity_class"].isNull()) {
    // Stored, but the model was unreachable. Treat as worst case locally so
    // the alert still fires; the backend will classify it on retry.
    last_crash_result.severity_class = SEVERITY_SEVERE;
    last_crash_result.severity_name  = "Pending";
    Serial.println(F("[BACKEND] Stored - classification pending"));
  } else {
    last_crash_result.severity_class = doc["severity_class"].as<int>();
    switch (last_crash_result.severity_class) {
      case SEVERITY_NORMAL:   last_crash_result.severity_name = "Normal";   break;
      case SEVERITY_MODERATE: last_crash_result.severity_name = "Moderate"; break;
      case SEVERITY_SEVERE:   last_crash_result.severity_name = "Severe";   break;
      default:                last_crash_result.severity_name = "Unknown";  break;
    }
  }
  last_crash_result.confidence       = doc["confidence"] | 0.0f;
  last_crash_result.peak_magnitude_g = doc["peak_g"] | trigger_peak_g;

  displayCrashResults();
  const char* source = doc["label_source"] | "unknown";
  Serial.print(F("[BACKEND] P(crash)="));
  Serial.print((float)(doc["p_crash"] | 0.0f), 3);
  Serial.print(F("  source="));
  Serial.print(source);
  Serial.println(F("  device_profile=" PROFILE_NAME));
  bool backend_rc = strcmp(source, "rc_demo_threshold") == 0;
  bool device_rc = strlen(PROFILE_LABEL_SOURCE) > 0;
  if (!doc["severity_class"].isNull() && backend_rc != device_rc) {
    Serial.println(F("[PROFILE] MISMATCH: device is " PROFILE_NAME
                     " but the backend classified under a different profile."));
    Serial.println(F("[PROFILE] Set SIGNATURE_PROFILE on the backend (rc_demo for the RC car)."));
  }
}

/** POST the impact-centred window to the backend, retrying transport faults. */
bool sendEventToBackend() {
  if (!WiFi.isConnected()) {
    Serial.println(F("[ERROR] WiFi not connected!"));
    return false;
  }

  String event_id = makeEventId();
  String body;
  buildEventBody(body, event_id);

  Serial.print(F("[BACKEND] Body "));
  Serial.print(body.length());
  Serial.print(F(" bytes | free heap "));
  Serial.println(ESP.getFreeHeap());

  String url = String(BACKEND_URL) + "/api/v1/events";

  for (uint8_t attempt = 1; attempt <= MAX_POST_ATTEMPTS; attempt++) {
    xSemaphoreTake(http_mutex, portMAX_DELAY);

    HTTPClient http;
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-API-Key", API_KEY);

    unsigned long t0 = millis();
    int code = http.POST(body);
    unsigned long elapsed = millis() - t0;

    Serial.print(F("[BACKEND] Attempt "));
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

    // 4xx is a contract violation; retrying cannot fix it.
    if (code >= 400 && code < 500) {
      Serial.print(F("[BACKEND] Rejected: "));
      Serial.println(http.getString().substring(0, 160));
      http.end();
      xSemaphoreGive(http_mutex);
      return false;
    }

    http.end();
    xSemaphoreGive(http_mutex);

    if (attempt < MAX_POST_ATTEMPTS) {
      Serial.print(F("[BACKEND] Retrying in "));
      Serial.print(RETRY_BACKOFF_MS[attempt - 1]);
      Serial.println(F(" ms (server is idempotent on event_id)"));
      delay(RETRY_BACKOFF_MS[attempt - 1]);
    }
  }

  Serial.println(F("[BACKEND] Upload failed after all attempts"));
  return false;
}

/** Liveness beacon. This is what puts the node on the dashboard as "online"
 *  before any crash has ever happened. */
void heartbeatTask(void* param) {
  (void)param;
  vTaskDelay(pdMS_TO_TICKS(5000));

  for (;;) {
    if (wifi_ready && WiFi.isConnected()) {
      // Never wait on the lock: a heartbeat is expendable, a crash upload is not.
      if (xSemaphoreTake(http_mutex, 0) == pdTRUE) {
        String body;
        body.reserve(320);
        body += F("{\"device_id\":\"");
        body += DEVICE_ID;
        body += F("\",\"gps\":{");
        if (gps_data.valid && gps_data.has_lock) {
          body += F("\"lat\":");
          appendFloat(body, gps_data.latitude, 6);
          body += F(",\"lon\":");
          appendFloat(body, gps_data.longitude, 6);
          body += F(",\"valid\":true,\"satellites\":");
          body += String(gps_data.satellites);
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
          Serial.print(F("[HB] "));
          Serial.println(code);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
  }
}

// ═══════════════════════════════════════════════════════════════

void displayCrashResults() {
  Serial.println();
  Serial.println(F("+========================================================+"));
  Serial.println(F("|              CRASH ANALYSIS COMPLETE                   |"));
  Serial.println(F("+========================================================+"));
  Serial.print(F("| Incident #"));
  Serial.println(crash_count);
  Serial.print(F("| Severity Class: "));
  Serial.println(last_crash_result.severity_class);
  Serial.print(F("| Severity Name: "));
  Serial.println(last_crash_result.severity_name);
  Serial.print(F("| Confidence: "));
  Serial.print(last_crash_result.confidence * 100, 1);
  Serial.println(F("%"));
  Serial.print(F("| Peak Acceleration: "));
  Serial.print(last_crash_result.peak_magnitude_g, 2);
  Serial.println(F("g"));

  if (gps_data.has_lock && gps_data.valid) {
    Serial.println(F("+--------------------------------------------------------+"));
    Serial.println(F("| GPS LOCATION                                           |"));
    Serial.print(F("| Latitude:  "));
    Serial.println(gps_data.latitude, 6);
    Serial.print(F("| Longitude: "));
    Serial.println(gps_data.longitude, 6);
    Serial.print(F("| Altitude:  "));
    Serial.print(gps_data.altitude, 1);
    Serial.println(F(" m"));
    Serial.print(F("| Satellites: "));
    Serial.println(gps_data.satellites);
    Serial.print(F("| Map: https://maps.google.com/?q="));
    Serial.print(gps_data.latitude, 6);
    Serial.print(F(","));
    Serial.println(gps_data.longitude, 6);
  } else {
    Serial.println(F("+--------------------------------------------------------+"));
    Serial.println(F("| GPS STATUS: No lock (awaiting satellites)              |"));
  }

  // GSM status block
  Serial.println(F("+--------------------------------------------------------+"));
  Serial.println(F("| GSM STATUS                                             |"));
  Serial.print(F("| Modem: "));
  Serial.println(gsm_data.module_found ? F("present") : F("NOT FOUND"));
  Serial.print(F("| Network: "));
  Serial.println(gsm_data.network_reg ? F("registered") : F("not registered"));
  Serial.print(F("| Signal (CSQ): "));
  Serial.println(gsm_data.signal_quality);
  Serial.print(F("| SMS sent / failed: "));
  Serial.print(gsm_data.sms_sent);
  Serial.print(F(" / "));
  Serial.println(gsm_data.sms_failed);

  Serial.println(F("+========================================================+"));
  Serial.println();
}

// ═══════════════════════════════════════════════════════════════
// BANNER
// ═══════════════════════════════════════════════════════════════

void printBanner() {
  Serial.println();
  Serial.println(F("+========================================================+"));
  Serial.println(F("|   ESP32 COMPLETE CRASH DETECTION SYSTEM                |"));
  Serial.println(F("|   RGB LED + GPS + GSM EMERGENCY NOTIFICATION           |"));
  Serial.println(F("|                                                        |"));
  Serial.println(F("|   Components:                                          |"));
  Serial.println(F("|   - MPU6050  (Crash Detection)      GPIO 21/22         |"));
  Serial.println(F("|   - WiFi     (ML API severity)                         |"));
  Serial.println(F("|   - GPS      NEO-6M  Serial2        GPIO 16/17         |"));
  Serial.println(F("|   - GSM      SIM800L Serial1                           |"));
  Serial.println(F("|       ESP32 GPIO19 TXD --> SIM800L RXD                 |"));
  Serial.println(F("|       ESP32 GPIO18 RXD <-- SIM800L TXD                 |"));
  Serial.println(F("|   - RGB LED  (common cathode, direct to ESP32)         |"));
  Serial.println(F("|       R = GPIO 4  via 330R                             |"));
  Serial.println(F("|       G = GPIO 27 via 100R                             |"));
  Serial.println(F("|       B = GPIO 23 via 100R                             |"));
  Serial.println(F("|       Common (long leg) -> GND                         |"));
  Serial.println(F("|   - Buzzer                          GPIO 32            |"));
  Serial.println(F("|                                                        |"));
  Serial.println(F("|   Old LED pins 25 / 26 / 33 are UNUSED in this build.  |"));
  Serial.println(F("|   They stay soldered but are never driven.             |"));
  Serial.println(F("|                                                        |"));
  Serial.println(F("|   Alert Colours:                                       |"));
  Serial.println(F("|   GREEN  = NORMAL   : no alert, no SMS                 |"));
  Serial.println(F("|   AMBER  = MODERATE : 3 pulses + SMS to contacts       |"));
  Serial.println(F("|   RED    = SEVERE   : 5 flashes + SMS + voice call     |"));
  Serial.println(F("|   WHITE  = crash trigger fired, capturing window       |"));
  Serial.println(F("|   BLUE   = WiFi connected (boot indicator)             |"));
  Serial.println(F("|                                                        |"));
  Serial.println(F("|   Resilience:                                          |"));
  Serial.println(F("|   If WiFi/ML API is down, the crash is classified      |"));
  Serial.println(F("|   on-device and the SMS is still sent over GSM.        |"));
  Serial.println(F("|                                                        |"));
  Serial.println(F("|   *** SIM800L NEEDS ITS OWN 4V / 2A SUPPLY ***         |"));
  Serial.println(F("|   *** DO NOT POWER IT FROM THE ESP32 3V3 PIN ***       |"));
  Serial.println(F("+========================================================+"));
  Serial.println(F("|   THRESHOLD PROFILE: " PROFILE_NAME));
  Serial.printf("|   trigger: accel >= %.1f g AND jerk >= %.1f g/s\n",
                ACCEL_THRESHOLD_G, JERK_THRESHOLD_G_PER_SEC);
  Serial.printf("|   local fallback (raw): %.1f g <= peak < %.1f g, %d-%d ms, Severe impulse >= %.3f g.s\n",
                SIG_PEAK_MIN_G, SIG_PEAK_MAX_G, SIG_DUR_MIN_MS, SIG_DUR_MAX_MS, SIG_SEVERE_IMPULSE);
#ifdef RC_DEMO_PROFILE
  Serial.println(F("|   backend rc_demo (filtered): 2.0-16 g, 10-250 ms, Severe >= 0.10 g.s"));
  Serial.println(F("|   !!! RC-CAR DEMONSTRATION THRESHOLDS - provisional, not the thesis taxonomy !!!"));
#endif
  Serial.println(F("+========================================================+"));
  Serial.println();
}





