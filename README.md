# sentinel-firmware

ESP32 crash-detection node for the Sentinel system.
Sketch: `Sentinel_Crash_Detection/Sentinel_Crash_Detection.ino`

Derived from `ESP32_Complete_3LED_System.ino`. The 100 Hz sampling, the
2.0 g + 5.0 g/s dual-threshold trigger, the 3-colour LED scheme and the buzzer
patterns are unchanged.

## What changed

1. **Uploads to the backend, not the ML API.** `POST {BACKEND_URL}/api/v1/events`
   with an `X-API-Key` header. The device no longer knows the ML API exists.
2. **Unique event IDs.** `{DEVICE_ID}-{boot:04d}-{millis:08d}`, boot count
   persisted in NVS so IDs survive resets.
3. **Retries.** Up to 3 attempts with 1 s / 3 s backoff. Safe because the server
   is idempotent on `event_id`. 4xx is not retried.
4. **Impact-centred window.** A continuous 500-sample ring holds 2.5 s of
   pre-trigger history; on trigger the device waits for 250 more samples and
   ships 250 pre + 250 post. Still exactly 500 samples @ 100 Hz, so the API
   contract is unchanged — but the ~70 ms crash pulse now sits mid-window
   instead of at the edge, and the alert goes out ~2.5 s sooner.
5. **Heartbeat task.** Every 30 s on core 0 at low priority, so it can never
   block or preempt IMU sampling (which owns core 1 via `loop()`).
6. **Memory discipline.** The ~28 kB body is built with a heap `String` and
   `reserve(40000)`, never a `StaticJsonDocument` on the 8 kB loop-task stack.
   Only the 246-byte flat ACK is parsed with ArduinoJson, and `severity_name` is
   mapped to permanent literals rather than pointers into a scoped document.

Plus NTP time sync, so `detected_at` is the trigger instant and the dashboard's
latency breakdown has its first leg.

## ⚠ Two things to check before flashing

**(a) Accelerometer range — fixed here, please verify.**
The previous firmware called `mpu.initialize()`, which the MPU6050 library sets
to `MPU6050_ACCEL_FS_2` (±2 g), while the pin comment claimed "±16g range".
`RAW_ACCEL_TO_G = 1/16384` is the correct scale *for ±2 g*, so units were right —
but the sensor physically **clips at 2 g per axis**. The model's crash band is
2–7 g, so no real crash could be measured in-band; the ceiling was
√(3 × 2²) = 3.46 g and only with all three axes saturated. This build sets ±16 g
explicitly (2048 LSB/g).

**(b) GPS and GSM blocks are reconstructed, not copied.**
`ESP32_Crash_Detection_GPS_GSM.ino` was not present in the workspace, so those
sections were written from the hardware description in the brief (GPS on Serial2
GPIO 16/17 @ 9600; SIM800L on Serial1). **`GSM_RX_PIN` is currently 26, which
collides with `LED_GREEN` 26** — that placeholder is left visible rather than
silently guessed, because guessing pin maps is how this project got bitten
before. `ENABLE_GSM` is `0` until you set the real UART pins.

## Build

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 Sentinel_Crash_Detection \
  --libraries <your Arduino/libraries>
```

Verified against ESP32 core 3.3.8:

```
Sketch uses 1113191 bytes (84%) of program storage space.
Global variables use 64828 bytes (19%) of dynamic memory,
leaving 262852 bytes for local variables.
```

262 kB free heap against a 40 kB payload reserve — comfortable.

Libraries: `MPU6050`, `ArduinoJson`, `TinyGPSPlus`, plus the ESP32 core's WiFi,
HTTPClient and Preferences.

## Configure

```cpp
const char* WIFI_SSID     = "...";
const char* WIFI_PASSWORD = "...";
const char* BACKEND_URL   = "http://192.168.x.x:8080";  // LAN IP, not localhost
const char* API_KEY       = "...";                      // must match the backend
const char* DEVICE_ID     = "ESP32_ACC_001";
```

The device cannot reach `localhost`. Use the LAN IP of the machine running the
backend (`ipconfig` → IPv4), keep both on the same network, and allow port 8080
through the firewall.

## SMS fail-safe

The SIM800L path fires on the device's own threshold decision and is sent whether
or not the POST succeeded. It does not consult the backend and does not wait for
a classification. That independence is deliberate: WiFi is the assumption most
likely to be false at a real crash site, and the fail-safe must not share a
failure mode with the thing it guards against.

The device keeps its own hardcoded recipient list for the same reason — the
backend's contact table is for the Sentinel app.
