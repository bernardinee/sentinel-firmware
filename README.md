# sentinel-firmware

ESP32 crash-detection node for the Sentinel system.

**Sketch: `ESP32_Crash_Detection_GPS_GSM_RGB_v3/`**

This is your working `..._RGB_v2` sketch with the backend integration applied.
Everything that already worked is untouched: the RGB LED scheme on GPIO 4/27/23,
GPS on Serial2 (16/17), SIM800L on Serial1 (18/19), the SMS and emergency-call
logic, the offline fallback classifier, and the mount-orientation check.

> An earlier `Sentinel_Crash_Detection/` sketch was removed. It was written
> before the real GPS/GSM firmware was available and guessed the SIM800L UART as
> GPIO 26/27 — which collides with the RGB green channel on 27. Flashing it
> would have broken the LED and the modem. It stays in git history if ever
> needed; nothing references it.

## What changed from v2

1. **Reports to the backend, not the ML API.** `POST {BACKEND_URL}/api/v1/events`
   with an `X-API-Key` header. The device no longer knows the ML API exists; the
   backend classifies, stores the raw window, and pushes the incident to the
   dispatch console. A classification used to live only in the serial log and
   vanished when the buffer scrolled.
2. **Unique event ids.** `{DEVICE_ID}-{boot:04d}-{millis:08d}`, with the boot
   counter persisted in NVS so ids survive a reset.
3. **Retries.** Up to 3 attempts with 1 s / 3 s backoff. Safe because the server
   is idempotent on `event_id`; 4xx is not retried, since repeating cannot fix a
   contract violation. SMS still fires on the local decision regardless.
4. **Heartbeat task** every 30 s on core 0 at low priority, so it can never
   preempt the 100 Hz sampling loop on core 1. This is what shows the node as
   *online* on the dashboard before any crash has happened.
5. **Memory discipline.** The ~28 kB body is built with a heap `String` and
   `reserve()`, replacing `StaticJsonDocument<16384>` — 500×6 floats does not fit
   an 8 kB loop-task stack, and an oversized document there has overflowed this
   project before.
6. **NTP sync**, so `detected_at` is the real trigger instant. It is back-dated
   past the post-roll, so the dashboard's detected→received figure measures the
   network rather than our own 2.5 s wait. Omitted entirely when NTP has not
   synced — an absent timestamp beats a wrong one.

Already correct in v2 and left alone: the impact-centred window
(`POST_TRIGGER_SAMPLES 250`), the ±16 g accelerometer range, ±500 dps gyro, and
GPS in signed decimal degrees.

## Configure before flashing

```cpp
const char* WIFI_SSID     = "...";   // 2.4 GHz only — the ESP32 has no 5 GHz radio
const char* WIFI_PASSWORD = "...";
const char* BACKEND_URL   = "https://<your-backend>.up.railway.app";
const char* API_KEY       = "...";   // must match ROLE_KEYS on the backend
const char* DEVICE_ID     = "ESP32_ACC_001";
```

`API_KEY` should be a **device**-scoped key, never the responder key the
dashboard uses. Firmware flash can be read back over USB, so what is stored here
must only be able to report incidents — not dispatch or resolve them. Register
it on the backend with:

```
ROLE_KEYS={"<your device key>": "device"}
```

To run against a laptop instead of Railway, set `BACKEND_URL` to
`http://<LAN-IP>:8080`; the URL scheme decides whether TLS is used.

## Build

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 ESP32_Crash_Detection_GPS_GSM_RGB_v3 \
  --libraries <your Arduino/libraries>
```

Verified against ESP32 core 3.3.8:

```
Sketch uses 1128747 bytes (86%) of program storage space.
Global variables use 66708 bytes (20%) of dynamic memory,
leaving 260972 bytes for local variables.
```

261 kB free heap against a 40 kB payload reserve plus TLS buffers.

Libraries: `MPU6050`, `ArduinoJson` (v7), plus the ESP32 core's WiFi,
HTTPClient, Preferences and time.

## What you should see on boot

```
[NVS] Boot count: 1
[MPU6050] Connected
[WiFi] OK - Connected!
[NTP] Syncing... OK - UTC set
[TASK] Heartbeat started on core 0
```

Within 30 s the dashboard's device panel shows the node **online**, with a
heartbeat age that counts up and resets. That confirms the link before any crash.

## Power warning (unchanged)

The SIM800L draws up to 2 A in bursts and cannot run off the ESP32 3.3 V pin.
Use a separate 4 V supply rated 2 A or more, a 1000 µF capacitor across its
VCC/GND, and tie all grounds together.
