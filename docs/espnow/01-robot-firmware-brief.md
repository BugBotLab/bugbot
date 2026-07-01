# Robot Firmware Brief: add an ESP-NOW mode

**Target:** `Firmware/BugBot_Firmware` (Arduino + FreeRTOS, XIAO ESP32-S3).
**Implements:** the robot side of `docs/espnow/00-protocol.md`.

## Goal

Add an "ESP-NOW mode" as a new option in the EXISTING serial config-override
system. When enabled, ESP-NOW becomes the SOLE communication protocol: every
WiFi-related feature is disabled (STA, AP, WebSocket, HTTP config portal, mDNS,
Apple MIDI, UDP telemetry). The robot is then controlled by a USB dongle plus a
host PC, reusing the existing motion, actuator, and sensor services unchanged.

**Do not rewrite motion, actuator, or sensor code.** Route incoming commands into
the existing services. The owner likes how the robot behaves today; we are only
swapping the transport (ESP-NOW in place of WebSocket), not the behaviour.

## 1. Config fields (`lib/config/AppConfig.h`, `SystemRuntimeConfig`)

Add:
```cpp
bool   espnowEnabled = false;
String deviceId      = "";   // 16 hex chars (64-bit). Empty until provisioned.
String pairingKey    = "";   // 16 hex chars (64-bit) secret. Empty until provisioned.
uint8_t espnowChannel = 1;
```

## 2. Config-override keys (`lib/services/ConfigService.cpp`)

In `setKey()`, `handleSystem_()`, `saveAll()`, and the help/list printers, add:
- `espnow_enabled=0|1`
- `espnow_channel=<int>`
- `device_id=<hex>` (manual override; normally auto-generated)

Persist `deviceId`, `pairingKey`, `espnowEnabled`, `espnowChannel` to
`/config/system.cfg` through the existing `saveAll` path. Do NOT expose
`pairing_key` as a plain `key=value` setter; it is only set by the init command
below, so it can be rotated atomically.

## 3. Serial provisioning commands (`BugBot_Firmware.ino`, `handleSerialLine_`)

Add these special commands alongside the existing `save` / `reload` / `list`:

- **`espnow_init`**
  1. If `deviceId` is empty, generate a 64-bit random id (`esp_random()` x2) and save. Keep it stable on later inits.
  2. ALWAYS generate a fresh 64-bit `pairing_key` and save. (Rotating the key is what locks out old hosts.)
  3. Set `espnow_enabled=1`, save.
  4. Print one machine-readable line for the host init script:
     `ESPNOW_INIT id=<16hex> key=<16hex> mac=<aabbccddeeff> ch=<n> fw=<ver>`
  5. Reboot so the comms mode takes effect.

- **`regenerate_key`**: roll only the `pairing_key`, save, print a fresh
  `ESPNOW_INIT ...` line. (Re-bind to a new host without changing identity.)

- **`espnow_off`**: set `espnow_enabled=0`, save, reboot (return to WiFi mode).

## 4. Comms-mode selection (`BugBot_Firmware.ino` `setup()`, ~lines 627-684)

Branch the existing network startup so it is fully SKIPPED in ESP-NOW mode:
```cpp
if (runtimeCfg.system.espnowEnabled) {
  espnow.begin(runtimeCfg.system);              // STA radio, ESP-NOW only
  espnow.onCommand([](const Command& c){ /* route into motionSvc / actuatorSvc */ });
} else {
  // existing wifi.configure / wifi.start / midiSvc.begin / udp / mDNS ... unchanged
}
```
In ESP-NOW mode, NONE of WiFi STA/AP, WebSocket, HTTP portal, mDNS, MIDI, or UDP
may start.

## 5. New `EspNowService` (`lib/services/EspNowService.h/.cpp`)

Parallel to `WiFiService`. Responsibilities:
- Init WiFi in STA mode WITHOUT connecting (`WIFI_STA`, no AP), set the channel,
  `esp_now_init`, register receive and send callbacks, add the broadcast peer.
- Validate every packet: `magic`, `version`, and `device_id == mine`.
- Auth: for CLAIM / COMMAND / HEARTBEAT / RELEASE / BLINK, check
  `pairing_key == mine`. If it does not match, reply `AUTH_FAIL(BAD_KEY)` and stop
  (rate-limit to ~1/sec/sender). The robot must NOT act on a wrong key, but it
  MUST answer, so a stale host can clean itself up.
- BEACON: broadcast about once a second with `{status, battery, fw}`.
- PROBE: reply PROBE_ACK (lets the dongle learn our MAC).
- BLINK: flash the RGB LED a few times via the existing LED/actuator code.
- CLAIM: if FREE, store owner MAC, issue a random `session_token`, reply
  CLAIM_ACK(OK, token), set status CLAIMED. If owned by a different MAC, reply
  CLAIM_ACK(DENIED).
- COMMAND: validate token + owner MAC, decode `sub_type`, call the matching
  existing service (MotionService, ActuatorService, or a sensor READ that returns
  RESPONSE).
- HEARTBEAT: renew the lease.
- RELEASE: clear owner, set status FREE.

## 6. Session state machine and safety (critical)

State: `FREE` and `OWNED(owner_mac, session_token, lease_expiry)`.

- **Lease, ~10 s:** if no HEARTBEAT for 10 s, clear the owner, stop the motors,
  return to FREE, beacon FREE.
- RELEASE or power-cycle returns to FREE immediately.
- Clamp every command value to a safe range. A robot must NEVER keep driving
  after the link is lost.

## 7. Identity and pairing summary

- `device_id`: stable 64-bit identity, generated once, kept across re-inits.
- `pairing_key`: 64-bit secret, ROLLED on every `espnow_init` and `regenerate_key`.
  The robot obeys only auth packets whose key matches; a stale host gets
  AUTH_FAIL and purges itself host-side.

## 8. Deliverables

- `EspNowService` + the config fields + the serial commands + the `setup()` branch.
- A shared protocol header matching `00-protocol.md`.
- A bench-test path: in ESP-NOW mode, log received packets, answer PROBE, and
  accept a DRIVE command from a second ESP32, so it can be tested before the
  dongle and host exist.
- WiFi-mode behaviour must be unchanged. `espnow_enabled=0` is the default.
