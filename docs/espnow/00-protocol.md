# BugBot ESP-NOW Protocol (v1)

Shared wire contract for the three components that must agree byte-for-byte:

1. **Robot firmware** (XIAO ESP32-S3, existing BugBot firmware)
2. **Dongle firmware** (XIAO ESP32-C3, serial-to-ESP-NOW bridge)
3. **Host Python library** (`bugbot`)

Keep a shared C header (firmware + dongle) and a matching Python module
(`bugbot/protocol.py`). On ANY wire change, bump `version` and update this doc.

---

## Transport

- ESP-NOW, 2.4 GHz, fixed channel (default 1). No WiFi association, no AP, no IP.
- Max ESP-NOW payload is 250 bytes. Every packet must fit.
- Robot talks to the dongle over ESP-NOW; the dongle bridges to the PC over USB
  serial (see the dongle brief).
- ESP-NOW addresses peers by MAC. The host addresses robots by `device_id`; the
  dongle learns each robot's MAC at connect time (PROBE then PROBE_ACK, or from
  any received packet) and unicasts after that. Broadcast (FF:FF:FF:FF:FF:FF) is
  used for BEACON and PROBE.

## Identity and auth fields

- **device_id**: 64-bit (8 bytes). Unique robot identity. Stable across re-inits.
  Public (used as an address). Random-generated, or derived from the MAC.
- **pairing_key**: 64-bit (8 bytes) secret. Rotatable. The robot only obeys
  auth-bearing packets whose `pairing_key` matches its stored key. Re-init rolls
  this, which instantly locks out any old host.
- **session_token**: 32-bit (4 bytes). Issued by the robot on CLAIM. Ties a live
  session; carried by COMMAND / HEARTBEAT / RELEASE.

### String encoding (host side)

`device_id` and `pairing_key` cross the serial link and live in the roster as
**16-character hex strings** (as printed in the `ESPNOW_INIT` line). They are
parsed **MSB-first** into the 8-byte wire field. The dongle and the `bugbot`
library MUST use this same convention so the bytes on the air match what the
robot stored.

## Common header (every packet)

| Offset | Size | Field      | Notes                                            |
|-------:|-----:|------------|--------------------------------------------------|
| 0      | 1    | magic      | 0xB6, filters out non-BugBot frames              |
| 1      | 1    | version    | protocol version, = 1                            |
| 2      | 1    | type       | packet type (see table)                          |
| 3      | 8    | device_id  | target robot (host to robot) or sender (robot to host) |
| 11     | ..   | payload    | type-specific                                    |

## Auth block (present in CLAIM, COMMAND, HEARTBEAT, RELEASE, BLINK)

Immediately after the header:

| Size | Field         | Notes                                       |
|-----:|---------------|---------------------------------------------|
| 8    | pairing_key   |                                             |
| 4    | session_token | 0 for CLAIM and BLINK (no session yet)      |

## Packet types

| Type | Name      | Direction               | Auth         | Payload                                  |
|-----:|-----------|-------------------------|--------------|------------------------------------------|
| 0x01 | BEACON    | robot to broadcast ~1Hz | no           | status(1), battery(1, %), fw(2)          |
| 0x02 | PROBE     | host to broadcast       | no           | (none; target is device_id in header)    |
| 0x03 | PROBE_ACK | robot to host           | no           | status(1), battery(1)                    |
| 0x10 | BLINK     | host to robot           | yes          | (none)                                   |
| 0x20 | CLAIM     | host to robot           | yes (key)    | dongle_id(8)                             |
| 0x21 | CLAIM_ACK | robot to host           | no           | result(1: 0=OK,1=DENIED), session_token(4)|
| 0x30 | COMMAND   | host to robot           | yes          | sub_type(1), args(..)                    |
| 0x31 | RESPONSE  | robot to host           | no           | req_id(2), data(..)                      |
| 0x40 | HEARTBEAT | host to robot           | yes          | (none)                                   |
| 0x50 | RELEASE   | host to robot           | yes          | (none)                                   |
| 0xE0 | AUTH_FAIL | robot to host           | no           | reason(1: 0=BAD_KEY,1=DENIED,2=NO_CLAIM) |

The robot replies **AUTH_FAIL** whenever an auth-bearing packet carries a wrong
`pairing_key`. The header `device_id` tells the host which roster entry to purge.
Rate-limit AUTH_FAIL to about once per second per sender.

## COMMAND sub-types

Each maps to an EXISTING firmware service call (see the robot brief). Scalars are
float32 little-endian unless noted.

| Sub  | Name      | Args                                   | Maps to (existing firmware)                |
|-----:|-----------|----------------------------------------|--------------------------------------------|
| 0x01 | DRIVE     | dir(1 = DriveDir), speed(f32)          | MotionService::setCommand(DriveDir, speed) |
| 0x02 | DRIVE_VEC | longitudinal(f32), lateral(f32), rot(f32)| MotionService::setCommandVec(...)        |
| 0x03 | STOP      | (none)                                 | MotionService::setCommand(Stop, 0)         |
| 0x10 | LED       | r(1), g(1), b(1)                       | ActuatorService (led)                      |
| 0x11 | SERVO     | index(1), angle(f32 deg)               | ActuatorService (servo)                    |
| 0x12 | BUZZER    | freq(u16 Hz)                           | ActuatorService (buzzer)                   |
| 0x20 | READ      | sensor(1)                              | returns RESPONSE (see sensor table)        |
| 0x30 | PHOTO     | (reserved, chunked, future)            | stub for now                               |

**DriveDir** enum (must match firmware `lib/core/DriveDefs.hpp`):
0 Stop, 1 Fwd, 2 Back, 3 StrafeL, 4 StrafeR, 5 TurnL, 6 TurnR.

## READ sensor ids and RESPONSE data

| sensor   | id   | RESPONSE data                         |
|----------|-----:|---------------------------------------|
| distance | 0x01 | f32 cm (ToF)                          |
| heading  | 0x02 | f32 deg (IMU yaw)                     |
| pose     | 0x03 | x(f32), y(f32), heading(f32) odometry |
| battery  | 0x04 | u8 %                                  |

Keep ids stable; extend as new sensors are exposed.

## Versioning

Bump `version` on any wire change. Receivers must reject a mismatched version
cleanly (drop the packet, log once) rather than misparsing it.
