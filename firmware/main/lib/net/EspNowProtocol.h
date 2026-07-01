// EspNowProtocol.h
// Shared ESP-NOW wire contract for the BugBot ecosystem. This header MUST stay
// byte-for-byte in sync with docs/espnow/00-protocol.md and the dongle firmware
// + host Python module (bugbot/protocol.py). On ANY wire change, bump
// ESPNOW_PROTO_VERSION and update the doc.
//
// Transport: ESP-NOW, 2.4 GHz, fixed channel. Max payload 250 bytes — every
// packet defined here fits. All multi-byte scalars are little-endian, which is
// the native layout on the ESP32 (Xtensa LX7), so the packed structs below map
// directly onto the wire with no byte swapping.
#pragma once
#include <stdint.h>

// ── Framing constants ──────────────────────────────────────────────────────────
static constexpr uint8_t ESPNOW_MAGIC         = 0xB6;  // filters out non-BugBot frames
static constexpr uint8_t ESPNOW_PROTO_VERSION = 1;     // bump on any wire change
static constexpr uint8_t ESPNOW_MAX_PAYLOAD   = 250;   // ESP-NOW hard limit

// ── Packet types ───────────────────────────────────────────────────────────────
enum EspNowType : uint8_t {
  ENT_BEACON    = 0x01,  // robot -> broadcast ~1Hz
  ENT_PROBE     = 0x02,  // host  -> broadcast
  ENT_PROBE_ACK = 0x03,  // robot -> host
  ENT_BLINK     = 0x10,  // host  -> robot (auth)
  ENT_CLAIM     = 0x20,  // host  -> robot (auth: key)
  ENT_CLAIM_ACK = 0x21,  // robot -> host
  ENT_COMMAND   = 0x30,  // host  -> robot (auth)
  ENT_RESPONSE  = 0x31,  // robot -> host
  ENT_HEARTBEAT = 0x40,  // host  -> robot (auth)
  ENT_RELEASE   = 0x50,  // host  -> robot (auth)
  ENT_AUTH_FAIL = 0xE0,  // robot -> host
};

// ── Packet types (extended) ────────────────────────────────────────────────────
// These complement EspNowType above; kept separate to avoid touching the existing
// enum and breaking switch exhaustion warnings.
static constexpr uint8_t ENT_SCRIPT_LOG     = 0x60;  // robot -> host: stdout line (UTF-8)
static constexpr uint8_t ENT_SCRIPT_DONE    = 0x61;  // robot -> host: script exited
static constexpr uint8_t ENT_SCRIPT_STARTED = 0x62;  // robot -> host: new script is now executing

// ── COMMAND sub-types ──────────────────────────────────────────────────────────
enum EspNowCmdSub : uint8_t {
  ECS_DRIVE        = 0x01,  // dir(1=DriveDir), speed(f32)            -> MotionService::setCommand
  ECS_DRIVE_VEC    = 0x02,  // longitudinal(f32), lateral(f32), rot(f32) -> setCommandVec
  ECS_STOP         = 0x03,  // (none)                                 -> setCommand(Stop, 0)
  ECS_LED          = 0x10,  // r(1), g(1), b(1)                       -> ActuatorService led
  ECS_SERVO        = 0x11,  // index(1), angle(f32 deg)               -> ActuatorService servo
  ECS_BUZZER       = 0x12,  // freq(u16 Hz)                           -> ActuatorService buzzer
  ECS_READ         = 0x20,  // sensor(1)                              -> returns RESPONSE
  ECS_PHOTO        = 0x30,  // reserved (chunked, future)             -> stub
  // Script runner commands (host -> robot)
  ECS_SCRIPT_BEGIN = 0x40,  // total_size(u32 LE) + script_id(u8)    -> ScriptService
  ECS_SCRIPT_CHUNK = 0x41,  // chunk_idx(u16 LE) + data(1..220)      -> ScriptService
  ECS_SCRIPT_RUN   = 0x42,  // script_id(u8)                         -> ScriptService
  ECS_SCRIPT_STOP  = 0x43,  // (none)                                -> ScriptService
};

// ── READ sensor ids ────────────────────────────────────────────────────────────
enum EspNowSensor : uint8_t {
  ESEN_DISTANCE = 0x01,  // RESPONSE: f32 cm (ToF / LiDAR)
  ESEN_HEADING  = 0x02,  // RESPONSE: f32 deg (IMU yaw)
  ESEN_POSE     = 0x03,  // RESPONSE: x(f32), y(f32), heading(f32) odometry
  ESEN_BATTERY  = 0x04,  // RESPONSE: u8 %
};

// ── CLAIM_ACK result / AUTH_FAIL reason ────────────────────────────────────────
enum EspNowClaimResult : uint8_t { ECLAIM_OK = 0, ECLAIM_DENIED = 1 };
enum EspNowAuthReason  : uint8_t { EAUTH_BAD_KEY = 0, EAUTH_DENIED = 1, EAUTH_NO_CLAIM = 2 };

// ── Robot status (BEACON / PROBE_ACK status byte) ──────────────────────────────
enum EspNowStatus : uint8_t { ESTAT_FREE = 0, ESTAT_CLAIMED = 1 };

// Packed wire structs. #pragma pack(1) — no padding, exact byte offsets.
#pragma pack(push, 1)

// Common header on every packet (11 bytes). device_id is the TARGET robot for
// host->robot frames and the SENDER robot for robot->host frames.
struct EspNowHeader {
  uint8_t  magic;       // 0  ESPNOW_MAGIC
  uint8_t  version;     // 1  ESPNOW_PROTO_VERSION
  uint8_t  type;        // 2  EspNowType
  uint8_t  device_id[8];// 3  64-bit robot identity (little-endian byte order)
};
static_assert(sizeof(EspNowHeader) == 11, "EspNowHeader must be 11 bytes");

// Auth block, present immediately after the header in CLAIM/COMMAND/HEARTBEAT/
// RELEASE/BLINK. session_token is 0 for CLAIM and BLINK (no session yet).
struct EspNowAuth {
  uint8_t  pairing_key[8];  // 8-byte secret
  uint32_t session_token;   // 0 for CLAIM/BLINK
};
static_assert(sizeof(EspNowAuth) == 12, "EspNowAuth must be 12 bytes");

// 0x01 BEACON payload: {status(1), battery(1,%), fw(2)}
struct EspNowBeacon {
  uint8_t  status;    // EspNowStatus
  uint8_t  battery;   // percent 0..100
  uint16_t fw;        // firmware version
};

// 0x03 PROBE_ACK payload: {status(1), battery(1)}
struct EspNowProbeAck {
  uint8_t  status;
  uint8_t  battery;
};

// 0x20 CLAIM payload (after auth): {dongle_id(8)}
struct EspNowClaim {
  uint8_t  dongle_id[8];
};

// 0x21 CLAIM_ACK payload: {result(1), session_token(4)}
struct EspNowClaimAck {
  uint8_t  result;        // EspNowClaimResult
  uint32_t session_token; // 0 if denied
};

// 0xE0 AUTH_FAIL payload: {reason(1)}
struct EspNowAuthFail {
  uint8_t  reason;        // EspNowAuthReason
};

// 0x31 RESPONSE payload header: {req_id(2), data(..)}. Data layout depends on the
// sensor that was READ (see EspNowSensor). The data follows this 2-byte prefix.
struct EspNowResponseHdr {
  uint16_t req_id;
};

// COMMAND argument payloads (each preceded by a 1-byte sub_type on the wire).
struct EspNowCmdDrive    { uint8_t dir;  float speed; };                 // ECS_DRIVE
struct EspNowCmdDriveVec { float longitudinal, lateral, rotational; };   // ECS_DRIVE_VEC
struct EspNowCmdLed      { uint8_t r, g, b; };                           // ECS_LED
struct EspNowCmdServo    { uint8_t index; float angle; };               // ECS_SERVO
struct EspNowCmdBuzzer   { uint16_t freq; };                            // ECS_BUZZER
struct EspNowCmdRead     { uint8_t sensor; };                          // ECS_READ

// Script runner payloads (all host -> robot, preceded by 1-byte ECS_SCRIPT_* sub_type).
struct EspNowCmdScriptBegin { uint32_t totalSize; uint8_t scriptId; };  // ECS_SCRIPT_BEGIN
struct EspNowCmdScriptChunk { uint16_t chunkIdx; };  // ECS_SCRIPT_CHUNK: data follows inline
struct EspNowCmdScriptRun   { uint8_t scriptId; };   // ECS_SCRIPT_RUN
// ECS_SCRIPT_STOP has no arguments.

// Script runner payloads (robot -> host). Sent as raw ENT_SCRIPT_LOG/DONE frames.
// ENT_SCRIPT_LOG: header(11) + utf8_line(variable, no NUL)
// ENT_SCRIPT_DONE: header(11) + exit_code(u8)
struct EspNowScriptDone { uint8_t exitCode; };

#pragma pack(pop)

// Firmware version reported in BEACON.fw and the ESPNOW_INIT serial line.
static constexpr uint16_t ESPNOW_FW_VERSION = 1;

// Broadcast MAC used for BEACON and PROBE.
static const uint8_t ESPNOW_BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
