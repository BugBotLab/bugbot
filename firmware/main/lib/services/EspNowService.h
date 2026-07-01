// EspNowService.h
// ESP-NOW transport for the BugBot robot. Parallel to WiFiService, but instead
// of associating to an AP / serving WebSocket, it brings the radio up in STA
// mode WITHOUT connecting, runs esp_now, and implements the robot side of the
// BugBot ESP-NOW protocol (see lib/net/EspNowProtocol.h and docs/espnow/00-protocol.md).
//
// Responsibilities:
//   - STA radio, fixed channel, no AP, no IP, esp_now_init + broadcast peer.
//   - Validate magic/version/device_id, auth-check the pairing_key.
//   - BEACON ~1 Hz, answer PROBE, BLINK the LED, CLAIM/RELEASE leasing.
//   - Route COMMAND into the existing MotionService / ActuatorService, and
//     answer sensor READ with a RESPONSE.
//   - Safety: motion deadman (~500 ms) and lease (~10 s) enforced from tick().
//
// All incoming traffic is parsed in the ESP-NOW RX callback (runs in the WiFi
// task context) into a small lock-free mailbox; the heavy lifting (service
// calls, sends) happens in tick() on the caller's task to keep the callback
// short and avoid calling blocking services from the WiFi stack context.
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "../config/AppConfig.h"
#include "../net/EspNowProtocol.h"

// Forward decls — we route into the existing services, never reimplement them.
class MotionService;
class ActuatorService;
class LidarService;
class OdomService;
class PoseBus;
class BoardPowerLib;
class ScriptService;

class EspNowService {
public:
  // Wire the existing services in before begin(). Any may be null (the matching
  // command/sensor is then simply ignored).
  void attach(MotionService* motion, ActuatorService* actuator,
              LidarService* lidar, PoseBus* pose, BoardPowerLib* power) {
    motion_ = motion; actuator_ = actuator; lidar_ = lidar; pose_ = pose; power_ = power;
  }

  // Wire the script runner. May be null (script commands are ignored).
  void attachScript(ScriptService* script) { script_ = script; }

  // Called by ScriptService to send ENT_SCRIPT_LOG / ENT_SCRIPT_DONE frames.
  // Public because ScriptService runs in its own task and calls back here.
  void sendScriptPacket_(const uint8_t* mac, uint8_t pktType,
                         const uint8_t* payload, uint8_t plen);

  // Bring the radio up in STA mode (no connect), init esp_now, add broadcast
  // peer. Reads identity/key/channel from sys. Returns false on radio failure.
  bool begin(const SystemRuntimeConfig& sys);

  // Tear the radio + esp_now down (mirrors WiFiService::stop()).
  void stop();

  // Drive periodic work: BEACON, motion deadman, lease expiry, and draining the
  // RX mailbox. Call from loop() frequently (every few ms is fine).
  void tick();

  bool started() const { return started_; }

private:
  // ── Session state machine ────────────────────────────────────────────────
  enum class State : uint8_t { FREE, OWNED };

  // ── RX mailbox ───────────────────────────────────────────────────────────
  // The esp_now RX callback is static; it forwards to the singleton instance.
  // Incoming frames are copied into a tiny ring so tick() can process them off
  // the WiFi task. 250-byte frames, a handful deep — plenty at protocol rates.
  static constexpr uint8_t kMailboxDepth = 8;
  struct RxSlot {
    uint8_t mac[6];
    uint8_t data[ESPNOW_MAX_PAYLOAD];
    uint8_t len;
  };

  static void onRecvThunk_(const esp_now_recv_info_t* info, const uint8_t* data, int len);
  static void onSentThunk_(const uint8_t* mac_addr, esp_now_send_status_t status);
  void handleFrame_(const uint8_t* mac, const uint8_t* data, uint8_t len);

  // Packet handlers (called from tick() context).
  void onProbe_(const uint8_t* mac);
  void onBlink_(const uint8_t* mac);
  void onClaim_(const uint8_t* mac, const uint8_t* payload, uint8_t plen);
  void onCommand_(const uint8_t* mac, const uint8_t* payload, uint8_t plen);
  void onHeartbeat_(const uint8_t* mac);
  void onRelease_(const uint8_t* mac);

  // Sends.
  void sendBeacon_();
  void sendProbeAck_(const uint8_t* mac);
  void sendClaimAck_(const uint8_t* mac, uint8_t result, uint32_t token);
  void sendResponse_(const uint8_t* mac, uint16_t reqId, const uint8_t* data, uint8_t dataLen);
  void sendAuthFail_(const uint8_t* mac, uint8_t reason);
  void sendRaw_(const uint8_t* mac, const uint8_t* buf, uint8_t len);

  // Helpers.
  void ensurePeer_(const uint8_t* mac);
  bool headerOk_(const uint8_t* data, uint8_t len) const;     // magic/version/device_id
  bool authOk_(const uint8_t* auth) const;                    // pairing_key match
  void writeHeader_(uint8_t* buf, uint8_t type) const;        // fills 11-byte header
  uint8_t batteryPercent_() const;
  uint8_t statusByte_() const;
  bool isOwner_(const uint8_t* mac) const;
  void releaseToFree_(const char* why);                       // stop motors, FREE, beacon

  // ── Identity / auth (parsed from config at begin) ────────────────────────
  uint8_t  deviceId_[8] = {0};
  uint8_t  pairingKey_[8] = {0};
  uint8_t  channel_ = 1;

  // ── Services ─────────────────────────────────────────────────────────────
  MotionService*  motion_   = nullptr;
  ActuatorService* actuator_= nullptr;
  LidarService*   lidar_    = nullptr;
  PoseBus*        pose_     = nullptr;
  BoardPowerLib*  power_    = nullptr;
  ScriptService*  script_   = nullptr;

  // ── Session / safety ─────────────────────────────────────────────────────
  State    state_ = State::FREE;
  uint8_t  ownerMac_[6] = {0};
  uint32_t sessionToken_ = 0;
  uint32_t lastHeartbeatMs_ = 0;   // for lease (~10 s)
  uint32_t lastCommandMs_   = 0;   // for motion deadman (~500 ms)
  bool     motorsStopped_   = true;

  static constexpr uint32_t kBeaconPeriodMs   = 1000;
  static constexpr uint32_t kDeadmanMs        = 500;
  static constexpr uint32_t kLeaseMs          = 10000;
  static constexpr uint32_t kAuthFailMinGapMs = 1000;  // rate-limit per sender
  uint32_t lastBeaconMs_ = 0;

  // AUTH_FAIL rate-limit: remember the last sender + time we replied.
  uint8_t  lastAuthFailMac_[6] = {0};
  uint32_t lastAuthFailMs_ = 0;

  // ── RX mailbox ───────────────────────────────────────────────────────────
  volatile RxSlot mailbox_[kMailboxDepth];
  volatile uint8_t mbHead_ = 0;   // written by callback
  volatile uint8_t mbTail_ = 0;   // read by tick()

  bool started_ = false;

  // Singleton pointer so the static esp_now callbacks can reach the instance.
  static EspNowService* self_;
};
