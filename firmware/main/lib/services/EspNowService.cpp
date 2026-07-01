#include "EspNowService.h"

#include <esp_wifi.h>
#include <string.h>

#include "MotionService.h"
#include "ActuatorService.h"
#include "LidarService.h"
#include "ScriptService.h"
#include "../core/PoseBus.h"
#include "../core/OdometryLib.h"
#include "../core/DriveDefs.hpp"
#include "../drivers/BoardPowerLib.h"

extern volatile bool g_sleepEntryInProgress;

EspNowService* EspNowService::self_ = nullptr;

// ── small helpers ──────────────────────────────────────────────────────────────
namespace {
// Parse up to 16 hex chars (64-bit) into 8 little-endian bytes. Matches the
// ESPNOW_INIT serial line (id/key are printed big-endian-readable, byte 0 = MSB
// of the printed string). We keep the on-wire layout identical between robot and
// host by treating the hex string as a big-endian number and storing it MSB-first
// here; the host module mirrors this. Empty/short strings yield zeros.
void hex16ToBytes_(const String& hex, uint8_t out[8]) {
  memset(out, 0, 8);
  if (hex.length() < 16) return;
  for (int i = 0; i < 8; i++) {
    auto nib = [](char c) -> uint8_t {
      if (c >= '0' && c <= '9') return c - '0';
      if (c >= 'a' && c <= 'f') return c - 'a' + 10;
      if (c >= 'A' && c <= 'F') return c - 'A' + 10;
      return 0;
    };
    out[i] = (nib(hex[i * 2]) << 4) | nib(hex[i * 2 + 1]);
  }
}

float clampf_(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
}  // namespace

// ── lifecycle ──────────────────────────────────────────────────────────────────
bool EspNowService::begin(const SystemRuntimeConfig& sys) {
  self_ = this;

  hex16ToBytes_(sys.deviceId, deviceId_);
  hex16ToBytes_(sys.pairingKey, pairingKey_);
  channel_ = sys.espnowChannel ? sys.espnowChannel : 1;

  // STA radio, NO connection, NO AP. Just enough to run ESP-NOW on a fixed channel.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);   // ensure we are not associated
  esp_wifi_set_ps(WIFI_PS_NONE);   // ESP-NOW must not sleep the modem

  // Pin the channel. esp_now requires the primary channel to match the peer's.
  esp_wifi_set_channel(channel_, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] esp_now_init FAILED");
    started_ = false;
    return false;
  }

  esp_now_register_recv_cb(&EspNowService::onRecvThunk_);
  esp_now_register_send_cb(&EspNowService::onSentThunk_);

  // Add the broadcast peer for BEACON / PROBE_ACK fallbacks.
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, ESPNOW_BROADCAST_MAC, 6);
  peer.channel = channel_;
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESPNOW] add broadcast peer FAILED (continuing)");
  }

  state_ = State::FREE;
  sessionToken_ = 0;
  motorsStopped_ = true;
  mbHead_ = mbTail_ = 0;

  uint8_t mac[6] = {0};
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("[ESPNOW] up: mac=%02x%02x%02x%02x%02x%02x ch=%u id=%02x%02x%02x%02x%02x%02x%02x%02x\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], channel_,
                deviceId_[0], deviceId_[1], deviceId_[2], deviceId_[3],
                deviceId_[4], deviceId_[5], deviceId_[6], deviceId_[7]);

  started_ = true;
  return true;
}

void EspNowService::stop() {
  if (!started_) return;
  esp_now_unregister_recv_cb();
  esp_now_unregister_send_cb();
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  started_ = false;
  self_ = nullptr;
  Serial.println("[ESPNOW] stopped");
}

// ── RX callbacks (WiFi task context — keep short) ───────────────────────────────
void EspNowService::onRecvThunk_(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!self_ || len <= 0 || len > ESPNOW_MAX_PAYLOAD) return;
  EspNowService& s = *self_;

  const uint8_t head = s.mbHead_;
  const uint8_t next = (head + 1) % kMailboxDepth;
  if (next == s.mbTail_) return;  // mailbox full — drop (protocol tolerates loss)

  volatile RxSlot& slot = s.mailbox_[head];
  memcpy((void*)slot.mac, info->src_addr, 6);
  memcpy((void*)slot.data, data, len);
  slot.len = (uint8_t)len;
  s.mbHead_ = next;
}

void EspNowService::onSentThunk_(const uint8_t* /*mac_addr*/, esp_now_send_status_t /*status*/) {
  // Best-effort transport; nothing to do on send completion for now.
}

// ── periodic work ───────────────────────────────────────────────────────────────
void EspNowService::tick() {
  if (!started_) return;
  const uint32_t now = millis();

  // 1. Drain the RX mailbox.
  while (mbTail_ != mbHead_) {
    const uint8_t tail = mbTail_;
    volatile RxSlot& slot = mailbox_[tail];
    uint8_t mac[6];
    uint8_t buf[ESPNOW_MAX_PAYLOAD];
    const uint8_t len = slot.len;
    memcpy(mac, (const void*)slot.mac, 6);
    memcpy(buf, (const void*)slot.data, len);
    mbTail_ = (tail + 1) % kMailboxDepth;
    handleFrame_(mac, buf, len);
  }

  // 2. Motion deadman: if we own a session but no COMMAND/HEARTBEAT for ~500 ms,
  //    stop the motors. Safety critical — a robot must never coast after the link
  //    goes quiet.
  if (state_ == State::OWNED && !motorsStopped_ &&
      (now - lastCommandMs_) >= kDeadmanMs) {
    if (motion_ && !g_sleepEntryInProgress) motion_->setCommand(DriveDir::Stop, 0.0f);
    motorsStopped_ = true;
    Serial.println("[ESPNOW] deadman: motors stopped (no command 500ms)");
  }

  // 3. Lease expiry: no HEARTBEAT for ~10 s -> release, stop, beacon FREE.
  if (state_ == State::OWNED && (now - lastHeartbeatMs_) >= kLeaseMs) {
    releaseToFree_("lease expired");
  }

  // 4. BEACON ~1 Hz.
  if ((now - lastBeaconMs_) >= kBeaconPeriodMs) {
    lastBeaconMs_ = now;
    sendBeacon_();
  }
}

// ── frame dispatch ──────────────────────────────────────────────────────────────
void EspNowService::handleFrame_(const uint8_t* mac, const uint8_t* data, uint8_t len) {
  if (!headerOk_(data, len)) return;

  const uint8_t type = data[2];
  const uint8_t* payload = data + sizeof(EspNowHeader);
  const uint8_t plen = len - sizeof(EspNowHeader);

  // Auth-bearing types: validate pairing_key first, reply AUTH_FAIL on mismatch.
  const bool authBearing =
      (type == ENT_CLAIM || type == ENT_COMMAND || type == ENT_HEARTBEAT ||
       type == ENT_RELEASE || type == ENT_BLINK);

  if (authBearing) {
    if (plen < sizeof(EspNowAuth)) return;  // malformed
    if (!authOk_(payload)) {
      sendAuthFail_(mac, EAUTH_BAD_KEY);
      return;  // never act on a wrong key
    }
  }

  switch (type) {
    case ENT_PROBE:     onProbe_(mac); break;
    case ENT_BLINK:     onBlink_(mac); break;
    case ENT_CLAIM:     onClaim_(mac, payload + sizeof(EspNowAuth), plen - sizeof(EspNowAuth)); break;
    case ENT_COMMAND:   onCommand_(mac, payload + sizeof(EspNowAuth), plen - sizeof(EspNowAuth)); break;
    case ENT_HEARTBEAT: onHeartbeat_(mac); break;
    case ENT_RELEASE:   onRelease_(mac); break;
    default: break;  // BEACON/ACK/RESPONSE/AUTH_FAIL are robot->host; ignore inbound
  }
}

// ── validation ──────────────────────────────────────────────────────────────────
bool EspNowService::headerOk_(const uint8_t* data, uint8_t len) const {
  if (len < sizeof(EspNowHeader)) return false;
  if (data[0] != ESPNOW_MAGIC) return false;
  if (data[1] != ESPNOW_PROTO_VERSION) {
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 2000) {  // log once-ish, don't spam
      Serial.printf("[ESPNOW] dropped: version %u != %u\n", data[1], ESPNOW_PROTO_VERSION);
      lastLog = millis();
    }
    return false;
  }
  // device_id must address us.
  return memcmp(data + 3, deviceId_, 8) == 0;
}

bool EspNowService::authOk_(const uint8_t* auth) const {
  // auth points at the EspNowAuth block (pairing_key first 8 bytes).
  return memcmp(auth, pairingKey_, 8) == 0;
}

bool EspNowService::isOwner_(const uint8_t* mac) const {
  return state_ == State::OWNED && memcmp(mac, ownerMac_, 6) == 0;
}

// ── handlers ────────────────────────────────────────────────────────────────────
void EspNowService::onProbe_(const uint8_t* mac) {
  Serial.printf("[ESPNOW] PROBE from %02x:%02x:%02x:%02x:%02x:%02x\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  sendProbeAck_(mac);
}

void EspNowService::onBlink_(const uint8_t* mac) {
  Serial.println("[ESPNOW] BLINK");
  if (!actuator_) return;
  // Flash white a few times via the existing LED driver, restoring afterwards.
  const uint8_t r0 = actuator_->ledR(), g0 = actuator_->ledG(), b0 = actuator_->ledB();
  for (int i = 0; i < 3; i++) {
    actuator_->setLed(255, 255, 255);
    delay(120);
    actuator_->setLed(0, 0, 0);
    delay(120);
  }
  actuator_->setLed(r0, g0, b0);
}

void EspNowService::onClaim_(const uint8_t* mac, const uint8_t* /*payload*/, uint8_t /*plen*/) {
  // payload carries dongle_id(8) — informational; ownership is keyed on src MAC.
  if (state_ == State::FREE || isOwner_(mac)) {
    memcpy(ownerMac_, mac, 6);
    sessionToken_ = ((uint32_t)esp_random());
    if (sessionToken_ == 0) sessionToken_ = 1;  // 0 is reserved
    state_ = State::OWNED;
    const uint32_t now = millis();
    lastHeartbeatMs_ = now;
    lastCommandMs_   = now;
    Serial.printf("[ESPNOW] CLAIM OK owner=%02x:%02x:%02x:%02x:%02x:%02x token=%08lx\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], (unsigned long)sessionToken_);
    sendClaimAck_(mac, ECLAIM_OK, sessionToken_);
  } else {
    Serial.println("[ESPNOW] CLAIM DENIED (owned by another)");
    sendClaimAck_(mac, ECLAIM_DENIED, 0);
  }
}

void EspNowService::onHeartbeat_(const uint8_t* mac) {
  // Validate token + owner. Token sits in the auth block we already verified the
  // key on; re-extract it from the frame is not needed here since handleFrame_
  // passed auth — but we still must confirm session ownership.
  if (!isOwner_(mac)) {
    sendAuthFail_(mac, EAUTH_NO_CLAIM);
    return;
  }
  lastHeartbeatMs_ = millis();
}

void EspNowService::onRelease_(const uint8_t* mac) {
  if (!isOwner_(mac)) {
    sendAuthFail_(mac, EAUTH_NO_CLAIM);
    return;
  }
  releaseToFree_("RELEASE");
}

void EspNowService::onCommand_(const uint8_t* mac, const uint8_t* payload, uint8_t plen) {
  if (!isOwner_(mac)) {
    sendAuthFail_(mac, EAUTH_NO_CLAIM);
    return;
  }
  if (plen < 1) return;

  const uint32_t now = millis();
  lastHeartbeatMs_ = now;  // a valid command also proves the link is alive
  lastCommandMs_   = now;

  const uint8_t sub = payload[0];
  const uint8_t* args = payload + 1;
  const uint8_t alen = plen - 1;

  switch (sub) {
    case ECS_DRIVE: {
      if (alen < sizeof(EspNowCmdDrive)) break;
      EspNowCmdDrive c; memcpy(&c, args, sizeof(c));
      if (motion_ && !g_sleepEntryInProgress) {
        const float speed = clampf_(c.speed, -1.0f, 1.0f);
        const DriveDir dir = (c.dir <= (uint8_t)DriveDir::TurnR)
                                 ? (DriveDir)c.dir : DriveDir::Stop;
        motion_->setCommand(dir, speed);
        motorsStopped_ = (dir == DriveDir::Stop);
      }
      break;
    }
    case ECS_DRIVE_VEC: {
      if (alen < sizeof(EspNowCmdDriveVec)) break;
      EspNowCmdDriveVec c; memcpy(&c, args, sizeof(c));
      if (motion_ && !g_sleepEntryInProgress) {
        motion_->setCommandVec(clampf_(c.longitudinal, -1.0f, 1.0f),
                               clampf_(c.lateral, -1.0f, 1.0f),
                               clampf_(c.rotational, -1.0f, 1.0f));
        motorsStopped_ = false;
      }
      break;
    }
    case ECS_STOP: {
      if (motion_ && !g_sleepEntryInProgress) motion_->setCommand(DriveDir::Stop, 0.0f);
      motorsStopped_ = true;
      break;
    }
    case ECS_LED: {
      if (alen < sizeof(EspNowCmdLed)) break;
      EspNowCmdLed c; memcpy(&c, args, sizeof(c));
      if (actuator_) actuator_->setLed(c.r, c.g, c.b);
      break;
    }
    case ECS_SERVO: {
      if (alen < sizeof(EspNowCmdServo)) break;
      EspNowCmdServo c; memcpy(&c, args, sizeof(c));
      const float deg = clampf_(c.angle, 0.0f, 180.0f);
      if (actuator_) { if (c.index == 0) actuator_->setServo1Deg(deg); else actuator_->setServo2Deg(deg); }
      break;
    }
    case ECS_BUZZER: {
      if (alen < sizeof(EspNowCmdBuzzer)) break;
      EspNowCmdBuzzer c; memcpy(&c, args, sizeof(c));
      if (actuator_) { if (c.freq == 0) actuator_->buzzerOff(); else actuator_->setBuzzerTone(c.freq); }
      break;
    }
    case ECS_READ: {
      if (alen < sizeof(EspNowCmdRead)) break;
      EspNowCmdRead c; memcpy(&c, args, sizeof(c));
      // req_id is not carried by READ in v1; echo 0. Build the RESPONSE data per sensor.
      uint8_t resp[16];
      uint8_t n = 0;
      switch (c.sensor) {
        case ESEN_DISTANCE: {
          float cm = -1.0f;
          if (lidar_) {
            uint16_t strip[4];
            if (lidar_->getStrip(strip)) {
              // front-row minimum non-zero range, mm -> cm
              uint16_t best = 0;
              for (int i = 0; i < 4; i++) if (strip[i] && (best == 0 || strip[i] < best)) best = strip[i];
              if (best) cm = best / 10.0f;
            }
          }
          memcpy(resp, &cm, 4); n = 4;
          break;
        }
        case ESEN_HEADING: {
          float deg = 0.0f;
          if (pose_) { Pose2D p = pose_->get(); deg = p.yaw_rad * 180.0f / (float)M_PI; }
          memcpy(resp, &deg, 4); n = 4;
          break;
        }
        case ESEN_POSE: {
          float xyz[3] = {0, 0, 0};
          if (pose_) { Pose2D p = pose_->get(); xyz[0] = p.x_mm; xyz[1] = p.y_mm; xyz[2] = p.yaw_rad * 180.0f / (float)M_PI; }
          memcpy(resp, xyz, 12); n = 12;
          break;
        }
        case ESEN_BATTERY: {
          resp[0] = batteryPercent_(); n = 1;
          break;
        }
        default: break;
      }
      if (n) sendResponse_(mac, 0, resp, n);
      break;
    }
    case ECS_PHOTO:
      // Reserved for future chunked transfer.
      break;

    // ── Script runner ──────────────────────────────────────────────────────
    case ECS_SCRIPT_BEGIN: {
      if (!script_ || alen < sizeof(EspNowCmdScriptBegin)) break;
      EspNowCmdScriptBegin c; memcpy(&c, args, sizeof(c));
      script_->onBegin(mac, c.totalSize, c.scriptId);
      break;
    }
    case ECS_SCRIPT_CHUNK: {
      if (!script_ || alen < sizeof(EspNowCmdScriptChunk)) break;
      EspNowCmdScriptChunk c; memcpy(&c, args, sizeof(c));
      const uint8_t* chunkData = args + sizeof(c);
      uint8_t chunkLen = alen - (uint8_t)sizeof(c);
      script_->onChunk(c.chunkIdx, chunkData, chunkLen);
      break;
    }
    case ECS_SCRIPT_RUN: {
      if (!script_) break;
      script_->onRun(mac, (alen >= 1) ? args[0] : 0);
      break;
    }
    case ECS_SCRIPT_STOP: {
      if (!script_) break;
      script_->onStop();
      break;
    }

    default:
      break;
  }
}

void EspNowService::releaseToFree_(const char* why) {
  if (motion_ && !g_sleepEntryInProgress) motion_->setCommand(DriveDir::Stop, 0.0f);
  motorsStopped_ = true;
  state_ = State::FREE;
  sessionToken_ = 0;
  memset(ownerMac_, 0, 6);
  Serial.printf("[ESPNOW] -> FREE (%s)\n", why ? why : "");
  sendBeacon_();  // announce FREE promptly
}

// ── senders ─────────────────────────────────────────────────────────────────────
void EspNowService::writeHeader_(uint8_t* buf, uint8_t type) const {
  buf[0] = ESPNOW_MAGIC;
  buf[1] = ESPNOW_PROTO_VERSION;
  buf[2] = type;
  memcpy(buf + 3, deviceId_, 8);  // robot->host: device_id is the sender (us)
}

void EspNowService::sendRaw_(const uint8_t* mac, const uint8_t* buf, uint8_t len) {
  ensurePeer_(mac);
  esp_now_send(mac, buf, len);
}

void EspNowService::ensurePeer_(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = channel_;
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void EspNowService::sendBeacon_() {
  uint8_t buf[sizeof(EspNowHeader) + sizeof(EspNowBeacon)];
  writeHeader_(buf, ENT_BEACON);
  EspNowBeacon b;
  b.status  = statusByte_();
  b.battery = batteryPercent_();
  b.fw      = ESPNOW_FW_VERSION;
  memcpy(buf + sizeof(EspNowHeader), &b, sizeof(b));
  esp_now_send(ESPNOW_BROADCAST_MAC, buf, sizeof(buf));
}

void EspNowService::sendProbeAck_(const uint8_t* mac) {
  uint8_t buf[sizeof(EspNowHeader) + sizeof(EspNowProbeAck)];
  writeHeader_(buf, ENT_PROBE_ACK);
  EspNowProbeAck p;
  p.status  = statusByte_();
  p.battery = batteryPercent_();
  memcpy(buf + sizeof(EspNowHeader), &p, sizeof(p));
  sendRaw_(mac, buf, sizeof(buf));
}

void EspNowService::sendClaimAck_(const uint8_t* mac, uint8_t result, uint32_t token) {
  uint8_t buf[sizeof(EspNowHeader) + sizeof(EspNowClaimAck)];
  writeHeader_(buf, ENT_CLAIM_ACK);
  EspNowClaimAck a;
  a.result        = result;
  a.session_token = token;
  memcpy(buf + sizeof(EspNowHeader), &a, sizeof(a));
  sendRaw_(mac, buf, sizeof(buf));
}

void EspNowService::sendResponse_(const uint8_t* mac, uint16_t reqId, const uint8_t* data, uint8_t dataLen) {
  uint8_t buf[ESPNOW_MAX_PAYLOAD];
  writeHeader_(buf, ENT_RESPONSE);
  uint8_t off = sizeof(EspNowHeader);
  EspNowResponseHdr rh; rh.req_id = reqId;
  memcpy(buf + off, &rh, sizeof(rh)); off += sizeof(rh);
  if (dataLen > (ESPNOW_MAX_PAYLOAD - off)) dataLen = ESPNOW_MAX_PAYLOAD - off;
  memcpy(buf + off, data, dataLen); off += dataLen;
  sendRaw_(mac, buf, off);
}

void EspNowService::sendAuthFail_(const uint8_t* mac, uint8_t reason) {
  // Rate-limit to ~1/sec per sender so a flapping stale host can't flood us.
  const uint32_t now = millis();
  if (memcmp(mac, lastAuthFailMac_, 6) == 0 && (now - lastAuthFailMs_) < kAuthFailMinGapMs) {
    return;
  }
  memcpy(lastAuthFailMac_, mac, 6);
  lastAuthFailMs_ = now;

  uint8_t buf[sizeof(EspNowHeader) + sizeof(EspNowAuthFail)];
  writeHeader_(buf, ENT_AUTH_FAIL);
  EspNowAuthFail f; f.reason = reason;
  memcpy(buf + sizeof(EspNowHeader), &f, sizeof(f));
  sendRaw_(mac, buf, sizeof(buf));
}

// ── script packet sender ────────────────────────────────────────────────────────
// Called from ScriptService (running on Core 0 script task) — must be thread-safe.
// esp_now_send() uses an internal queue and is documented as thread-safe.
void EspNowService::sendScriptPacket_(const uint8_t* mac, uint8_t pktType,
                                       const uint8_t* payload, uint8_t plen) {
  if (!started_) return;
  uint8_t buf[ESPNOW_MAX_PAYLOAD];
  writeHeader_(buf, pktType);
  uint8_t off = sizeof(EspNowHeader);
  if (plen > (ESPNOW_MAX_PAYLOAD - off)) plen = ESPNOW_MAX_PAYLOAD - off;
  if (plen) memcpy(buf + off, payload, plen);
  sendRaw_(mac, buf, off + plen);
}

// ── status helpers ──────────────────────────────────────────────────────────────
uint8_t EspNowService::statusByte_() const {
  return (state_ == State::OWNED) ? ESTAT_CLAIMED : ESTAT_FREE;
}

uint8_t EspNowService::batteryPercent_() const {
  if (!power_) return 0;
  // Linear 3.30 V (0%) .. 4.20 V (100%) — matches the WiFi-mode battery LED scale.
  const float v = power_->readBatteryVolts();
  float pct = (v - 3.30f) / (4.20f - 3.30f) * 100.0f;
  pct = clampf_(pct, 0.0f, 100.0f);
  return (uint8_t)(pct + 0.5f);
}
