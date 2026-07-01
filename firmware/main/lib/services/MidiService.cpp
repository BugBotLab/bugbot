#include "MidiService.h"

#include "ActuatorService.h"
#include "MotionService.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <AppleMIDI.h>
#include <MIDI.h>
#include <math.h>

namespace {
constexpr byte kChBuzzer = 1;
constexpr byte kChMotion = 2;
constexpr byte kChLed    = 1;
constexpr byte kChServo  = 4;

constexpr byte kCcMotionLong = 20;
constexpr byte kCcMotionLat  = 21;
constexpr byte kCcMotionRot  = 22;

constexpr byte kCcLedBrightness = 30;
constexpr byte kCcLedR = 31;
constexpr byte kCcLedG = 32;
constexpr byte kCcLedB = 33;

constexpr byte kCcServo1 = 40;
constexpr byte kCcServo2 = 41;

constexpr float kPitchBendSemitoneRange = 1.0f;   // narrower = cleaner on buzzer
constexpr uint32_t kMotionTimeoutMs = 350;
constexpr float kFixedDuty = 0.50f;               // fixed strong duty for cleaner tone
constexpr uint32_t kBuzzerUpdateIntervalMs = 12;  // ~83 Hz bend stepping
constexpr float kMinFreqStepHz = 3.0f;            // ignore tiny retunes
constexpr int kPitchBendDeadband = 160;
constexpr bool kPitchBendEnabled = true;

WiFiUDP g_midiUdp;
APPLEMIDI_CREATE_INSTANCE(WiFiUDP, g_appleMidi, "BugBot", DEFAULT_CONTROL_PORT);
MidiService* g_midiSelf = nullptr;
}

bool MidiService::begin(ActuatorService* actuator, MotionService* motion) {
  actuator_ = actuator;
  motion_ = motion;

  if (!actuator_ || !motion_) {
    Serial.println("[MIDI] begin failed: missing service pointers");
    started_ = false;
    return false;
  }

  g_midiSelf = this;
  g_appleMidi.begin(MIDI_CHANNEL_OMNI);

  g_appleMidi.setHandleNoteOn([](byte ch, byte note, byte vel) {
    if (g_midiSelf) g_midiSelf->handleNoteOn_(ch, note, vel);
  });

  g_appleMidi.setHandleNoteOff([](byte ch, byte note, byte vel) {
    if (g_midiSelf) g_midiSelf->handleNoteOff_(ch, note, vel);
  });

  g_appleMidi.setHandleControlChange([](byte ch, byte cc, byte val) {
    if (g_midiSelf) g_midiSelf->handleControlChange_(ch, cc, val);
  });

  g_appleMidi.setHandlePitchBend([](byte ch, int bend) {
    if (g_midiSelf) g_midiSelf->handlePitchBend_(ch, bend);
  });

  started_ = true;
  Serial.println("[MIDI] AppleMIDI started");
  Serial.println("[MIDI] Ch1 buzzer note/vel/pitchbend | Ch2 CC20-22 motion | Ch3 CC30-33 LED | Ch4 CC40-41 servos");
  Serial.println("[MIDI] Clean-tone mode: fixed duty + quantized bend");
  return true;
}

void MidiService::update() {
  if (!started_) return;
  g_appleMidi.read();
  updateBuzzerOutput_();
  applyMotion_();
}

void MidiService::handleNoteOn_(byte channel, byte note, byte velocity) {
  if (channel != kChBuzzer) return;
  if (velocity == 0) {
    handleNoteOff_(channel, note, velocity);
    return;
  }

  noteActive_ = true;
  note_ = note;
  velocity_ = velocity;
  updateBuzzerTarget_();

  // Immediate, crisp note attack.
  buzzerCurrentFreqHz_ = buzzerTargetFreqHz_;
  buzzerLastAppliedFreqHz_ = buzzerCurrentFreqHz_;
  lastAppliedPitchBend_ = pitchBend_;
  buzzerLastUpdateMs_ = millis();
  actuator_->setBuzzerTone((uint32_t)lroundf(buzzerCurrentFreqHz_), buzzerDuty_);
}

void MidiService::handleNoteOff_(byte channel, byte note, byte velocity) {
  (void)note;
  (void)velocity;
  if (channel != kChBuzzer) return;
  stopBuzzer_();
}

void MidiService::handlePitchBend_(byte channel, int bend) {
  if (channel != kChBuzzer) return;
  if (!kPitchBendEnabled) return;

  if (abs(bend - pitchBend_) < kPitchBendDeadband) return;
  pitchBend_ = bend;
  if (noteActive_) updateBuzzerTarget_();
}

void MidiService::handleControlChange_(byte channel, byte control, byte value) {
  if (!actuator_ || !motion_) return;

  if (channel == kChMotion) {
    if (control == kCcMotionLong) {
      midiLong_ = ccToSigned_(value);
    } else if (control == kCcMotionLat) {
      midiLat_ = ccToSigned_(value);
    } else if (control == kCcMotionRot) {
      midiRot_ = ccToSigned_(value);
    } else {
      return;
    }
    motionActive_ = true;
    motionTimedOutSent_ = false;
    motionLastUpdateMs_ = millis();
    motion_->setCommandVec(midiLong_, midiLat_, midiRot_);
    return;
  }

  if (channel == kChLed) {
    if (control == kCcLedBrightness) {
      ledBrightness_ = value;
    } else if (control == kCcLedR) {
      ledR_ = (uint8_t)map(value, 0, 127, 0, 255);
    } else if (control == kCcLedG) {
      ledG_ = (uint8_t)map(value, 0, 127, 0, 255);
    } else if (control == kCcLedB) {
      ledB_ = (uint8_t)map(value, 0, 127, 0, 255);
    } else {
      return;
    }

    lastLedCcMs_ = millis();
    uint8_t r = ledR_, g = ledG_, b = ledB_;
    scaleLedLevel_(r, g, b);
    actuator_->setLed(r, g, b);
    return;
  }

  if (channel == kChServo) {
    if (control == kCcServo1) {
      actuator_->setServo1Deg((float)map(value, 0, 127, 0, 180));
    } else if (control == kCcServo2) {
      actuator_->setServo2Deg((float)map(value, 0, 127, 0, 180));
    }
    return;
  }
}

void MidiService::updateBuzzerTarget_() {
  if (!actuator_ || !noteActive_) return;

  const float baseFreq = noteToFreqHz_(note_);
  float freqHz = baseFreq;

  if (kPitchBendEnabled) {
    const float bendNorm = ((float)pitchBend_ - 8192.0f) / 8192.0f;
    const float bendSemis = bendNorm * kPitchBendSemitoneRange;
    freqHz = baseFreq * powf(2.0f, bendSemis / 12.0f);
  }

  // Quantize to integer Hz so the buzzer is not constantly retuned.
  buzzerTargetFreqHz_ = floorf(freqHz + 0.5f);
  // Cleaner tone: fixed strong duty. Velocity is ignored here on purpose.
  buzzerDuty_ = kFixedDuty;
}

void MidiService::updateBuzzerOutput_() {
  if (!actuator_) return;
  if (!noteActive_) return;

  const uint32_t now = millis();
  if ((now - buzzerLastUpdateMs_) < kBuzzerUpdateIntervalMs) return;
  buzzerLastUpdateMs_ = now;

  const float deltaHz = fabsf(buzzerTargetFreqHz_ - buzzerLastAppliedFreqHz_);
  if (deltaHz < kMinFreqStepHz) return;

  buzzerCurrentFreqHz_ = buzzerTargetFreqHz_;
  buzzerLastAppliedFreqHz_ = buzzerCurrentFreqHz_;
  lastAppliedPitchBend_ = pitchBend_;
  actuator_->setBuzzerTone((uint32_t)lroundf(buzzerCurrentFreqHz_), buzzerDuty_);
}

void MidiService::stopBuzzer_() {
  noteActive_ = false;
  if (actuator_) actuator_->buzzerOff();
}

void MidiService::applyMotion_() {
  if (!motion_) return;
  if (!motionActive_) return;

  const uint32_t now = millis();
  if ((now - motionLastUpdateMs_) > kMotionTimeoutMs) {
    if (!motionTimedOutSent_) {
      motion_->setCommandVec(0.0f, 0.0f, 0.0f);
      motionTimedOutSent_ = true;
    }
    motionActive_ = false;
  }
}

void MidiService::scaleLedLevel_(uint8_t& r, uint8_t& g, uint8_t& b) const {
  const float s = (float)ledBrightness_ / 127.0f;
  r = (uint8_t)lroundf(r * s);
  g = (uint8_t)lroundf(g * s);
  b = (uint8_t)lroundf(b * s);
}

float MidiService::ccToSigned_(byte value) {
  float v = ((float)value - 64.0f) / 63.0f;
  if (fabsf(v) < 0.06f) v = 0.0f;
  if (v < -1.0f) v = -1.0f;
  if (v > 1.0f) v = 1.0f;
  return v;
}

float MidiService::noteToFreqHz_(byte note) {
  return 440.0f * powf(2.0f, ((float)note - 69.0f) / 12.0f);
}
