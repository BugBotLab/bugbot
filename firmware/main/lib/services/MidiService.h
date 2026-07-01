// MidiService.h
// Apple MIDI (RTP-MIDI) service that maps MIDI note/CC/pitch-bend events to buzzer
// tones, RGB LED colours, and motor commands via ActuatorService and MotionService.
#pragma once
#include <Arduino.h>

class ActuatorService;
class MotionService;

class MidiService {
public:
  bool begin(ActuatorService* actuator, MotionService* motion);
  void update();
  bool started() const { return started_; }
  bool midiLedActive() const {
    return lastLedCcMs_ > 0 && (millis() - lastLedCcMs_) < 20000;
  }

private:
  void handleNoteOn_(byte channel, byte note, byte velocity);
  void handleNoteOff_(byte channel, byte note, byte velocity);
  void handleControlChange_(byte channel, byte control, byte value);
  void handlePitchBend_(byte channel, int bend);
  void updateBuzzerTarget_();
  void updateBuzzerOutput_();
  void stopBuzzer_();
  void applyMotion_();
  void scaleLedLevel_(uint8_t& r, uint8_t& g, uint8_t& b) const;
  static float ccToSigned_(byte value);
  static float noteToFreqHz_(byte note);

  ActuatorService* actuator_ = nullptr;
  MotionService* motion_ = nullptr;
  bool started_ = false;

  // Buzzer state
  bool noteActive_ = false;
  byte note_ = 69;
  byte velocity_ = 100;
  int pitchBend_ = 8192;
  int lastAppliedPitchBend_ = 8192;
  float buzzerDuty_ = 0.50f;
  float buzzerTargetFreqHz_ = 440.0f;
  float buzzerCurrentFreqHz_ = 440.0f;
  float buzzerLastAppliedFreqHz_ = 440.0f;
  uint32_t buzzerLastUpdateMs_ = 0;

  // LED state
  uint8_t ledR_ = 0;
  uint8_t ledG_ = 0;
  uint8_t ledB_ = 0;
  uint8_t ledBrightness_ = 127;
  uint32_t lastLedCcMs_ = 0;

  // MIDI motion state (newest source wins while active)
  float midiLong_ = 0.0f;
  float midiLat_ = 0.0f;
  float midiRot_ = 0.0f;
  bool motionActive_ = false;
  uint32_t motionLastUpdateMs_ = 0;
  bool motionTimedOutSent_ = false;
};
