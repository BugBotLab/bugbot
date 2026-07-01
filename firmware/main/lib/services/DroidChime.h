#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "ActuatorService.h"

struct DroidChimeStep {
  uint16_t freq;        // buzzer frequency in Hz; 0 = silence
  uint8_t  r, g, b;    // LED colour for this step
  uint16_t dur_ms;      // step duration
};

// Non-blocking Star-Wars-style audio+LED event cues.
// A dedicated low-priority FreeRTOS task steps through tone/colour sequences.
// Calling play() while a sequence is in progress interrupts it immediately.
// notifyTagId() queues politely without interrupting — short chirps play naturally.
class DroidChime {
public:
  enum Event : uint8_t {
    STARTUP      = 0,   // boot complete — ascending R2 chirp
    CONNECTED    = 1,   // first WS client joined — happy greeting
    DISCONNECTED = 2,   // last WS client left — sad descending tones
    LOW_BATTERY  = 3,   // battery warning — triple alarm pulse
    FOUND_OBJECT = 4,   // AprilTag first seen (debounced) — excited chirps
    CALIB_START  = 5,   // calibration triggered — attention tones
    CALIB_DONE   = 6,   // calibration succeeded — fanfare
    CALIB_ERROR  = 7,   // calibration failed — error descend
    TAG_ID       = 8,   // per-tag chirp (internal use; data byte = tag id)
  };

  void begin(ActuatorService& act, UBaseType_t prio = 1, BaseType_t core = 1);

  // Post a system event; interrupts any in-progress sequence.
  void play(Event ev);

  // Play a short two-note chirp with a unique frequency + LED colour for this
  // tag ID (0-15). Debounced per ID; does not interrupt other sequences.
  void notifyTagId(uint8_t tagId);

  // Legacy: plays FOUND_OBJECT on first detection after 6 s silence.
  void notifyTagFound();

private:
  static void taskFn(void* arg);
  void        runTask();
  void        playSequence_(const DroidChimeStep* seq, uint8_t n);
  void        playTagChirp_(uint8_t id);
  bool        waitMs_(uint16_t ms);

  ActuatorService* act_   = nullptr;
  // Queue holds uint16_t: high byte = Event, low byte = data (tag id for TAG_ID)
  QueueHandle_t    queue_ = nullptr;
  volatile bool    stop_  = false;

  uint32_t lastTagFoundMs_    = 0;
  uint32_t lastTagMs_[16]     = {};

  static constexpr uint32_t TAG_COOLDOWN_MS    = 6000;
  static constexpr uint32_t TAG_ID_COOLDOWN_MS = 3000;
};
