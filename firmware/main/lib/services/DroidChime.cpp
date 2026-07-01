#include "DroidChime.h"
#include <freertos/idf_additions.h>

// ── Tone sequences ────────────────────────────────────────────────────────────
// Each Step: { freq_hz, r, g, b, dur_ms }
// freq=0 → silence (buzzer off).  Colours: cyan=boot, green=good, red=bad,
// yellow=discovery, blue=calibration, amber=warn.
// All sequences end with buzzer off implicitly; LED fades back to battery pulse.

// STARTUP — ascending R2-style boot chirp
static const DroidChimeStep SEQ_STARTUP[] = {
  { 659,   0,120,180,  70},   // E5  cyan
  { 784,   0,140,200,  55},   // G5  cyan
  { 988,   0,140,200,  55},   // B5  cyan
  {   0,   0,  0,  0,  25},   // rest
  {1319,  60,120,255,  85},   // E6  cool-white
  {   0,   0,  0,  0,  18},   // rest
  { 988,   0,140,200,  50},   // B5  cyan
  {1568,  60,120,255, 140},   // G6  cool-white
};

// CONNECTED — happy ascending greeting (first client)
static const DroidChimeStep SEQ_CONNECTED[] = {
  { 784,   0,180,50,  60},    // G5  green
  { 988,   0,200,60,  55},    // B5  green
  {1319,   0,255,80,  85},    // E6  bright-green
  {   0,   0,  0,  0,  18},   // rest
  {1568,   0,255,80,  80},    // G6  bright-green
};

// DISCONNECTED — sad descending farewell (last client)
static const DroidChimeStep SEQ_DISCONNECTED[] = {
  { 880, 200,100,  0,  80},   // A5  amber
  { 784, 180, 70,  0,  75},   // G5  amber
  { 659, 200, 30,  0, 100},   // E5  red-orange
  {   0,   0,  0,  0,  28},   // rest
  { 587, 150,  0,  0, 140},   // D5  dark-red
};

// LOW_BATTERY — urgent triple alarm pulse
static const DroidChimeStep SEQ_LOW_BATTERY[] = {
  { 880, 255,  0,  0, 140},   // A5  red
  {   0,   0,  0,  0, 110},   // silence
  { 880, 255,  0,  0, 140},   // A5  red
  {   0,   0,  0,  0, 110},   // silence
  { 880, 255,  0,  0, 140},   // A5  red
  {   0,   0,  0,  0,  70},   // silence
};

// FOUND_OBJECT — excited R2 discovery chirp
static const DroidChimeStep SEQ_FOUND_OBJECT[] = {
  {1319, 200,180,  0,  50},   // E6  yellow
  {1760, 255,230,  0,  48},   // A6  bright-yellow
  {1319, 200,180,  0,  38},   // E6  yellow
  {   0,   0,  0,  0,  20},   // rest
  {1760, 255,230,  0,  55},   // A6  bright-yellow
  {2093, 255,240,100,  58},   // C7  yellow-white
  {   0,   0,  0,  0,  22},   // rest
  {1568, 220,200,  0,  35},   // G6  gold
  {1760, 255,230,  0, 100},   // A6  bright-yellow
};

// CALIB_START — two-tap attention signal
static const DroidChimeStep SEQ_CALIB_START[] = {
  { 880,   0, 80,255,  75},   // A5  blue
  {   0,   0,  0,  0,  35},   // rest
  { 880,   0, 80,255,  75},   // A5  blue
  {   0,   0,  0,  0,  35},   // rest
  {1175,   0,160,255, 180},   // D6  bright-blue
};

// CALIB_DONE — ascending fanfare
static const DroidChimeStep SEQ_CALIB_DONE[] = {
  { 784,   0,180,50,  65},    // G5  green
  { 988,   0,200,60,  65},    // B5  green
  {1319,   0,220,70,  65},    // E6  green
  {1568,   0,255,80, 170},    // G6  bright-green
};

// CALIB_ERROR — double alarm then low moan
static const DroidChimeStep SEQ_CALIB_ERROR[] = {
  { 784, 220,  0,  0,  80},   // G5  red
  {   0,   0,  0,  0,  40},   // rest
  { 784, 220,  0,  0,  80},   // G5  red
  {   0,   0,  0,  0,  40},   // rest
  { 587, 140,  0,  0, 180},   // D5  dark-red
};

// ── Sequence table ────────────────────────────────────────────────────────────

struct SeqEntry { const DroidChimeStep* steps; uint8_t n; };

static const SeqEntry SEQ_TABLE[] = {
  { SEQ_STARTUP,      sizeof(SEQ_STARTUP)      / sizeof(SEQ_STARTUP[0])      },
  { SEQ_CONNECTED,    sizeof(SEQ_CONNECTED)    / sizeof(SEQ_CONNECTED[0])    },
  { SEQ_DISCONNECTED, sizeof(SEQ_DISCONNECTED) / sizeof(SEQ_DISCONNECTED[0]) },
  { SEQ_LOW_BATTERY,  sizeof(SEQ_LOW_BATTERY)  / sizeof(SEQ_LOW_BATTERY[0])  },
  { SEQ_FOUND_OBJECT, sizeof(SEQ_FOUND_OBJECT) / sizeof(SEQ_FOUND_OBJECT[0]) },
  { SEQ_CALIB_START,  sizeof(SEQ_CALIB_START)  / sizeof(SEQ_CALIB_START[0])  },
  { SEQ_CALIB_DONE,   sizeof(SEQ_CALIB_DONE)   / sizeof(SEQ_CALIB_DONE[0])   },
  { SEQ_CALIB_ERROR,  sizeof(SEQ_CALIB_ERROR)  / sizeof(SEQ_CALIB_ERROR[0])  },
  // Index 8 (TAG_ID) is handled dynamically in playTagChirp_()
};

// ── Per-tag identity ──────────────────────────────────────────────────────────
// Pentatonic notes across three octaves — each ID sounds clearly distinct.
// C-E-G-A pattern avoids dissonant intervals; pleasant even when two robots
// are heard simultaneously.
static const uint16_t kTagNotes[16] = {
   262,  330,  392,  440,   // C4  E4  G4  A4
   523,  659,  784,  880,   // C5  E5  G5  A5
  1047, 1319, 1568, 1760,   // C6  E6  G6  A6
  2093, 2637, 3136, 3520,   // C7  E7  G7  A7
};

// Unique LED colours per tag ID — vivid, maximally distinct.
static const uint8_t kTagColors[16][3] = {
  {255,   0,   0},  //  0  red
  {255,  80,   0},  //  1  orange
  {220, 200,   0},  //  2  yellow
  { 80, 255,   0},  //  3  lime
  {  0, 200,   0},  //  4  green
  {  0, 255, 180},  //  5  cyan
  {  0, 150, 255},  //  6  sky
  {  0,   0, 255},  //  7  blue
  {100,   0, 255},  //  8  purple
  {200,   0, 255},  //  9  magenta
  {255,   0, 150},  // 10  pink
  {255, 255, 255},  // 11  white
  {200, 100,   0},  // 12  amber
  {150, 180,   0},  // 13  gold-green
  {  0, 200, 150},  // 14  teal
  {120,   0, 200},  // 15  violet
};

// ── API ───────────────────────────────────────────────────────────────────────

void DroidChime::begin(ActuatorService& act, UBaseType_t prio, BaseType_t core) {
  act_   = &act;
  queue_ = xQueueCreate(1, sizeof(uint16_t));  // uint16_t: high=Event, low=data
  if (!queue_) return;
  xTaskCreatePinnedToCoreWithCaps(taskFn, "chime", 6144, this, prio, nullptr, core,
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

void DroidChime::play(Event ev) {
  if (!queue_) return;
  stop_ = true;                              // interrupt any playing sequence
  uint16_t msg = (uint16_t)ev << 8;
  xQueueOverwrite(queue_, &msg);
}

void DroidChime::notifyTagId(uint8_t tagId) {
  if (!queue_ || tagId >= 16) return;
  const uint32_t now = millis();
  if ((now - lastTagMs_[tagId]) < TAG_ID_COOLDOWN_MS) return;
  lastTagMs_[tagId] = now;
  // Queue politely without interrupting — tag chirps are short (120 ms) and
  // drop silently if another event is already pending.
  uint16_t msg = ((uint16_t)TAG_ID << 8) | tagId;
  xQueueSend(queue_, &msg, 0);
}

void DroidChime::notifyTagFound() {
  const uint32_t now = millis();
  if ((now - lastTagFoundMs_) >= TAG_COOLDOWN_MS) {
    play(FOUND_OBJECT);
  }
  lastTagFoundMs_ = now;
}

// ── Task ─────────────────────────────────────────────────────────────────────

/*static*/ void DroidChime::taskFn(void* arg) {
  static_cast<DroidChime*>(arg)->runTask();
}

void DroidChime::runTask() {
  for (;;) {
    uint16_t msg;
    if (xQueueReceive(queue_, &msg, portMAX_DELAY) == pdTRUE) {
      stop_ = false;
      const uint8_t ev   = (uint8_t)(msg >> 8);
      const uint8_t data = (uint8_t)(msg & 0xFF);
      if (ev == TAG_ID) {
        playTagChirp_(data);
      } else if (ev < sizeof(SEQ_TABLE) / sizeof(SEQ_TABLE[0])) {
        const SeqEntry& e = SEQ_TABLE[ev];
        playSequence_(e.steps, e.n);
      }
    }
  }
}

bool DroidChime::waitMs_(uint16_t dur_ms) {
  uint32_t remaining = dur_ms;
  while (remaining > 0) {
    if (stop_) return false;
    uint16_t tmp;
    if (xQueuePeek(queue_, &tmp, 0) == pdTRUE) return false;
    const uint32_t chunk = (remaining > 10) ? 10 : remaining;
    vTaskDelay(pdMS_TO_TICKS(chunk));
    remaining -= chunk;
  }
  return true;
}

void DroidChime::playSequence_(const DroidChimeStep* seq, uint8_t n) {
  if (!act_ || !seq || n == 0) return;

  uint32_t total_ms = 0;
  for (uint8_t i = 0; i < n; i++) total_ms += seq[i].dur_ms;
  act_->lockLed(total_ms + 100);

  for (uint8_t i = 0; i < n; i++) {
    if (stop_) break;
    const DroidChimeStep& s = seq[i];
    if (s.freq > 0) act_->setBuzzerTone(s.freq, 0.35f);
    else             act_->buzzerOff();
    act_->setLed(s.r, s.g, s.b);
    if (s.dur_ms > 0 && !waitMs_(s.dur_ms)) break;
  }

  act_->buzzerOff();
}

// Two-note rising chirp with the tag's unique note and LED colour.
// Total duration ~120 ms — short enough to not be disruptive.
void DroidChime::playTagChirp_(uint8_t id) {
  if (!act_ || id >= 16) return;
  const uint16_t f1 = kTagNotes[id];
  const uint16_t f2 = (uint16_t)(f1 * 1.26f);  // major third up
  const uint8_t  r  = kTagColors[id][0];
  const uint8_t  g  = kTagColors[id][1];
  const uint8_t  b  = kTagColors[id][2];
  const DroidChimeStep seq[] = {
    { f1, (uint8_t)(r >> 1), (uint8_t)(g >> 1), (uint8_t)(b >> 1), 65 },
    { f2, r, g, b, 55 },
  };
  playSequence_(seq, 2);
}
