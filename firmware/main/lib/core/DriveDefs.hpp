#pragma once
#include <stdint.h>

// Single source of truth for drive directions.
// Numeric values must match the Python sender.
enum class DriveDir : uint8_t {
  Stop = 0,
  Fwd,
  Back,
  StrafeL,
  StrafeR,
  TurnL,
  TurnR
};
