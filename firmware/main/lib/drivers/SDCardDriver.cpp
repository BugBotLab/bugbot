#include "SDCardDriver.h"

bool SDCardDriver::begin() {
  mounted_ = false;
  cardSizeBytes_ = 0;

  if (!SD.begin()) {
    Serial.println("[SD] Mount failed");
    return false;
  }

  if (SD.cardType() == CARD_NONE) {
    Serial.println("[SD] No card attached");
    return false;
  }

  mounted_ = true;
  cardSizeBytes_ = SD.cardSize();
  Serial.printf("[SD] Mounted OK (%llu MB)\n", cardSizeBytes_ / (1024ULL * 1024ULL));
  return true;
}

void SDCardDriver::end() {
  mounted_ = false;
  cardSizeBytes_ = 0;
  SD.end();
}

bool SDCardDriver::exists(const char* path) const {
  if (!mounted_ || !path) return false;
  return SD.exists(path);
}

File SDCardDriver::open(const char* path, const char* mode) const {
  if (!mounted_ || !path) return File();
  return SD.open(path, mode);
}
