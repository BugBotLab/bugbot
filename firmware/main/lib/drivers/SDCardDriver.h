// SDCardDriver.h
// SD card driver wrapping the Arduino SD library. Handles mount/unmount, reports
// card size, and exposes file exists/open. Optional — only used when logToSd is set.
#pragma once
#include <Arduino.h>
#include <FS.h>
#include <SD.h>

class SDCardDriver {
public:
  bool begin();
  void end();

  bool mounted() const { return mounted_; }
  uint64_t cardSizeBytes() const { return cardSizeBytes_; }

  bool exists(const char* path) const;
  File open(const char* path, const char* mode = FILE_READ) const;

private:
  bool mounted_ = false;
  uint64_t cardSizeBytes_ = 0;
};
