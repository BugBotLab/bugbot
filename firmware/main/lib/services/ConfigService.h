// ConfigService.h
// Loads and saves RuntimeConfig from LittleFS key-value files
// (/config/wifi.cfg, motion.cfg, system.cfg, arena.cfg). Mounts LittleFS
// on first beginAndLoad(); writes defaults if files are absent.
#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include "../config/AppConfig.h"

class ConfigService {
public:
  bool beginAndLoad(RuntimeConfig& cfg);
  bool reload(RuntimeConfig& cfg);
  bool saveAll(const RuntimeConfig& cfg);
  bool ensureDefaultsExist();

  // Set a single config key by name (same keys as the .cfg files).
  // Returns true if the key was recognised and the field updated.
  bool setKey(const String& key, const String& value, RuntimeConfig& cfg);

  bool fsMounted() const { return fsMounted_; }
  size_t totalBytes() const;
  size_t usedBytes() const;

private:
  bool beginFs_();
  bool loadAll_(RuntimeConfig& cfg);
  bool loadKeyValueFile_(const char* path,
                         void (*handler)(const String&, const String&, RuntimeConfig&),
                         RuntimeConfig& cfg);
  bool saveText_(const char* path, const String& content);

  static void handleWifi_(const String& key, const String& value, RuntimeConfig& cfg);
  static void handleMotion_(const String& key, const String& value, RuntimeConfig& cfg);
  static void handleSystem_(const String& key, const String& value, RuntimeConfig& cfg);
  static void handleArena_(const String& key, const String& value, RuntimeConfig& cfg);

  static String trimCopy_(const String& s);
  static bool toBool_(const String& s, bool defaultValue);

  bool fsMounted_ = false;
};
