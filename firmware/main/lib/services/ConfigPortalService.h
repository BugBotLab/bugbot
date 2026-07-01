// ConfigPortalService.h
// Attaches HTTP routes to WebWS for a browser-accessible runtime configuration
// portal. GET /config returns JSON or an HTML form; POST /config applies changes
// and saves them to LittleFS via ConfigService.
#pragma once
#include <Arduino.h>
#include "../config/AppConfig.h"

class WebWS;
class ConfigService;

class ConfigPortalService {
public:
  void attach(WebWS& webws, ConfigService& configSvc, RuntimeConfig& runtimeCfg);

private:
  static String jsonEscape_(const String& s);
  static String htmlEscape_(const String& s);
  static bool getBoolParam_(class AsyncWebServerRequest* req, const char* name, bool defaultVal);
  static String getStringParam_(class AsyncWebServerRequest* req, const char* name, const String& defaultVal);
  static uint32_t getUIntParam_(class AsyncWebServerRequest* req, const char* name, uint32_t defaultVal);
  static float getFloatParam_(class AsyncWebServerRequest* req, const char* name, float defaultVal);
  static String buildJson_(const RuntimeConfig& cfg, ConfigService& svc);
  static String buildHtml_(const RuntimeConfig& cfg);
  static void scheduleRestart_();
};
