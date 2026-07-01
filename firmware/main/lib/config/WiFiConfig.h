// WiFiConfig.h
// Compile-time Wi-Fi credentials and network topology constants (BugBotNet namespace).
//
// The credentials below are PLACEHOLDERS. Set your own before flashing, or leave the
// STA fields blank and configure Wi-Fi at runtime via AppConfig.h / /config/wifi.cfg.
// Do NOT commit real credentials to a public repository.
#pragma once
#include <Arduino.h>
#include <IPAddress.h>

namespace BugBotNet {

// ---- Station (join existing Wi-Fi) ----
static const bool USE_STA = true;
static const char* WIFI_SSID = "";            // e.g. "my-network" (or set at runtime)
static const char* WIFI_PASS = "";            // e.g. "my-password" (or set at runtime)
static const bool WIFI_SLEEP = false;     // false = lower jitter

// Optional static IP for STA
static const bool USE_STATIC_IP = false;
static const IPAddress STATIC_IP(192,168,1,42);
static const IPAddress GATEWAY  (192,168,1,1);
static const IPAddress SUBNET   (255,255,255,0);
static const IPAddress DNS1     (8,8,8,8);

// Fixed STA target (e.g., the host PC receiving pose telemetry)
static const IPAddress REMOTE_IP(192,168,1,100);
static const uint16_t  UDP_PORT = 45100;      // TX pose

// ---- Access Point (create Wi-Fi) ----
static const bool USE_AP = true;
static const char* AP_SSID = "BugBot";
static const char* AP_PASS = "changeme123";   // >=8 chars if not open; change before deployment

// Optional AP IP/net (defaults to 192.168.4.1/24 if disabled)
static const bool AP_USE_STATIC_IP = false;
static const IPAddress AP_IP      (192,168,4,1);
static const IPAddress AP_GATEWAY (192,168,4,1);
static const IPAddress AP_SUBNET  (255,255,255,0);

// Broadcast pose on AP? (in addition to registered clients)
static const bool AP_BROADCAST    = false;    // set true to also send to 192.168.4.255

// ---- Client registration (AP & STA) ----
static const uint16_t REG_PORT    = 45102;    // clients send any packet here to register
static const uint32_t CLIENT_TTL_MS = 10000;  // drop if not seen for 10 s

// ---- mDNS (optional) ----
static const bool USE_MDNS = false;
static const char* MDNS_NAME = "bugbot";

} // namespace
