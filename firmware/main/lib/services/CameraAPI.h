// lib/services/CameraAPI.h
#pragma once
#include <Arduino.h>
bool CameraAPI_begin(uint16_t http_port = 81);  // returns true on success
void CameraAPI_end();
