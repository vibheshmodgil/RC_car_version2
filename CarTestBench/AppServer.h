#pragma once
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "MotorChannel.h"
#include "EncoderReader.h"
#include "Imu.h"

// Owns the WiFi access point, the HTTP pages + REST API, and the WebSocket
// telemetry broadcaster. Subsystems are passed in by reference so the server
// stays a thin transport layer over them.
class AppServer {
public:
    AppServer(MotorChannel* const (&motors)[NUM_MOTORS], EncoderReader& enc, Imu& imu);

    void begin();
    void loop();        // call from loop(): pushes telemetry at WS_BROADCAST_MS

private:
    void  routes();
    void  emergencyStop();
    void  clearEmergencyStop();
    MotorChannel* channel(const String& name);   // "lf"/"lr"/"rf"/"rr" -> instance
    void  broadcast();

    MotorChannel*  _m[NUM_MOTORS];
    EncoderReader& _enc;
    Imu&           _imu;

    AsyncWebServer _server{80};
    AsyncWebSocket _ws{"/ws"};
    Preferences    _prefs;
    uint32_t       _lastBroadcast = 0;
    uint32_t       _lastWifiCheck = 0;   // STA reconnect watchdog
    bool           _wifiWasUp     = false;
};
