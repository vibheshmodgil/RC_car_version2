#include "AppServer.h"
#include "WebUI.h"

// Channel keys used by the REST API + UI, index-aligned with PINS_MOTOR /
// PINS_ENCODER: 0=lf, 1=lr, 2=rf, 3=rr.
static const char* MOTOR_KEYS[NUM_MOTORS] = { "lf", "lr", "rf", "rr" };

AppServer::AppServer(MotorChannel* const (&motors)[NUM_MOTORS], EncoderReader& enc, Imu& imu)
    : _enc(enc), _imu(imu) {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) _m[i] = motors[i];
}

void AppServer::begin() {
#if WIFI_STATION_MODE
    // Pi-centric phase: join the Pi's AP as a station with a static IP.
    // Non-blocking — the HTTP server starts immediately and answers as
    // soon as the association completes.
    WiFi.mode(WIFI_STA);
    IPAddress ip, gw, sn;
    ip.fromString(STA_STATIC_IP);
    gw.fromString(STA_GATEWAY);
    sn.fromString(STA_SUBNET);
    WiFi.config(ip, gw, sn);
    WiFi.setAutoReconnect(true);
    WiFi.begin(AP_SSID, AP_PASS);
    Serial.printf("STA mode: joining '%s' as %s (fallback UI stays at that IP)\n",
                  AP_SSID, STA_STATIC_IP);
#else
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("AP started. Connect to '");
    Serial.print(AP_SSID);
    Serial.print("' then browse to http://");
    Serial.println(WiFi.softAPIP());
#endif

    // Restore saved CPR and per-wheel direction-invert flags.
    _prefs.begin("bench", true);
    _enc.setCPR(_prefs.getFloat("cpr", DEFAULT_CPR));
    uint8_t invMask = _prefs.getUChar("invMask", 0);
    _prefs.end();
    for (uint8_t i = 0; i < NUM_MOTORS; i++) _m[i]->setInverted(invMask & (1 << i));

    _ws.onEvent([](AsyncWebSocket*, AsyncWebSocketClient* c, AwsEventType t, void*, uint8_t*, size_t) {
        if (t == WS_EVT_CONNECT) Serial.printf("WS client %u connected\n", c->id());
    });
    _server.addHandler(&_ws);

    routes();
    _server.begin();
}

// --------------------------------------------------------------- helpers
MotorChannel* AppServer::channel(const String& name) {
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
        if (name == MOTOR_KEYS[i]) return _m[i];
    return nullptr;
}

void AppServer::emergencyStop() {
    for (auto* m : _m) m->emergencyStop();   // all disarmed => STBY drops (hardware cut)
}

void AppServer::clearEmergencyStop() {
    for (auto* m : _m) m->clearEmergencyStop();
}

// ---------------------------------------------------------------- routes
void AppServer::routes() {
    // ---- Pages (one URL per subsystem) ----
    _server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(200, "text/html", web::page("Overview", "home", web::HOME_BODY(), web::HOME_JS()));
    });
    _server.on("/motors", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(200, "text/html", web::page("Motors", "motors", web::MOTORS_BODY(), web::MOTORS_JS()));
    });
    _server.on("/drive", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(200, "text/html", web::page("Drive", "drive", web::DRIVE_BODY(), web::DRIVE_JS()));
    });
    _server.on("/encoders", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(200, "text/html", web::page("Encoders", "encoders", web::ENCODERS_BODY(), web::ENCODERS_JS()));
    });
    _server.on("/imu", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(200, "text/html", web::page("IMU", "imu", web::IMU_BODY(), web::IMU_JS()));
    });
    _server.on("/camera", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(200, "text/html", web::page("Camera", "camera", web::CAMERA_BODY(), web::CAMERA_JS().c_str()));
    });

    // ---- Motor control (ch = lf|lr|rf|rr|all) ----
    _server.on("/api/motor", HTTP_POST, [this](AsyncWebServerRequest* r) {
        String ch = r->hasParam("ch") ? r->getParam("ch")->value() : "";
        Serial.printf("HTTP /api/motor ch=%s arm=%s pwm=%s mode=%s inv=%s\n",
            ch.length()         ? ch.c_str()                              : "-",
            r->hasParam("arm")  ? r->getParam("arm")->value().c_str()  : "-",
            r->hasParam("pwm")  ? r->getParam("pwm")->value().c_str()  : "-",
            r->hasParam("mode") ? r->getParam("mode")->value().c_str() : "-",
            r->hasParam("inv")  ? r->getParam("inv")->value().c_str()  : "-");

        MotorChannel* targets[NUM_MOTORS];
        uint8_t n = 0;
        if (ch == "all")                  { for (auto* m : _m) targets[n++] = m; }
        else if (MotorChannel* m = channel(ch)) { targets[n++] = m; }
        else { r->send(400, "text/plain", "bad ch"); return; }

        for (uint8_t i = 0; i < n; i++) {
            MotorChannel* m = targets[i];
            if (r->hasParam("arm")) {
                if (r->getParam("arm")->value().toInt() == 1) m->arm();
                else                                          m->disarm();
            }
            else if (r->hasParam("pwm")) {
                m->setPWM(r->getParam("pwm")->value().toInt());
                _lastMoveCmdMs = millis();   // feeds the drive deadman
            }
            else if (r->hasParam("inv")) m->setInverted(r->getParam("inv")->value().toInt() == 1);
            else if (r->hasParam("mode")) {
                String mode = r->getParam("mode")->value();
                if (mode == "brake")       m->brake();
                else if (mode == "disarm") m->disarm();
                else                       m->coast();
            }
        }

        // Persist invert flags so wiring fixes survive reboot.
        if (r->hasParam("inv")) {
            uint8_t invMask = 0;
            for (uint8_t i = 0; i < NUM_MOTORS; i++)
                if (_m[i]->isInverted()) invMask |= 1 << i;
            _prefs.begin("bench", false);
            _prefs.putUChar("invMask", invMask);
            _prefs.end();
        }
        r->send(200, "text/plain", "OK");
    });

    // ---- Drive (whole-car motion: all four wheels in one request) ----
    // dir = fwd|rev|left|right|stop, pwm = 0..255 magnitude.
    // left/right are in-place spins: sides run in opposite directions.
    // Wheels must already be armed; setPWM() ignores disarmed channels.
    _server.on("/api/drive", HTTP_POST, [this](AsyncWebServerRequest* r) {
        String dir = r->hasParam("dir") ? r->getParam("dir")->value() : "";
        int16_t pwm = r->hasParam("pwm")
            ? constrain((int16_t)r->getParam("pwm")->value().toInt(), (int16_t)0, PWM_MAX) : 0;
        Serial.printf("HTTP /api/drive dir=%s pwm=%d\n",
            dir.length() ? dir.c_str() : "-", pwm);

        int8_t left, right;   // per-side sign; _m order is LF,LR,RF,RR
        if      (dir == "fwd")   { left = +1; right = +1; }
        else if (dir == "rev")   { left = -1; right = -1; }
        else if (dir == "left")  { left = -1; right = +1; }   // spin CCW
        else if (dir == "right") { left = +1; right = -1; }   // spin CW
        else if (dir == "stop")  { left =  0; right =  0; }
        else { r->send(400, "text/plain", "bad dir"); return; }

        _m[0]->setPWM(left  * pwm);
        _m[1]->setPWM(left  * pwm);
        _m[2]->setPWM(right * pwm);
        _m[3]->setPWM(right * pwm);
        _lastMoveCmdMs = millis();   // feeds the drive deadman
        r->send(200, "text/plain", "OK");
    });

    // ---- Safety ----
    _server.on("/api/estop", HTTP_POST, [this](AsyncWebServerRequest* r) {
        Serial.println("HTTP /api/estop  (EMERGENCY STOP pressed)");
        emergencyStop(); r->send(200, "text/plain", "ESTOP");
    });
    _server.on("/api/estop/clear", HTTP_POST, [this](AsyncWebServerRequest* r) {
        Serial.println("HTTP /api/estop/clear");
        clearEmergencyStop(); r->send(200, "text/plain", "OK");
    });

    // ---- Encoders ----
    _server.on("/api/encoder/reset", HTTP_POST, [this](AsyncWebServerRequest* r) {
        String ch = r->hasParam("ch") ? r->getParam("ch")->value() : "all";
        if (ch == "all") _enc.resetAll();
        else             _enc.resetCount(ch.toInt());
        r->send(200, "text/plain", "OK");
    });
    _server.on("/api/encoder/cpr", HTTP_GET, [this](AsyncWebServerRequest* r) {
        JsonDocument d; d["cpr"] = _enc.cpr();
        String out; serializeJson(d, out);
        r->send(200, "application/json", out);
    });
    _server.addHandler(new AsyncCallbackJsonWebHandler("/api/encoder/cpr",
        [this](AsyncWebServerRequest* r, JsonVariant& json) {
            float cpr = json["cpr"] | DEFAULT_CPR;
            _enc.setCPR(cpr);
            _prefs.begin("bench", false);
            _prefs.putFloat("cpr", _enc.cpr());
            _prefs.end();
            r->send(200, "text/plain", "OK");
        }));

    _server.onNotFound([](AsyncWebServerRequest* r) { r->send(404, "text/plain", "not found"); });
}

// ------------------------------------------------------------ telemetry
void AppServer::broadcast() {
    if (_ws.count() == 0) return;

    JsonDocument d;
    d["up"] = millis() / 1000;

    bool estop = false;
    JsonArray m = d["m"].to<JsonArray>();     // applied PWM per wheel (LF,LR,RF,RR)
    JsonArray a = d["a"].to<JsonArray>();     // armed flag per wheel
    JsonArray inv = d["inv"].to<JsonArray>(); // direction-invert flag per wheel
    for (auto* mc : _m) {
        estop |= mc->isEStopped();
        m.add(mc->currentPWM());
        a.add(mc->isArmed());
        inv.add(mc->isInverted());
    }
    d["estop"] = estop;

    JsonArray enc = d["enc"].to<JsonArray>();
    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
        JsonObject e = enc.add<JsonObject>();
        e["c"] = _enc.count(i);
        e["r"] = _enc.rpm(i);
    }

    JsonObject im = d["imu"].to<JsonObject>();
    im["en"] = _imu.enabled();
    Imu::Reading rd = _imu.reading();
    im["ok"] = rd.ok;
    im["h"]  = rd.heading;
    im["r"]  = rd.roll;
    im["p"]  = rd.pitch;
    im["cs"] = rd.calSys;
    im["cg"] = rd.calGyro;
    im["ca"] = rd.calAccel;
    im["cm"] = rd.calMag;

    String out; serializeJson(d, out);
    _ws.textAll(out);
}

void AppServer::loop() {
    uint32_t now = millis();

#if WIFI_STATION_MODE
    // Auto-reconnect watchdog. setAutoReconnect() covers momentary drops;
    // this poll also recovers the boot-order case where the Pi's AP was
    // not up yet when WiFi.begin() first ran.
    if (now - _lastWifiCheck >= STA_RECONNECT_MS) {
        _lastWifiCheck = now;
        bool up = WiFi.status() == WL_CONNECTED;
        if (up && !_wifiWasUp)
            Serial.printf("WiFi connected: http://%s\n", WiFi.localIP().toString().c_str());
        if (!up) {
            Serial.println("WiFi down, retrying...");
            WiFi.reconnect();
        }
        _wifiWasUp = up;
    }
#endif

    // Drive deadman: a wheel may only keep spinning while pwm/drive commands
    // keep arriving (both UIs re-send every 150 ms while held). Covers a
    // crashed browser, a dead Pi, or a dropped WiFi link mid-drive. Coast
    // only — wheels stay armed, so the next command drives again.
    if (now - _lastMoveCmdMs > DRIVE_DEADMAN_MS) {
        bool moving = false;
        for (auto* m : _m) moving |= m->currentPWM() != 0;
        if (moving) {
            for (auto* m : _m) m->coast();
            Serial.printf("DEADMAN: no drive command for %lu ms, coasting all wheels\n",
                          (unsigned long)(now - _lastMoveCmdMs));
        }
    }

    if (now - _lastBroadcast >= WS_BROADCAST_MS) {
        _lastBroadcast = now;
        _ws.cleanupClients();
        broadcast();
    }
}
