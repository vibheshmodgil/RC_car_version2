// =====================================================================
//  CamStreamer.ino  —  ESP32-CAM firmware for the RC Car Test Bench
//
//  THIS SKETCH GOES ON THE ESP32-CAM BOARD (AI-Thinker), NOT the DevKit.
//  The DevKit keeps running CarTestBench/CarTestBench.ino unchanged roles:
//  motors + encoders. This board joins the home WiFi router as a station
//  with a static IP and serves the video itself — the DevKit never
//  touches video.
//
//  Endpoints (from a phone/Pi on the home WiFi):
//      http://192.168.1.52:81/stream    MJPEG live stream (used by /camera page)
//      http://192.168.1.52/capture      single JPEG snapshot
//      http://192.168.1.52/control?var=framesize|quality|vflip|hmirror&val=n
//      http://192.168.1.52/             tiny status page
//
//  Build/flash (Arduino IDE):
//    * Board: "AI Thinker ESP32-CAM"  (or "ESP32 Dev Module" + PSRAM enabled
//      + partition scheme "Huge APP").
//    * No USB on this board: wire a USB-serial adapter to U0T/U0R + GND,
//      tie GPIO0 to GND, press RST, upload, then remove the GPIO0 strap
//      and press RST again.
//    * Power from 5 V >= 2 A with a fat (470-1000 uF) cap near the board;
//      brownouts during WiFi TX are the #1 cause of random reboots.
//
//  Keep AP_SSID/AP_PASS/CAM_IP in sync with CarTestBench/config.h
//  (AP_SSID, AP_PASS, CAM_HOST). Duplicated here because Arduino sketches
//  cannot include headers from a sibling sketch folder.
// =====================================================================
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// ---- Must match CarTestBench/config.h ----
static const char* AP_SSID = "Airtel_kuma_9602";
static const char* AP_PASS = "air71417";
static const IPAddress CAM_IP(192, 168, 1, 52);   // = CAM_HOST
static const IPAddress GATEWAY(192, 168, 1, 1);   // home WiFi router
static const IPAddress SUBNET(255, 255, 255, 0);

// ---- AI-Thinker ESP32-CAM pin map (OV2640) ----
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM   0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM     5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22
#define FLASH_LED_PIN   4    // onboard flash LED; kept off (heat + glare)

static httpd_handle_t web_httpd    = NULL;   // :80  / , /capture, /control, /status
static httpd_handle_t stream_httpd = NULL;   // :81  /stream

// Stream statistics for /status. The camera page polls them once a second
// and derives FPS / bitrate from the deltas.
static volatile uint32_t s_frames    = 0;    // total frames sent to stream clients
static volatile uint64_t s_bytes     = 0;    // total JPEG bytes sent
static volatile bool     s_streaming = false;

// ------------------------------------------------------------- handlers
static esp_err_t index_handler(httpd_req_t* req) {
    static const char html[] =
        "<html><body style='font-family:sans-serif;background:#111;color:#eee'>"
        "<h2>RC Car CamStreamer</h2>"
        "<p><a style='color:#4d8dff' href=':81/stream'>MJPEG stream (port 81)</a> &middot; "
        "<a style='color:#4d8dff' href='/capture'>snapshot</a></p>"
        "<p>Open the test bench page at http://192.168.1.50/camera for the full UI.</p>"
        "</body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t capture_handler(httpd_req_t* req) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "capture failed");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

// multipart/x-mixed-replace: one JPEG per part, forever, one client at a time.
static esp_err_t stream_handler(httpd_req_t* req) {
    static const char* BOUNDARY = "\r\n--frame\r\n";
    static const char* PART     = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
    char part_buf[64];

    esp_err_t res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    s_streaming = true;
    while (true) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) { res = ESP_FAIL; break; }
        size_t hlen = snprintf(part_buf, sizeof(part_buf), PART, fb->len);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, BOUNDARY, strlen(BOUNDARY));
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
        if (res == ESP_OK) { s_frames++; s_bytes += fb->len; }
        esp_camera_fb_return(fb);
        if (res != ESP_OK) break;   // client disconnected
    }
    s_streaming = false;
    return res;
}

// Cumulative counters + device clock; the client turns deltas into rates.
static esp_err_t status_handler(httpd_req_t* req) {
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "{\"frames\":%lu,\"bytes\":%llu,\"ms\":%lu,\"streaming\":%s}",
        (unsigned long)s_frames, (unsigned long long)s_bytes,
        (unsigned long)millis(), s_streaming ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, len);
}

static esp_err_t control_handler(httpd_req_t* req) {
    char query[64] = {0}, var[16] = {0}, val[8] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "var", var, sizeof(var)) != ESP_OK ||
        httpd_query_key_value(query, "val", val, sizeof(val)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need var & val");
        return ESP_FAIL;
    }
    int v = atoi(val);
    sensor_t* s = esp_camera_sensor_get();
    int res = -1;
    if      (!strcmp(var, "framesize")) res = s->set_framesize(s, (framesize_t)v);
    else if (!strcmp(var, "quality"))   res = s->set_quality(s, v);
    else if (!strcmp(var, "vflip"))     res = s->set_vflip(s, v);
    else if (!strcmp(var, "hmirror"))   res = s->set_hmirror(s, v);
    if (res != 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad var/val");
        return ESP_FAIL;
    }
    Serial.printf("control %s=%d\n", var, v);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

// -------------------------------------------------------------- startup
static void startServers() {
    httpd_uri_t idx  = { "/",        HTTP_GET, index_handler,   NULL };
    httpd_uri_t cap  = { "/capture", HTTP_GET, capture_handler, NULL };
    httpd_uri_t ctl  = { "/control", HTTP_GET, control_handler, NULL };
    httpd_uri_t stat = { "/status",  HTTP_GET, status_handler,  NULL };
    httpd_uri_t strm = { "/stream",  HTTP_GET, stream_handler,  NULL };

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    if (httpd_start(&web_httpd, &cfg) == ESP_OK) {
        httpd_register_uri_handler(web_httpd, &idx);
        httpd_register_uri_handler(web_httpd, &cap);
        httpd_register_uri_handler(web_httpd, &ctl);
        httpd_register_uri_handler(web_httpd, &stat);
    }

    // Stream gets its own server (port 81) so a stalled stream client can
    // never block snapshots/control on port 80.
    cfg.server_port = 81;
    cfg.ctrl_port  += 1;
    if (httpd_start(&stream_httpd, &cfg) == ESP_OK)
        httpd_register_uri_handler(stream_httpd, &strm);
}

static bool initCamera() {
    camera_config_t c = {};
    c.ledc_channel = LEDC_CHANNEL_0;
    c.ledc_timer   = LEDC_TIMER_0;
    c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
    c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
    c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
    c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
    c.pin_xclk  = XCLK_GPIO_NUM;
    c.pin_pclk  = PCLK_GPIO_NUM;
    c.pin_vsync = VSYNC_GPIO_NUM;
    c.pin_href  = HREF_GPIO_NUM;
    c.pin_sccb_sda = SIOD_GPIO_NUM;
    c.pin_sccb_scl = SIOC_GPIO_NUM;
    c.pin_pwdn  = PWDN_GPIO_NUM;
    c.pin_reset = RESET_GPIO_NUM;
    c.xclk_freq_hz = 20000000;
    c.pixel_format = PIXFORMAT_JPEG;

    if (psramFound()) {                       // normal AI-Thinker case
        c.frame_size  = FRAMESIZE_VGA;        // 640x480 default; UI can change it
        c.jpeg_quality = 12;                  // lower number = better quality
        c.fb_count    = 2;
        c.fb_location = CAMERA_FB_IN_PSRAM;
        c.grab_mode   = CAMERA_GRAB_LATEST;   // stream shows freshest frame
    } else {                                  // fallback: no PSRAM detected
        c.frame_size  = FRAMESIZE_QVGA;
        c.jpeg_quality = 14;
        c.fb_count    = 1;
        c.fb_location = CAMERA_FB_IN_DRAM;
        c.grab_mode   = CAMERA_GRAB_WHEN_EMPTY;
    }

    esp_err_t err = esp_camera_init(&c);
    if (err != ESP_OK) {
        Serial.printf("Camera init FAILED 0x%x (check ribbon cable seating)\n", err);
        return false;
    }
    return true;
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== RC Car CamStreamer ===");

    pinMode(FLASH_LED_PIN, OUTPUT);
    digitalWrite(FLASH_LED_PIN, LOW);

    if (!initCamera()) {
        delay(5000);
        ESP.restart();      // ribbon glitches often clear on a power cycle
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);   // modem sleep causes stream stutter
    WiFi.config(CAM_IP, GATEWAY, SUBNET);
    WiFi.begin(AP_SSID, AP_PASS);
    Serial.printf("Joining '%s'", AP_SSID);
    uint32_t attemptStart = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        Serial.print('.');
        if (millis() - attemptStart > 15000) {   // AP may not be up yet; retry
            Serial.println(" retry");
            WiFi.disconnect();
            WiFi.begin(AP_SSID, AP_PASS);
            attemptStart = millis();
        }
    }
    Serial.printf("\nConnected. Stream: http://%s:81/stream\n",
                  WiFi.localIP().toString().c_str());

    startServers();
    Serial.println("Ready.");
}

void loop() {
    // HTTP is served by esp_http_server tasks; just babysit the WiFi link.
    static uint32_t lastCheck = 0;
    if (millis() - lastCheck >= 5000) {
        lastCheck = millis();
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi lost - reconnecting");
            WiFi.reconnect();
        }
    }
    delay(50);
}
