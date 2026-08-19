#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <string.h>

/* =========================
   WIFI AP CONFIG
========================= */
const char* ap_ssid     = "Drone";
const char* ap_password = "taisa0phaicho";

/* =========================
   SERVER + WEBSOCKET
========================= */
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

/* =========================
   UART → STM32
   TX0 (GPIO1) → STM32 PA10 (RX)
   RX0 (GPIO3) → STM32 PA9  (TX)
========================= */
#define STM32_BAUD 115200

/* =========================
   BUFFER nhận dữ liệu MPU từ STM32
========================= */
char    inputBuffer[128];
size_t bufferIndex = 0;

/* =========================
   LED STATUS
========================= */
#define WIFI_LED 2      // LED onboard (Nhấp nháy khi chờ, sáng im khi kết nối)
#define CONN_LED 16     // LED ngoại vi IO16 (Làm LED báo nguồn - Cứ có điện là sáng)

/* =========================
   CONTROL STATE -> STM32
========================= */
int  ctrlThrottle = 1000;
int  ctrlRoll     = 0;     
int  ctrlPitch    = 0;     
int  ctrlYaw      = 0;     
bool ctrlArmed    = false;
bool ctrlPidActive = true; // Biến bật tắt PID (Mặc định ON)

#define CTRL_SEND_INTERVAL_MS 20
#define CTRL_THROTTLE_STEP    5
#define CTRL_THROTTLE_MIN     1000
#define CTRL_THROTTLE_MAX     2000
#define CTRL_ANGLE_CMD        500
#define CTRL_YAW_RATE_CMD     30

static int clampInt(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void sendControlToSTM32()
{
    // Gửi gói tin 6 tham số xuống STM32: CTRL <thr> <roll> <pitch> <yaw> <arm> <pid>
    Serial.printf("CTRL %d %d %d %d %d %d\n",
                  ctrlThrottle,
                  ctrlRoll,
                  ctrlPitch,
                  ctrlYaw,
                  ctrlArmed ? 1 : 0,
                  ctrlPidActive ? 1 : 0);
}

static void stopControl()
{
    ctrlThrottle = 1000;
    ctrlRoll     = 0;
    ctrlPitch    = 0;
    ctrlYaw      = 0;
    ctrlArmed    = false;
}

static void applyCommand(const char *cmd)
{
    if (strcmp(cmd, "PID_TOGGLE") == 0) {
        ctrlPidActive = !ctrlPidActive;
        return;
    }

    ctrlRoll  = 0;
    ctrlPitch = 0;
    ctrlYaw   = 0;

    if (strcmp(cmd, "STOP") == 0) {
        stopControl();
    } else if (strcmp(cmd, "HOLD") == 0) {
        // Giữ nguyên ga, trả các trục góc về tâm
    } else if (strcmp(cmd, "T_UP") == 0) {
        ctrlArmed = true;
        ctrlThrottle = clampInt(ctrlThrottle + CTRL_THROTTLE_STEP,
                                CTRL_THROTTLE_MIN, CTRL_THROTTLE_MAX);
    } else if (strcmp(cmd, "T_DOWN") == 0) {
        ctrlThrottle = clampInt(ctrlThrottle - CTRL_THROTTLE_STEP,
                                CTRL_THROTTLE_MIN, CTRL_THROTTLE_MAX);
        if (ctrlThrottle <= CTRL_THROTTLE_MIN) ctrlArmed = false;
    } else if (strcmp(cmd, "FORWARD") == 0) {
        ctrlPitch = CTRL_ANGLE_CMD;
    } else if (strcmp(cmd, "BACK") == 0) {
        ctrlPitch = -CTRL_ANGLE_CMD;
    } else if (strcmp(cmd, "LEFT") == 0) {
        ctrlRoll = -CTRL_ANGLE_CMD;
    } else if (strcmp(cmd, "RIGHT") == 0) {
        ctrlRoll = CTRL_ANGLE_CMD;
    } else if (strcmp(cmd, "YAW_LEFT") == 0) {
        ctrlYaw = -CTRL_YAW_RATE_CMD;
    } else if (strcmp(cmd, "YAW_RIGHT") == 0) {
        ctrlYaw = CTRL_YAW_RATE_CMD;
    }
}

/* =========================
   HTML GIAO DIỆN CHUẨN RESPONSIVE
========================= */
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>Drone Control</title>
<style>
* { box-sizing: border-box; }
body {
    background: #111; color: white; text-align: center; font-family: Arial, sans-serif; margin: 0; padding: 10px;
    -webkit-user-select: none; user-select: none; 
    overflow: auto; /* Cho phép cuộn nếu màn hình quá bé */
}
h1 { font-size: 20px; margin: 5px 0; }
#status { font-size: 15px; font-weight: bold; color: red; margin-bottom: 4px; }
#ping_ms { font-size: 12px; color: #555; margin-bottom: 10px; }

/* Container linh hoạt kéo dãn */
.main-container { 
    display: flex; 
    justify-content: center; 
    align-items: center; 
    flex-wrap: wrap; 
    gap: 15px;
    max-width: 1200px;
    margin: 0 auto;
}

.joy-cluster { display: inline-block; width: 260px; max-width: 100%; }
.telemetry-cluster { 
    width: 280px; max-width: 100%; background: #1c1c1e; padding: 12px; 
    border-radius: 15px; border: 1px solid #333; box-shadow: 0 4px 15px rgba(0,0,0,0.5); 
}
.telemetry-title { font-size: 13px; color: #8e8e93; margin-bottom: 8px; text-transform: uppercase; letter-spacing: 1px; }
.data-field { font-size: 15px; font-family: 'Courier New', monospace; color: #00ff88; margin: 6px 0; text-align: left; padding-left: 10px; }

/* CSS Nút bấm - chặn touch action tại đây để tránh giật màn hình khi lái */
button { 
    width: 75px; height: 60px; margin: 5px; font-size: 13px; font-weight: bold; 
    border: none; border-radius: 12px; background: #2c2c2e; color: #0a84ff; 
    touch-action: none; cursor: pointer; 
}
button:active { background: #00aa55; color: white; }
.stop-btn { background: #ff453a; color: white; }
.stop-btn:active { background: #ff3b30; }

/* CSS cho nút PID riêng biệt */
.pid-btn {
    width: 100%; height: 42px; background: #30d158; color: white; 
    margin: 8px 0; font-size: 14px; border-radius: 8px;
}

.grid-row { display: flex; justify-content: center; }
.space-holder { width: 75px; height: 60px; margin: 5px; }

/* Tối ưu hóa khi màn hình xoay dọc (Điện thoại đứng) */
@media (max-width: 768px) {
    .main-container {
        flex-direction: column; /* Xếp dọc toàn bộ */
    }
}
</style>
</head>
<body>
<h1>🚀 Drone Control</h1>
<div id="status">CONNECTING...</div>
<div id="ping_ms">-- ms</div>
<div class="main-container">
    <div class="joy-cluster">
        <div class="grid-row">
            <button ontouchstart="event.preventDefault();hold('T_UP')" ontouchend="event.preventDefault();release('HOLD')" onmousedown="hold('T_UP')" onmouseup="release('HOLD')">&#9650; HIGH</button>
        </div>
        <div class="grid-row">
            <button ontouchstart="event.preventDefault();hold('YAW_LEFT')" ontouchend="event.preventDefault();release('HOLD')" onmousedown="hold('YAW_LEFT')" onmouseup="release('HOLD')">&#9664; YAW</button>
            <div class="space-holder"></div>
            <button ontouchstart="event.preventDefault();hold('YAW_RIGHT')" ontouchend="event.preventDefault();release('HOLD')" onmousedown="hold('YAW_RIGHT')" onmouseup="release('HOLD')">YAW &#9654;</button>
        </div>
        <div class="grid-row">
            <button ontouchstart="event.preventDefault();hold('T_DOWN')" ontouchend="event.preventDefault();release('HOLD')" onmousedown="hold('T_DOWN')" onmouseup="release('HOLD')">&#9660; LOW</button>
        </div>
    </div>
    
    <div class="telemetry-cluster"> 
        <div class="telemetry-title">MPU6050 Telemetry</div>
        <div class="data-field">Roll:  <span id="roll"> 0.00</span>&deg; (Tgt: <span id="tgt_roll">0.0</span>)</div>
        <div class="data-field">Pitch: <span id="pitch"> 0.00</span>&deg; (Tgt: <span id="tgt_pitch">0.0</span>)</div>
        <div class="data-field">Yaw:   <span id="yaw"> 0.00</span>&deg;/s (Tgt: <span id="tgt_yaw">0.0</span>)</div>
        <hr style="border-color:#333; margin:6px 0">
        <div class="data-field" style="color: #0a84ff;">Base Thr: <span id="base_thr">1000</span></div>
        
        <button id="pid_btn" class="pid-btn" onclick="sendCmd('PID_TOGGLE')">PID STATUS: ON</button>
        
        <div class="data-field" style="color: #ff9f0a; font-size: 13px;">PID Out: <span id="pid_r">0</span>|<span id="pid_p">0</span>|<span id="pid_y">0</span></div>
        <hr style="border-color:#333; margin:6px 0">
        <div class="data-field" style="font-size:13px; color:#aaa;">M1:<span id="m1">100</span> | M2:<span id="m2">100</span></div>
        <div class="data-field" style="font-size:13px; color:#aaa;">M3:<span id="m3">100</span> | M4:<span id="m4">100</span></div>
    </div>
    
    <div class="joy-cluster">
        <div class="grid-row">
            <button ontouchstart="event.preventDefault();hold('FORWARD')" ontouchend="event.preventDefault();release('HOLD')" onmousedown="hold('FORWARD')" onmouseup="release('HOLD')">&#9650; FWD</button>
        </div>
        <div class="grid-row">
            <button ontouchstart="event.preventDefault();hold('LEFT')" ontouchend="event.preventDefault();release('HOLD')" onmousedown="hold('LEFT')" onmouseup="release('HOLD')">&#9664; L</button>
            <button class="stop-btn" ontouchstart="event.preventDefault();sendCmd('STOP')" onmousedown="sendCmd('STOP')">STOP</button>
            <button ontouchstart="event.preventDefault();hold('RIGHT')" ontouchend="event.preventDefault();release('HOLD')" onmousedown="hold('RIGHT')" onmouseup="release('HOLD')">R &#9654;</button>
        </div>
        <div class="grid-row">
            <button ontouchstart="event.preventDefault();hold('BACK')" ontouchend="event.preventDefault();release('HOLD')" onmousedown="hold('BACK')" onmouseup="release('HOLD')">&#9660; BACK</button>
        </div>
    </div>
</div>
<script>
var ws = null; var holdInterval = null; var reconnectDelay = 500; var pingTimer = null; var pingTs = 0;
function connect() {
    if (ws && ws.readyState === WebSocket.OPEN) return;
    ws = new WebSocket('ws://' + location.hostname + '/ws');
    ws.onopen = function() {
        document.getElementById('status').textContent = 'CONNECTED';
        document.getElementById('status').style.color = '#00ff88';
        reconnectDelay = 500;
        clearInterval(pingTimer);
        pingTimer = setInterval(function() {
            if (ws && ws.readyState === WebSocket.OPEN) { pingTs = Date.now(); ws.send('PING'); }
        }, 150);
    };
    ws.onclose = function() {
        document.getElementById('status').textContent = 'RECONNECTING...';
        document.getElementById('status').style.color = '#ff9f0a';
        clearInterval(pingTimer);
        setTimeout(connect, reconnectDelay);
        reconnectDelay = Math.min(reconnectDelay * 2, 4000);
    };
    ws.onerror = function() { ws.close(); };
    ws.onmessage = function(event) {
        var d = event.data.trim();
        if (d === 'PONG') { document.getElementById('ping_ms').textContent = (Date.now() - pingTs) + ' ms'; return; }
        var t = d.split(/\s+/);
        if (t.length >= 14) {
            document.getElementById('roll').textContent  = parseFloat(t[0]).toFixed(2);
            document.getElementById('pitch').textContent = parseFloat(t[1]).toFixed(2);
            document.getElementById('yaw').textContent   = parseFloat(t[2]).toFixed(2);
            document.getElementById('m1').textContent = t[3]; document.getElementById('m2').textContent = t[4];
            document.getElementById('m3').textContent = t[5]; document.getElementById('m4').textContent = t[6];
            document.getElementById('base_thr').textContent = t[7];
            document.getElementById('tgt_roll').textContent  = parseFloat(t[8]).toFixed(1);
            document.getElementById('tgt_pitch').textContent = parseFloat(t[9]).toFixed(1);
            document.getElementById('tgt_yaw').textContent   = parseFloat(t[10]).toFixed(1);
            document.getElementById('pid_r').textContent = Math.round(parseFloat(t[11]));
            document.getElementById('pid_p').textContent = Math.round(parseFloat(t[12]));
            document.getElementById('pid_y').textContent = Math.round(parseFloat(t[13]));
        }
    };
}
function sendCmd(cmd) { 
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(cmd); 
        if(cmd === 'PID_TOGGLE') {
            var btn = document.getElementById('pid_btn');
            if(btn.textContent.includes("ON")) {
                btn.textContent = "PID STATUS: OFF"; btn.style.background = "#ff453a";
            } else {
                btn.textContent = "PID STATUS: ON"; btn.style.background = "#30d158";
            }
        }
    }
}
function hold(cmd) { clearInterval(holdInterval); sendCmd(cmd); holdInterval = setInterval(function() { sendCmd(cmd); }, 100); }
function release(cmd) { clearInterval(holdInterval); holdInterval = null; sendCmd(cmd); }
document.addEventListener('visibilitychange', function() { if (!document.hidden && (!ws || ws.readyState !== WebSocket.OPEN)) { reconnectDelay = 500; connect(); } });
window.oncontextmenu = function(e) { e.preventDefault(); return false; };
connect();
</script>
</body>
</html>
)rawliteral";

/* =========================
   WEBSOCKET EVENT HANDLER
========================= */
void onWebSocketEvent(AsyncWebSocket *server,
                      AsyncWebSocketClient *client,
                      AwsEventType type,
                      void *arg,
                      uint8_t *data,
                      size_t len)
{
    if (type == WS_EVT_DATA)
    {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (!info->final || info->index != 0 || info->len != len) return;
        if (info->opcode != WS_TEXT) return;
        if (len >= 32) return;

        char cmd[32];
        memcpy(cmd, data, len);
        cmd[len] = '\0';

        if (strcmp(cmd, "PING") == 0)
        {
            client->text("PONG");
            return;
        }

        applyCommand(cmd);
    }
}

/* =========================
   SETUP
========================= */
void setup()
{
    Serial.begin(STM32_BAUD);

    pinMode(WIFI_LED, OUTPUT);
    digitalWrite(WIFI_LED, LOW);

    // CẤP NGUỒN LÀ SÁNG: LED ngoài IO16 bật ngay lập tức
    pinMode(CONN_LED, OUTPUT);
    digitalWrite(CONN_LED, HIGH);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, ap_password, 6, 0, 4);

    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", index_html);
    });
    server.begin();

    digitalWrite(WIFI_LED, HIGH);
}

/* =========================
   LOOP
========================= */
void loop()
{
    unsigned long now = millis();

    // Logic LED trạng thái onboard (WIFI_LED)
    {
        static unsigned long lastBlink = 0;
        static bool ledState = false;
        if (ws.count() > 0) {
            digitalWrite(WIFI_LED, HIGH); 
        } else {
            if (now - lastBlink > 400) {
                ledState = !ledState;
                digitalWrite(WIFI_LED, ledState); 
                lastBlink = now;
            }
        }
    }

    // Gửi dữ liệu điều khiển xuống STM32 liên tục
    {
        static unsigned long lastCtrlSend = 0;
        if (now - lastCtrlSend >= CTRL_SEND_INTERVAL_MS) {
            sendControlToSTM32();
            lastCtrlSend = now;
        }
    }

    // ĐỌC DỮ LIỆU MPU TỪ STM32
    while (Serial.available())
    {
        char c = (char)Serial.read();
        if (c == '\n')
        {
            inputBuffer[bufferIndex] = '\0';
            if (bufferIndex > 0 && ws.count() > 0)
                ws.textAll(inputBuffer);
            bufferIndex = 0;
        }
        else if (c != '\r')
        {
            if (bufferIndex < sizeof(inputBuffer) - 1)
                inputBuffer[bufferIndex++] = c;
            else
            {
                inputBuffer[bufferIndex] = '\0';
                if (ws.count() > 0) ws.textAll(inputBuffer);
                bufferIndex = 0;
            }
        }
    }

    // Dọn dẹp WebSocket định kỳ
    {
        static unsigned long lastCleanup = 0;
        if (now - lastCleanup > 1000) {
            ws.cleanupClients();
            lastCleanup = now;
        }
    }
}
