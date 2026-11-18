// Luka's McLaren F1 ECU, stage 1a: rear drive, tail light, pedal + dash switches, phone control.
// Board: ESP32-S3 DevKitC-1 or a classic ESP32 DevKit (pins picked automatically). Arduino-ESP32 core 2.x or 3.x.
//
// The ESP32 makes its own Wi-Fi network. Join it from your phone and open http://192.168.4.1.
// Luka drives with the pedal; the Fwd/Rev and High/Low switches work like stock.
// Your phone overrides him: hold the throttle pad to drive, E-STOP to stop everything.
// By default the pedal only works while your phone page is open (PEDAL_NEEDS_PHONE).
// Steering stays manual (Luka steers) until the second driver arrives (stage 1).

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copy secrets.example.h to secrets.h and set your own Wi-Fi password (secrets.h is gitignored)."
#endif

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// Garage mode: define HOME_WIFI_SSID in secrets.h and the car also joins your home Wi-Fi, so new
// firmware can be sent over the air (no laptop at the car) and http://mclaren.local/status shows
// live state. Driving still only works from the car's own Wi-Fi network.
#if defined(HOME_WIFI_SSID)
#include <ArduinoOTA.h>
#define GARAGE_MODE 1
#if !defined(OTA_PASSWORD)
#error "Garage mode needs OTA_PASSWORD in secrets.h"
#endif
#else
#define GARAGE_MODE 0
#endif

#define FW_VERSION "stage1a " __DATE__ " " __TIME__  // shows on /status, so you can tell which build is running

// ---------------- pins (match the wiring tutorial) ----------------
// One sketch, two boards: the right map is picked from the board you select in the IDE.
#if defined(CONFIG_IDF_TARGET_ESP32S3)        // ESP32-S3 DevKitC-1 (the long board)
const int PIN_REAR_RPWM = 4;   // forward PWM  -> BTS7960 RPWM
const int PIN_REAR_LPWM = 5;   // reverse PWM  -> BTS7960 LPWM
const int PIN_REAR_EN   = 6;   // R_EN + L_EN, 10k pull-down to GND on the driver side
const int PIN_MAX_DIN   = 11;  // MAX7219 DIN
const int PIN_MAX_CLK   = 12;  // MAX7219 CLK
const int PIN_MAX_CS    = 10;  // MAX7219 CS/LOAD
const int PIN_PEDAL     = 13;  // pedal switch to GND
const int PIN_FWD_REV   = 14;  // Fwd/Rev switch to GND
const int PIN_HIGH_LOW  = 17;  // High/Low switch to GND
const int PIN_MANUAL_RC = 18;  // Manual/RC switch to GND (leave unwired if the car has none)
#define BOARD_NAME "ESP32-S3"
#elif defined(CONFIG_IDF_TARGET_ESP32)        // classic ESP32 DevKit (WROOM-32, DOIT V1 and clones)
const int PIN_REAR_RPWM = 16;  // labeled RX2 on some 30-pin boards
const int PIN_REAR_LPWM = 17;  // labeled TX2 on some 30-pin boards
const int PIN_REAR_EN   = 4;
const int PIN_MAX_DIN   = 23;
const int PIN_MAX_CLK   = 18;
const int PIN_MAX_CS    = 5;
const int PIN_PEDAL     = 32;
const int PIN_FWD_REV   = 33;
const int PIN_HIGH_LOW  = 19;
const int PIN_MANUAL_RC = 21;
#define BOARD_NAME "classic ESP32"
#else
#error "Pin map is for ESP32-S3 or classic ESP32. Pick one of those boards in the IDE."
#endif

// ---------------- tuning ----------------
const bool  INVERT_DRIVE     = false;  // both wheels spin backward on "forward"? set true
const float DEFAULT_MAX_DUTY = 0.30f;  // speed cap until you trust it (phone slider can change it, up to HARD_MAX_DUTY)
const float HARD_MAX_DUTY    = 0.80f;  // absolute ceiling the phone can never exceed
const float ACCEL_PER_S      = 0.6f;   // duty gained per second (0 -> 30% in 0.5 s, 0 -> 80% in 1.3 s)
const float DECEL_PER_S      = 1.0f;   // duty lost per second when you ease off
const uint32_t LINK_TIMEOUT_MS = 500;  // no phone command for this long = ramp to stop
const uint32_t PWM_HZ        = 20000;  // above hearing range, so the motors don't whine
const uint8_t  PWM_BITS      = 10;
const uint32_t PWM_MAX       = (1u << PWM_BITS) - 1;

// cockpit: "closed" = the switch connects the pin to GND. The phone page shows each switch
// live, so flip these if a switch reads backwards.
const bool  PEDAL_NEEDS_PHONE   = true;   // pedal works only while your phone page is connected (you're supervising)
const bool  REVERSE_WHEN_CLOSED = true;   // Fwd/Rev: closed = reverse
const bool  HIGH_WHEN_CLOSED    = true;   // High/Low: closed = high speed
const bool  RC_ONLY_WHEN_CLOSED = true;   // Manual/RC: closed = parent only, pedal ignored
const float LOW_GEAR_FACTOR     = 0.5f;   // Low = half of the max speed
const float REVERSE_FACTOR      = 0.6f;   // reverse tops out slower than forward

// tail light orientation, flip if the arrow points the wrong way once mounted
const bool MATRIX_FLIP_ROWS = false;
const bool MATRIX_FLIP_COLS = false;

// ---------------- state ----------------
WebServer server(80);
float    throttle = 0;           // -1..1 from the phone
float    maxDuty = DEFAULT_MAX_DUTY;
float    duty = 0;               // what the motors actually get, -1..1 (ramped)
float    target = 0;
bool     estop = false;
bool     enabled = false;
uint32_t lastCmdMs = 0;
IPAddress ownerIp;               // only one phone drives at a time
uint32_t lastLoopUs = 0;

enum TailState { TAIL_STOPPED, TAIL_SLOWING, TAIL_FWD, TAIL_REV };
TailState tail = TAIL_STOPPED;
enum Source { SRC_NONE, SRC_PHONE, SRC_PEDAL };
Source source = SRC_NONE;

// debounced switch input: closed = pin pulled to GND, stable for 3 samples (30 ms)
struct Input {
  int pin;
  bool closed = false;
  uint8_t count = 0;
  void begin() { pinMode(pin, INPUT_PULLUP); closed = digitalRead(pin) == LOW; }
  void update() {
    bool now = digitalRead(pin) == LOW;
    if (now == closed) { count = 0; return; }
    if (++count >= 3) { closed = now; count = 0; }
  }
};
Input inPedal{PIN_PEDAL}, inFwdRev{PIN_FWD_REV}, inHighLow{PIN_HIGH_LOW}, inMode{PIN_MANUAL_RC};
bool pedalArmed = false;  // pedal must be seen released once after boot / e-stop before it can drive
bool reverseSel = false, highSel = false, rcOnly = false;

// ---------------- PWM (works on core 2.x and 3.x) ----------------
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmSetup(int pin, int /*ch*/) { ledcAttach(pin, PWM_HZ, PWM_BITS); ledcWrite(pin, 0); }
void pwmWrite(int pin, int /*ch*/, uint32_t v) { ledcWrite(pin, v); }
#else
void pwmSetup(int pin, int ch) { ledcSetup(ch, PWM_HZ, PWM_BITS); ledcAttachPin(pin, ch); ledcWrite(ch, 0); }
void pwmWrite(int /*pin*/, int ch, uint32_t v) { ledcWrite(ch, v); }
#endif
const int CH_R = 0, CH_L = 1;

void driveOutputs() {
  float d = INVERT_DRIVE ? -duty : duty;
  uint32_t r = 0, l = 0;
  if (d > 0) r = (uint32_t)(d * PWM_MAX + 0.5f);
  if (d < 0) l = (uint32_t)(-d * PWM_MAX + 0.5f);
  // never both at once: zero the idle side first
  if (r) { pwmWrite(PIN_REAR_LPWM, CH_L, 0); pwmWrite(PIN_REAR_RPWM, CH_R, r); }
  else   { pwmWrite(PIN_REAR_RPWM, CH_R, 0); pwmWrite(PIN_REAR_LPWM, CH_L, l); }
  digitalWrite(PIN_REAR_EN, enabled ? HIGH : LOW);
}

// ---------------- MAX7219 (bit-banged, no library needed) ----------------
void maxSend(uint8_t reg, uint8_t val) {
  digitalWrite(PIN_MAX_CS, LOW);
  shiftOut(PIN_MAX_DIN, PIN_MAX_CLK, MSBFIRST, reg);
  shiftOut(PIN_MAX_DIN, PIN_MAX_CLK, MSBFIRST, val);
  digitalWrite(PIN_MAX_CS, HIGH);
}
uint8_t reverseBits(uint8_t b) { uint8_t r = 0; for (int i = 0; i < 8; i++) if (b & (1 << i)) r |= 1 << (7 - i); return r; }
void maxShow(const uint8_t rows[8], uint8_t intensity) {
  maxSend(0x0A, intensity & 0x0F);
  for (int i = 0; i < 8; i++) {
    uint8_t v = rows[MATRIX_FLIP_ROWS ? 7 - i : i];
    maxSend(i + 1, MATRIX_FLIP_COLS ? reverseBits(v) : v);
  }
}
void maxInit() {
  pinMode(PIN_MAX_CS, OUTPUT); digitalWrite(PIN_MAX_CS, HIGH);
  pinMode(PIN_MAX_DIN, OUTPUT); pinMode(PIN_MAX_CLK, OUTPUT);
  maxSend(0x0F, 0);  // display test off
  maxSend(0x09, 0);  // no BCD decode, raw pixels
  maxSend(0x0B, 7);  // scan all 8 rows
  maxSend(0x0C, 1);  // wake up
}
const uint8_t PAT_UP[8]   = {0x18, 0x3C, 0x7E, 0xFF, 0x18, 0x18, 0x18, 0x18};
const uint8_t PAT_DOWN[8] = {0x18, 0x18, 0x18, 0x18, 0xFF, 0x7E, 0x3C, 0x18};
const uint8_t PAT_FULL[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t PAT_OFF[8]  = {0, 0, 0, 0, 0, 0, 0, 0};

void updateTail(uint32_t now) {
  static int lastKey = -1;
  TailState s;
  if (fabsf(duty) < 0.01f) s = TAIL_STOPPED;
  else if (fabsf(target) < fabsf(duty) - 0.02f) s = TAIL_SLOWING;
  else s = duty > 0 ? TAIL_FWD : TAIL_REV;
  tail = s;
  bool flickerOn = (now / 60) % 2;  // ~8 Hz flicker while slowing
  int key = s * 2 + (s == TAIL_SLOWING ? flickerOn : 0);
  if (key == lastKey) return;  // only talk to the chip when the picture changes
  lastKey = key;
  switch (s) {
    case TAIL_STOPPED: maxShow(PAT_FULL, 15); break;
    case TAIL_SLOWING: maxShow(flickerOn ? PAT_FULL : PAT_OFF, 15); break;
    case TAIL_FWD:     maxShow(PAT_UP, 3); break;
    case TAIL_REV:     maxShow(PAT_DOWN, 3); break;
  }
}

// ---------------- control loop ----------------
void controlStep(uint32_t now, float dt) {
  inPedal.update(); inFwdRev.update(); inHighLow.update(); inMode.update();
  reverseSel = inFwdRev.closed == REVERSE_WHEN_CLOSED;
  highSel    = inHighLow.closed == HIGH_WHEN_CLOSED;
  rcOnly     = inMode.closed == RC_ONLY_WHEN_CLOSED;
  if (!inPedal.closed && !estop) pedalArmed = true;

  bool linkOk = (now - lastCmdMs) < LINK_TIMEOUT_MS;
  bool phoneActive = linkOk && fabsf(throttle) > 0.02f;
  bool pedalAllowed = pedalArmed && !rcOnly && (!PEDAL_NEEDS_PHONE || linkOk);
  float t = 0;
  source = SRC_NONE;
  if (estop) {
    t = 0;
  } else if (phoneActive) {                       // parent overrides Luka
    t = throttle * maxDuty; source = SRC_PHONE;
  } else if (inPedal.closed && pedalAllowed) {
    t = (reverseSel ? -1.0f : 1.0f) * maxDuty * (highSel ? 1.0f : LOW_GEAR_FACTOR); source = SRC_PEDAL;
  }
  if (t < 0) t *= REVERSE_FACTOR;
  // never flip direction while moving: come to a stop first
  if (t != 0 && duty != 0 && ((t > 0) != (duty > 0))) t = 0;
  target = t;

  if (estop) {
    duty = 0;  // e-stop: outputs off immediately, the car coasts
  } else {
    bool speedingUp = fabsf(target) > fabsf(duty);
    float step = (speedingUp ? ACCEL_PER_S : DECEL_PER_S) * dt;
    float diff = target - duty;
    if (fabsf(diff) <= step) duty = target; else duty += (diff > 0 ? step : -step);
  }
  enabled = !estop && (duty != 0 || target != 0);
  driveOutputs();
  updateTail(now);
}

// ---------------- phone page ----------------
const char PAGE[] PROGMEM = R"rawliteral(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>McLaren F1</title><style>
*{box-sizing:border-box;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}
body{margin:0;background:#0e141b;color:#e6edf3;font:16px -apple-system,Helvetica,Arial,sans-serif;display:flex;flex-direction:column;height:100vh;padding:12px;gap:10px}
h1{font-size:18px;margin:0;color:#ff8000}
#pad{flex:1;border:2px solid #2a3a4c;border-radius:18px;position:relative;touch-action:none;background:linear-gradient(#13301a,#0e141b 50%,#301313)}
#pad .mid{position:absolute;left:0;right:0;top:50%;border-top:1px dashed #47627f}
#pad .lab{position:absolute;left:0;right:0;text-align:center;color:#8fa3b8;font-size:14px}
#knob{position:absolute;left:50%;top:50%;width:90px;height:90px;margin:-45px;border-radius:50%;background:#ff8000;box-shadow:0 0 20px #ff800088}
.row{display:flex;gap:10px;align-items:center}
button{flex:1;font-size:18px;font-weight:700;padding:16px;border-radius:12px;border:0}
#stop{background:#c62828;color:#fff}#reset{background:#2a3a4c;color:#e6edf3}
#stat{font:14px ui-monospace,Menlo,monospace;color:#8fa3b8}
input[type=range]{flex:1;accent-color:#ff8000}
</style></head><body>
<h1>Luka's McLaren F1</h1>
<div id="stat">connecting...</div><div id="sw" style="font:13px ui-monospace,Menlo,monospace;color:#8fa3b8"></div>
<div id="pad"><div class="lab" style="top:8px">FORWARD</div><div class="mid"></div><div class="lab" style="bottom:8px">REVERSE</div><div id="knob"></div></div>
<div class="row"><span>Max speed</span><input id="max" type="range" min="10" max="80" value="30"><b id="maxv">30%</b></div>
<div class="row"><button id="stop">E-STOP</button><button id="reset">Reset</button></div>
<script>
let t=0, busy=false; const pad=document.getElementById('pad'), knob=document.getElementById('knob'), stat=document.getElementById('stat'), mx=document.getElementById('max');
function setFromY(y){const r=pad.getBoundingClientRect();let v=(r.top+r.height/2-y)/(r.height/2-45);t=Math.max(-1,Math.min(1,v));knob.style.top=(50-t*(50-45/r.height*100))+'%';}
function release(){t=0;knob.style.top='50%';}
pad.addEventListener('pointerdown',e=>{pad.setPointerCapture(e.pointerId);setFromY(e.clientY);});
pad.addEventListener('pointermove',e=>{if(e.buttons||e.pointerType==='touch')setFromY(e.clientY);});
['pointerup','pointercancel','lostpointercapture'].forEach(n=>pad.addEventListener(n,release));
document.addEventListener('visibilitychange',release);
mx.oninput=()=>document.getElementById('maxv').textContent=mx.value+'%';
document.getElementById('stop').onclick=()=>fetch('/estop');
document.getElementById('reset').onclick=()=>fetch('/reset');
setInterval(async()=>{ if(busy) return; busy=true;
  try{ const r=await fetch(`/c?t=${Math.round(t*100)}&m=${mx.value}`,{cache:'no-store'}); const j=await r.json();
    stat.textContent = j.busy ? 'another phone is driving' : `speed ${j.d}%  ${j.s}  (${j.src})${j.e?'  E-STOP (press Reset)':''}`;
    document.getElementById('sw').textContent = `pedal ${j.p?'PRESSED':'up'}${j.p&&!j.arm?' (release first)':''} | ${j.rev?'REV':'FWD'} | ${j.hi?'HIGH':'LOW'} | ${j.rc?'parent only':'Luka can drive'}`;
  }catch(e){ stat.textContent='no link: car is stopping'; }
  busy=false; },100);
</script></body></html>)rawliteral";

const char* tailName() {
  switch (tail) { case TAIL_STOPPED: return "stopped"; case TAIL_SLOWING: return "slowing"; case TAIL_FWD: return "forward"; default: return "reverse"; }
}
const char* sourceName() { return source == SRC_PHONE ? "phone" : source == SRC_PEDAL ? "pedal" : "idle"; }
void sendStatus(bool busy) {
  char buf[280];
  snprintf(buf, sizeof buf,
           "{\"d\":%d,\"s\":\"%s\",\"e\":%s,\"busy\":%s,\"src\":\"%s\",\"p\":%d,\"arm\":%d,\"rev\":%d,\"hi\":%d,\"rc\":%d,\"fw\":\"%s\",\"board\":\"%s\"}",
           (int)lroundf(duty * 100), tailName(), estop ? "true" : "false", busy ? "true" : "false",
           sourceName(), inPedal.closed, pedalArmed, reverseSel, highSel, rcOnly, FW_VERSION, BOARD_NAME);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}
bool fromCarWifi() {
#if GARAGE_MODE
  return server.client().localIP() == WiFi.softAPIP();  // came in through the car's own network
#else
  return true;  // the car's own network is the only network
#endif
}
void handleCmd() {
  if (!fromCarWifi()) { server.send(403, "application/json", "{\"err\":\"drive only from the car's own Wi-Fi\"}"); return; }
  uint32_t now = millis();
  IPAddress ip = server.client().remoteIP();
  bool ownerActive = (now - lastCmdMs) < 3000;
  if (ownerActive && !(ownerIp == IPAddress()) && !(ip == ownerIp)) { sendStatus(true); return; }
  ownerIp = ip;
  float t = constrain(server.arg("t").toInt(), -100, 100) / 100.0f;
  float m = constrain(server.arg("m").toInt(), 10, 100) / 100.0f;
  throttle = t;
  maxDuty = min(m, HARD_MAX_DUTY);
  lastCmdMs = now;
  sendStatus(false);
}

// ---------------- setup / loop ----------------
void setup() {
  // motors OFF before anything else
  pinMode(PIN_REAR_EN, OUTPUT); digitalWrite(PIN_REAR_EN, LOW);
  pwmSetup(PIN_REAR_RPWM, CH_R);
  pwmSetup(PIN_REAR_LPWM, CH_L);
  maxInit();
  maxShow(PAT_FULL, 15);
  inPedal.begin(); inFwdRev.begin(); inHighLow.begin(); inMode.begin();

  Serial.begin(115200);
#if GARAGE_MODE
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(HOME_WIFI_SSID, HOME_WIFI_PASSWORD);
  for (uint32_t t0 = millis(); WiFi.status() != WL_CONNECTED && millis() - t0 < 12000;) delay(100);
  bool home = WiFi.status() == WL_CONNECTED;
  if (!home) {  // say why, so it can be fixed without guessing
    int why = WiFi.status();  // 1 = network not found, 4 = connect failed (often a wrong password)
    WiFi.disconnect();  // a scan can't run while a connection attempt is in progress
    delay(200);
    int n = WiFi.scanNetworks();
    bool seen = false;
    for (int i = 0; i < n; i++) if (WiFi.SSID(i) == HOME_WIFI_SSID) {
      seen = true;
      Serial.printf("Garage mode: %s visible on channel %d, signal %d dBm, but not connected (status %d)\n",
                    HOME_WIFI_SSID, (int)WiFi.channel(i), (int)WiFi.RSSI(i), why);
    }
    for (int i = 0; i < n && !seen; i++) if (WiFi.SSID(i).equalsIgnoreCase(HOME_WIFI_SSID))
      Serial.printf("Garage mode: no exact match, but \"%s\" is visible. Wi-Fi names are case-sensitive; check secrets.h\n", WiFi.SSID(i).c_str());
    if (!seen) Serial.printf("Garage mode: %s not visible on 2.4 GHz (%d networks seen). This chip has no 5 GHz radio.\n", HOME_WIFI_SSID, n < 0 ? 0 : n);
    WiFi.scanDelete();
    WiFi.begin(HOME_WIFI_SSID, HOME_WIFI_PASSWORD);  // keep trying in the background
  }
  WiFi.softAP(WIFI_AP_NAME, WIFI_AP_PASSWORD, home ? WiFi.channel() : 1, 0, 2);  // AP must share the home channel
  ArduinoOTA.setHostname("mclaren");
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([] {  // never update with the motors live
    estop = true; throttle = 0; duty = 0; target = 0; enabled = false; driveOutputs(); maxShow(PAT_OFF, 0);
    Serial.println("OTA update starting, motors off");
  });
  ArduinoOTA.begin();  // also starts mDNS as mclaren.local
  if (home) Serial.printf("Garage mode: home Wi-Fi %s, http://mclaren.local/status\n", WiFi.localIP().toString().c_str());
  else Serial.println("Garage mode: home Wi-Fi not found, car network only (will keep trying)");
#else
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_NAME, WIFI_AP_PASSWORD, 1, 0, 2);
  MDNS.begin("mclaren");
#endif
  server.on("/", [] { server.send_P(200, "text/html", PAGE); });
  server.on("/c", handleCmd);
  server.on("/estop", [] { estop = true; throttle = 0; pedalArmed = false; sendStatus(false); Serial.println("E-STOP"); });
  server.on("/reset", [] {  // only from the car's network: clearing an e-stop needs someone at the car
    if (!fromCarWifi()) { server.send(403, "application/json", "{\"err\":\"reset only from the car's own Wi-Fi\"}"); return; }
    estop = false; throttle = 0; sendStatus(false); Serial.println("reset");
  });
  server.on("/status", [] { sendStatus(false); });  // read-only, from either network
  server.begin();
  Serial.printf("McLaren ECU up on %s. Join Wi-Fi \"%s\", open http://192.168.4.1\n", BOARD_NAME, WIFI_AP_NAME);
  lastLoopUs = micros();
}

void loop() {
  server.handleClient();
#if GARAGE_MODE
  ArduinoOTA.handle();
#endif
  uint32_t nowUs = micros();
  if (nowUs - lastLoopUs >= 10000) {  // 100 Hz control loop
    float dt = (nowUs - lastLoopUs) / 1e6f;
    lastLoopUs = nowUs;
    controlStep(millis(), dt);
  }
  static uint32_t lastLog = 0;
  if (millis() - lastLog > 500) {
    lastLog = millis();
    Serial.printf("src %-5s throttle %+.2f target %+.2f duty %+.2f EN %d tail %-7s | pedal %d%s %s %s %s%s\n",
                  sourceName(), throttle, target, duty, enabled, tailName(),
                  inPedal.closed, (inPedal.closed && !pedalArmed) ? "(release first)" : "",
                  reverseSel ? "REV" : "FWD", highSel ? "HIGH" : "LOW", rcOnly ? "parent-only" : "luka-ok",
                  estop ? "  ESTOP" : "");
  }
}
