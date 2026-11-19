// Luka's McLaren F1 ECU, stage 1a: rear drive, tail light, pedal + dash switches, phone control.
// Board: ESP32-S3 DevKitC-1 or a classic ESP32 DevKit (pins picked automatically). Arduino-ESP32 core 2.x or 3.x.
//
// The ESP32 makes its own Wi-Fi network. Join it from your phone and open http://192.168.4.1.
// Luka drives with the pedal; Fwd/Rev, High/Low, Manual/RC and the power button work like stock.
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
#include <esp_system.h>

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

#define FW_VERSION "stage1a-tabs " __DATE__ " " __TIME__  // shows on /status, so you can tell which build is running

// ---------------- pins (match the wiring tutorial) ----------------
// One sketch, two boards: the right map is picked from the board you select in the IDE.
#if defined(CONFIG_IDF_TARGET_ESP32S3)        // ESP32-S3 DevKitC-1 (the long board)
const int PIN_REAR_RPWM = 4;   // forward PWM  -> BTS7960 RPWM
const int PIN_REAR_LPWM = 5;   // reverse PWM  -> BTS7960 LPWM
const int PIN_REAR_EN   = 6;   // R_EN + L_EN, 10k pull-down to GND on the driver side
const int PIN_MAX_DIN   = 11;  // MAX7219 DIN
const int PIN_MAX_CLK   = 12;  // MAX7219 CLK
const int PIN_MAX_CS    = 10;  // MAX7219 CS/LOAD
const int PIN_GO_FWD    = 13;  // dash ORANGE: closes to GND when pedal down + rocker Forward
const int PIN_GO_REV    = 14;  // dash BLUE:   closes to GND when pedal down + rocker Reverse
const int PIN_HIGH_LOW  = 17;  // dash YELLOW: closes to GND in turtle (slow)
const int PIN_MANUAL_RC = 18;  // dash GREEN:  closes to GND in remote mode
const int PIN_POWER_BTN = 8;   // dash WHITE (PINK goes to GND): closed = power button on
#define BOARD_NAME "ESP32-S3"
#elif defined(CONFIG_IDF_TARGET_ESP32)        // classic ESP32 DevKit (WROOM-32, DOIT V1 and clones)
const int PIN_REAR_RPWM = 16;  // labeled RX2 on some 30-pin boards
const int PIN_REAR_LPWM = 17;  // labeled TX2 on some 30-pin boards
const int PIN_REAR_EN   = 4;
const int PIN_MAX_DIN   = 23;
const int PIN_MAX_CLK   = 18;
const int PIN_MAX_CS    = 5;
const int PIN_GO_FWD    = 32;  // dash ORANGE
const int PIN_GO_REV    = 33;  // dash BLUE
const int PIN_HIGH_LOW  = 19;  // dash YELLOW
const int PIN_MANUAL_RC = 21;  // dash GREEN
const int PIN_POWER_BTN = 22;  // dash WHITE (PINK goes to GND)
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
// On the McLaren the pedal and the Fwd/Rev rocker are chained inside the dash: the ESP32 sees one wire
// for "go forward" (orange) and one for "go backward" (blue). The 7-wire dash plug maps straight onto pins.
const bool  PEDAL_NEEDS_PHONE   = true;   // pedal works only while your phone page is connected (you're supervising)
const bool  USE_POWER_BUTTON    = true;   // dash power button off = car disabled (phone included), like stock
const bool  HIGH_WHEN_CLOSED    = false;  // High/Low: closed = LOW (McLaren: yellow wire closes in turtle mode)
const bool  RC_ONLY_WHEN_CLOSED = true;   // Manual/RC: closed = parent only, pedal ignored (McLaren: green wire closes in remote mode)
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

// ---------------- telemetry: event log + loop health ----------------
const int EV_N = 30;
struct Ev { uint32_t ms; char msg[44]; };
Ev evs[EV_N]; int evHead = 0, evCount = 0;
void logEvent(const char* msg) {
  Ev& e = evs[evHead]; e.ms = millis(); strncpy(e.msg, msg, sizeof e.msg - 1); e.msg[sizeof e.msg - 1] = 0;
  evHead = (evHead + 1) % EV_N; if (evCount < EV_N) evCount++;
  Serial.printf("EVENT %s\n", msg);
}
// Extra telemetry: register any value and it shows up in the dashboard's "More sensors" card.
// Add your own in setup(), e.g. addMetric("Battery", "V", [] { return analogReadMilliVolts(35) * 6.55f / 1000; }, 2);
struct Metric { const char* name; const char* unit; float (*get)(); int decimals; };
const int METRIC_MAX = 16;
Metric metrics[METRIC_MAX]; int metricCount = 0;
void addMetric(const char* name, const char* unit, float (*get)(), int decimals) {
  if (metricCount < METRIC_MAX) metrics[metricCount++] = {name, unit, get, decimals};
}
uint32_t loopsThisSec = 0, loopHz = 0, maxGapUs = 0, maxGapThisSec = 0, lastSecMs = 0, prevLoopUs = 0;
const char* resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "power on";
    case ESP_RST_SW:       return "update / restart";
    case ESP_RST_PANIC:    return "CRASH";
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "WATCHDOG";
    case ESP_RST_BROWNOUT: return "BROWNOUT (voltage dip)";
    case ESP_RST_EXT:      return "reset button";
    case ESP_RST_DEEPSLEEP:return "deep sleep";
    default:               return "other";
  }
}

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
Input inGoFwd{PIN_GO_FWD}, inGoRev{PIN_GO_REV}, inHighLow{PIN_HIGH_LOW}, inMode{PIN_MANUAL_RC}, inPower{PIN_POWER_BTN};
bool pedalArmed = false;  // pedal must be seen released once after boot / e-stop / power-on before it can drive
bool pedalDown = false, reverseSel = false, highSel = false, rcOnly = false, powerOn = true;

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
  inGoFwd.update(); inGoRev.update(); inHighLow.update(); inMode.update(); inPower.update();
  bool fwdReq = inGoFwd.closed && !inGoRev.closed, revReq = inGoRev.closed && !inGoFwd.closed;  // both at once = fault, ignore
  pedalDown  = fwdReq || revReq;
  reverseSel = revReq;
  highSel    = inHighLow.closed == HIGH_WHEN_CLOSED;
  rcOnly     = inMode.closed == RC_ONLY_WHEN_CLOSED;
  powerOn    = !USE_POWER_BUTTON || inPower.closed;
  if (!powerOn) pedalArmed = false;
  else if (!inGoFwd.closed && !inGoRev.closed && !estop) pedalArmed = true;

  bool linkOk = (now - lastCmdMs) < LINK_TIMEOUT_MS;
  bool phoneActive = linkOk && fabsf(throttle) > 0.02f;
  bool pedalAllowed = pedalArmed && !rcOnly && (!PEDAL_NEEDS_PHONE || linkOk);
  float t = 0;
  source = SRC_NONE;
  if (estop || !powerOn) {
    t = 0;
  } else if (phoneActive) {                       // parent overrides Luka
    t = throttle * maxDuty; source = SRC_PHONE;
  } else if (pedalDown && pedalAllowed) {
    t = (reverseSel ? -1.0f : 1.0f) * maxDuty * (highSel ? 1.0f : LOW_GEAR_FACTOR); source = SRC_PEDAL;
  }
  if (t < 0) t *= REVERSE_FACTOR;
  // never flip direction while moving: come to a stop first
  if (t != 0 && duty != 0 && ((t > 0) != (duty > 0))) t = 0;
  target = t;

  if (estop || !powerOn) {
    duty = 0;  // e-stop or power button off: outputs off immediately, the car coasts
  } else {
    bool speedingUp = fabsf(target) > fabsf(duty);
    float step = (speedingUp ? ACCEL_PER_S : DECEL_PER_S) * dt;
    float diff = target - duty;
    if (fabsf(diff) <= step) duty = target; else duty += (diff > 0 ? step : -step);
  }
  enabled = !estop && powerOn && (duty != 0 || target != 0);
  driveOutputs();
  updateTail(now);

  // log every change, so the dashboard shows exactly what the car saw and when
  static int pPower = -1, pPedal = -1, pHigh = -1, pRc = -1, pEstop = -1, pLink = -1, pSrc = -1, pWait = -1;
  int pedalState = pedalDown ? (reverseSel ? 2 : 1) : 0;
  bool link = lastCmdMs != 0 && linkOk;
  bool armWait = pedalDown && !pedalArmed && powerOn;
  if (powerOn != pPower) { logEvent(powerOn ? "power button ON" : "power button OFF"); pPower = powerOn; }
  if (pedalState != pPedal) { logEvent(pedalState == 1 ? "pedal down, FORWARD" : pedalState == 2 ? "pedal down, REVERSE" : "pedal up"); pPedal = pedalState; }
  if (highSel != pHigh) { logEvent(highSel ? "speed HIGH (rabbit)" : "speed LOW (turtle)"); pHigh = highSel; }
  if (rcOnly != pRc) { logEvent(rcOnly ? "mode REMOTE (parent only)" : "mode MANUAL (pedal enabled)"); pRc = rcOnly; }
  if (estop != pEstop) { if (pEstop != -1 || estop) logEvent(estop ? "E-STOP" : "E-STOP cleared"); pEstop = estop; }
  if (link != pLink) { if (pLink != -1 || link) logEvent(link ? "phone connected" : "phone link lost"); pLink = link; }
  if ((int)source != pSrc) { if (pSrc != -1) logEvent(source == SRC_PEDAL ? "driving: pedal" : source == SRC_PHONE ? "driving: phone" : "idle"); pSrc = source; }
  if (armWait != pWait) { if (armWait) logEvent("pedal held: release to arm"); pWait = armWait; }
}

// ---------------- phone page ----------------
const char PAGE[] PROGMEM = R"rawliteral(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>McLaren F1</title><meta name="apple-mobile-web-app-capable" content="yes"><meta name="mobile-web-app-capable" content="yes"><meta name="apple-mobile-web-app-status-bar-style" content="black"><meta name="theme-color" content="#0e141b"><style>
.nav{display:flex;align-items:center;gap:10px}.nav h1{flex:1}.tabs{display:flex;background:#151e28;border:1px solid #2a3a4c;border-radius:10px;overflow:hidden}.tabs a,.tabs b{padding:8px 14px;font-size:14px;color:#8fa3b8;text-decoration:none;font-weight:600}.tabs b{background:#ff8000;color:#1b0f00}
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
<div class="nav"><h1>McLaren F1</h1><span class="tabs"><b>Drive</b><a href="/dash">Telemetry</a></span></div>
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
    stat.textContent = j.busy ? 'another phone is driving' : `${j.ro?'VIEW ONLY on home Wi-Fi (join McLaren-F1 to drive) · ':''}speed ${j.d}%  ${j.s}  (${j.src})${j.e?'  E-STOP (press Reset)':''}`;
    document.getElementById('sw').textContent = `power ${j.pw?'ON':'OFF'} | pedal ${j.p?(j.rev?'down REVERSE':'down FORWARD'):'up'}${j.p&&!j.arm?' (release first)':''} | ${j.hi?'HIGH':'LOW'} | ${j.rc?'parent only':'pedal enabled'}`;
  }catch(e){ stat.textContent='no link: car is stopping'; }
  busy=false; },100);
</script></body></html>)rawliteral";

const char* tailName() {
  switch (tail) { case TAIL_STOPPED: return "stopped"; case TAIL_SLOWING: return "slowing"; case TAIL_FWD: return "forward"; default: return "reverse"; }
}
const char DASH[] PROGMEM = R"dash(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>McLaren F1 · live</title><meta name="apple-mobile-web-app-capable" content="yes"><meta name="mobile-web-app-capable" content="yes"><meta name="apple-mobile-web-app-status-bar-style" content="black"><meta name="theme-color" content="#0e141b"><style>
.nav{display:flex;align-items:center;gap:10px}.nav h1{flex:1}.tabs{display:flex;background:#151e28;border:1px solid #2a3a4c;border-radius:10px;overflow:hidden}.tabs a,.tabs b{padding:8px 14px;font-size:14px;color:#8fa3b8;text-decoration:none;font-weight:600}.tabs b{background:#ff8000;color:#1b0f00}
#nav{display:none;margin:2px 0 6px}
:root{--bg:#0e141b;--card:#151e28;--line:#2a3a4c;--txt:#e6edf3;--mut:#8fa3b8;--org:#ff8000}
*{box-sizing:border-box}body{margin:0 auto;background:var(--bg);color:var(--txt);font:15px -apple-system,Helvetica,Arial,sans-serif;padding:12px;max-width:760px}
header{display:flex;align-items:center;gap:10px;margin:2px 0 8px}h1{font-size:18px;margin:0;flex:1}h1 small{display:block;font-size:12px;color:var(--mut);font-weight:400;margin-top:2px}
.pill{padding:7px 12px;border-radius:999px;font-weight:800;font-size:13px;letter-spacing:.04em;background:#2a3a4c;white-space:nowrap}
.pill.ready{background:#1b5e20}.pill.drive{background:var(--org);color:#1b0f00}.pill.stop{background:#c62828}.pill.off{background:#37474f}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:12px;margin:10px 0}
.card h2{font-size:12px;letter-spacing:.08em;text-transform:uppercase;color:var(--mut);margin:0 0 10px;font-weight:700}
.lamps{display:grid;grid-template-columns:repeat(5,1fr);gap:8px}
.lamp{background:#0f1720;border:1px solid var(--line);border-radius:12px;padding:10px 4px 8px;text-align:center;font-size:11.5px;color:var(--mut)}
.lamp i{display:block;width:18px;height:18px;border-radius:50%;margin:0 auto 6px;background:#26323d}
.lamp b{display:block;font-size:13px;color:var(--txt)}.lamp.on{border-color:#3d5a3f}.lamp.on i{background:#66bb6a;box-shadow:0 0 12px #66bb6a}
.lamp.amber.on{border-color:#6b5513}.lamp.amber.on i{background:#ffb300;box-shadow:0 0 12px #ffb300}
.note{margin-top:10px;padding:8px 10px;border-radius:10px;background:#33250a;border:1px solid #6b4a0f;font-size:13px;display:none}
.drive{display:flex;align-items:center;gap:14px}.num{font:800 42px ui-monospace,Menlo,monospace;min-width:128px}
.bar{flex:1;height:14px;background:#0f1720;border-radius:7px;border:1px solid var(--line);position:relative;overflow:hidden}
.bar i{position:absolute;top:0;bottom:0;background:var(--org)}.bar b{position:absolute;left:50%;top:0;bottom:0;width:2px;background:#47627f}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:10px}
.tile{background:#0f1720;border:1px solid var(--line);border-radius:10px;padding:8px 10px;min-width:0}
.tile span{display:block;font-size:11.5px;color:var(--mut)}.tile b{display:block;font:600 16px ui-monospace,Menlo,monospace;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.mx{display:grid;grid-template-columns:repeat(8,7px);gap:2px;margin-top:4px}.mx i{width:7px;height:7px;border-radius:50%;background:#3a0b0b}.mx i.on{background:#ff3b30}
.log{font:13px ui-monospace,Menlo,monospace;max-height:280px;overflow:auto}.log div{padding:5px 0;border-bottom:1px solid #1d2935;display:flex;gap:10px}.log span{color:var(--mut);min-width:62px}
.bad{color:#ff8a80!important}.warn{color:#ffd54f!important}.good{color:#a5d6a7!important}
.foot{display:flex;gap:12px;align-items:center;color:var(--mut);font-size:12px;margin:12px 0 20px}
button{background:#8e1b1b;color:#fff;border:0;border-radius:10px;padding:12px 18px;font-weight:800;font-size:14px;white-space:nowrap;flex:none}
@media(max-width:480px){.lamps{grid-template-columns:repeat(3,1fr)}.grid{grid-template-columns:1fr 1fr}.num{font-size:34px;min-width:100px}}
</style></head><body>
<div class="nav" id="nav"><span style="flex:1"></span><span class="tabs"><a href="/">Drive</a><b>Telemetry</b></span></div>
<header><h1>McLaren F1 · live<small id="sub">connecting…</small></h1><div class="pill" id="pill">…</div></header>
<div class="card"><h2>Dash controls</h2><div class="lamps">
<div class="lamp" id="lPow"><i></i><b>POWER</b><em></em></div><div class="lamp" id="lFwd"><i></i><b>FORWARD</b><em>pedal + rocker</em></div>
<div class="lamp" id="lRev"><i></i><b>REVERSE</b><em>pedal + rocker</em></div><div class="lamp amber" id="lSpd"><i></i><b>SPEED</b><em></em></div>
<div class="lamp amber" id="lMode"><i></i><b>MODE</b><em></em></div></div><div class="note" id="arm">Pedal is held: the car waits for a release before it will drive.</div></div>
<div class="card"><h2>Drive</h2><div class="drive"><div class="num" id="duty">0%</div><div class="bar"><b></b><i id="bar"></i></div></div>
<div class="grid"><div class="tile"><span>Commanded by</span><b id="src">idle</b></div><div class="tile"><span>Target</span><b id="tgt">0%</b></div>
<div class="tile"><span>Motor enable</span><b id="en">OFF</b></div><div class="tile"><span>Speed cap</span><b id="max"></b></div>
<div class="tile"><span>Tail light</span><b id="tail"></b><div class="mx" id="mx"></div></div><div class="tile"><span>Battery</span><b class="warn">not wired yet</b></div></div></div>
<div class="card"><h2>Links</h2><div class="grid"><div class="tile"><span>Driving phone</span><b id="phone"></b></div>
<div class="tile"><span>On car Wi-Fi</span><b id="ap"></b></div><div class="tile"><span>Home Wi-Fi</span><b id="rssi"></b></div></div></div>
<div class="card"><h2>System</h2><div class="grid"><div class="tile"><span>Up for</span><b id="up"></b></div><div class="tile"><span>Last restart</span><b id="rst"></b></div>
<div class="tile"><span>Free memory</span><b id="heap"></b></div><div class="tile"><span>Control loop</span><b id="hz"></b></div><div class="tile"><span>Worst delay (1 s)</span><b id="gap"></b></div>
<div class="tile"><span>Address</span><b id="ip"></b></div></div></div>
<div class="card" id="moreCard" style="display:none"><h2>More sensors</h2><div class="grid" id="more"></div></div>
<div class="card"><h2>Event log</h2><div class="log" id="log"></div></div>
<div class="foot"><button id="stop">E-STOP</button><span id="footTxt">View only. Driving works on the car's own Wi-Fi (McLaren-F1).</span></div>
<script>
const $=id=>document.getElementById(id), P={forward:[24,60,126,255,24,24,24,24],reverse:[24,24,24,24,255,126,60,24],stopped:Array(8).fill(255),off:Array(8).fill(0)};
for(let k=0;k<64;k++) $('mx').appendChild(document.createElement('i'));
const ago=ms=>ms<1000?'now':ms<60000?Math.round(ms/1000)+'s ago':ms<3600000?Math.round(ms/60000)+'m ago':Math.round(ms/3600000)+'h ago';
const dur=s=>s<60?s+'s':s<3600?Math.floor(s/60)+'m '+(s%60)+'s':Math.floor(s/3600)+'h '+Math.floor(s%3600/60)+'m';
function lamp(id,on,txt){ $(id).classList.toggle('on',!!on); if(txt!==undefined) $(id).querySelector('em').textContent=txt; }
function set(id,txt,cls){ const e=$(id); e.textContent=txt; e.className=cls||''; }
let fails=0, last=null;
function render(j){
  const pill=$('pill');
  if(j.estop){pill.textContent='E-STOP';pill.className='pill stop';} else if(!j.power){pill.textContent='POWER OFF';pill.className='pill off';}
  else if(j.src!=='idle'){pill.textContent='DRIVING';pill.className='pill drive';} else {pill.textContent='READY';pill.className='pill ready';}
  $('sub').textContent=j.fw+' · '+j.board;
  lamp('lPow',j.power,j.power?'on':'off'); lamp('lFwd',j.pedal===1); lamp('lRev',j.pedal===2);
  lamp('lSpd',!j.high,j.high?'rabbit · fast':'turtle · slow'); lamp('lMode',j.remote,j.remote?'remote · parent':'manual · pedal');
  $('arm').style.display=(j.pedal&&!j.armed&&j.power)?'block':'none';
  $('duty').textContent=(j.duty>0?'+':'')+j.duty+'%'; const b=$('bar'), d=Math.max(-100,Math.min(100,j.duty));
  b.style.left=(d>=0?50:50+d/2)+'%'; b.style.width=Math.abs(d)/2+'%';
  set('src',j.src,j.src==='idle'?'':'warn'); set('tgt',(j.target>0?'+':'')+j.target+'%'); set('en',j.en?'ON':'OFF',j.en?'good':'');
  set('max',j.max+'% (cap '+j.hardMax+'%)'); set('tail',j.tail);
  const pat=j.tail==='slowing'?((Date.now()/60|0)%2?P.stopped:P.off):(P[j.tail]||P.off); [...$('mx').children].forEach((c,i)=>c.classList.toggle('on',!!(pat[i>>3]&(128>>(i&7)))));
  set('phone',j.phoneAgeMs<0?'none yet':j.phoneAgeMs<800?'connected':'last '+ago(j.phoneAgeMs),j.phoneAgeMs>=0&&j.phoneAgeMs<800?'good':'');
  set('ap',j.apClients+' device'+(j.apClients===1?'':'s'));
  set('rssi',j.home?j.rssi+' dBm '+(j.rssi>-60?'strong':j.rssi>-70?'good':j.rssi>-80?'weak':'very weak'):'not connected',j.home?(j.rssi>-70?'good':j.rssi>-80?'warn':'bad'):'warn');
  set('up',dur(j.up)); set('rst',j.reset,/BROWNOUT|CRASH|WATCHDOG/.test(j.reset)?'bad':'');
  set('heap',Math.round(j.heap/1024)+' KB'); set('hz',j.loopHz+' Hz',j.loopHz<90?'warn':'good'); set('gap',j.gapMs.toFixed(1)+' ms',j.gapMs>50?'warn':'');
  set('ip',j.ip||'car Wi-Fi only');
  $('nav').style.display=j.carNet?'flex':'none';
  $('footTxt').textContent=j.carNet?'On the car\'s Wi-Fi: switch to the Drive tab to drive. Leaving the Drive tab stops the car.':'View only. Driving works on the car\'s own Wi-Fi (McLaren-F1).';
  const M=$('more'); $('moreCard').style.display=(j.extra&&j.extra.length)?'block':'none'; M.innerHTML='';
  for(const [name,unit,val] of (j.extra||[])){ const t=document.createElement('div'); t.className='tile'; t.innerHTML='<span></span><b></b>'; t.children[0].textContent=name; t.children[1].textContent=(val===null?'—':val)+(unit?' '+unit:''); M.appendChild(t); }
  const L=$('log'); L.innerHTML=''; for(const [ms,msg] of j.ev){ const r=document.createElement('div'); r.innerHTML='<span></span><b></b>';
    r.children[0].textContent=ago(j.up*1000-ms); r.children[1].textContent=msg; if(/E-STOP|lost|BROWNOUT|CRASH/.test(msg)) r.children[1].className='bad'; L.appendChild(r); }
}
async function tick(){ try{ const r=await fetch('/telemetry',{cache:'no-store'}); last=await r.json(); fails=0; render(last); }
  catch(e){ if(++fails>2){ $('pill').textContent='OFFLINE'; $('pill').className='pill stop'; $('sub').textContent='car not answering'; } } }
setInterval(tick,333); tick();
$('stop').onclick=()=>fetch('/estop');
</script></body></html>)dash";

void sendTelemetry() {
  uint32_t now = millis();
  bool home = WiFi.status() == WL_CONNECTED;
  String j; j.reserve(2800);
  char b[480];
  snprintf(b, sizeof b, "{\"fw\":\"%s\",\"board\":\"%s\",\"up\":%lu,\"reset\":\"%s\",\"heap\":%u,\"heapMin\":%u,\"loopHz\":%lu,\"gapMs\":%.1f,"
           "\"power\":%d,\"pedal\":%d,\"high\":%d,\"remote\":%d,\"armed\":%d,\"estop\":%d,"
           "\"src\":\"%s\",\"duty\":%d,\"target\":%d,\"en\":%d,\"tail\":\"%s\",\"max\":%d,\"hardMax\":%d,",
           FW_VERSION, BOARD_NAME, (unsigned long)(now / 1000), resetReasonName(), (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
           (unsigned long)loopHz, maxGapUs / 1000.0f, (int)powerOn, pedalDown ? (reverseSel ? 2 : 1) : 0, (int)highSel, (int)rcOnly, (int)pedalArmed, (int)estop,
           sourceName(), (int)lroundf(duty * 100), (int)lroundf(target * 100), (int)enabled, tailName(), (int)lroundf(maxDuty * 100), (int)lroundf(HARD_MAX_DUTY * 100));
  j += b;
  long phoneAge = lastCmdMs ? (long)(now - lastCmdMs) : -1;
  snprintf(b, sizeof b, "\"phoneAgeMs\":%ld,\"apClients\":%d,\"home\":%d,\"rssi\":%d,\"ip\":\"%s\",\"carNet\":%d,\"extra\":[",
           phoneAge, (int)WiFi.softAPgetStationNum(), (int)home, home ? (int)WiFi.RSSI() : 0, home ? WiFi.localIP().toString().c_str() : "", (int)fromCarWifi());
  j += b;
  for (int i = 0; i < metricCount; i++) {
    float v = metrics[i].get();
    if (isfinite(v)) snprintf(b, sizeof b, "%s[\"%s\",\"%s\",%.*f]", i ? "," : "", metrics[i].name, metrics[i].unit, metrics[i].decimals, v);
    else snprintf(b, sizeof b, "%s[\"%s\",\"%s\",null]", i ? "," : "", metrics[i].name, metrics[i].unit);
    j += b;
  }
  j += "],\"ev\":[";
  for (int i = 0; i < evCount; i++) {
    const Ev& e = evs[(evHead - 1 - i + EV_N) % EV_N];
    snprintf(b, sizeof b, "%s[%lu,\"%s\"]", i ? "," : "", (unsigned long)e.ms, e.msg);
    j += b;
  }
  j += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

const char* sourceName() { return source == SRC_PHONE ? "phone" : source == SRC_PEDAL ? "pedal" : "idle"; }
void sendStatus(bool busy, bool viewOnly) {
  char buf[280];
  snprintf(buf, sizeof buf,
           "{\"d\":%d,\"s\":\"%s\",\"e\":%s,\"busy\":%s,\"src\":\"%s\",\"p\":%d,\"arm\":%d,\"rev\":%d,\"hi\":%d,\"rc\":%d,\"pw\":%d,\"ro\":%s,\"fw\":\"%s\",\"board\":\"%s\"}",
           (int)lroundf(duty * 100), tailName(), estop ? "true" : "false", busy ? "true" : "false",
           sourceName(), pedalDown, pedalArmed, reverseSel, highSel, rcOnly, powerOn, viewOnly ? "true" : "false", FW_VERSION, BOARD_NAME);
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
  if (!fromCarWifi()) { sendStatus(false, true); return; }  // home network: live readout only, commands ignored
  uint32_t now = millis();
  IPAddress ip = server.client().remoteIP();
  bool ownerActive = (now - lastCmdMs) < 3000;
  if (ownerActive && !(ownerIp == IPAddress()) && !(ip == ownerIp)) { sendStatus(true, false); return; }
  ownerIp = ip;
  float t = constrain(server.arg("t").toInt(), -100, 100) / 100.0f;
  float m = constrain(server.arg("m").toInt(), 10, 100) / 100.0f;
  throttle = t;
  maxDuty = min(m, HARD_MAX_DUTY);
  lastCmdMs = now;
  sendStatus(false, false);
}

// ---------------- setup / loop ----------------
void setup() {
  // motors OFF before anything else
  pinMode(PIN_REAR_EN, OUTPUT); digitalWrite(PIN_REAR_EN, LOW);
  pwmSetup(PIN_REAR_RPWM, CH_R);
  pwmSetup(PIN_REAR_LPWM, CH_L);
  maxInit();
  maxShow(PAT_FULL, 15);
  inGoFwd.begin(); inGoRev.begin(); inHighLow.begin(); inMode.begin(); inPower.begin();

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
    logEvent("firmware update starting");
  });
  ArduinoOTA.begin();  // also starts mDNS as mclaren.local
  if (home) { Serial.printf("Garage mode: home Wi-Fi %s, http://mclaren.local\n", WiFi.localIP().toString().c_str()); logEvent("joined home Wi-Fi"); }
  else Serial.println("Garage mode: home Wi-Fi not found, car network only (will keep trying)");
#else
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_NAME, WIFI_AP_PASSWORD, 1, 0, 2);
  MDNS.begin("mclaren");
#endif
  server.on("/", [] { server.send_P(200, "text/html", fromCarWifi() ? PAGE : DASH); });  // car Wi-Fi: drive page; home Wi-Fi: dashboard
  server.on("/dash", [] { server.send_P(200, "text/html", DASH); });
  server.on("/telemetry", sendTelemetry);
  server.on("/c", handleCmd);
  server.on("/estop", [] { estop = true; throttle = 0; pedalArmed = false; sendStatus(false, false); Serial.println("E-STOP"); });
  server.on("/reset", [] {  // only from the car's network: clearing an e-stop needs someone at the car
    if (!fromCarWifi()) { server.send(403, "application/json", "{\"err\":\"reset only from the car's own Wi-Fi\"}"); return; }
    estop = false; throttle = 0; sendStatus(false, false); Serial.println("reset");
  });
  server.on("/status", [] { sendStatus(false, false); });  // read-only, from either network
  server.begin();
  { char b[44]; snprintf(b, sizeof b, "boot: %s", resetReasonName()); logEvent(b); }
  addMetric("Chip temperature", "°C", [] { return temperatureRead(); }, 0);   // rough: the ESP32's own die sensor
  addMetric("Phone max speed", "%", [] { return maxDuty * 100; }, 0);
  Serial.printf("McLaren ECU up on %s. Join Wi-Fi \"%s\", open http://192.168.4.1\n", BOARD_NAME, WIFI_AP_NAME);
  lastLoopUs = micros();
}

void loop() {
  server.handleClient();
#if GARAGE_MODE
  ArduinoOTA.handle();
#endif
  uint32_t nowUs = micros();
  if (prevLoopUs && nowUs - prevLoopUs > maxGapThisSec) maxGapThisSec = nowUs - prevLoopUs;  // worst stall between loop passes
  prevLoopUs = nowUs;
  if (nowUs - lastLoopUs >= 10000) {  // 100 Hz control loop
    float dt = (nowUs - lastLoopUs) / 1e6f;
    lastLoopUs = nowUs;
    controlStep(millis(), dt);
    loopsThisSec++;
  }
  if (millis() - lastSecMs >= 1000) { lastSecMs = millis(); loopHz = loopsThisSec; loopsThisSec = 0; maxGapUs = maxGapThisSec; maxGapThisSec = 0; }
  static uint32_t lastLog = 0;
  if (millis() - lastLog > 500) {
    lastLog = millis();
    Serial.printf("src %-5s throttle %+.2f target %+.2f duty %+.2f EN %d tail %-7s | power %s pedal %s%s %s %s%s\n",
                  sourceName(), throttle, target, duty, enabled, tailName(), powerOn ? "on" : "OFF",
                  pedalDown ? (reverseSel ? "REV" : "FWD") : "up", (pedalDown && !pedalArmed) ? "(release first)" : "",
                  highSel ? "HIGH" : "LOW", rcOnly ? "parent-only" : "pedal-ok", estop ? "  ESTOP" : "");
  }
}
