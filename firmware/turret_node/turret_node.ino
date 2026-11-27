// Lawn Watering Turret - Turret Node firmware rev B (ESP32-WROOM-32, Arduino core 3.x)
//
// Drives the pan and tilt servos, the 24 V zone valve and the upstream master
// valve, counts flow, reads the pressure sensor and soil probes, and answers
// the Sensor Unit Pi over half-duplex RS-485 (115200 8N1).
//
// Rev B changes (design review, Nov 2026):
//   - The node only ever transmits in reply to a command (no unsolicited EVT
//     lines that could collide with the Pi on the half-duplex bus). Dose
//     progress and results are polled with DOSE?.
//   - Times are compared as signed differences, read after the serial poll,
//     so a millisecond tick between two reads can no longer latch a fault.
//   - Doses carry a sequence id; a repeated DOSE with the same id is ignored,
//     so a retried command can never restart a dose.
//   - Master valve in series upstream of the zone valve: double block, and the
//     only way to stop water if the zone valve sticks open.
//   - STUCK_OPEN is judged on pulses integrated over 30 s (stray pulses no
//     longer trip it) and survives a reboot (stored in flash).
//   - High-flow trip (burst tube), task watchdog, CFG range checks, glitch
//     filter on the flow input, hardware RS-485 direction control.
//
// Safety rules (all local to this node):
//   - the valves open only with an explicit time limit (max 120 s per command)
//   - link watchdog: no valid frame for 3 s while open closes both valves
//   - no-flow (< 0.5 L/min after 4 s) and low pressure (< 10 psi after 4 s) close them
//   - high flow (> 1.6 x nominal after 2 s) closes them (tube off the barb)
//   - leak: >= 20 pulses in 30 s with both valves shut latches STUCK_OPEN
//     (master stays shut; the leak is then through the master too, so the Pi alerts)
//   - task watchdog resets the chip if loop() hangs; the gate pull-downs hold
//     both valves shut through the reset
//   - any latched fault blocks the valves until CLR; STUCK_OPEN needs CLR STUCK
//   - both valves are normally closed, so power loss closes them
//
// Frame format (both directions):  <TEXT>*<HH>\n
//   HH = XOR of all bytes of <TEXT>, two upper-case hex digits.
//
// Commands (Pi -> node)              Replies (node -> Pi)
//   PING                             OK PONG <fw>
//   MOVE <pan> <tilt>                OK MOVE <eta_ms>          degrees; approached from below (anti-backlash)
//   VALVE 1 <max_ms>                 OK VALVE 1                or ERR FAULT <bits>
//   VALVE 0                          OK VALVE 0
//   DOSE <id> <ml> <max_ms>          OK DOSE <id>              or ERR FAULT <bits>; repeated id -> OK DOSE <id> (no restart)
//   DOSE?                            OK DOSEST <id> <RUN|END> <ml> <ms> <reason>   reason: DONE TIMEOUT FAULT STOP PARK -
//   STAT                             OK STAT <pan> <tilt> <valve> <lpm> <ml_total> <psi> <vin> <p1> <p2> <p3> <faults> <uptime_s>
//   CFG <key> <value>                OK CFG                    or ERR KEY / ERR RANGE
//   CLR                              OK CLR                    clears faults except STUCK_OPEN
//   CLR STUCK                        OK CLR                    also clears STUCK_OPEN (after a human has checked)
//   PARK                             OK PARK                   valves shut, nozzle down to drain
//
// Fault bits: 1 LINK, 2 NO_FLOW, 4 LOW_PSI, 8 STUCK_OPEN, 16 VALVE_TIMEOUT, 32 HIGH_FLOW
//
// CFG keys and accepted ranges:
//   pan_c, tilt_c  servo centre pulse (us)      1000..2000
//   pan_k, tilt_k  us per degree (sign = dir)   |k| 4..20
//   flow_k         pulses per litre             100..2000
//   psi_k          psi per volt                 5..100
//   psi_v0         volts at 0 psi               0..1
//   nom_lpm        nominal flow (L/min)         1..20

#include <Arduino.h>
#include <Preferences.h>
#include <esp_task_wdt.h>

#define FW_VERSION "2.0.0"

// ----------------------------------------------------------------- pins
const int PIN_PAN = 25;
const int PIN_TILT = 26;
const int PIN_VALVE = 27;       // zone valve MOSFET gate, 10k pull-down on the board
const int PIN_MASTER = 13;      // master valve MOSFET gate, 10k pull-down on the board
const int PIN_FLOW = 33;        // flow sensor pulses via 10k/20k divider, 1 nF to ground
const int PIN_PSI = 34;         // pressure transducer via 4.7k/10k divider
const int PIN_VIN = 36;         // 24 V input via 100k/12k divider
const int PIN_PROBE[3] = {35, 32, 39};
const int PIN_RS485_TX = 17;
const int PIN_RS485_RX = 16;    // 10k pull-up on the board (RO floats while transmitting)
const int PIN_RS485_DE = 4;     // DE and /RE tied, 10k pull-down (receive during boot)
const int PIN_LED = 2;

// ----------------------------------------------------------------- limits
const float PAN_MIN = -135.0, PAN_MAX = 135.0;
const float TILT_MIN = -45.0, TILT_MAX = 40.0;   // below about -50 deg the jet hits the head
const float SLEW_DEG_S = 120.0;
const float BACKLASH_DEG = 2.0;             // overshoot, then approach every target from below
const int32_t VALVE_MAX_MS = 120000;
const int32_t LINK_TIMEOUT_MS = 3000;
const int32_t FLOW_CHECK_MS = 4000;
const int32_t HIGH_FLOW_CHECK_MS = 2000;
const float NO_FLOW_LPM = 0.5;
const float LOW_PSI = 10.0;
const float HIGH_FLOW_FACTOR = 1.6;
const int32_t LEAK_SETTLE_MS = 5000;        // let the tube drain after closing
const int32_t LEAK_WINDOW_MS = 30000;
const uint32_t LEAK_PULSES = 20;            // ~45 mL in 30 s = 0.09 L/min
const uint32_t FLOW_GLITCH_US = 2000;       // 30 L/min max = 225 Hz, so pulses < 2 ms apart are noise
const uint32_t WDT_TIMEOUT_MS = 4000;

enum Fault : uint8_t { F_LINK = 1, F_NOFLOW = 2, F_LOWPSI = 4, F_STUCK = 8, F_TIMEOUT = 16, F_HIGHFLOW = 32 };

// ----------------------------------------------------------------- config (flash)
Preferences prefs;
struct Cfg {
  float pan_c = 1500, tilt_c = 1500;
  float pan_k = 2000.0 / 270.0;      // 270 deg servo, 500..2500 us
  float tilt_k = 2000.0 / 180.0;     // 180 deg servo
  float flow_k = 450.0;              // nominal; calibrate with a bucket
  float psi_k = 30.0 / 4.0;          // rev B sensor: 0.5-4.5 V = 0-30 psi (use 100/4 for a 0-100 psi sensor)
  float psi_v0 = 0.5;
  float nom_lpm = 7.0;
} cfg;

// ----------------------------------------------------------------- state
volatile uint32_t flowPulses = 0;
volatile uint32_t lastPulseUs = 0;
float panCmd = 0, tiltCmd = -45, panNow = 0, tiltNow = -45;
float panGoal = 0, tiltGoal = -45;          // final targets (after the anti-backlash leg)
bool valveOn = false;
uint32_t valveOpenedAt = 0, valveLimitMs = 0, valveClosedAt = 0;
uint8_t faults = 0;
uint32_t lastFrameMs = 0;
float lpm = 0;
uint32_t lastFlowCalcMs = 0, lastFlowPulses = 0;
uint32_t leakWindowStart = 0, leakWindowPulses = 0;
String rx;

struct Dose {
  long id = -1;
  bool running = false;
  uint32_t startPulses = 0, targetPulses = 0;
  float ml = 0;
  uint32_t ms = 0;
  const char *reason = "-";
} dose;

void IRAM_ATTR onFlowPulse() {
  uint32_t t = micros();
  if (t - lastPulseUs >= FLOW_GLITCH_US) {
    flowPulses++;
    lastPulseUs = t;
  }
}

// elapsed time since a stamp; signed so a stamp taken "after" now reads as 0, not 49 days
inline int32_t since(uint32_t now, uint32_t stamp) {
  int32_t d = (int32_t)(now - stamp);
  return d < 0 ? 0 : d;
}

// ----------------------------------------------------------------- helpers
uint8_t checksum(const String &s) {
  uint8_t c = 0;
  for (size_t i = 0; i < s.length(); i++) c ^= (uint8_t)s[i];
  return c;
}

void send(const String &text) {
  char tail[6];
  snprintf(tail, sizeof(tail), "*%02X\n", checksum(text));
  Serial2.print(text);                  // UART drives DE itself (RS-485 half-duplex mode)
  Serial2.print(tail);
}

void writeServo(int pin, float us) {
  us = constrain(us, 500.0f, 2500.0f);
  ledcWrite(pin, (uint32_t)(us * 65535.0f / 20000.0f));    // 50 Hz, 16-bit
}

float readVolts(int pin) { return analogReadMilliVolts(pin) / 1000.0f; }

float readPsi() {
  float v_pin = readVolts(PIN_PSI);
  float v_sensor = v_pin * (4.7f + 10.0f) / 10.0f;          // undo the divider
  return max(0.0f, (v_sensor - cfg.psi_v0) * cfg.psi_k);
}

float readVin() { return readVolts(PIN_VIN) * (100.0f + 12.0f) / 12.0f; }

void setValves(bool on, uint32_t limitMs) {
  if (on && faults) return;
  valveOn = on;
  // open master first and close it last, so the zone valve never sees full line shock alone
  if (on) {
    digitalWrite(PIN_MASTER, HIGH);
    delay(50);
    digitalWrite(PIN_VALVE, HIGH);
    valveOpenedAt = millis();
    valveLimitMs = min((int32_t)limitMs, VALVE_MAX_MS);
  } else {
    digitalWrite(PIN_VALVE, LOW);
    delay(50);
    digitalWrite(PIN_MASTER, LOW);
    valveClosedAt = millis();
    leakWindowStart = valveClosedAt;
    leakWindowPulses = flowPulses;
  }
}

void endDose(const char *reason) {
  if (!dose.running) return;
  dose.ml = 1000.0f * (flowPulses - dose.startPulses) / cfg.flow_k;
  dose.ms = since(millis(), valveOpenedAt);
  dose.reason = reason;
  dose.running = false;
  setValves(false, 0);
}

void trip(uint8_t f) {
  faults |= f;
  if (dose.running) endDose("FAULT");
  else if (valveOn) setValves(false, 0);
  if (f & F_STUCK) {                    // survives a reboot: a human must check the valve
    prefs.begin("lwt", false);
    prefs.putUChar("stuck", 1);
    prefs.end();
  }
}

void loadCfg() {
  prefs.begin("lwt", true);
  cfg.pan_c = prefs.getFloat("pan_c", cfg.pan_c);
  cfg.tilt_c = prefs.getFloat("tilt_c", cfg.tilt_c);
  cfg.pan_k = prefs.getFloat("pan_k", cfg.pan_k);
  cfg.tilt_k = prefs.getFloat("tilt_k", cfg.tilt_k);
  cfg.flow_k = prefs.getFloat("flow_k", cfg.flow_k);
  cfg.psi_k = prefs.getFloat("psi_k", cfg.psi_k);
  cfg.psi_v0 = prefs.getFloat("psi_v0", cfg.psi_v0);
  cfg.nom_lpm = prefs.getFloat("nom_lpm", cfg.nom_lpm);
  if (prefs.getUChar("stuck", 0)) faults |= F_STUCK;
  prefs.end();
}

// returns 0 ok, 1 unknown key, 2 out of range
int saveCfg(const String &key, float v) {
  struct { const char *k; float *p; float lo, hi; bool absRange; } t[] = {
    {"pan_c", &cfg.pan_c, 1000, 2000, false}, {"tilt_c", &cfg.tilt_c, 1000, 2000, false},
    {"pan_k", &cfg.pan_k, 4, 20, true},       {"tilt_k", &cfg.tilt_k, 4, 20, true},
    {"flow_k", &cfg.flow_k, 100, 2000, false}, {"psi_k", &cfg.psi_k, 5, 100, false},
    {"psi_v0", &cfg.psi_v0, 0, 1, false},      {"nom_lpm", &cfg.nom_lpm, 1, 20, false},
  };
  for (auto &e : t) {
    if (key != e.k) continue;
    float a = e.absRange ? fabs(v) : v;
    if (!(a >= e.lo && a <= e.hi)) return 2;
    *e.p = v;
    prefs.begin("lwt", false);
    prefs.putFloat(e.k, v);
    prefs.end();
    return 0;
  }
  return 1;
}

// ----------------------------------------------------------------- commands
void handle(const String &text) {
  String tok[5];
  int n = 0, start = 0;
  for (int i = 0; i <= (int)text.length() && n < 5; i++) {
    if (i == (int)text.length() || text[i] == ' ') {
      if (i > start) tok[n++] = text.substring(start, i);
      start = i + 1;
    }
  }
  if (n == 0) return;
  const String &c = tok[0];

  if (c == "PING") {
    send(String("OK PONG ") + FW_VERSION);
  } else if (c == "MOVE" && n == 3) {
    panGoal = constrain(tok[1].toFloat(), PAN_MIN, PAN_MAX);
    tiltGoal = constrain(tok[2].toFloat(), TILT_MIN, TILT_MAX);
    // anti-backlash: go BACKLASH_DEG below each goal first, then come up to it,
    // so the gear train always rests against the same tooth flank
    panCmd = max(PAN_MIN, panGoal - BACKLASH_DEG);
    tiltCmd = max(TILT_MIN, tiltGoal - BACKLASH_DEG);
    float d = max(fabs(panCmd - panNow), fabs(tiltCmd - tiltNow)) + BACKLASH_DEG;
    send(String("OK MOVE ") + (uint32_t)(1000.0f * d / SLEW_DEG_S + 150));
  } else if (c == "VALVE" && n >= 2) {
    if (tok[1] == "1") {
      if (faults) { send(String("ERR FAULT ") + faults); return; }
      if (dose.running) { send("ERR BUSY"); return; }
      uint32_t lim = n >= 3 ? (uint32_t)tok[2].toInt() : 10000;
      setValves(true, lim);
      send("OK VALVE 1");
    } else {
      if (dose.running) endDose("STOP"); else setValves(false, 0);
      send("OK VALVE 0");
    }
  } else if (c == "DOSE" && n == 4) {
    long id = tok[1].toInt();
    if (id == dose.id) { send(String("OK DOSE ") + id); return; }   // retried command: do not restart
    if (faults) { send(String("ERR FAULT ") + faults); return; }
    if (valveOn) { send("ERR BUSY"); return; }
    float ml = tok[2].toFloat();
    if (!(ml > 0 && ml < 20000)) { send("ERR RANGE"); return; }
    dose.id = id;
    dose.startPulses = flowPulses;
    dose.targetPulses = (uint32_t)(ml / 1000.0f * cfg.flow_k + 0.5f);
    dose.ml = 0; dose.ms = 0; dose.reason = "-";
    setValves(true, (uint32_t)tok[3].toInt());
    dose.running = valveOn;
    send(String("OK DOSE ") + id);
  } else if (c == "DOSE?") {
    float ml = dose.running ? 1000.0f * (flowPulses - dose.startPulses) / cfg.flow_k : dose.ml;
    uint32_t ms = dose.running ? since(millis(), valveOpenedAt) : dose.ms;
    send(String("OK DOSEST ") + dose.id + " " + (dose.running ? "RUN" : "END") + " " +
         String(ml, 0) + " " + ms + " " + dose.reason);
  } else if (c == "STAT") {
    String s = String("OK STAT ") + String(panNow, 1) + " " + String(tiltNow, 1) + " " + (valveOn ? 1 : 0) +
               " " + String(lpm, 2) + " " + String(1000.0f * flowPulses / cfg.flow_k, 0) +
               " " + String(readPsi(), 1) + " " + String(readVin(), 1);
    for (int i = 0; i < 3; i++) s += " " + String(analogReadMilliVolts(PIN_PROBE[i]));
    s += " " + String(faults) + " " + String(millis() / 1000);
    send(s);
  } else if (c == "CFG" && n == 3) {
    int r = saveCfg(tok[1], tok[2].toFloat());
    send(r == 0 ? "OK CFG" : r == 1 ? "ERR KEY" : "ERR RANGE");
  } else if (c == "CLR") {
    if (n >= 2 && tok[1] == "STUCK") {
      faults = 0;
      prefs.begin("lwt", false);
      prefs.putUChar("stuck", 0);
      prefs.end();
    } else {
      faults &= F_STUCK;
    }
    send("OK CLR");
  } else if (c == "PARK") {
    if (dose.running) endDose("PARK"); else setValves(false, 0);
    panGoal = panCmd = panNow;
    tiltGoal = tiltCmd = TILT_MIN;
    send("OK PARK");
  } else {
    send("ERR CMD");
  }
}

void pollSerial() {
  while (Serial2.available()) {
    char ch = (char)Serial2.read();
    if (ch == '\r') continue;
    if (ch != '\n') {
      if (rx.length() < 96) rx += ch;
      continue;
    }
    int star = rx.lastIndexOf('*');
    if (star > 0 && star + 3 == (int)rx.length()) {
      String text = rx.substring(0, star);
      uint8_t want = (uint8_t)strtol(rx.substring(star + 1).c_str(), nullptr, 16);
      if (checksum(text) == want) {
        lastFrameMs = millis();
        handle(text);
      } else {
        send("ERR CHECKSUM");
      }
    }
    rx = "";
  }
}

// ----------------------------------------------------------------- setup / loop
void setup() {
  pinMode(PIN_VALVE, OUTPUT);
  digitalWrite(PIN_VALVE, LOW);                 // valves closed before anything else
  pinMode(PIN_MASTER, OUTPUT);
  digitalWrite(PIN_MASTER, LOW);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_FLOW, INPUT);                     // external pull-up through the divider
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), onFlowPulse, FALLING);
  analogSetAttenuation(ADC_11db);
  loadCfg();
  ledcAttach(PIN_PAN, 50, 16);
  ledcAttach(PIN_TILT, 50, 16);
  writeServo(PIN_PAN, cfg.pan_c + cfg.pan_k * panNow);
  writeServo(PIN_TILT, cfg.tilt_c + cfg.tilt_k * tiltNow);
  Serial2.begin(115200, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
  Serial2.setPins(PIN_RS485_RX, PIN_RS485_TX, -1, PIN_RS485_DE);   // RTS pin drives DE
  Serial2.setMode(UART_MODE_RS485_HALF_DUPLEX);
  esp_task_wdt_config_t wdt = { .timeout_ms = WDT_TIMEOUT_MS, .idle_core_mask = 0, .trigger_panic = true };
  esp_task_wdt_reconfigure(&wdt);
  esp_task_wdt_add(NULL);
  lastFrameMs = valveClosedAt = leakWindowStart = millis();
  leakWindowPulses = flowPulses;
}

void loop() {
  static uint32_t lastServo = millis();
  esp_task_wdt_reset();
  pollSerial();
  uint32_t now = millis();                      // read AFTER the poll: no stamp can be newer than now

  // servo slew at SLEW_DEG_S, updated every 20 ms; second leg of the anti-backlash move
  if (since(now, lastServo) >= 20) {
    float step = SLEW_DEG_S * since(now, lastServo) / 1000.0f;
    lastServo = now;
    panNow += constrain(panCmd - panNow, -step, step);
    tiltNow += constrain(tiltCmd - tiltNow, -step, step);
    if (fabs(panNow - panCmd) < 0.01f && fabs(tiltNow - tiltCmd) < 0.01f) {
      panCmd = panGoal;
      tiltCmd = tiltGoal;
    }
    writeServo(PIN_PAN, cfg.pan_c + cfg.pan_k * panNow);
    writeServo(PIN_TILT, cfg.tilt_c + cfg.tilt_k * tiltNow);
  }

  // flow rate over 500 ms windows
  if (since(now, lastFlowCalcMs) >= 500) {
    uint32_t p = flowPulses;
    lpm = (p - lastFlowPulses) / cfg.flow_k * 60000.0f / since(now, lastFlowCalcMs);
    lastFlowPulses = p;
    lastFlowCalcMs = now;
  }

  // ---- safety interlocks
  if (valveOn) {
    int32_t open = since(now, valveOpenedAt);
    if (since(now, lastFrameMs) > LINK_TIMEOUT_MS) trip(F_LINK);
    else if (dose.running && flowPulses - dose.startPulses >= dose.targetPulses) endDose("DONE");
    else if (open > (int32_t)valveLimitMs) {
      if (dose.running) endDose("TIMEOUT");
      trip(F_TIMEOUT);
    } else if (open > HIGH_FLOW_CHECK_MS && lpm > HIGH_FLOW_FACTOR * cfg.nom_lpm) trip(F_HIGHFLOW);
    else if (open > FLOW_CHECK_MS && lpm < NO_FLOW_LPM) trip(F_NOFLOW);
    else if (open > FLOW_CHECK_MS && readPsi() < LOW_PSI) trip(F_LOWPSI);
  } else if (since(now, valveClosedAt) > LEAK_SETTLE_MS) {
    if (since(now, leakWindowStart) >= LEAK_WINDOW_MS) {
      if (flowPulses - leakWindowPulses >= LEAK_PULSES && !(faults & F_STUCK)) trip(F_STUCK);
      leakWindowStart = now;
      leakWindowPulses = flowPulses;
    }
  } else {
    leakWindowStart = now;                      // still draining: restart the window
    leakWindowPulses = flowPulses;
  }

  // status LED: solid = valve open, fast blink = fault, slow blink = idle
  bool led = valveOn ? true : faults ? ((now / 150) % 2) : ((now / 1000) % 2);
  digitalWrite(PIN_LED, led);
}
