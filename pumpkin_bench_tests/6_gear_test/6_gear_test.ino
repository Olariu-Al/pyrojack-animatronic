// TEST 6 - GEAR MECHANISM: runs the sector gear, ratchet, rack and deadbolt through every step of the cycle
// and checks each step with the hall sensor (is the gear parked?) and the lid switch (is the trapdoor shut?).
// Board: ESP32 DevKit (ESP32-WROOM-32, "ESP32 Dev Module"). PlatformIO: env:gear_test. Library: ESP32Servo.
// Calibrate the MG996R with 4_zero_test first and copy REVERSED / TRIM below.
//
// Wiring: MG996R signal -> GPIO26, MG90S signal -> GPIO27, buzzer -> GPIO25 (servos on the 6 V rail, common GND)
//         A3144 hall: VCC -> 3V3, GND -> GND, OUT -> GPIO32   (open collector, uses the internal pull-up)
//         KW11-3Z lid switch: COM -> GND, NO -> GPIO33         (closed = lid shut)
//
// The cycle (cat_eye_pumpkin.scad):
//   ARMED     horn 0, gear parked (hall ON), lid shut and bolted, plunger pressed down by the lid
//   1 FIRE    bolt opens, the spring throws the lid open and the flame up 47.2 mm
//   2 COCK    horn 0 -> 134: the gear picks the rack up at ~9 deg and pulls the flame back down,
//             then the horn goes limp for 1 s - the anti-reverse pawl alone must hold the rack
//   3 LID     you close the trapdoor by hand -> lid switch
//   4 RELEASE bolt in, horn 134 -> 180: the last tooth slips off at ~162 deg, the spring now pushes on the lid
//   5 RESET   horn 180 -> 0: the drive pawls ratchet back, the gear must stay parked (hall stays ON)
//
// Commands (type, then Enter; + - x m work without Enter). Add ! to skip a safety check, e.g. 4!
//   1 2 3 4 5   run one step                    c      full cycle (waits for you to close the lid)
//   d           dry cycle: ignores the lid switch - run it with the lid off, the flame fires up at every release
//   e<n>        endurance: n dry cycles, stops at the first failure, e.g. e20
//   h<s>        hold test: horn limp at the hold angle for s seconds, the gear must not slip back, e.g. h30
//   w           slow mode on / off: 1 deg every 150 ms, to watch the pick-up and the slip
//   m           mark: print the horn angle now (press it when the rack picks up or slips off)
//   a<deg>      move the horn, e.g. a90         + / -  jog by j<deg> (default 1)    v<ms>  manual speed, ms per deg
//   l / o       bolt lock / open (l<deg> / o<deg> also set that angle)
//   g<deg>      hold angle (default 134)        i      show the servos and sensors
//   x           STOP: both servos limp          p      print the settings to copy into the other sketches

#include <ESP32Servo.h>
#include <stdarg.h>

const int PIN_MG996R = 26, PIN_MG90S = 27, PIN_BUZZ = 25, PIN_HALL = 32, PIN_LID = 33;

// ---------------- calibration (from 4_zero_test) ----------------
const bool  REVERSED = false;   // MG996R direction
const float TRIM     = 0;       // MG996R zero trim, degrees
const int   CK_MIN_US = 500, CK_MAX_US = 2500;   // MG996R pulse at 0 / 180 deg
int LOCK_DEG = 45, OPEN_DEG = 135;               // MG90S deadbolt locked / open

// ---------------- mechanism (cat_eye_pumpkin.scad) ----------------
float HOLD_DEG = 134;           // HOLD_G = 132 is the last tooth on the pitch line; +2 so the anti-reverse pawl drops in
const float PICKUP_DEG = 9, SLIP_DEG = 162.25;   // leading tooth picks up the rack / last tooth slips off (REL_G)
const int   COCK_MS = 8, RELEASE_MS = 6, RESET_MS = 3, SLOW_MS = 150;   // ms per degree
const float HALL_OFF_BY = 20;   // cocking from 0: the gear must have left the park magnet by this horn angle
const uint32_t BOLT_MS = 300, LID_OPEN_MS = 1500, LID_WAIT_MS = 60000, HOLD_CHECK_MS = 1000;
const bool  HALL_ACTIVE_LOW = true, LID_SHUT_LOW = true;

Servo cock, latch;
float cockPos = -1;             // logical horn angle, -1 = unknown (limp since boot)
bool boltLocked = false, boltKnown = false, slowMode = false;
int manualMs = 8; float jogDeg = 1;

// ---------------- servos ----------------
void cockWrite(float logical) {
  float s = logical + TRIM;
  if (REVERSED) s = 180 - s;
  s = constrain(s, 0.0f, 180.0f);
  if (!cock.attached()) cock.attach(PIN_MG996R, CK_MIN_US, CK_MAX_US);
  cock.writeMicroseconds(CK_MIN_US + (int)(s * (CK_MAX_US - CK_MIN_US) / 180.0f));
  cockPos = logical;
}
void latchWrite(int deg) {
  if (!latch.attached()) latch.attach(PIN_MG90S, 500, 2500);
  latch.writeMicroseconds(500 + constrain(deg, 0, 180) * 2000 / 180);
}
void beep(int ms) { digitalWrite(PIN_BUZZ, HIGH); delay(ms); digitalWrite(PIN_BUZZ, LOW); }

// ---------------- sensors (edges are logged with the horn angle) ----------------
bool hallState = false, lidState = false, lidRaw = false;
uint32_t lidChangedAt = 0;
int hallOnEvents = 0, hallOffEvents = 0;
float firstHallOffAt = -1;

bool readHall() { return digitalRead(PIN_HALL) == (HALL_ACTIVE_LOW ? LOW : HIGH); }
bool readLid()  { return digitalRead(PIN_LID) == (LID_SHUT_LOW ? LOW : HIGH); }
void resetWatch() { hallOnEvents = hallOffEvents = 0; firstHallOffAt = -1; }

void pollInputs() {
  bool h = readHall();
  if (h != hallState) {
    hallState = h;
    if (h) hallOnEvents++;
    else { hallOffEvents++; if (firstHallOffAt < 0) firstHallOffAt = cockPos; }
    Serial.printf("    hall %s (horn %.0f)\n", h ? "ON - gear parked" : "off", cockPos);
  }
  bool l = readLid();                                       // 20 ms debounce
  if (l != lidRaw) { lidRaw = l; lidChangedAt = millis(); }
  if (lidRaw != lidState && millis() - lidChangedAt > 20) {
    lidState = lidRaw;
    Serial.printf("    lid %s\n", lidState ? "SHUT" : "open");
  }
}

// ---------------- results ----------------
int fails = 0;
void report(const char *tag, const char *fmt, va_list ap) {
  char msg[200];
  vsnprintf(msg, sizeof msg, fmt, ap);
  Serial.printf("  %s  %s\n", tag, msg);
}
void pass(const char *fmt, ...) { va_list ap; va_start(ap, fmt); report("PASS", fmt, ap); va_end(ap); }
void warn(const char *fmt, ...) { va_list ap; va_start(ap, fmt); report("WARN", fmt, ap); va_end(ap); }
void fail(const char *fmt, ...) {
  va_list ap; va_start(ap, fmt); report("FAIL", fmt, ap); va_end(ap);
  fails++; beep(80); delay(80); beep(80);
}
bool blocked(const char *fmt, ...) { va_list ap; va_start(ap, fmt); report("NO  ", fmt, ap); va_end(ap); return false; }

// ---------------- keyboard while a step runs ----------------
bool aborted = false, busyNoted = false;

void stopAll() {
  cock.detach(); latch.detach(); digitalWrite(PIN_BUZZ, LOW);
  aborted = true;
  Serial.println("STOPPED - both servos limp");
}
void mark() {
  Serial.printf("  MARK  horn %.0f deg   (pick-up expected ~%.0f, slip ~%.0f)\n", cockPos, PICKUP_DEG, SLIP_DEG);
}

// keeps the sensors and the keyboard alive during a step; false once x was pressed
bool service() {
  pollInputs();
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'x' || ch == 'X') stopAll();
    else if (ch == 'm') mark();
    else if (ch > ' ' && !busyNoted) { busyNoted = true; Serial.println("  (busy - x stops, m marks)"); }
  }
  return !aborted;
}
bool waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  do { if (!service()) return false; delay(1); } while (millis() - t0 < ms);
  return true;
}

bool moveTo(float target, int msPerDeg) {
  target = constrain(target, 0.0f, 180.0f);
  if (cockPos < 0) {                                        // unknown: only a move back to 0 is safe (the ratchet free-wheels)
    if (target > 0.5f) return blocked("horn position unknown - run 5 first (moving back to 0 is always safe)");
    cockWrite(0);
    return waitMs(800);
  }
  if (slowMode) msPerDeg = max(msPerDeg, SLOW_MS);
  while (fabsf(target - cockPos) >= 1.0f) {
    cockWrite(cockPos + (target > cockPos ? 1.0f : -1.0f));
    if (!waitMs(msPerDeg)) return false;
  }
  cockWrite(target);
  return true;
}
bool bolt(bool lock) {
  latchWrite(lock ? LOCK_DEG : OPEN_DEG);
  boltLocked = lock; boltKnown = true;
  bool ok = waitMs(BOLT_MS);
  latch.detach();
  return ok;
}

// turning the gear forward with the lid shut drives the rack (still pressed down by the lid) into the channel bottom
bool forwardAllowed(float target, bool force) {
  if (force || target <= cockPos + 0.5f || !lidState) return true;
  return blocked("lid is shut, so the plunger is still pressed down: turning the gear forward would jam the rack. Fire first (1) or add !");
}

// ---------------- the steps ----------------
bool stepFire(bool dry) {
  Serial.println("STEP 1 FIRE: bolt open");
  bool wasShut = lidState;
  uint32_t t0 = millis();
  digitalWrite(PIN_BUZZ, HIGH);
  bool ok = bolt(false);
  digitalWrite(PIN_BUZZ, LOW);
  if (!ok || dry) return ok;
  if (!wasShut) { warn("lid was already open - nothing to fire"); return true; }
  while (lidState) {
    if (!waitMs(5)) return false;
    if (millis() - t0 > LID_OPEN_MS) { fail("lid still shut 1.5 s after the bolt opened - bolt stuck in the keeper or plunger not loaded"); return false; }
  }
  pass("lid opened %lu ms after the bolt command", (unsigned long)(millis() - t0));
  return true;
}

bool holdCheck(uint32_t ms) {
  cock.detach();                                            // the anti-reverse pawl takes the load
  resetWatch();
  if (!waitMs(ms)) return false;
  if (hallOnEvents) { fail("gear slipped back to the park position with the horn limp - anti-reverse pawl not engaging"); return false; }
  pass("anti-reverse pawl held the rack for %.1f s with the horn limp", ms / 1000.0f);
  return true;
}

bool stepCock(bool force, bool dry) {
  Serial.printf("STEP 2 COCK: horn %.0f -> %.0f\n", cockPos, HOLD_DEG);
  if (cockPos < 0) return blocked("horn position unknown - run 5 first");
  if (cockPos >= HOLD_DEG - 1) return blocked("already at the hold angle - run 4 (release) or 5 (reset)");
  if (!dry && !forwardAllowed(HOLD_DEG, force)) return false;
  int f0 = fails;
  bool fromZero = cockPos < 1, parked = hallState;
  if (fromZero && !parked) warn("hall off at 0 deg - gear not parked, or the magnet / sensor is misaligned");
  if (slowMode) Serial.printf("  watch the rack: it should start moving at ~%.0f deg - press m when it does\n", PICKUP_DEG);
  resetWatch();
  uint32_t t0 = millis();
  if (!moveTo(HOLD_DEG, COCK_MS)) return false;
  Serial.printf("  cocked in %lu ms\n", (unsigned long)(millis() - t0));
  if (fromZero && parked) {
    if (firstHallOffAt < 0) fail("gear never left the park position - drive pawls not catching the ratchet (disk loose on the horn?)");
    else if (firstHallOffAt > HALL_OFF_BY) fail("gear only started moving at %.0f deg - pawls skipped a tooth or the zero is off (4_zero_test)", firstHallOffAt);
    else pass("gear followed the horn (left the park magnet at %.0f deg)", firstHallOffAt);
  }
  if (hallState) fail("hall ON at the hold angle - the gear is not where the horn is");
  if (!waitMs(300)) return false;
  return holdCheck(HOLD_CHECK_MS) && fails == f0;
}

bool stepLid(bool dry) {
  if (dry) { Serial.println("STEP 3 LID: dry run - not waiting for the lid"); return waitMs(500); }
  Serial.println("STEP 3 LID: close the trapdoor now (60 s, x cancels)");
  uint32_t t0 = millis();
  while (!lidState) {
    if (!waitMs(10)) return false;
    if (millis() - t0 > LID_WAIT_MS) { fail("lid not closed within 60 s - still holding, nothing released"); return false; }
  }
  pass("lid shut");
  return waitMs(500);
}

bool stepRelease(bool force, bool dry) {
  Serial.printf("STEP 4 RELEASE: bolt in, horn %.0f -> 180\n", cockPos);
  if (cockPos < HOLD_DEG - 5) return blocked("not cocked - run 2 first");
  if (cockPos >= 179) return blocked("already released - run 5 (reset)");
  if (!dry && !force && !lidState) return blocked("lid is open: releasing now fires the flame straight back up. Close the lid (3) or add !");
  int f0 = fails;
  if (!bolt(true)) return false;
  if (slowMode) Serial.printf("  watch the last tooth: it should slip off at ~%.0f deg - press m when it does\n", SLIP_DEG);
  resetWatch();
  if (!moveTo(180, RELEASE_MS)) return false;
  if (!waitMs(300)) return false;
  if (hallState) pass("gear parked at 180 (hall ON)");
  else fail("hall off at 180 - the gear did not reach its park position (skipped tooth or hall misaligned)");
  if (!dry) {
    if (lidState) pass("lid still shut and bolted - the flame is armed");
    else fail("lid opened at the release - the bolt did not hold the spring");
  }
  return fails == f0;
}

bool stepReset() {
  Serial.println("STEP 5 RESET: horn -> 0, the gear must stay parked");
  bool known = cockPos >= 0, parked = hallState;
  int f0 = fails;
  resetWatch();
  if (!moveTo(0, RESET_MS)) return false;
  if (!waitMs(300)) return false;
  cock.detach();
  if (!known) { Serial.println("  horn at 0"); return true; }
  if (!parked) warn("gear was not parked before the reset - free-wheel check skipped");
  else if (hallOffEvents) fail("gear moved during the reset (hall went off %d times) - the drive pawls drag it back past the anti-reverse pawl", hallOffEvents);
  else pass("ratchet free-wheeled, the gear stayed parked");
  return fails == f0;
}

bool holdTest(float s) {
  Serial.printf("HOLD TEST: horn limp for %.0f s\n", s);
  if (cockPos < HOLD_DEG - 5 || cockPos >= 179) return blocked("cock first (2)");
  return holdCheck((uint32_t)(s * 1000));
}

bool cycle(bool dry) {
  if (cockPos < 0 || cockPos > 1) return blocked("the horn must be at 0 - run 5 (reset) first");
  Serial.printf("=== %s CYCLE ===\n", dry ? "DRY" : "FULL");
  uint32_t t0 = millis();
  bool ok;
  if (!dry && !lidState) { Serial.println("STEP 1 skipped: lid already open"); ok = true; }
  else ok = stepFire(dry);
  ok = ok && stepCock(false, dry) && stepLid(dry) && stepRelease(false, dry) && stepReset();
  Serial.printf("=== cycle %s, %.1f s ===\n", ok ? "OK" : aborted ? "STOPPED" : "FAILED", (millis() - t0) / 1000.0f);
  return ok;
}

void endurance(int n) {
  Serial.printf("ENDURANCE: %d dry cycles (x stops)\n", n);
  int done = 0;
  uint32_t t0 = millis();
  while (done < n && cycle(true)) { done++; Serial.printf("--- %d / %d ---\n", done, n); if (!waitMs(500)) break; }
  Serial.printf("ENDURANCE: %d / %d cycles OK, %.1f s per cycle\n", done, n, done ? (millis() - t0) / 1000.0f / done : 0.0f);
}

// ---------------- controls ----------------
void info() {
  char horn[24];
  if (cockPos < 0) strcpy(horn, "unknown");
  else snprintf(horn, sizeof horn, "%.0f deg", cockPos);
  Serial.printf("horn %s (%s) | bolt %s | hall %s | lid %s | hold %.0f | slow %s | fails %d\n",
                horn, cock.attached() ? "powered" : "limp", boltKnown ? (boltLocked ? "LOCKED" : "open") : "?",
                hallState ? "ON (parked)" : "off", lidState ? "SHUT" : "open", HOLD_DEG, slowMode ? "on" : "off", fails);
}
void printSettings() {
  Serial.println("Settings (copy into the other sketches):");
  Serial.printf("  const int LOCK_DEG = %d;\n  const int OPEN_DEG = %d;\n  hold angle = %.0f\n", LOCK_DEG, OPEN_DEG, HOLD_DEG);
}
void help() {
  Serial.println("1-5 one step | c full cycle | d dry cycle | e<n> endurance | h<s> hold test | w slow | m mark");
  Serial.println("a<deg> move | + - jog | j<deg> jog step | v<ms> speed | l o bolt | g<deg> hold angle | i info | x STOP | p settings");
  Serial.println("add ! to skip a safety check, e.g. 4!");
}

void command(String in) {
  in.trim();
  if (in.length() == 0) return;
  bool force = in.endsWith("!");
  if (force) in.remove(in.length() - 1);
  char c = in.charAt(0);
  String arg = in.substring(1);
  float v = arg.toFloat();
  bool hasArg = arg.length() > 0;
  aborted = false; busyNoted = false;
  switch (c) {
    case '1': stepFire(false); break;
    case '2': stepCock(force, false); break;
    case '3': stepLid(false); break;
    case '4': stepRelease(force, false); break;
    case '5': stepReset(); break;
    case 'c': cycle(false); break;
    case 'd': cycle(true); break;
    case 'e': endurance(hasArg ? max(1, (int)v) : 10); break;
    case 'h': holdTest(hasArg ? v : 10); break;
    case 'w': slowMode = !slowMode; Serial.printf("Slow mode %s\n", slowMode ? "on" : "off"); return;
    case 'm': mark(); return;
    case 'a': if (forwardAllowed(v, force)) moveTo(v, manualMs); break;
    case '+': if (forwardAllowed(cockPos + jogDeg, false)) moveTo(cockPos + jogDeg, manualMs); break;
    case '-': moveTo(cockPos - jogDeg, manualMs); break;
    case 'j': jogDeg = constrain(v, 0.5f, 30.0f); Serial.printf("Jog step %.1f deg\n", jogDeg); return;
    case 'v': manualMs = constrain((int)v, 1, 500); Serial.printf("Manual speed %d ms per deg\n", manualMs); return;
    case 'l': if (hasArg) LOCK_DEG = constrain((int)v, 0, 180); bolt(true); break;
    case 'o': if (hasArg) OPEN_DEG = constrain((int)v, 0, 180); bolt(false); break;
    case 'g': HOLD_DEG = constrain(v, 120.0f, 160.0f); Serial.printf("Hold angle %.0f deg\n", HOLD_DEG); return;
    case 'i': break;
    case 'x': stopAll(); return;
    case 'p': printSettings(); return;
    default: help(); return;
  }
  info();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  cock.setPeriodHertz(50); latch.setPeriodHertz(50);
  pinMode(PIN_BUZZ, OUTPUT); digitalWrite(PIN_BUZZ, LOW);
  pinMode(PIN_HALL, INPUT_PULLUP);
  pinMode(PIN_LID, INPUT_PULLUP);
  hallState = readHall(); lidState = lidRaw = readLid();
  Serial.println("Gear mechanism test - servos limp until you send a command");
  if (!hallState) Serial.println("Hall off: the gear is not parked, or the sensor is not wired (GPIO32)");
  help();
  info();
  Serial.println("Start with 5 (horn to 0), then c (full cycle) or d (dry cycle).");
}

String buf;

void loop() {
  pollInputs();
  while (Serial.available()) {
    char ch = Serial.read();
    if (buf.length() == 0 && (ch == '+' || ch == '-' || ch == 'x' || ch == 'm')) { command(String(ch)); continue; }   // instant keys
    if (ch == '\n' || ch == '\r') { command(buf); buf = ""; }
    else buf += ch;
  }
}
