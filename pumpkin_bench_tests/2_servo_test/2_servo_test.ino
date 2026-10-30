// TEST 2 - GEAR SERVO ONLY: automatic MG996R test, also controllable from the Serial Monitor (115200)
// Run this with the horn OFF, or before the plunger is fitted: a move past ~162 deg releases the plunger.
// Works on ESP32 DevKit (ESP32-WROOM-32, board "ESP32 Dev Module"), ESP8266 (NodeMCU) and ESP32-S3 -
// just pick the board in Tools > Board. PlatformIO: env:servo_test.
// ESP32 needs the "ESP32Servo" library; the ESP8266 uses its built-in Servo library.
//
// Starts on its own 3 s after boot and repeats the mechanism cycle, 2 s between rounds:
//   at 0 -> COCK 0 -> 134 -> HOLD 1 s limp -> RELEASE 134 -> 180 -> RESET 180 -> 0
// Seen from the front (horn side) the cock must turn CLOCKWISE. If it turns the other way, send f.
// x stops the automatic test (servo limp), t starts it again. While it is stopped you can use:
//
// Commands (type, then Enter; + - x work without Enter):
//   a<deg>   move to an angle, e.g. a90           + / -   jog by j<deg> (default 1)
//   j<deg>   jog step                              v<ms>   speed for a / + / -, ms per degree (default 8)
//   s        full sweep 0 -> 180 -> 0
//   c        cock     0 -> hold angle (pulls the rack down)
//   r        release  hold -> 180 (last tooth slips off at ~162)
//   z        reset    -> 0 (ratchet free-wheels back)
//   k        scare motion: cock, hold 1 s limp, release, reset
//   l<n>     loop the scare motion n times, 2 s apart, e.g. l10
//   g<deg>   hold angle (default 134)
//   t        start the automatic test again          f   flip the direction (copy the result into the other sketches)
//   i        status                                 x   STOP: servo limp, automatic test off

// ---------------- servo pin ----------------
#if defined(ESP8266)
  #include <Servo.h>
  const int PIN_MG996R = 12;  // D6
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  #include <ESP32Servo.h>
  const int PIN_MG996R = 17;
#else
  // ESP32 DevKit
  #include <ESP32Servo.h>
  const int PIN_MG996R = 26;
#endif

// ---------------- calibration (from 4_zero_test) ----------------
bool        REVERSED = true;    // MG996R direction (f flips it live)
const float TRIM     = 0;       // MG996R zero trim, degrees
const int   CK_MIN_US = 500, CK_MAX_US = 2500;   // MG996R pulse at 0 / 180 deg - narrow it if the servo buzzes at an end

// ---------------- motion ----------------
float HOLD_DEG = 134;                            // last tooth on the rack (+2 deg so the anti-reverse pawl drops in)
const int COCK_MS = 8, RELEASE_MS = 6, RESET_MS = 3, SWEEP_MS = 10;   // ms per degree
int moveMs = 8; float jogDeg = 1;

Servo cock;
float cockPos = -1;             // logical angle, -1 = unknown (limp since boot)
bool aborted = false, autoMode = true;
const uint32_t AUTO_DELAY_MS = 3000, AUTO_GAP_MS = 2000;   // start after boot / pause between rounds

void cockWrite(float logical) {
  float s = logical + TRIM;
  if (REVERSED) s = 180 - s;
  s = constrain(s, 0.0f, 180.0f);
  if (!cock.attached()) cock.attach(PIN_MG996R, CK_MIN_US, CK_MAX_US);
  cock.writeMicroseconds(CK_MIN_US + (int)(s * (CK_MAX_US - CK_MIN_US) / 180.0f));
  cockPos = logical;
}

void stopServo() {
  cock.detach(); aborted = true;
  if (autoMode) { autoMode = false; Serial.println("STOPPED - servo limp, automatic test off (t starts it again)"); }
  else Serial.println("STOPPED - servo limp");
}

// waits while still listening for x; false once stopped
bool waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  do {
    while (Serial.available()) {
      char ch = Serial.read();
      if (ch == 'x') stopServo();
      else if (ch > ' ' && autoMode) Serial.println("  (automatic test running - x stops it)");
    }
    if (aborted) return false;
    delay(1);
  } while (millis() - t0 < ms);
  return true;
}

bool moveTo(float target, int msPerDeg) {
  target = constrain(target, 0.0f, 180.0f);
  if (cockPos < 0) {                             // position unknown: jump there at the servo's own speed
    Serial.printf("Position unknown - jumping straight to %.0f\n", target);
    cockWrite(target);
    return waitMs(800);
  }
  while (fabsf(target - cockPos) >= 1.0f) {
    cockWrite(cockPos + (target > cockPos ? 1.0f : -1.0f));
    if (!waitMs(msPerDeg)) return false;
  }
  cockWrite(target);
  return true;
}

bool scare() {
  if (cockPos != 0) { Serial.println("  to 0"); if (!moveTo(0, RESET_MS) || !waitMs(300)) return false; }
  Serial.printf("  COCK 0 -> %.0f (clockwise from the front)\n", HOLD_DEG);
  uint32_t t0 = millis();
  if (!moveTo(HOLD_DEG, COCK_MS)) return false;
  Serial.printf("  HOLD 1 s, servo limp (cocked in %lu ms)\n", (unsigned long)(millis() - t0));
  if (!waitMs(300)) return false;
  cock.detach();                                 // the anti-reverse pawl holds the rack
  if (!waitMs(1000)) return false;
  Serial.printf("  RELEASE %.0f -> 180\n", HOLD_DEG);
  if (!moveTo(180, RELEASE_MS) || !waitMs(200)) return false;
  Serial.println("  RESET 180 -> 0");
  if (!moveTo(0, RESET_MS) || !waitMs(300)) return false;
  cock.detach();
  return true;
}

void loopScare(int n) {
  int done = 0;
  while (done < n && scare()) {
    done++;
    Serial.printf("--- %d / %d ---\n", done, n);
    if (done < n && !waitMs(2000)) break;
  }
  Serial.printf("Loop: %d / %d scare motions done\n", done, n);
}

uint32_t rounds = 0;

bool autoRound() {
  Serial.printf("=== automatic round %lu ===\n", (unsigned long)(rounds + 1));
  uint32_t t0 = millis();
  if (!scare()) return false;
  rounds++;
  Serial.printf("=== round %lu done in %.1f s - next in %lu s ===\n", (unsigned long)rounds, (millis() - t0) / 1000.0f, (unsigned long)(AUTO_GAP_MS / 1000));
  return true;
}

void status() {
  if (cockPos < 0) Serial.print("angle unknown");
  else Serial.printf("angle %.0f", cockPos);
  Serial.printf(" (%s) | reversed %s | speed %d ms/deg | jog %.1f | hold %.0f | auto %s, %lu rounds\n", cock.attached() ? "powered" : "limp",
                REVERSED ? "true" : "false", moveMs, jogDeg, HOLD_DEG, autoMode ? "on" : "off", (unsigned long)rounds);
}

void help() {
  Serial.println("a<deg> move | + - jog | j<deg> jog step | v<ms> speed | s sweep | c cock | r release | z reset");
  Serial.println("k scare motion | l<n> loop it n times | g<deg> hold angle | t automatic test | f flip direction | i status | x STOP");
}

void command(String in) {
  in.trim();
  if (in.length() == 0) return;
  char c = in.charAt(0);
  String arg = in.substring(1);
  float v = arg.toFloat();
  aborted = false;
  switch (c) {
    case 'a': moveTo(v, moveMs); break;
    case '+': moveTo(cockPos + jogDeg, moveMs); break;
    case '-': moveTo(cockPos - jogDeg, moveMs); break;
    case 'j': jogDeg = constrain(v, 0.5f, 90.0f); break;
    case 'v': moveMs = constrain((int)v, 1, 500); break;
    case 's': Serial.println("Sweep 0 -> 180 -> 0"); if (moveTo(0, RESET_MS) && waitMs(300) && moveTo(180, SWEEP_MS) && waitMs(500)) moveTo(0, SWEEP_MS / 2); break;
    case 'c': moveTo(HOLD_DEG, COCK_MS); break;
    case 'r': moveTo(180, RELEASE_MS); break;
    case 'z': moveTo(0, RESET_MS); break;
    case 'k': scare(); break;
    case 'l': loopScare(arg.length() ? max(1, (int)v) : 5); break;
    case 'g': HOLD_DEG = constrain(v, 90.0f, 179.0f); break;
    case 't': autoMode = true; Serial.println("Automatic test on - x stops it"); return;
    case 'f':
      REVERSED = !REVERSED;
      Serial.printf("Direction flipped: const bool REVERSED = %s;  (copy into 3_full_bench, 4_zero_test, 6_gear_test)\n", REVERSED ? "true" : "false");
      if (cockPos >= 0 && cock.attached()) cockWrite(cockPos);   // the horn swaps to the mirrored angle
      break;
    case 'i': break;
    case 'x': stopServo(); return;
    default: help(); return;
  }
  status();
}

void setup() {
  Serial.begin(115200);
  delay(500);
#if !defined(ESP8266)
  ESP32PWM::allocateTimer(0);
  cock.setPeriodHertz(50);
#endif
  Serial.printf("Gear servo test - automatic test starts in %lu s (x stops it)\n", (unsigned long)(AUTO_DELAY_MS / 1000));
  help();
  waitMs(AUTO_DELAY_MS);
}

String buf;

void loop() {
  if (autoMode) {
    aborted = false;
    if (autoRound()) waitMs(AUTO_GAP_MS);
    return;
  }
  while (Serial.available()) {
    char ch = Serial.read();
    if (buf.length() == 0 && (ch == '+' || ch == '-' || ch == 'x')) { command(String(ch)); continue; }   // instant keys
    if (ch == '\n' || ch == '\r') { command(buf); buf = ""; }
    else buf += ch;
  }
}
