// TEST 4 - MG996R ZERO CALIBRATION: find REVERSED and TRIM, then copy them into the other tests
// Works on ESP32 DevKit (ESP32-WROOM-32, board "ESP32 Dev Module"), ESP8266 (NodeMCU) and ESP32-S3 -
// just pick the board in Tools > Board. ESP32 needs the "ESP32Servo" library.
// Type commands in the Serial Monitor (115200) and press Enter. "+" and "-" work without Enter.
//
//   w        direction test on the BARE spline: seen from the front it must turn clockwise, else send v
//   v        flip direction (REVERSED)
//   z        hold logical 0 deg - fit the drive disk now (pawl just touching a ratchet tooth)
//   t<deg>   set trim, e.g. t3 or t-4.5      + / -   nudge trim by 0.5 deg
//   a<deg>   move to a logical angle, e.g. a132 (hold on the last tooth), a180 (slip release)
//   c        check cycle: 0 -> 134 (hold 2 s) -> 180 -> back to 0
//   x        servo limp (no signal)
//   p        print the values to copy into the sketches

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

// ---------------- starting values (edit to resume a previous calibration) ----------------
bool  REVERSED = false;   // MG996R direction
float TRIM     = 0;       // MG996R zero trim, degrees
const int CK_MIN_US = 500, CK_MAX_US = 2500;   // MG996R pulse at 0 / 180 deg

Servo cock;
float cockPos = 0;

void cockWrite(float logical) {
  float s = logical + TRIM;
  if (REVERSED) s = 180 - s;
  s = constrain(s, 0.0f, 180.0f);
  if (!cock.attached()) cock.attach(PIN_MG996R, CK_MIN_US, CK_MAX_US);
  cock.writeMicroseconds(CK_MIN_US + (int)(s * (CK_MAX_US - CK_MIN_US) / 180.0f));
  cockPos = logical;
}

void sweepCock(float target, int msPerDeg) {
  while (fabsf(target - cockPos) >= 1.0f) {
    cockWrite(cockPos + (target > cockPos ? 1.0f : -1.0f));
    delay(msPerDeg);
  }
  cockWrite(target);
}

void status() {
  Serial.printf("angle=%.1f  trim=%.1f  reversed=%s\n", cockPos, TRIM, REVERSED ? "true" : "false");
}

void printValues() {
  Serial.println("Copy these into 2_servo_test, 3_full_bench:");
  Serial.printf("  const bool  REVERSED = %s;\n", REVERSED ? "true" : "false");
  Serial.printf("  const float TRIM     = %.1f;\n", TRIM);
}

void command(String in) {
  in.trim();
  if (in.length() == 0) return;
  char c = in.charAt(0);
  float v = in.substring(1).toFloat();
  switch (c) {
    case 'w':
      cock.detach();
      if (!cock.attach(PIN_MG996R, CK_MIN_US, CK_MAX_US)) break;
      for (int us = 1100; us <= 1500; us += 5) { cock.writeMicroseconds(us); delay(10); }
      cockPos = -1;   // raw position, not logical
      Serial.println("Seen from the front: clockwise -> keep; anticlockwise -> send v");
      return;
    case 'v': REVERSED = !REVERSED; Serial.println("Direction flipped - run w again to check"); return;
    case 'z': sweepCock(0, 8); break;
    case 't': TRIM = v; cockWrite(cockPos); break;
    case '+': TRIM += 0.5f; cockWrite(cockPos); break;
    case '-': TRIM -= 0.5f; cockWrite(cockPos); break;
    case 'a': sweepCock(constrain(v, 0.0f, 180.0f), 8); break;
    case 'c':
      Serial.println("0 -> 134 (hold 2 s) -> 180 -> 0");
      sweepCock(0, 8);   delay(500);
      sweepCock(134, 8); delay(2000);
      sweepCock(180, 6); delay(500);
      sweepCock(0, 3);
      break;
    case 'x': cock.detach(); Serial.println("Servo limp"); return;
    case 'p': printValues(); return;
    default: Serial.println("? commands: w v z t<deg> + - a<deg> c x p"); return;
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
  Serial.println("MG996R zero calibration: w v z t<deg> + - a<deg> c x p");
  Serial.println("Servo stays limp until you send a command. Start with w (horn off).");
}

String buf;

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if ((ch == '+' || ch == '-') && buf.length() == 0) { command(String(ch)); continue; }   // instant nudge
    if (ch == '\n' || ch == '\r') { command(buf); buf = ""; }
    else buf += ch;
  }
}
