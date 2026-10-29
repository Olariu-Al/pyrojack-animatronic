// TEST 2 - SERVOS ONLY: MG995 sweep, MG90S lock/open, then the scare motion, repeating every 15 s
// Run this with the servo horns OFF, or before the plunger is fitted:
// the 0 -> 180 sweep would release the plunger and fire the flame.
// Works on ESP32 DevKit (ESP32-WROOM-32, board "ESP32 Dev Module"), ESP8266 (NodeMCU) and ESP32-S3 -
// just pick the board in Tools > Board.
// ESP32 needs the "ESP32Servo" library; the ESP8266 uses its built-in Servo library.

// ---------------- servo + buzzer pins ----------------
#if defined(ESP8266)
  #include <Servo.h>
  const int PIN_MG995 = 12;  // D6
  const int PIN_MG90S = 2;   // D4  (also the on-board LED - it will flicker, that's fine)
  const int PIN_BUZZ  = 16;  // D0
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  #include <ESP32Servo.h>
  const int PIN_MG995 = 17, PIN_MG90S = 18, PIN_BUZZ = 40;
#else
  // ESP32 DevKit
  #include <ESP32Servo.h>
  const int PIN_MG995 = 26, PIN_MG90S = 27, PIN_BUZZ = 25;
#endif

// ---------------- calibration (from the helper sketch) ----------------
const bool  REVERSED = false;   // MG995 direction
const float TRIM     = 0;       // MG995 zero trim, degrees
const int   LOCK_DEG = 45;      // MG90S deadbolt locked
const int   OPEN_DEG = 135;     // MG90S deadbolt open

Servo cock, latch;
float cockPos = 0;

void cockWrite(float logical) {
  float s = logical + TRIM;
  if (REVERSED) s = 180 - s;
  s = constrain(s, 0.0f, 180.0f);
  if (!cock.attached()) cock.attach(PIN_MG995, 500, 2500);
  cock.writeMicroseconds(500 + (int)(s * 2000.0f / 180.0f));
  cockPos = logical;
}
void latchWrite(int deg) {
  if (!latch.attached()) latch.attach(PIN_MG90S, 500, 2500);
  latch.writeMicroseconds(500 + constrain(deg, 0, 180) * 2000 / 180);
}
void servoInit() {
#if !defined(ESP8266)
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  cock.setPeriodHertz(50); latch.setPeriodHertz(50);
#endif
  pinMode(PIN_BUZZ, OUTPUT); digitalWrite(PIN_BUZZ, LOW);
}

void sweepCock(float target, int msPerDeg) {
  while (fabsf(target - cockPos) >= 1.0f) {
    cockWrite(cockPos + (target > cockPos ? 1.0f : -1.0f));
    delay(msPerDeg);
  }
  cockWrite(target);
}

void beep(int ms) { digitalWrite(PIN_BUZZ, HIGH); delay(ms); digitalWrite(PIN_BUZZ, LOW); }

void setup() {
  Serial.begin(115200);
  delay(500);
  servoInit();
  cockWrite(0); latchWrite(LOCK_DEG);
  delay(1000);
  Serial.println("Servo test starting");
}

void loop() {
  uint32_t start = millis();

  Serial.println("1) MG995 full sweep 0 -> 180 -> 0");
  sweepCock(180, 10); delay(500);
  sweepCock(0, 5);    delay(500);
  cock.detach();

  Serial.println("2) MG90S lock / open x3");
  for (int i = 0; i < 3; i++) { latchWrite(LOCK_DEG); delay(700); latchWrite(OPEN_DEG); delay(700); }

  Serial.println("3) Scare motion: open, cock 0->134, lock, release ->180, reset ->0");
  beep(200);
  latchWrite(OPEN_DEG); delay(1000);
  sweepCock(134, 8);    delay(300); cock.detach(); delay(1000);
  latchWrite(LOCK_DEG); delay(300);
  sweepCock(180, 6);    delay(200);
  sweepCock(0, 3);      delay(300);
  cock.detach(); latch.detach();

  Serial.println("Done - repeating");
  while (millis() - start < 15000) delay(50);
}
