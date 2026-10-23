// Cat-Eye Pumpkin - assembly & calibration helper (not the show firmware)
// Board: ESP32S3 Dev Module, USB CDC On Boot: Enabled. Libraries: ESP32Servo, Adafruit NeoPixel.
// Serial Monitor: 115200 baud, line ending "Newline".
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>

// ---- write your calibration results here so they survive a reboot ----
bool  REVERSED = false;   // MG995 direction  (command v)
float TRIM     = 0;       // MG995 zero trim, degrees (command t)
int   LOCK_DEG = 45;      // MG90S deadbolt locked (command L)
int   OPEN_DEG = 135;     // MG90S deadbolt open   (command O)

const int PIN_MG995 = 17, PIN_MG90S = 18, PIN_LID = 41, PIN_HALL = 47;
const int PIN_LED = 39, PIN_BUZZ = 40, PIN_VBAT = 2;

Servo cock, latch;
Adafruit_NeoPixel flame(4, PIN_LED, NEO_GRB + NEO_KHZ800);
float cockPos = 0;        // last logical MG995 angle

void cockWrite(float logical) {
  float s = logical + TRIM;
  if (REVERSED) s = 180 - s;
  s = constrain(s, 0.0f, 180.0f);
  if (!cock.attached()) cock.attach(PIN_MG995, 500, 2500);
  cock.writeMicroseconds(500 + (int)(s * 2000.0f / 180.0f));
  cockPos = logical;
}

void cockMove(float target, int msPerDeg) {      // slow, 1 degree per step
  while (fabsf(target - cockPos) >= 1.0f) {
    cockWrite(cockPos + (target > cockPos ? 1.0f : -1.0f));
    delay(msPerDeg);
  }
  cockWrite(target);
}

void latchWrite(int deg) {
  if (!latch.attached()) latch.attach(PIN_MG90S, 500, 2500);
  latch.write(constrain(deg, 0, 180));
}

bool lidClosed()  { return digitalRead(PIN_LID) == LOW; }
bool magnetSeen() { return digitalRead(PIN_HALL) == LOW; }
float vbat()      { return analogReadMilliVolts(PIN_VBAT) / 1000.0f * (122.0f / 22.0f); }

void flameOn(bool on) {
  for (int i = 0; i < 4; i++) flame.setPixelColor(i, on ? flame.Color(255, 70, 0) : 0);
  flame.show();
}

void beep(int ms) { digitalWrite(PIN_BUZZ, HIGH); delay(ms); digitalWrite(PIN_BUZZ, LOW); }

void status() {
  Serial.printf("lid=%s hall=%s vbat=%.2fV | cock=%.0f reversed=%d trim=%.1f lock=%d open=%d\n",
                lidClosed() ? "CLOSED" : "open", magnetSeen() ? "MAGNET" : "-", vbat(),
                cockPos, REVERSED ? 1 : 0, TRIM, LOCK_DEG, OPEN_DEG);
}

void waitLidClosed() {                           // closed and steady for 250 ms
  uint32_t since = 0;
  while (true) {
    if (lidClosed()) { if (!since) since = millis(); if (millis() - since > 250) return; }
    else since = 0;
    delay(10);
  }
}

void fullCycle() {                               // the 5-step hand-off, start from ARMED
  Serial.println("FIRE: bolt open");
  flameOn(true); beep(300); latchWrite(OPEN_DEG);
  delay(3000); flameOn(false);
  Serial.println("Cock 0 -> 134 (2 deg past the 132 notch), hold on the pawl");
  cockMove(0, 4); cockMove(134, 8); delay(300); cock.detach();
  Serial.println("Close the lid...");
  waitLidClosed();
  latchWrite(LOCK_DEG); delay(300);
  Serial.println("Release 132 -> 180, then reset 180 -> 0");
  cockMove(180, 6); delay(200);
  cockMove(0, 3); delay(300);
  cock.detach(); latch.detach();
  Serial.println("ARMED");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LID, INPUT_PULLUP);
  pinMode(PIN_HALL, INPUT_PULLUP);
  pinMode(PIN_BUZZ, OUTPUT); digitalWrite(PIN_BUZZ, LOW);
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  cock.setPeriodHertz(50); latch.setPeriodHertz(50);
  flame.begin(); flame.show();
  delay(1500);
  Serial.println("Pumpkin helper: a<deg> b<deg> t<trim> v w x f z s L<deg> O<deg> F");
}

void loop() {
  if (!Serial.available()) return;
  String in = Serial.readStringUntil('\n'); in.trim();
  if (in.length() == 0) return;
  char c = in.charAt(0);
  float v = in.substring(1).toFloat();
  switch (c) {
    case 'a': cockMove(constrain(v, 0.0f, 180.0f), 8); break;      // MG995 to logical angle
    case 'b': latchWrite((int)v); break;                           // MG90S to raw angle
    case 't': TRIM = v; cockWrite(cockPos); break;                 // zero trim
    case 'v': REVERSED = !REVERSED; break;                         // flip MG995 direction
    case 'w':                                                      // direction test, bare spline only
      if (!cock.attached()) cock.attach(PIN_MG995, 500, 2500);
      for (int us = 1100; us <= 1500; us += 5) { cock.writeMicroseconds(us); delay(10); }
      Serial.println("Seen from the front: clockwise -> keep; anticlockwise -> send v");
      break;
    case 'x': cock.detach(); latch.detach(); break;                // both servos limp
    case 'f': flameOn(true); delay(3000); flameOn(false); break;   // LED test
    case 'z': beep(200); break;                                    // buzzer test
    case 'L': LOCK_DEG = (int)v; latchWrite(LOCK_DEG); break;
    case 'O': OPEN_DEG = (int)v; latchWrite(OPEN_DEG); break;
    case 'F': fullCycle(); break;
    case 's': break;
    default: Serial.println("?"); break;
  }
  status();
}
