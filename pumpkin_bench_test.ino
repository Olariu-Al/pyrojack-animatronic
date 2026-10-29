// Cat-Eye Pumpkin - BENCH TEST: 1 LCD (left eye) + MG995 + MG90S
// Runs the scare cycle every 10 s while the eye animates continuously.
// Board: ESP32S3 Dev Module, USB CDC On Boot: Enabled
// Libraries: "GFX Library for Arduino" (moononournation), "ESP32Servo"

#include <Arduino_GFX_Library.h>
#include <ESP32Servo.h>

// ---------- calibration (copy from the helper sketch) ----------
const bool  REVERSED     = false;  // MG995 direction
const float TRIM         = 0;      // MG995 zero trim, degrees
const int   LOCK_DEG     = 45;     // MG90S deadbolt locked
const int   OPEN_DEG     = 135;    // MG90S deadbolt open
const bool  WAIT_FOR_LID = false;  // true = wait for the lid switch before locking
const uint32_t CYCLE_MS  = 10000;  // one scare every 10 s

// ---------- pins ----------
const int PIN_MG995 = 17, PIN_MG90S = 18, PIN_LID = 41, PIN_BUZZ = 40;
const int LCD_SCK = 12, LCD_MOSI = 11, LCD_CS = 10, LCD_DC = 13, LCD_RST = 14;

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, LCD_RST, 0 /*rotation*/, true /*IPS*/);

Servo cock, latch;

// ======================= cat-eye fire graphics =======================
const int CX = 120, CY = 120;       // screen centre
const int EYE_HH = 45;              // half height of the almond
float halfW[2 * EYE_HH + 1];        // almond half-width per row
const float SHEAR = -0.32f;         // slants the eye (about 18 deg)

uint16_t rgb(int r, int g, int b) {
  r = constrain(r, 0, 255); g = constrain(g, 0, 255); b = constrain(b, 0, 255);
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void buildEye() {                   // vesica: two circles -> sharp almond
  const float L = 200, H = 2 * EYE_HH;
  const float R = H / 4 + L * L / (4 * H), d = R - H / 2;
  for (int y = -EYE_HH; y <= EYE_HH; y++) {
    float t = R * R - (fabsf(y) + d) * (fabsf(y) + d);
    halfW[y + EYE_HH] = t > 0 ? sqrtf(t) : 0;
  }
}

// one horizontal flame segment from the pupil edge outward: yellow -> orange -> red
void flameSegment(int xInner, int xOuter, int y, float bright) {
  int dir = xOuter > xInner ? 1 : -1;
  int len = abs(xOuter - xInner);
  if (len <= 0) return;
  int a = len * random(20, 40) / 100;          // yellow core
  int b = len * random(30, 45) / 100;          // orange band
  int c = len - a - b;                         // red rim
  float f = bright * random(70, 101) / 100.0f; // per-row flicker
  int x = xInner;
  gfx->drawFastHLine(dir > 0 ? x : x - a + 1, y, a, rgb(255 * f, 220 * f, 70 * f)); x += dir * a;
  gfx->drawFastHLine(dir > 0 ? x : x - b + 1, y, b, rgb(255 * f, 110 * f, 0));     x += dir * b;
  gfx->drawFastHLine(dir > 0 ? x : x - c + 1, y, c, rgb(190 * f, 15 * f, 0));
}

// bright: 0..1 flame brightness, pupil: max half-width of the slit in pixels
void drawEye(float bright, float pupil) {
  for (int y = -EYE_HH; y <= EYE_HH; y++) {
    float w = halfW[y + EYE_HH];
    if (w < 1) continue;
    int sy = CY + y;
    int cx = CX + (int)(SHEAR * y);
    float k = 1.0f - (float)(y * y) / (EYE_HH * EYE_HH);
    int p = (int)(pupil * sqrtf(k > 0 ? k : 0)) + 1;       // vertical slit pupil
    int left = cx - (int)w, right = cx + (int)w;
    gfx->drawFastHLine(cx - p, sy, 2 * p, 0x0000);         // black slit
    flameSegment(cx - p - 1, left, sy, bright);
    flameSegment(cx + p, right, sy, bright);
  }
}

// ======================= servos =======================
float cockPos = 0, cockTarget = 0;
int stepMs = 8;
uint32_t lastStep = 0;

void cockWrite(float logical) {
  float s = logical + TRIM;
  if (REVERSED) s = 180 - s;
  s = constrain(s, 0.0f, 180.0f);
  if (!cock.attached()) cock.attach(PIN_MG995, 500, 2500);
  cock.writeMicroseconds(500 + (int)(s * 2000.0f / 180.0f));
  cockPos = logical;
}
void cockGo(float target, int msPerDeg) { cockTarget = target; stepMs = msPerDeg; }
bool cockStep(uint32_t now) {       // true when the target is reached
  if (fabsf(cockTarget - cockPos) < 1.0f) { cockWrite(cockTarget); return true; }
  if (now - lastStep >= (uint32_t)stepMs) {
    lastStep = now;
    cockWrite(cockPos + (cockTarget > cockPos ? 1.0f : -1.0f));
  }
  return false;
}
void latchWrite(int deg) {
  if (!latch.attached()) latch.attach(PIN_MG90S, 500, 2500);
  latch.write(constrain(deg, 0, 180));
}

// ======================= scare cycle =======================
enum Phase { IDLE, FIRE, COCK, HOLD, LOCK, RELEASE, RESETTING };
Phase phase = IDLE;
uint32_t cycleStart = 0, phaseStart = 0, arrivedAt = 0, lidSince = 0;

void enter(Phase p, uint32_t now) { phase = p; phaseStart = now; arrivedAt = 0; Serial.printf("[%lu] phase %d\n", (unsigned long)now, p); }

void runCycle(uint32_t now) {
  switch (phase) {
    case IDLE:
      if (now - cycleStart >= CYCLE_MS) {
        cycleStart = now;
        latchWrite(OPEN_DEG);                     // FIRE: bolt open, flame pops
        digitalWrite(PIN_BUZZ, HIGH);
        enter(FIRE, now);
      }
      break;
    case FIRE:
      if (now - phaseStart > 300) digitalWrite(PIN_BUZZ, LOW);
      if (now - phaseStart > 3000) { cockGo(134, 8); enter(COCK, now); }   // 2 deg past the 132 notch
      break;
    case COCK:
      if (cockStep(now)) {
        if (!arrivedAt) arrivedAt = now;
        if (now - arrivedAt > 300) { cock.detach(); enter(HOLD, now); }  // pawl holds the plunger
      }
      break;
    case HOLD: {
      bool ready;
      if (WAIT_FOR_LID) {
        if (digitalRead(PIN_LID) == LOW) { if (!lidSince) lidSince = now; } else lidSince = 0;
        ready = lidSince && now - lidSince > 250;
      } else {
        ready = now - phaseStart > 1000;          // bench: pretend the lid was closed
      }
      if (ready) { lidSince = 0; latchWrite(LOCK_DEG); enter(LOCK, now); }
      break;
    }
    case LOCK:
      if (now - phaseStart > 300) { cockGo(180, 6); enter(RELEASE, now); }
      break;
    case RELEASE:
      if (cockStep(now)) {
        if (!arrivedAt) arrivedAt = now;
        if (now - arrivedAt > 200) { cockGo(0, 3); enter(RESETTING, now); }
      }
      break;
    case RESETTING:
      if (cockStep(now)) {
        if (!arrivedAt) arrivedAt = now;
        if (now - arrivedAt > 300) { cock.detach(); latch.detach(); enter(IDLE, now); }
      }
      break;
  }
}

// ======================= setup / loop =======================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LID, INPUT_PULLUP);
  pinMode(PIN_BUZZ, OUTPUT); digitalWrite(PIN_BUZZ, LOW);

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  cock.setPeriodHertz(50); latch.setPeriodHertz(50);

  if (!gfx->begin()) Serial.println("LCD init failed - check wiring");
  gfx->fillScreen(0x0000);
  buildEye();

  cockWrite(0); latchWrite(LOCK_DEG);            // start ARMED
  delay(500);
  cock.detach(); latch.detach();
  cycleStart = millis() - CYCLE_MS + 3000;       // first scare 3 s after boot
  Serial.println("Bench test running: scare every 10 s");
}

void loop() {
  uint32_t now = millis();
  runCycle(now);

  // eye: blazing with a thin slit during FIRE, dim ember with a wide slit otherwise
  bool blaze = (phase == FIRE);
  float bright = blaze ? 1.0f : 0.35f + 0.08f * sinf(now / 300.0f);
  float pupil  = blaze ? 4.0f : 13.0f;
  drawEye(bright, pupil);
}
