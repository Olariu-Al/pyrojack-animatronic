// TEST 3 - LCD + BOTH SERVOS: scare cycle every 10 s while the eye animates non-stop
// Run with the horns off or the plunger out first: step 4 releases the plunger every cycle.
// Works on ESP32 DevKit (ESP32-WROOM-32, board "ESP32 Dev Module"), ESP8266 (NodeMCU) and ESP32-S3 -
// just pick the board in Tools > Board.
// Libraries: "GFX Library for Arduino" (moononournation); ESP32 also needs "ESP32Servo".

#include <Arduino_GFX_Library.h>

// ---------------- LCD pins ----------------
#if defined(ESP8266)
  // NodeMCU: SCL -> D5 (GPIO14), SDA -> D7 (GPIO13)  [fixed hardware SPI pins]
  const int LCD_DC = 4;    // D2
  const int LCD_CS = 15;   // D8
  const int LCD_RST = 5;   // D1
  Arduino_DataBus *bus = new Arduino_ESP8266SPI(LCD_DC, LCD_CS);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  // ESP32-S3: SCL -> 12, SDA -> 11
  const int LCD_SCK = 12, LCD_MOSI = 11, LCD_CS = 10, LCD_DC = 13, LCD_RST = 14;
  Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
#else
  // ESP32 DevKit (VSPI hardware pins): SCL -> GPIO18, SDA -> GPIO23
  const int LCD_SCK = 18, LCD_MOSI = 23, LCD_CS = 5, LCD_DC = 4, LCD_RST = 22;
  Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
#endif
const bool LCD_IPS = true;   // colours look inverted? set to false
Arduino_GFX *gfx = new Arduino_GC9A01(bus, LCD_RST, 0 /*rotation*/, LCD_IPS);

// ---------------- servo + buzzer pins ----------------
#if defined(ESP8266)
  #include <Servo.h>
  const int PIN_MG996R = 12;  // D6
  const int PIN_MG90S = 2;   // D4  (also the on-board LED - it will flicker, that's fine)
  const int PIN_BUZZ  = 16;  // D0
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  #include <ESP32Servo.h>
  const int PIN_MG996R = 17, PIN_MG90S = 18, PIN_BUZZ = 40;
#else
  // ESP32 DevKit
  #include <ESP32Servo.h>
  const int PIN_MG996R = 26, PIN_MG90S = 27, PIN_BUZZ = 25;
#endif

// ---------------- calibration (from the helper sketch) ----------------
const bool  REVERSED = false;   // MG996R direction
const float TRIM     = 0;       // MG996R zero trim, degrees
const int   CK_MIN_US = 500, CK_MAX_US = 2500;   // MG996R pulse at 0 / 180 deg - narrow it if the servo buzzes at an end
const int   LOCK_DEG = 45;      // MG90S deadbolt locked
const int   OPEN_DEG = 135;     // MG90S deadbolt open

// declared before any function so the auto-generated prototypes (PlatformIO / Arduino) can see it
enum Phase { IDLE, FIRE, COCK, HOLD, LOCK, RELEASE, RESETTING };

Servo cock, latch;
float cockPos = 0;

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
void servoInit() {
#if !defined(ESP8266)
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  cock.setPeriodHertz(50); latch.setPeriodHertz(50);
#endif
  pinMode(PIN_BUZZ, OUTPUT); digitalWrite(PIN_BUZZ, LOW);
}

// ---------------- fire cat-eye graphics ----------------
// The outline is the face cut-out from cat_eye_pumpkin.scad (almond_2d): a 31 x 13.5 mm two-arc almond with the
// angry hood, tilted 18 deg, centred on the LCD and scaled to its 32.4 mm active area. A flame burns in the pupil.
const int   EYE_SIDE = 1;                 // 1 = eye on the viewer's right, -1 = viewer's left (mirrored)
const float PX_MM    = 240 / 32.4f;       // screen pixels per mm
const float EYE_L    = 31 * PX_MM, EYE_H = 13.5f * PX_MM;
const float EYE_R    = EYE_H / 4 + EYE_L * EYE_L / (4 * EYE_H), EYE_D = EYE_R - EYE_H / 2;
const float TILT_C   = cosf(18 * DEG_TO_RAD), TILT_S = sinf(18 * DEG_TO_RAD);
const float HOOD_M   = (0.5f * EYE_H) / (0.8f * EYE_L);    // upper lid: nasal corner (-L/2, 0) -> (0.3 L, H/2)
const float HOOD_K   = 1 / sqrtf(1 + HOOD_M * HOOD_M);
const float GROW     = 5;                 // draw a bit past the cut-out so the bezel hides any misalignment
const float FLAME_X  = 16, FLAME_BASE = -60, HALO = 7;     // flame position (px from centre) and its dark rim

int16_t spanX0[240], spanX1[240];         // eye pixels on each row (empty when x1 < x0)
uint16_t palette[64];                     // heat 0..1 -> colour
float sinLut[256];
uint16_t line[240];

uint16_t rgb(int r, int g, int b) {
  r = constrain(r, 0, 255); g = constrain(g, 0, 255); b = constrain(b, 0, 255);
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

inline float fsin(float a) { return sinLut[(int)(a * 40.743665f) & 255]; }   // 256-step sine, a in radians

// signed distance to the eye outline in pixels, positive inside. X right, Y up, from the screen centre.
float eyeDist(float X, float Y) {
  X *= EYE_SIDE;
  float x = X * TILT_C + Y * TILT_S, y = Y * TILT_C - X * TILT_S;   // undo the tilt
  float lower = EYE_R - sqrtf(x * x + (y - EYE_D) * (y - EYE_D));
  float upper = EYE_R - sqrtf(x * x + (y + EYE_D) * (y + EYE_D));
  float hood  = (HOOD_M * (x + EYE_L / 2) - y) * HOOD_K;
  return fminf(fminf(lower, upper), hood);
}

void buildEye() {
  for (int i = 0; i < 256; i++) sinLut[i] = sinf(i * TWO_PI / 256);
  // black -> deep red -> red -> orange -> yellow -> near white
  const float key[][4] = { { 0, 0, 0, 0 }, { 0.25f, 120, 0, 0 }, { 0.5f, 220, 30, 0 }, { 0.72f, 255, 120, 0 }, { 0.88f, 255, 210, 40 }, { 1, 255, 250, 190 } };
  for (int i = 0; i < 64; i++) {
    float h = i / 63.0f; int k = 0;
    while (h > key[k + 1][0]) k++;
    float f = (h - key[k][0]) / (key[k + 1][0] - key[k][0]);
    palette[i] = rgb(key[k][1] + f * (key[k + 1][1] - key[k][1]), key[k][2] + f * (key[k + 1][2] - key[k][2]), key[k][3] + f * (key[k + 1][3] - key[k][3]));
  }
  for (int py = 0; py < 240; py++) {
    spanX0[py] = 240; spanX1[py] = -1;
    for (int px = 0; px < 240; px++)
      if (eyeDist(px - 119.5f, 119.5f - py) > -GROW) { if (px < spanX0[py]) spanX0[py] = px; spanX1[py] = px; }
  }
}

// t: seconds, power: 0 = smouldering ember, 1 = full blaze
void drawEye(float t, float power) {
  float hf  = (90 + 60 * power) * (0.92f + 0.08f * fsin(t * 13) * fsin(t * 7.3f));   // flame height, flickering
  float wf  = 22 + 16 * power;              // flame width
  float amp = 10 + 12 * power;              // sideways lick of the tip
  static float ts = 0, lastT = t;           // animation phase, sped up smoothly by power
  ts += (t - lastT) * (1 + 0.8f * power); lastT = t;
  float hot = 0.95f + 0.45f * power;        // iris heat
  for (int py = 0; py < 240; py++) {
    int x0 = spanX0[py], x1 = spanX1[py];
    if (x1 < x0) continue;
    float Y = 119.5f - py;
    float v = (Y - FLAME_BASE) / hf;        // 0 at the flame base, 1 at its tip
    float wob = amp * v * v * (0.6f * fsin(Y * 0.035f - ts * 1.9f) + 0.4f * fsin(Y * 0.08f - ts * 3.7f));
    float w = (v > 0 && v < 1) ? wf * 1.7f * sqrtf(v) * (1 - v) * (1 + 0.18f * fsin(Y * 0.11f - ts * 7)) : 0;
    float rowN = 1.6f * fsin(Y * 0.045f - ts * 2.2f);
    for (int px = x0; px <= x1; px++) {
      float Xs = px - 119.5f, X = Xs * EYE_SIDE;
      float rim = constrain(eyeDist(Xs, Y) / 28, 0.0f, 1.0f);   // darker towards the lids
      float n = 0.5f + 0.25f * fsin(X * 0.07f + rowN) + 0.25f * fsin(Y * 0.09f - ts * 5.1f + X * 0.03f);
      float heat = hot * (0.12f + 0.5f * n * n) * (0.35f + 0.65f * rim);    // rising embers in the iris
      if (w > 0) {
        float dd = fabsf(X - FLAME_X - wob) - w;                            // < 0 inside the flame
        if (dd < 0) heat = (0.62f + 0.38f * sqrtf(-dd / w)) * (1 - 0.3f * v) * (0.85f + 0.15f * power);
        else if (dd < HALO) heat *= 0.2f + 0.8f * dd / HALO;                // dark slit around the flame
      }
      line[px] = palette[(int)(constrain(heat, 0.0f, 1.0f) * 63)];
    }
    gfx->draw16bitRGBBitmap(x0, py, line + x0, x1 - x0 + 1, 1);
  }
}

const uint32_t CYCLE_MS = 10000;
float power = 0;

// non-blocking MG996R stepping, so the eye keeps animating
float cockTarget = 0; int stepMs = 8; uint32_t lastStep = 0;
void cockGo(float target, int msPerDeg) { cockTarget = target; stepMs = msPerDeg; }
bool cockStep(uint32_t now) {
  if (fabsf(cockTarget - cockPos) < 1.0f) { cockWrite(cockTarget); return true; }
  if (now - lastStep >= (uint32_t)stepMs) { lastStep = now; cockWrite(cockPos + (cockTarget > cockPos ? 1.0f : -1.0f)); }
  return false;
}

Phase phase = IDLE;
uint32_t cycleStart = 0, phaseStart = 0, arrivedAt = 0;
const char *PHASE_NAMES[] = { "IDLE", "FIRE", "COCK", "HOLD", "LOCK", "RELEASE", "RESET" };

void enter(Phase p, uint32_t now) {
  phase = p; phaseStart = now; arrivedAt = 0;
  Serial.printf("[%lu ms] %s\n", (unsigned long)now, PHASE_NAMES[p]);
}

bool arrivedFor(uint32_t now, uint32_t ms) {       // target reached and settled for ms
  if (!cockStep(now)) return false;
  if (!arrivedAt) arrivedAt = now;
  return now - arrivedAt > ms;
}

void runCycle(uint32_t now) {
  switch (phase) {
    case IDLE:
      if (now - cycleStart >= CYCLE_MS) {
        cycleStart = now;
        latchWrite(OPEN_DEG); digitalWrite(PIN_BUZZ, HIGH);
        enter(FIRE, now);
      }
      break;
    case FIRE:
      if (now - phaseStart > 300) digitalWrite(PIN_BUZZ, LOW);
      if (now - phaseStart > 3000) { cockGo(134, 8); enter(COCK, now); }
      break;
    case COCK:
      if (arrivedFor(now, 300)) { cock.detach(); enter(HOLD, now); }
      break;
    case HOLD:
      if (now - phaseStart > 1000) { latchWrite(LOCK_DEG); enter(LOCK, now); }   // pretend the lid was closed
      break;
    case LOCK:
      if (now - phaseStart > 300) { cockGo(180, 6); enter(RELEASE, now); }
      break;
    case RELEASE:
      if (arrivedFor(now, 200)) { cockGo(0, 3); enter(RESETTING, now); }
      break;
    case RESETTING:
      if (arrivedFor(now, 300)) { cock.detach(); latch.detach(); enter(IDLE, now); }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  servoInit();
  if (!gfx->begin()) Serial.println("LCD init failed - check wiring");
  gfx->fillScreen(0x0000);
  buildEye();
  cockWrite(0); latchWrite(LOCK_DEG);              // start ARMED
  delay(500);
  cock.detach(); latch.detach();
  cycleStart = millis() - CYCLE_MS + 3000;         // first scare 3 s after boot
  Serial.println("Full bench test: scare every 10 s");
}

void loop() {
  uint32_t now = millis();
  runCycle(now);
  power += ((phase == FIRE ? 1.0f : 0.0f) - power) * 0.2f;   // flame blazes while the plunger fires
  drawEye(now / 1000.0f, power);
}
