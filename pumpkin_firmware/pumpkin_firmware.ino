// Cat-Eye Pumpkin - main firmware: fire cat-eye on the GC9A01 LCD, driven by pushes read from an MPU6050
// Board: ESP32 DevKit (ESP32-WROOM-32, "ESP32 Dev Module"). PlatformIO: env:pumpkin
// Library: "GFX Library for Arduino" (moononournation). Tune the push settings with pumpkin_bench_tests/5_accel_test.
//
// Every push on the head makes the flame grow one stage. The 4th push makes the eye blaze for 4 s, then it
// starts over. With no pushes for 30 s the eye cools down one stage at a time.
// Serial Monitor (115200) logs every push; type p + Enter to fake a push (works without the sensor).
//
// Wiring: LCD      VCC -> 3V3, GND -> GND, SCL -> 18, SDA -> 23, CS -> 5, DC -> 4, RST -> 22
//         MPU6050  VCC -> 3V3, GND -> GND, SDA -> 21, SCL -> 19

#include <Arduino_GFX_Library.h>
#include <Wire.h>

// ---------------- pins ----------------
const int LCD_SCK = 18, LCD_MOSI = 23, LCD_CS = 5, LCD_DC = 4, LCD_RST = 22;
const int I2C_SDA = 21, I2C_SCL = 19;    // GPIO22 (the usual SCL) is the LCD reset

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
const bool LCD_IPS = true;   // colours look inverted? set to false
Arduino_GFX *gfx = new Arduino_GC9A01(bus, LCD_RST, 0 /*rotation*/, LCD_IPS);

// ---------------- behaviour ----------------
const float    STAGE_POWER[] = { 0, 0.35f, 0.6f, 0.85f };   // flame power after 0, 1, 2, 3 pushes
const float    BLAZE_POWER = 1.25f;                         // 4th push
const uint32_t BLAZE_MS = 4000;                             // how long the blaze lasts
const uint32_t COOL_MS  = 30000;                            // no push for this long -> drop one stage

// ---------------- push detection (copy tuned values from 5_accel_test) ----------------
float PUSH_G = 0.25f, QUIET_G = 0.06f;          // motion (g) that counts as a push / as standing still
uint32_t COOLDOWN_MS = 1500, SETTLE_MS = 400;   // ignore the spring rebound after a push

float grav[3];                 // slow average of the readings = gravity in sensor axes
bool armed = false;
uint32_t lastPush = 0, quietSince = 0;

// true once per push. motion = acceleration with gravity removed, in g (works in any mounting orientation)
bool detectPush(const float a[3], uint32_t now, float &motion) {
  float m2 = 0;
  for (int i = 0; i < 3; i++) {
    grav[i] += (a[i] - grav[i]) * 0.006f;       // ~0.8 s time constant at 200 Hz
    float d = a[i] - grav[i];
    m2 += d * d;
  }
  motion = sqrtf(m2);
  if (motion > QUIET_G) quietSince = now;
  if (!armed) {
    if (now - lastPush > COOLDOWN_MS && now - quietSince > SETTLE_MS) armed = true;
    return false;
  }
  if (motion > PUSH_G) { armed = false; lastPush = now; return true; }
  return false;
}

// ---------------- MPU6050 ----------------
uint8_t mpuAddr = 0, whoAmI = 0;

bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

uint8_t mpuRead8(uint8_t reg) {
  Wire.beginTransmission(mpuAddr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(mpuAddr, (uint8_t)1) != 1) return 0;
  return Wire.read();
}

bool mpuBegin() {
  const uint8_t addrs[] = { 0x68, 0x69 };
  mpuAddr = 0;
  for (uint8_t a : addrs) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) { mpuAddr = a; break; } }
  if (!mpuAddr) return false;
  whoAmI = mpuRead8(0x75);
  mpuWrite(0x6B, 0x80); delay(100);              // reset
  mpuWrite(0x6B, 0x01);                          // wake up, gyro clock
  mpuWrite(0x1A, 0x03);                          // 44 Hz low-pass
  mpuWrite(0x19, 4);                             // 200 samples/s
  mpuWrite(0x1C, 0x08);                          // +-4 g
  if (whoAmI != 0x68) mpuWrite(0x1D, 0x03);      // MPU6500/9250: accel low-pass is a separate register
  return true;
}

bool mpuRead(float a[3]) {                       // acceleration in g
  Wire.beginTransmission(mpuAddr); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(mpuAddr, (uint8_t)6) != 6) return false;
  for (int i = 0; i < 3; i++) {
    int hi = Wire.read(), lo = Wire.read();
    a[i] = (int16_t)((hi << 8) | lo) / 8192.0f;
  }
  return true;
}

// sensor runs on its own core at 200 Hz, so drawing the eye never makes it miss a push
volatile uint32_t pushCount = 0;
volatile float motionG = 0;

void imuTask(void *) {
  float a[3], motion;
  while (!mpuRead(a)) vTaskDelay(pdMS_TO_TICKS(100));
  for (int i = 0; i < 3; i++) grav[i] = a[i];
  quietSince = millis();
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    if (mpuRead(a)) {
      if (detectPush(a, millis(), motion)) pushCount = pushCount + 1;
      motionG = motion;
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
  }
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
  float hot = 0.95f + 0.45f * fminf(power, 1.1f);   // iris heat (capped so the blaze doesn't wash out)
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

// ---------------- stages ----------------
int stage = 0;
bool blazing = false;
uint32_t blazeStart = 0, lastActivity = 0, seenPushes = 0;
float power = 0, flare = 0;

void onPush(uint32_t now) {
  if (blazing) return;                          // pushes during the blaze don't count
  lastActivity = now;
  flare = 0.5f;                                 // quick flare-up on every push
  if (++stage >= 4) { blazing = true; blazeStart = now; Serial.println("Push 4 -> BLAZE"); }
  else Serial.printf("Push %d -> stage %d\n", stage, stage);
}

uint32_t frames = 0, lastReport = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  if (!gfx->begin()) Serial.println("LCD init failed - check wiring");
  gfx->fillScreen(0x0000);
  buildEye();

  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  if (mpuBegin()) {
    Serial.printf("Accelerometer found at 0x%02X (WHO_AM_I 0x%02X)\n", mpuAddr, whoAmI);
    xTaskCreatePinnedToCore(imuTask, "imu", 4096, nullptr, 2, nullptr, 0);
  } else {
    Serial.println("No accelerometer found - check SDA -> 21, SCL -> 19. Type p to fake a push.");
  }
  Serial.println("Pumpkin running");
}

void loop() {
  uint32_t now = millis();

  while (seenPushes != pushCount) { seenPushes++; onPush(now); }
  while (Serial.available()) if (Serial.read() == 'p') onPush(now);

  if (blazing && now - blazeStart > BLAZE_MS) {
    blazing = false; stage = 0; lastActivity = now;
    Serial.println("Blaze over -> stage 0");
  }
  if (!blazing && stage > 0 && now - lastActivity > COOL_MS) {
    stage--; lastActivity = now;
    Serial.printf("No pushes -> cooling to stage %d\n", stage);
  }

  float target = blazing ? BLAZE_POWER : STAGE_POWER[stage];
  power += (target - power) * 0.15f;
  flare *= 0.85f;
  drawEye(now / 1000.0f, power + flare);

  frames++;
  if (now - lastReport > 10000) {
    Serial.printf("stage %d%s  %.1f fps  motion %.3f g\n", stage, blazing ? " (blaze)" : "",
                  frames * 1000.0f / (now - lastReport), (float)motionG);
    frames = 0; lastReport = now;
  }
}
