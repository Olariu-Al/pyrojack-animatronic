// TEST 1 - LCD ONLY: colour check, then the fire cat-eye (ember 5 s / blaze 3 s, repeating)
// Works on ESP32 DevKit (ESP32-WROOM-32, board "ESP32 Dev Module"), ESP8266 (NodeMCU) and ESP32-S3 -
// just pick the board in Tools > Board.
// Library: "GFX Library for Arduino" by moononournation (if the ESP8266 build fails, install version 1.4.9).

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

// ---------------- fire cat-eye graphics ----------------
const int CX = 120, CY = 120;       // screen centre
const int EYE_HH = 45;              // half height of the almond
float halfW[2 * EYE_HH + 1];        // almond half-width per row
const float SHEAR = -0.32f;         // slants the eye (about 18 deg)

uint16_t rgb(int r, int g, int b) {
  r = constrain(r, 0, 255); g = constrain(g, 0, 255); b = constrain(b, 0, 255);
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void buildEye() {                   // two circles -> sharp almond
  const float L = 200, H = 2 * EYE_HH;
  const float R = H / 4 + L * L / (4 * H), d = R - H / 2;
  for (int y = -EYE_HH; y <= EYE_HH; y++) {
    float t = R * R - (fabsf(y) + d) * (fabsf(y) + d);
    halfW[y + EYE_HH] = t > 0 ? sqrtf(t) : 0;
  }
}

// one flame segment from the pupil outward: yellow -> orange -> red
void flameSegment(int xInner, int xOuter, int y, float bright) {
  int dir = xOuter > xInner ? 1 : -1;
  int len = abs(xOuter - xInner);
  if (len <= 0) return;
  int a = len * random(20, 40) / 100;
  int b = len * random(30, 45) / 100;
  int c = len - a - b;
  float f = bright * random(70, 101) / 100.0f;
  int x = xInner;
  gfx->drawFastHLine(dir > 0 ? x : x - a + 1, y, a, rgb(255 * f, 220 * f, 70 * f)); x += dir * a;
  gfx->drawFastHLine(dir > 0 ? x : x - b + 1, y, b, rgb(255 * f, 110 * f, 0));     x += dir * b;
  gfx->drawFastHLine(dir > 0 ? x : x - c + 1, y, c, rgb(190 * f, 15 * f, 0));
}

// bright: 0..1, pupil: max half-width of the vertical slit in pixels
void drawEye(float bright, float pupil) {
  for (int y = -EYE_HH; y <= EYE_HH; y++) {
    float w = halfW[y + EYE_HH];
    if (w < 1) continue;
    int sy = CY + y;
    int cx = CX + (int)(SHEAR * y);
    float k = 1.0f - (float)(y * y) / (EYE_HH * EYE_HH);
    int p = (int)(pupil * sqrtf(k > 0 ? k : 0)) + 1;
    gfx->drawFastHLine(cx - p, sy, 2 * p, 0x0000);       // black slit pupil
    flameSegment(cx - p - 1, cx - (int)w, sy, bright);
    flameSegment(cx + p, cx + (int)w, sy, bright);
  }
}

void colourCheck() {               // red, green, blue, white: tells you if wiring and colours are right
  const uint16_t cols[] = { 0xF800, 0x07E0, 0x001F, 0xFFFF };
  const char *names[] = { "RED", "GREEN", "BLUE", "WHITE" };
  for (int i = 0; i < 4; i++) {
    Serial.printf("Screen should be %s\n", names[i]);
    gfx->fillScreen(cols[i]);
    delay(800);
  }
  gfx->fillScreen(0x0000);
}

uint32_t frames = 0, lastReport = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  if (!gfx->begin()) Serial.println("LCD init failed - check wiring");
  colourCheck();
  buildEye();
  Serial.println("Eye animation running");
}

void loop() {
  uint32_t now = millis();
  bool blaze = (now % 8000) > 5000;                     // 5 s ember, 3 s blaze
  float bright = blaze ? 1.0f : 0.35f + 0.08f * sinf(now / 300.0f);
  drawEye(bright, blaze ? 4.0f : 13.0f);
  frames++;
  if (now - lastReport > 2000) {
    Serial.printf("%s  %.1f fps\n", blaze ? "BLAZE" : "ember", frames * 1000.0f / (now - lastReport));
    frames = 0; lastReport = now;
  }
}
