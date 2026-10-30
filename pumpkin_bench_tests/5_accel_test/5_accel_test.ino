// TEST 5 - ACCELEROMETER ONLY: checks the ADXL345 and tunes the push detection used by pumpkin_firmware
// Board: ESP32 DevKit (ESP32-WROOM-32, "ESP32 Dev Module"). PlatformIO: env:accel_test. No extra libraries.
//
// Wiring: ADXL345 VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22, CS -> 3V3 (selects I2C)
//         SDO -> GND gives address 0x53, SDO -> 3V3 gives 0x1D (both are tried). INT1 / INT2 not connected.
//
// What you should see in the Serial Monitor (115200):
//   at rest:   total ~1.00 g, motion below ~0.03 g, state ARMED
//   push head: ">>> PUSH #n" exactly once per push, even while the head keeps wobbling on the spring
// Commands (type, then Enter):
//   t<g>   push threshold, e.g. t1.3         q<g>   "standing still" threshold, e.g. q0.05
//   c<ms>  minimum time between pushes        s<ms>  how long it must be still before the next push counts
//   l      live readings on / off             r      reset the push counter
//   p      print the settings to copy into pumpkin_firmware

#include <Wire.h>

const int I2C_SDA = 21, I2C_SCL = 22;    // standard ESP32 I2C pins

// ---------------- push detection ----------------
float PUSH_G = 1.3f, QUIET_G = 0.06f;           // total acceleration (g) that counts as a push / motion (g) that counts as standing still
uint32_t COOLDOWN_MS = 1500, SETTLE_MS = 400;   // ignore the spring rebound after a push

float grav[3];                 // slow average of the readings = gravity in sensor axes
bool armed = false;
uint32_t lastPush = 0, quietSince = 0;

// true once per push: total acceleration (gravity included, ~1.00 g at rest) goes over PUSH_G.
// motion = acceleration with gravity removed, in g - only used to tell when the head has stopped wobbling
bool detectPush(const float a[3], uint32_t now, float &motion) {
  float m2 = 0, t2 = 0;
  for (int i = 0; i < 3; i++) {
    grav[i] += (a[i] - grav[i]) * 0.006f;       // ~0.8 s time constant at 200 Hz
    float d = a[i] - grav[i];
    m2 += d * d;
    t2 += a[i] * a[i];
  }
  motion = sqrtf(m2);
  float total = sqrtf(t2);
  if (motion > QUIET_G) quietSince = now;
  if (!armed) {
    if (now - lastPush > COOLDOWN_MS && now - quietSince > SETTLE_MS) armed = true;
    return false;
  }
  if (total > PUSH_G) { armed = false; lastPush = now; return true; }
  return false;
}

// ---------------- ADXL345 ----------------
uint8_t adxlAddr = 0, devId = 0;

bool adxlWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(adxlAddr); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

uint8_t adxlRead8(uint8_t reg) {
  Wire.beginTransmission(adxlAddr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(adxlAddr, (uint8_t)1) != 1) return 0;
  return Wire.read();
}

bool adxlBegin() {
  const uint8_t addrs[] = { 0x53, 0x1D };
  adxlAddr = 0;
  for (uint8_t a : addrs) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) { adxlAddr = a; break; } }
  if (!adxlAddr) return false;
  devId = adxlRead8(0x00);                       // DEVID, 0xE5 on a genuine ADXL345
  adxlWrite(0x2D, 0x00);                         // POWER_CTL: standby while configuring
  adxlWrite(0x2C, 0x0B);                         // BW_RATE: 200 samples/s (100 Hz bandwidth)
  adxlWrite(0x31, 0x09);                         // DATA_FORMAT: full resolution, +-4 g (3.9 mg/LSB)
  adxlWrite(0x38, 0x00);                         // FIFO_CTL: bypass
  adxlWrite(0x2D, 0x08);                         // POWER_CTL: measure
  delay(10);
  return true;
}

bool adxlRead(float a[3]) {                      // acceleration in g
  Wire.beginTransmission(adxlAddr); Wire.write(0x32);   // DATAX0..DATAZ1, little-endian
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(adxlAddr, (uint8_t)6) != 6) return false;
  for (int i = 0; i < 3; i++) {
    int lo = Wire.read(), hi = Wire.read();
    a[i] = (int16_t)((hi << 8) | lo) * 0.0039f;
  }
  return true;
}

void scanBus() {
  Serial.print("I2C devices found:");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); n++; }
  }
  Serial.println(n ? "" : " none - check VCC, GND, SDA -> 21, SCL -> 22");
}

// ---------------- test ----------------
uint32_t pushes = 0, nextSample = 0, nextPrint = 0, nextRetry = 0;
float peak = 0;
bool live = true;
String buf;

void printSettings() {
  Serial.println("Settings (copy into pumpkin_firmware):");
  Serial.printf("  float PUSH_G = %.2f, QUIET_G = %.2f;\n", PUSH_G, QUIET_G);
  Serial.printf("  uint32_t COOLDOWN_MS = %lu, SETTLE_MS = %lu;\n", (unsigned long)COOLDOWN_MS, (unsigned long)SETTLE_MS);
}

void command(String in) {
  in.trim();
  if (in.length() == 0) return;
  char c = in.charAt(0);
  float v = in.substring(1).toFloat();
  switch (c) {
    case 't': PUSH_G = v; break;
    case 'q': QUIET_G = v; break;
    case 'c': COOLDOWN_MS = (uint32_t)v; break;
    case 's': SETTLE_MS = (uint32_t)v; break;
    case 'l': live = !live; Serial.println(live ? "Live readings on" : "Live readings off"); return;
    case 'r': pushes = 0; Serial.println("Push counter reset"); return;
    case 'p': break;
    default: Serial.println("? commands: t<g> q<g> c<ms> s<ms> l r p"); return;
  }
  printSettings();
}

bool startSensor() {
  if (!adxlBegin()) return false;
  Serial.printf("Accelerometer found at 0x%02X, DEVID = 0x%02X", adxlAddr, devId);
  Serial.println(devId == 0xE5 ? " (ADXL345)" : " (unexpected ID - wrong chip or clone?)");
  float a[3];
  if (!adxlRead(a)) return false;
  for (int i = 0; i < 3; i++) grav[i] = a[i];
  armed = false; quietSince = millis();
  nextSample = micros();
  Serial.println("Keep the head still for a moment, then push it.");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Accelerometer test");
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  scanBus();
  if (!startSensor()) Serial.println("No accelerometer at 0x53 / 0x1D - retrying every 2 s");
  printSettings();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { command(buf); buf = ""; }
    else buf += ch;
  }

  uint32_t now = millis();
  if (!adxlAddr) {
    if (now >= nextRetry) {
      nextRetry = now + 2000;
      if (!startSensor()) { Serial.print("Still no accelerometer. "); scanBus(); }
    }
    return;
  }

  if ((int32_t)(micros() - nextSample) < 0) return;          // 200 Hz
  nextSample += 5000;
  if ((int32_t)(micros() - nextSample) > 20000) nextSample = micros();   // fell behind, don't burst

  float a[3], motion;
  if (!adxlRead(a)) { Serial.println("Read failed - sensor disconnected?"); adxlAddr = 0; return; }
  if (detectPush(a, now, motion)) { pushes++; Serial.printf(">>> PUSH #%lu  (total %.2f g)\n", (unsigned long)pushes, sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])); }
  peak = fmaxf(peak, motion);

  if (live && now >= nextPrint) {
    nextPrint = now + 200;
    float total = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    Serial.printf("x %+.2f  y %+.2f  z %+.2f g | total %.2f g | motion %.3f g (peak %.3f) | %s\n",
                  a[0], a[1], a[2], total, motion, peak, armed ? "ARMED" : "settling");
    peak = 0;
  }
}
