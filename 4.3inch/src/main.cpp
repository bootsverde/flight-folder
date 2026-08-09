#include <Arduino.h>
#include <Arduino_GFX.h>
#include <databus/Arduino_ESP32RGBPanel.h>
#include <display/Arduino_RGB_Display.h>
#include <canvas/Arduino_Canvas.h>
#include <Wire.h>
#include <TAMC_GT911.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>
#include "jeep_bitmap.h"

// ==================== DISPLAY (Waveshare 4.3B RGB, 800x480) ====================
// Same carrier board / RGB pinout as the 7B (verified against Waveshare's own
// 4.3B example), only the timing constants, resolution and touch cal differ.
// bounce_buffer_size_px (last arg): 10 scanlines' worth of internal-SRAM bounce
// buffer -- the LCD peripheral DMA-reads from this instead of racing directly
// against PSRAM writes, which is what caused the tearing/shift glitches.
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
  5, 3, 46, 7,
  1, 2, 42, 41, 40,
  39, 0, 45, 48, 47, 21,
  14, 38, 18, 17, 10,
  0, 40, 48, 88,
  0, 13, 3, 32,
  1, 16000000, false,
  0, 0, 800 * 40
);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(800, 480, bus, 0, true);
Arduino_Canvas *canvas = new Arduino_Canvas(800, 480, gfx, 0, 0);

#define COLOR_GROUND canvas->color565(90, 60, 30)
#define COLOR_SKY    canvas->color565(0, 100, 200)
#define COLOR_TAPE   canvas->color565(28, 28, 28)
#define COLOR_BORDER canvas->color565(80, 80, 80)
#define COLOR_DIM    canvas->color565(120, 120, 120)
#define COLOR_JEEP   canvas->color565(255, 140, 0)
#define COLOR_AMBER  canvas->color565(255, 176, 32)

// ==================== HARDWARE ====================
#define IMU_ADDR     0x68
#define MAG_ADDR     0x0E   // IST8310 compass
#define TOUCH_INT    4

// NEO-M8N GPS is UART-only (no I2C on this breakout). GPIO17 is already the
// display's B3 data line, so UART2 can't use the Arduino default (16/17) TX pin.
#define GPS_RX_PIN   16   // ESP32 RX <- GPS TX
#define GPS_TX_PIN   15   // ESP32 TX -> GPS RX

Adafruit_BMP280 bmp;
TinyGPSPlus gps;
TAMC_GT911 ts = TAMC_GT911(8, 9, TOUCH_INT, -1, 800, 480);

uint8_t muxAddr = 0;
bool imuOk = false, magOk = false, bmpOk = false, gpsOk = false;

// ==================== STATE ====================
float pitch = 0, roll = 0, heading = 0;
float altitude_m = 0, speed_mph = 0;
bool  gpsFix = false;
unsigned long prevTime = 0;

// ==================== CALIBRATION ====================
float pitchOffset = 0, rollOffset = 0;
float gxBias = 0, gyBias = 0, gzBias = 0;
float magXOff = 0, magYOff = 0, magZOff = 0;
float seaLevelPressure_hPa = 1013.25;

// ==================== UI ====================
int  currentScreen = 0;
bool menuActive = false;
int  tapX = -1, tapY = -1;
bool touchDown = false, touchTap = false;

const char* menuItems[] = {"Horizon", "GPS", "Info", "Calibrate"};
const int   menuSize = 4;

// ==================== I2C HELPERS ====================
void imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

void selectMux() {
  if (!muxAddr) return;
  Wire.beginTransmission(muxAddr);
  Wire.write(0x0F);
  Wire.endTransmission();
}

void enableDisplay() {
  Wire.beginTransmission(0x24);
  Wire.write(0x02); Wire.write(0xFF);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(0x24);
  Wire.write(0x03); Wire.write(0xFF);
  Wire.endTransmission();
  delay(200);
}

// ==================== SENSOR INIT ====================
void initIMU() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B); Wire.write(0x80);
  imuOk = (Wire.endTransmission() == 0);
  if (!imuOk) return;
  delay(100);

  imuWrite(0x6B, 0x01);
  delay(10);
  imuWrite(0x1A, 0x03);
  imuWrite(0x1B, 0x00);
  imuWrite(0x1C, 0x00);
  imuWrite(0x1D, 0x03);
}

// IST8310 standalone compass (separate chip, not behind an MPU9250 bypass)
void initMag() {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x00);  // WHO_AM_I
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)1) != 1) return;
  if (Wire.read() != 0x10) return;

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0B); Wire.write(0x01);  // CNTRL2: soft reset
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x41); Wire.write(0x09);  // AVGCNTL: average 2 samples
  Wire.endTransmission();

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x42); Wire.write(0xC0);  // PDCNTL: required pulse duration setting
  Wire.endTransmission();

  magOk = true;
}

void calibrateGyro() {
  if (!imuOk) return;
  long sx = 0, sy = 0, sz = 0;
  int n = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    if (Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)6) == 6) {
      uint8_t b[6];
      for (int j = 0; j < 6; j++) b[j] = Wire.read();
      sx += (int16_t)((b[0] << 8) | b[1]);
      sy += (int16_t)((b[2] << 8) | b[3]);
      sz += (int16_t)((b[4] << 8) | b[5]);
      n++;
    }
    delay(5);
  }
  if (n > 0) {
    gxBias = (sx / (float)n) / 131.0f;
    gyBias = (sy / (float)n) / 131.0f;
    gzBias = (sz / (float)n) / 131.0f;
  }
}

// ==================== SENSOR READ ====================
void readIMU() {
  if (!imuOk) return;

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  if (Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)14) != 14) return;

  uint8_t buf[14];
  for (int i = 0; i < 14; i++) buf[i] = Wire.read();

  int16_t ax = (buf[0]  << 8) | buf[1];
  int16_t ay = (buf[2]  << 8) | buf[3];
  int16_t az = (buf[4]  << 8) | buf[5];
  int16_t gx = (buf[8]  << 8) | buf[9];
  int16_t gy = (buf[10] << 8) | buf[11];
  int16_t gz = (buf[12] << 8) | buf[13];

  if (ax == 0 && ay == 0 && az == 0) return;

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0f;
  if (dt <= 0 || dt > 0.5f) dt = 0.01f;
  prevTime = now;

  // GY-91 mounted vertical, pins at top: Y=up, X=lateral, Z=fore/aft
  float fax = ax / 16384.0f;
  float fay = ay / 16384.0f;
  float faz = az / 16384.0f;
  float accelPitch = atan2(faz, fay) * RAD_TO_DEG;
  float accelRoll  = atan2(-fax, fay) * RAD_TO_DEG;

  float gxRate = gx / 131.0f - gxBias;
  float gyRate = gy / 131.0f - gyBias;
  float gzRate = gz / 131.0f - gzBias;

  // vertical mount: gx=pitch, gz=roll(inverted), gy=yaw
  pitch = 0.96f * (pitch + gxRate * dt) + 0.04f * accelPitch;
  roll  = 0.96f * (roll  - gzRate * dt) + 0.04f * accelRoll;

  if (!magOk) {
    heading += gyRate * dt;
    heading = fmod(heading, 360.0f);
    if (heading < 0) heading += 360.0f;
  }
}

void readMag() {
  if (!magOk) return;

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A); Wire.write(0x01);  // CNTRL1: trigger single measurement
  Wire.endTransmission();

  bool ready = false;
  for (int i = 0; i < 10; i++) {
    delay(1);
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x02);  // STAT1
    Wire.endTransmission(false);
    if (Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)1) == 1 && (Wire.read() & 0x01)) {
      ready = true;
      break;
    }
  }
  if (!ready) return;

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x03);  // DATAXL
  Wire.endTransmission(false);
  if (Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)6) != 6) return;

  uint8_t mb[6];
  for (int i = 0; i < 6; i++) mb[i] = Wire.read();

  float mx = (int16_t)(mb[1] << 8 | mb[0]) - magXOff;
  float my = (int16_t)(mb[3] << 8 | mb[2]) - magYOff;
  float mz = (int16_t)(mb[5] << 8 | mb[4]) - magZOff;

  float pr = (pitch - pitchOffset) * DEG_TO_RAD;
  float rr = (roll  - rollOffset)  * DEG_TO_RAD;
  float mx2 = mx * cos(pr) + mz * sin(pr);
  float my2 = mx * sin(rr) * sin(pr) + my * cos(rr) - mz * sin(rr) * cos(pr);

  heading = atan2(-my2, mx2) * RAD_TO_DEG;
  if (heading < 0) heading += 360.0f;

  // As mounted, N and S read backwards but E/W are correct -- that's a mirror
  // about the E-W axis, not a uniform offset, so swap N/S only, leaving E/W be.
  heading = fmod(540.0f - heading, 360.0f);
}

void readGPS() {
  if (!gpsOk) return;
  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }
  gpsFix = gps.location.isValid();
  if (gpsFix) {
    speed_mph = gps.speed.mph();
    if (!bmpOk) altitude_m = gps.altitude.meters();
  }
}

void readBaro() {
  if (!bmpOk) return;
  altitude_m = bmp.readAltitude(seaLevelPressure_hPa);
}

// ==================== TOUCH ====================
void readTouch() {
  touchTap = false;
  ts.read();
  bool touching = ts.isTouched && ts.touches > 0;
  if (touching) {
    // NOTE: back-derived from the 7B's calibration (which itself was scaled up
    // from an 800x480 board), not measured directly on this panel. Recalibrate
    // against ts.points[0] raw values if taps land off.
    tapX = constrain(map(ts.points[0].x, 760, 86, 0, 799), 0, 799);
    tapY = constrain(map(ts.points[0].y, 427, 60, 0, 479), 0, 479);
    touchDown = true;
  } else if (touchDown) {
    touchTap = true;
    touchDown = false;
  }
}

void processTouch() {
  if (!touchTap) return;

  if (menuActive) {
    for (int i = 0; i < menuSize; i++) {
      int y = 60 + i * 100;
      if (tapY >= y && tapY < y + 100) {
        currentScreen = i;
        menuActive = false;
        return;
      }
    }
    return;
  }

  if (currentScreen != 0 && tapX >= 620 && tapY <= 76) {
    currentScreen = 0;
    return;
  }

  if (currentScreen != 0 && tapX < 180 && tapY > 415) {
    menuActive = true;
    return;
  }

  if (currentScreen == 0 && tapX < 137 && tapY > 415) {
    currentScreen = 3;
    return;
  }

  if (currentScreen == 3) {
    if (tapY >= 80 && tapY < 180) {
      if (tapX < 400) seaLevelPressure_hPa -= 1.0f;
      else            seaLevelPressure_hPa += 1.0f;
      return;
    }
    if (tapY >= 180 && tapY < 300) {
      pitchOffset = pitch;
      rollOffset = roll;
      return;
    }
    if (tapY >= 300 && tapY < 415) {
      heading = 0;
      return;
    }
  }
}

// ==================== DRAW: MENU ====================
void drawMenuButton() {
  canvas->fillRoundRect(0, 415, 170, 65, 10, canvas->color565(30, 30, 30));
  canvas->drawRoundRect(0, 415, 170, 65, 10, RGB565_WHITE);
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(30, 430);
  canvas->print("MENU");

  canvas->fillRoundRect(620, 10, 170, 55, 10, canvas->color565(40, 40, 40));
  canvas->drawRoundRect(620, 10, 170, 55, 10, RGB565_WHITE);
  canvas->setCursor(650, 22);
  canvas->print("< BACK");
}

void drawMenu() {
  canvas->fillScreen(RGB565_BLACK);
  for (int i = 0; i < menuSize; i++) {
    int y = 60 + i * 100;
    if (i == currentScreen)
      canvas->fillRect(0, y, 800, 100, canvas->color565(30, 30, 30));
    canvas->drawFastHLine(0, y, 800, canvas->color565(60, 60, 60));
    canvas->setTextSize(4);
    canvas->setTextColor(i == currentScreen ? RGB565_YELLOW : RGB565_WHITE);
    canvas->setCursor(90, y + 30);
    canvas->print(menuItems[i]);
  }
  canvas->drawFastHLine(0, 60 + menuSize * 100, 800, canvas->color565(60, 60, 60));
}

// Arduino_GFX's drawBitmap() has no scale param -- it reads the PROGMEM array
// at exactly JEEP_W/JEEP_H, so shrinking the icon means walking the source
// bits ourselves and block-filling each destination row/run at the smaller
// size, rather than regenerating the bitmap asset itself.
const float JEEP_SCALE = 0.5f;

void drawJeepScaled(int destCx, int destCy, float scale, uint16_t color) {
  const int rowBytes = (JEEP_W + 7) / 8;
  int dw = (int)(JEEP_W * scale + 0.5f);
  int dh = (int)(JEEP_H * scale + 0.5f);
  int ox = destCx - dw / 2;
  int oy = destCy - dh / 2;

  for (int sy = 0; sy < JEEP_H; sy++) {
    int dy0 = oy + (int)(sy * scale);
    int dy1 = oy + (int)((sy + 1) * scale);
    if (dy1 <= dy0) dy1 = dy0 + 1;

    int runStart = -1;
    for (int sx = 0; sx <= JEEP_W; sx++) {
      bool on = false;
      if (sx < JEEP_W) {
        uint8_t b = pgm_read_byte(&jeep_bitmap[sy * rowBytes + sx / 8]);
        on = b & (0x80 >> (sx % 8));
      }
      if (on && runStart < 0) {
        runStart = sx;
      } else if (!on && runStart >= 0) {
        int dx0 = ox + (int)(runStart * scale);
        int dx1 = ox + (int)(sx * scale);
        if (dx1 <= dx0) dx1 = dx0 + 1;
        canvas->fillRect(dx0, dy0, dx1 - dx0, dy1 - dy0, color);
        runStart = -1;
      }
    }
  }
}

// ==================== DRAW: HORIZON ====================
void drawHorizon() {
  const int W = 800, H = 368;
  int cx = W / 2, cy = H / 2;

  float r = (roll - rollOffset) * DEG_TO_RAD;
  int   p = (pitch - pitchOffset) * 4;

  float maxAngle = 85 * DEG_TO_RAD;
  if (r >  maxAngle) r =  maxAngle;
  if (r < -maxAngle) r = -maxAngle;
  float slope = tan(r);

  int y1 = cy + p + (int)((0 - cx) * slope);
  int y2 = cy + p + (int)((W - cx) * slope);

  for (int x = 0; x < W; x++) {
    int horizonY = y1 + (int)((long)(y2 - y1) * x / W);
    if (horizonY < 0) horizonY = 0;
    if (horizonY > H) horizonY = H;
    if (horizonY > 0)
      canvas->drawFastVLine(x, 0, horizonY, COLOR_SKY);
    if (horizonY < H)
      canvas->drawFastVLine(x, horizonY, H - horizonY, COLOR_GROUND);
  }
  canvas->drawLine(0, y1, W - 1, y2, RGB565_WHITE);
  canvas->drawLine(0, y1 + 1, W - 1, y2 + 1, RGB565_WHITE);

  const float pxPerDeg = 8.0;
  const int halfLen = 70, gap = 24;

  for (int pv = -30; pv <= 30; pv += 5) {
    if (pv == 0) continue;
    int yAtCx = cy + p - (int)(pv * pxPerDeg);
    if (yAtCx < 10 || yAtCx > H - 10) continue;

    int lInner = yAtCx + (int)((-gap) * slope);
    int rInner = yAtCx + (int)((gap) * slope);

    bool major = (pv % 10 == 0);
    int len = major ? halfLen : halfLen / 2;
    int lO = yAtCx + (int)((-len) * slope);
    int rO = yAtCx + (int)((len) * slope);

    if (pv < 0) {
      for (int s = 0; s < len; s += 8) {
        int s1 = s, s2 = min(s + 4, len);
        int ly1 = yAtCx + (int)((-len + s1) * slope);
        int ly2 = yAtCx + (int)((-len + s2) * slope);
        int ry1 = yAtCx + (int)((len - s2) * slope);
        int ry2 = yAtCx + (int)((len - s1) * slope);
        canvas->drawLine(cx - len + s1, ly1, cx - len + s2, ly2, RGB565_WHITE);
        canvas->drawLine(cx + len - s2, ry1, cx + len - s1, ry2, RGB565_WHITE);
      }
    } else {
      canvas->drawLine(cx - len, lO, cx - gap, lInner, RGB565_WHITE);
      canvas->drawLine(cx + gap, rInner, cx + len, rO, RGB565_WHITE);
    }

    if (major) {
      canvas->setTextSize(2);
      canvas->setTextColor(RGB565_WHITE);
      canvas->setCursor(cx - len - 23, lO - 6);
      canvas->print(abs(pv));
      canvas->setCursor(cx + len + 5, rO - 6);
      canvas->print(abs(pv));
    }
  }

  drawJeepScaled(cx, cy, JEEP_SCALE, COLOR_JEEP);
}

// ==================== DRAW: ROLL TAPE (left) ====================
void drawRollTape() {
  const int tapeW = 172, tapeH = 368;
  const int cy = tapeH / 2;
  const float scale = 5.0;

  canvas->fillRect(0, 0, tapeW, tapeH, COLOR_TAPE);
  canvas->drawFastVLine(tapeW, 0, tapeH, COLOR_BORDER);

  float r = roll - rollOffset;

  for (int deg = -90; deg <= 90; deg += 5) {
    int y = cy - (int)((deg - r) * scale);
    if (y < 0 || y >= tapeH) continue;

    bool major = (deg % 10 == 0);
    int tickLen = major ? 34 : 16;
    canvas->drawFastHLine(tapeW - tickLen, y, tickLen, RGB565_WHITE);

    // Skip the label (not the tick) within ~14px of the tape edges so the glyph
    // doesn't bleed past tapeH, where drawHeadingTape()'s fill paints over it
    // and reads as a flashing/partially-hidden number.
    if (major && y >= 14 && y <= tapeH - 14) {
      canvas->setTextSize(3);
      canvas->setTextColor(RGB565_WHITE);
      canvas->setCursor(tapeW - tickLen - 47, y - 8);
      canvas->print(deg);
    }
  }

  canvas->fillTriangle(tapeW + 1, cy, tapeW + 14, cy - 9, tapeW + 14, cy + 9, RGB565_WHITE);

  int rollVal = (int)round(r);
  canvas->fillRect(0, cy - 22, tapeW, 45, RGB565_BLACK);
  canvas->drawRect(0, cy - 22, tapeW + 1, 45, RGB565_WHITE);
  canvas->setTextSize(5);
  canvas->setTextColor(RGB565_GREEN);
  int digits = (abs(rollVal) >= 10) ? 2 : 1;
  if (rollVal < 0) digits++;
  canvas->setCursor((tapeW - digits * 30) / 2, cy - 19);
  canvas->print(rollVal);

  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(8, cy - 43);
  canvas->print("ROLL");
}

// ==================== DRAW: ALTITUDE TAPE (right) ====================
void drawAltitudeTape() {
  const int tapeW = 172, tapeH = 368;
  const int tapeX = 800 - tapeW;
  const int cy = tapeH / 2;
  const float scale = 2.0;

  canvas->fillRect(tapeX, 0, tapeW, tapeH, COLOR_TAPE);
  canvas->drawFastVLine(tapeX - 1, 0, tapeH, COLOR_BORDER);

  float altitude_ft = altitude_m * 3.28084f;
  int startFt = ((int)(altitude_ft / 20)) * 20 - 120;

  for (int ft = startFt; ft <= startFt + 240; ft += 20) {
    int y = cy - (int)((ft - altitude_ft) * scale);
    if (y < 0 || y >= tapeH) continue;

    bool major = (ft % 100 == 0);
    int tickLen = major ? 34 : 16;
    canvas->drawFastHLine(tapeX, y, tickLen, RGB565_WHITE);

    // Skip the label (not the tick) within ~14px of the tape edges so the glyph
    // doesn't bleed past tapeH, where drawHeadingTape()'s fill paints over it
    // and reads as a flashing/partially-hidden number.
    if (y >= 14 && y <= tapeH - 14) {
      canvas->setTextSize(3);
      canvas->setTextColor(RGB565_WHITE);
      canvas->setCursor(tapeX + tickLen + 5, y - 8);
      canvas->print(ft);
    }
  }

  // Amber trend vector: how fast/which way altitude is currently changing,
  // not just where it is -- same rate-of-change cue Boeing tapes use.
  static float prevAltFt = altitude_ft;
  static unsigned long prevAltMs = 0;
  unsigned long nowMs = millis();
  float dtSec = (prevAltMs == 0) ? 0.05f : (nowMs - prevAltMs) / 1000.0f;
  if (dtSec <= 0) dtSec = 0.05f;
  static float altTrendSmooth = 0;
  altTrendSmooth = 0.8f * altTrendSmooth + 0.2f * ((altitude_ft - prevAltFt) / dtSec);
  prevAltFt = altitude_ft;
  prevAltMs = nowMs;

  float trendPx = altTrendSmooth * scale * 3.0f;
  if (trendPx > 140) trendPx = 140;
  if (trendPx < -140) trendPx = -140;
  if (fabs(trendPx) > 3) {
    int tx = 796;
    int ty1 = cy - (int)trendPx;
    canvas->drawFastVLine(tx, min(cy, ty1), abs(ty1 - cy) + 1, COLOR_AMBER);
    canvas->drawFastVLine(tx - 1, min(cy, ty1), abs(ty1 - cy) + 1, COLOR_AMBER);
    int dir = trendPx < 0 ? -1 : 1;
    canvas->fillTriangle(tx, ty1, tx - 6, ty1 - dir * 9, tx + 3, ty1 - dir * 9, COLOR_AMBER);
  }

  canvas->fillTriangle(tapeX - 2, cy, tapeX - 16, cy - 9, tapeX - 16, cy + 9, RGB565_WHITE);

  int altVal = (int)round(altitude_ft);
  canvas->fillRect(tapeX, cy - 22, tapeW, 45, RGB565_BLACK);
  canvas->drawRect(tapeX - 1, cy - 22, tapeW + 1, 45, RGB565_WHITE);
  canvas->setTextSize(5);
  canvas->setTextColor(RGB565_GREEN);
  int altDigits = (abs(altVal) >= 10000) ? 5 : (abs(altVal) >= 1000) ? 4 : (abs(altVal) >= 100) ? 3 : (abs(altVal) >= 10) ? 2 : 1;
  if (altVal < 0) altDigits++;
  canvas->setCursor(tapeX + (tapeW - altDigits * 30) / 2, cy - 19);
  canvas->print(altVal);

  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(800 - 64, cy - 43);
  canvas->print("ALT");
}

// ==================== DRAW: HEADING ARC (bottom) ====================
// Boeing/Airbus-style shallow compass arc instead of a flat tape -- ticks sag
// away from center along a gentle parabola rather than running in a straight line.
void drawHeadingTape() {
  const int barY = 368, barH = 112;
  const int cx = 400;
  const float pxPerDeg = 3.9;
  const int topY = barY + 16;
  const int maxSag = 18;
  static const char* compassNames[16] = {
    "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
  };

  canvas->fillRect(0, barY, 800, barH, COLOR_TAPE);
  canvas->drawFastHLine(0, barY, 800, COLOR_BORDER);

  // Steps of 7.5 deg so every 3rd step lands exactly on a 22.5 deg (16-point) compass mark.
  for (int n = -8; n <= 8; n++) {
    float angleOffset = n * 7.5f;
    int x = cx + (int)(angleOffset * pxPerDeg);
    if (x < 0 || x >= 800) continue;

    float t = (x - cx) / 400.0f;
    int ty = topY + (int)(maxSag * t * t);

    if (n % 3 == 0) {
      float degF = fmod(heading + angleOffset + 360.0f, 360.0f);
      int idx = ((int)round(degF / 22.5f)) % 16;
      const char* label = compassNames[idx];
      int len = strlen(label);

      canvas->drawFastVLine(x, ty, 24, RGB565_WHITE);
      canvas->setTextSize(3);
      canvas->setTextColor(RGB565_WHITE);
      canvas->setCursor(x - len * 9, ty + 28);
      canvas->print(label);
    } else {
      canvas->drawFastVLine(x, ty, 13, RGB565_WHITE);
    }
  }

  canvas->fillTriangle(cx, topY - 2, cx - 8, topY - 14, cx + 8, topY - 14, RGB565_WHITE);

  canvas->fillRect(cx - 40, barY + 58, 80, 35, RGB565_BLACK);
  canvas->drawRect(cx - 40, barY + 58, 80, 35, RGB565_WHITE);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_GREEN);
  canvas->setCursor(cx - 36, barY + 64);
  canvas->printf("%03d", (int)heading % 360);
}

// Boeing/Airbus "unusual attitude" cue: black/amber hazard chevrons along the
// top of the sphere, in place of a plain text warning.
void drawAttitudeChevrons() {
  const int bandH = 20;
  const int x0 = 172, x1 = 800 - 172;
  const int thick = 10;

  canvas->fillRect(x0, 0, x1 - x0, bandH, RGB565_BLACK);
  for (int sx = x0 - bandH; sx < x1 + bandH; sx += 24) {
    canvas->fillTriangle(sx, bandH, sx + bandH, 0, sx + bandH + thick, 0, COLOR_AMBER);
    canvas->fillTriangle(sx, bandH, sx + bandH + thick, 0, sx + thick, bandH, COLOR_AMBER);
  }

  canvas->setTextSize(2);
  canvas->setTextColor(COLOR_AMBER);
  canvas->setCursor(400 - 58, bandH + 4);
  canvas->print("ATTITUDE");
}

// ==================== DRAW: OVERLAY ====================
void drawOverlay() {
  if (abs(roll - rollOffset) > 35 || abs(pitch - pitchOffset) > 35) {
    drawAttitudeChevrons();
  }

  canvas->fillRoundRect(30, 430, 90, 50, 6, canvas->color565(40, 40, 40));
  canvas->drawRoundRect(30, 430, 90, 50, 6, COLOR_BORDER);
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(41, 441);
  canvas->print("CAL");

  // Heading-source / GPS status badges, top-right corner of the heading bar.
  // Opaque boxes so they stay legible over whatever tick/label is underneath.
  // (Earlier "clipped" report was just the "---" no-fix glyph being naturally
  // short next to "MAG"'s full-height letters -- not an actual render bug.)
  canvas->setTextSize(3);

  canvas->fillRect(717, 371, 75, 27, RGB565_BLACK);
  canvas->drawRect(717, 371, 75, 27, COLOR_BORDER);
  canvas->setTextColor(magOk ? RGB565_GREEN : RGB565_RED);
  canvas->setCursor(725, 376);
  canvas->print(magOk ? "MAG" : "GYR");

  canvas->fillRect(636, 371, 75, 27, RGB565_BLACK);
  canvas->drawRect(636, 371, 75, 27, COLOR_BORDER);
  canvas->setTextColor(gpsFix ? RGB565_GREEN : RGB565_RED);
  canvas->setCursor(644, 376);
  canvas->print(gpsFix ? "GPS" : "---");
}

// ==================== DRAW: GPS SCREEN ====================
void drawGPSScreen() {
  canvas->fillScreen(RGB565_BLACK);

  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_CYAN);
  canvas->setCursor(40, 30);
  canvas->print("GPS");

  canvas->setTextSize(3);
  if (gpsFix) {
    canvas->setTextColor(RGB565_GREEN);
    canvas->setCursor(200, 30);
    canvas->print("FIX");

    canvas->setTextColor(RGB565_WHITE);
    canvas->setCursor(40, 110);
    canvas->printf("LAT: %.6f", gps.location.lat());
    canvas->setCursor(40, 160);
    canvas->printf("LON: %.6f", gps.location.lng());
    canvas->setCursor(40, 240);
    canvas->printf("SPD: %.1f mph", speed_mph);
    canvas->setCursor(40, 290);
    canvas->printf("ALT: %d ft", (int)(altitude_m * 3.28084f));
  } else {
    canvas->setTextColor(RGB565_RED);
    canvas->setCursor(200, 30);
    canvas->print("NO FIX");
  }

  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(40, 390);
  canvas->printf("SATS: %d", gps.satellites.value());

  drawMenuButton();
}

// ==================== DRAW: INFO SCREEN ====================
void drawInfoScreen() {
  canvas->fillScreen(RGB565_BLACK);

  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_CYAN);
  canvas->setCursor(40, 20);
  canvas->print("INFO");

  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(40, 100);
  canvas->printf("Pitch: %.1f", pitch - pitchOffset);
  canvas->setCursor(40, 150);
  canvas->printf("Roll:  %.1f", roll - rollOffset);
  canvas->setCursor(40, 200);
  canvas->printf("Hdg:   %.0f", heading);
  canvas->setCursor(40, 280);
  canvas->printf("Alt:   %d ft", (int)(altitude_m * 3.28084f));
  canvas->setCursor(40, 330);
  canvas->printf("QNH:   %.1f hPa", seaLevelPressure_hPa);

  canvas->setTextSize(2);
  canvas->setTextColor(COLOR_DIM);
  canvas->setCursor(40, 400);
  canvas->printf("IMU:%s  MAG:%s  BARO:%s  GPS:%s",
    imuOk ? "OK" : "--", magOk ? "OK" : "--",
    bmpOk ? "OK" : "--", gpsOk ? "OK" : "--");

  drawMenuButton();
}

// ==================== DRAW: CALIBRATE SCREEN ====================
void drawCalibrateScreen() {
  canvas->fillScreen(RGB565_BLACK);

  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(40, 22);
  canvas->print("CALIBRATE");

  canvas->drawFastHLine(0, 80, 800, canvas->color565(60, 60, 60));
  canvas->fillRoundRect(10, 88, 385, 74, 10, canvas->color565(50, 15, 15));
  canvas->drawRoundRect(10, 88, 385, 74, 10, RGB565_RED);
  canvas->fillRoundRect(406, 88, 385, 74, 10, canvas->color565(15, 50, 15));
  canvas->drawRoundRect(406, 88, 385, 74, 10, RGB565_GREEN);

  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(30, 102);
  canvas->print("QNH  -");
  canvas->setCursor(426, 102);
  canvas->print("QNH  +");

  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(280, 140);
  canvas->printf("%.1f hPa", seaLevelPressure_hPa);

  canvas->drawFastHLine(0, 180, 800, canvas->color565(60, 60, 60));
  canvas->fillRoundRect(10, 188, 780, 94, 10, canvas->color565(20, 20, 60));
  canvas->drawRoundRect(10, 188, 780, 94, 10, RGB565_CYAN);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(30, 200);
  canvas->print("LEVEL HORIZON");
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(30, 248);
  canvas->printf("P: %.1f   R: %.1f", pitch - pitchOffset, roll - rollOffset);

  canvas->drawFastHLine(0, 300, 800, canvas->color565(60, 60, 60));
  canvas->fillRoundRect(10, 308, 780, 94, 10, canvas->color565(20, 20, 60));
  canvas->drawRoundRect(10, 308, 780, 94, 10, RGB565_CYAN);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(30, 320);
  canvas->print("ZERO HEADING");
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(30, 368);
  canvas->printf("HDG: %.0f  %s", heading, magOk ? "(MAG)" : "(GYRO)");

  canvas->drawFastHLine(0, 415, 800, canvas->color565(60, 60, 60));

  drawMenuButton();
}

// ==================== BOOT ====================
void showBoot(int row) {
  canvas->setCursor(8, 8 + row * 16);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->print("tap to continue...");
  canvas->flush();

  delay(500);
  for (int t = 0; t < 600; t++) {
    ts.read();
    if (ts.isTouched && ts.touches > 0) break;
    delay(100);
  }
  for (int t = 0; t < 100; t++) {
    ts.read();
    if (!ts.isTouched) break;
    delay(50);
  }

  canvas->fillScreen(RGB565_BLACK);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_GREEN);
  canvas->setCursor(200, 180);
  canvas->print("WRANGLER");
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(260, 240);
  canvas->print("rv.3.0");
  canvas->flush();
  delay(2000);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);

  for (uint8_t a = 0x70; a <= 0x73; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { muxAddr = a; break; }
  }
  if (muxAddr) selectMux();

  enableDisplay();
  gfx->begin();
  canvas->begin(GFX_SKIP_OUTPUT_BEGIN);

  canvas->fillScreen(RGB565_BLACK);
  canvas->setTextSize(2);
  int row = 0;

  canvas->setTextColor(muxAddr ? RGB565_GREEN : RGB565_RED);
  canvas->setCursor(8, 8);
  if (muxAddr) canvas->printf("MUX: OK @ 0x%02X", muxAddr);
  else         canvas->print("MUX: NOT FOUND");
  row++;

  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(8, 8 + row * 16);
  canvas->print("I2C SCAN:");
  row++;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      canvas->setCursor(8, 8 + row * 16);
      canvas->printf("  0x%02X", addr);
      row++;
    }
  }

  initIMU();
  canvas->setCursor(8, 8 + row * 16); row++;
  canvas->setTextColor(imuOk ? RGB565_GREEN : RGB565_RED);
  canvas->printf("IMU: %s", imuOk ? "OK" : "FAIL");

  initMag();
  canvas->setCursor(8, 8 + row * 16); row++;
  canvas->setTextColor(magOk ? RGB565_GREEN : RGB565_RED);
  canvas->printf("MAG: %s (IST8310)", magOk ? "OK" : "FAIL");

  canvas->setCursor(8, 8 + row * 16); row++;
  canvas->setTextColor(RGB565_YELLOW);
  canvas->print("Calibrating gyro...");
  canvas->flush();
  calibrateGyro();
  canvas->setCursor(8, 8 + row * 16); row++;
  canvas->setTextColor(RGB565_GREEN);
  canvas->print("GYRO: OK");

  bmpOk = bmp.begin(0x77);
  if (!bmpOk) bmpOk = bmp.begin(0x76);
  canvas->setCursor(8, 8 + row * 16); row++;
  canvas->setTextColor(bmpOk ? RGB565_GREEN : RGB565_RED);
  canvas->printf("BARO: %s", bmpOk ? "OK" : "FAIL");
  if (bmpOk) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X4,
                    Adafruit_BMP280::FILTER_X4,
                    Adafruit_BMP280::STANDBY_MS_125);
  }

  Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  unsigned long gpsWaitStart = millis();
  while (millis() - gpsWaitStart < 2000 && !gpsOk) {
    if (Serial2.available()) gpsOk = true;
  }
  canvas->setCursor(8, 8 + row * 16); row++;
  canvas->setTextColor(gpsOk ? RGB565_GREEN : RGB565_RED);
  canvas->printf("GPS: %s (UART2)", gpsOk ? "OK" : "FAIL");

  canvas->setCursor(8, 8 + row * 16); row++;
  canvas->setTextColor(RGB565_WHITE);
  canvas->print("Touch init...");
  canvas->flush();

  ts.begin();
  ts.setRotation(ROTATION_NORMAL);

  prevTime = millis();
  showBoot(row);
}

// ==================== LOOP ====================
void loop() {
  readTouch();
  processTouch();

  selectMux();
  readIMU();
  readMag();
  readGPS();
  readBaro();

  if (menuActive) {
    drawMenu();
  } else {
    switch (currentScreen) {
      case 0:
        drawHorizon();
        drawRollTape();
        drawAltitudeTape();
        drawHeadingTape();
        drawOverlay();
        break;
      case 1: drawGPSScreen();       break;
      case 2: drawInfoScreen();      break;
      case 3: drawCalibrateScreen(); break;
    }
  }

  canvas->flush();
  // Single unbuffered PSRAM framebuffer (no bounce buffer in this toolchain) is
  // actively DMA-scanned while flush() writes it; keeping loop period >= panel's
  // ~52ms refresh reduces how often a write collides mid-scan (visible as a shift).
  delay(50);
}
