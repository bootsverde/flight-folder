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

// ==================== DISPLAY (Waveshare 7B RGB, 1024x600) ====================
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
  5, 3, 46, 7,
  1, 2, 42, 41, 40,
  39, 0, 45, 48, 47, 21,
  14, 38, 18, 17, 10,
  0, 48, 162, 152,
  0, 3, 45, 13,
  1, 12000000
);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(1024, 600, bus, 0, true);
Arduino_Canvas *canvas = new Arduino_Canvas(1024, 600, gfx, 0, 0);

#define COLOR_GROUND canvas->color565(90, 60, 30)
#define COLOR_SKY    canvas->color565(0, 100, 200)
#define COLOR_TAPE   canvas->color565(28, 28, 28)
#define COLOR_BORDER canvas->color565(80, 80, 80)
#define COLOR_DIM    canvas->color565(120, 120, 120)
#define COLOR_JEEP   canvas->color565(255, 140, 0)

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
TAMC_GT911 ts = TAMC_GT911(8, 9, TOUCH_INT, -1, 1024, 600);

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
    // NOTE: raw range proportionally scaled from the 800x480 board's calibration,
    // not re-measured on this panel. Recalibrate against ts.points[0] raw values if taps land off.
    tapX = constrain(map(ts.points[0].x, 973, 110, 0, 1023), 0, 1023);
    tapY = constrain(map(ts.points[0].y, 534, 75, 0, 599), 0, 599);
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
      int y = 75 + i * 125;
      if (tapY >= y && tapY < y + 125) {
        currentScreen = i;
        menuActive = false;
        return;
      }
    }
    return;
  }

  if (currentScreen != 0 && tapX >= 794 && tapY <= 95) {
    currentScreen = 0;
    return;
  }

  if (currentScreen != 0 && tapX < 230 && tapY > 519) {
    menuActive = true;
    return;
  }

  if (currentScreen == 0 && tapX < 175 && tapY > 519) {
    currentScreen = 3;
    return;
  }

  if (currentScreen == 3) {
    if (tapY >= 100 && tapY < 225) {
      if (tapX < 512) seaLevelPressure_hPa -= 1.0f;
      else            seaLevelPressure_hPa += 1.0f;
      return;
    }
    if (tapY >= 225 && tapY < 375) {
      pitchOffset = pitch;
      rollOffset = roll;
      return;
    }
    if (tapY >= 375 && tapY < 519) {
      heading = 0;
      return;
    }
  }
}

// ==================== DRAW: MENU ====================
void drawMenuButton() {
  canvas->fillRoundRect(0, 519, 218, 81, 12, canvas->color565(30, 30, 30));
  canvas->drawRoundRect(0, 519, 218, 81, 12, RGB565_WHITE);
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(38, 538);
  canvas->print("MENU");

  canvas->fillRoundRect(794, 13, 218, 69, 12, canvas->color565(40, 40, 40));
  canvas->drawRoundRect(794, 13, 218, 69, 12, RGB565_WHITE);
  canvas->setCursor(832, 27);
  canvas->print("< BACK");
}

void drawMenu() {
  canvas->fillScreen(RGB565_BLACK);
  for (int i = 0; i < menuSize; i++) {
    int y = 75 + i * 125;
    if (i == currentScreen)
      canvas->fillRect(0, y, 1024, 125, canvas->color565(30, 30, 30));
    canvas->drawFastHLine(0, y, 1024, canvas->color565(60, 60, 60));
    canvas->setTextSize(4);
    canvas->setTextColor(i == currentScreen ? RGB565_YELLOW : RGB565_WHITE);
    canvas->setCursor(115, y + 38);
    canvas->print(menuItems[i]);
  }
  canvas->drawFastHLine(0, 75 + menuSize * 125, 1024, canvas->color565(60, 60, 60));
}

// ==================== DRAW: HORIZON ====================
void drawHorizon() {
  const int W = 1024, H = 460;
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

  const float pxPerDeg = 10.0;
  const int halfLen = 90, gap = 31;

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
      canvas->setCursor(cx - len - 30, lO - 7);
      canvas->print(abs(pv));
      canvas->setCursor(cx + len + 6, rO - 7);
      canvas->print(abs(pv));
    }
  }

  canvas->drawBitmap(cx - JEEP_W / 2, cy - JEEP_H / 2, jeep_bitmap, JEEP_W, JEEP_H, COLOR_JEEP);
}

// ==================== DRAW: ROLL TAPE (left) ====================
void drawRollTape() {
  const int tapeW = 220, tapeH = 460;
  const int cy = tapeH / 2;
  const float scale = 6.25;

  canvas->fillRect(0, 0, tapeW, tapeH, COLOR_TAPE);
  canvas->drawFastVLine(tapeW, 0, tapeH, COLOR_BORDER);

  float r = roll - rollOffset;

  for (int deg = -90; deg <= 90; deg += 5) {
    int y = cy - (int)((deg - r) * scale);
    if (y < 0 || y >= tapeH) continue;

    bool major = (deg % 10 == 0);
    int tickLen = major ? 44 : 20;
    canvas->drawFastHLine(tapeW - tickLen, y, tickLen, COLOR_BORDER);

    // Skip the label (not the tick) within ~14px of the tape edges so the glyph
    // doesn't bleed past tapeH, where drawHeadingTape()'s fill paints over it
    // and reads as a flashing/partially-hidden number.
    if (major && y >= 14 && y <= tapeH - 14) {
      canvas->setTextSize(3);
      canvas->setTextColor(RGB565_WHITE);
      canvas->setCursor(tapeW - tickLen - 60, y - 10);
      canvas->print(deg);
    }
  }

  canvas->fillTriangle(tapeW + 1, cy, tapeW + 18, cy - 11, tapeW + 18, cy + 11, RGB565_WHITE);

  int rollVal = (int)round(r);
  canvas->fillRect(0, cy - 28, tapeW, 56, RGB565_BLACK);
  canvas->drawRect(0, cy - 28, tapeW + 1, 56, RGB565_WHITE);
  canvas->setTextSize(6);
  canvas->setTextColor(RGB565_GREEN);
  int digits = (abs(rollVal) >= 10) ? 2 : 1;
  if (rollVal < 0) digits++;
  canvas->setCursor((tapeW - digits * 36) / 2, cy - 24);
  canvas->print(rollVal);

  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(10, cy - 54);
  canvas->print("ROLL");
}

// ==================== DRAW: ALTITUDE TAPE (right) ====================
void drawAltitudeTape() {
  const int tapeW = 220, tapeH = 460;
  const int tapeX = 1024 - tapeW;
  const int cy = tapeH / 2;
  const float scale = 2.5;

  canvas->fillRect(tapeX, 0, tapeW, tapeH, COLOR_TAPE);
  canvas->drawFastVLine(tapeX - 1, 0, tapeH, COLOR_BORDER);

  float altitude_ft = altitude_m * 3.28084f;
  int startFt = ((int)(altitude_ft / 20)) * 20 - 120;

  for (int ft = startFt; ft <= startFt + 240; ft += 20) {
    int y = cy - (int)((ft - altitude_ft) * scale);
    if (y < 0 || y >= tapeH) continue;

    bool major = (ft % 100 == 0);
    int tickLen = major ? 44 : 20;
    canvas->drawFastHLine(tapeX, y, tickLen, COLOR_BORDER);

    // Skip the label (not the tick) within ~14px of the tape edges so the glyph
    // doesn't bleed past tapeH, where drawHeadingTape()'s fill paints over it
    // and reads as a flashing/partially-hidden number.
    if (y >= 14 && y <= tapeH - 14) {
      canvas->setTextSize(3);
      canvas->setTextColor(RGB565_WHITE);
      canvas->setCursor(tapeX + tickLen + 6, y - 10);
      canvas->print(ft);
    }
  }

  canvas->fillTriangle(tapeX - 2, cy, tapeX - 20, cy - 11, tapeX - 20, cy + 11, RGB565_WHITE);

  int altVal = (int)round(altitude_ft);
  canvas->fillRect(tapeX, cy - 28, tapeW, 56, RGB565_BLACK);
  canvas->drawRect(tapeX - 1, cy - 28, tapeW + 1, 56, RGB565_WHITE);
  canvas->setTextSize(6);
  canvas->setTextColor(RGB565_GREEN);
  int altDigits = (abs(altVal) >= 10000) ? 5 : (abs(altVal) >= 1000) ? 4 : (abs(altVal) >= 100) ? 3 : (abs(altVal) >= 10) ? 2 : 1;
  if (altVal < 0) altDigits++;
  canvas->setCursor(tapeX + (tapeW - altDigits * 36) / 2, cy - 24);
  canvas->print(altVal);

  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(1024 - 64, cy - 54);
  canvas->print("ALT");
}

// ==================== DRAW: HEADING BAR (bottom) ====================
void drawHeadingTape() {
  const int barY = 460, barH = 140;
  const int cx = 512;
  const float pxPerDeg = 5.0;
  static const char* compassNames[16] = {
    "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
  };

  canvas->fillRect(0, barY, 1024, barH, COLOR_TAPE);
  canvas->drawFastHLine(0, barY, 1024, COLOR_BORDER);

  // Steps of 7.5 deg so every 3rd step lands exactly on a 22.5 deg (16-point) compass mark.
  for (int n = -8; n <= 8; n++) {
    float angleOffset = n * 7.5f;
    int x = cx + (int)(angleOffset * pxPerDeg);
    if (x < 0 || x >= 1024) continue;

    if (n % 3 == 0) {
      float degF = fmod(heading + angleOffset + 360.0f, 360.0f);
      int idx = ((int)round(degF / 22.5f)) % 16;
      const char* label = compassNames[idx];
      int len = strlen(label);

      canvas->drawFastVLine(x, barY + 2, 30, RGB565_WHITE);
      canvas->setTextSize(3);
      canvas->setTextColor((idx % 4 == 0) ? RGB565_GREEN : RGB565_WHITE);
      canvas->setCursor(x - len * 9, barY + 38);
      canvas->print(label);
    } else {
      canvas->drawFastVLine(x, barY + 2, 16, COLOR_BORDER);
    }
  }

  canvas->fillTriangle(cx, barY + 1, cx - 10, barY - 10, cx + 10, barY - 10, RGB565_WHITE);

  canvas->fillRect(cx - 45, barY + 72, 90, 44, RGB565_BLACK);
  canvas->drawRect(cx - 45, barY + 72, 90, 44, RGB565_WHITE);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_GREEN);
  canvas->setCursor(cx - 32, barY + 80);
  canvas->printf("%03d", (int)heading % 360);
}

// ==================== DRAW: OVERLAY ====================
void drawOverlay() {
  if (abs(roll - rollOffset) > 35 || abs(pitch - pitchOffset) > 35) {
    canvas->setTextColor(RGB565_RED);
    canvas->setTextSize(4);
    canvas->setCursor(320, 10);
    canvas->print("! DANGER !");
  }

  canvas->fillRoundRect(38, 538, 115, 63, 8, canvas->color565(40, 40, 40));
  canvas->drawRoundRect(38, 538, 115, 63, 8, COLOR_BORDER);
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(53, 551);
  canvas->print("CAL");

  // Heading-source / GPS status badges, top-right corner of the heading bar.
  // Opaque boxes so they stay legible over whatever tick/label is underneath.
  // (Earlier "clipped" report was just the "---" no-fix glyph being naturally
  // short next to "MAG"'s full-height letters -- not an actual render bug.)
  canvas->setTextSize(3);

  canvas->fillRect(918, 464, 96, 34, RGB565_BLACK);
  canvas->drawRect(918, 464, 96, 34, COLOR_BORDER);
  canvas->setTextColor(magOk ? RGB565_GREEN : RGB565_RED);
  canvas->setCursor(928, 470);
  canvas->print(magOk ? "MAG" : "GYR");

  canvas->fillRect(814, 464, 96, 34, RGB565_BLACK);
  canvas->drawRect(814, 464, 96, 34, COLOR_BORDER);
  canvas->setTextColor(gpsFix ? RGB565_GREEN : RGB565_RED);
  canvas->setCursor(824, 470);
  canvas->print(gpsFix ? "GPS" : "---");
}

// ==================== DRAW: GPS SCREEN ====================
void drawGPSScreen() {
  canvas->fillScreen(RGB565_BLACK);

  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_CYAN);
  canvas->setCursor(51, 38);
  canvas->print("GPS");

  canvas->setTextSize(3);
  if (gpsFix) {
    canvas->setTextColor(RGB565_GREEN);
    canvas->setCursor(256, 38);
    canvas->print("FIX");

    canvas->setTextColor(RGB565_WHITE);
    canvas->setCursor(51, 138);
    canvas->printf("LAT: %.6f", gps.location.lat());
    canvas->setCursor(51, 200);
    canvas->printf("LON: %.6f", gps.location.lng());
    canvas->setCursor(51, 300);
    canvas->printf("SPD: %.1f mph", speed_mph);
    canvas->setCursor(51, 363);
    canvas->printf("ALT: %d ft", (int)(altitude_m * 3.28084f));
  } else {
    canvas->setTextColor(RGB565_RED);
    canvas->setCursor(256, 38);
    canvas->print("NO FIX");
  }

  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(51, 488);
  canvas->printf("SATS: %d", gps.satellites.value());

  drawMenuButton();
}

// ==================== DRAW: INFO SCREEN ====================
void drawInfoScreen() {
  canvas->fillScreen(RGB565_BLACK);

  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_CYAN);
  canvas->setCursor(51, 25);
  canvas->print("INFO");

  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(51, 125);
  canvas->printf("Pitch: %.1f", pitch - pitchOffset);
  canvas->setCursor(51, 188);
  canvas->printf("Roll:  %.1f", roll - rollOffset);
  canvas->setCursor(51, 250);
  canvas->printf("Hdg:   %.0f", heading);
  canvas->setCursor(51, 350);
  canvas->printf("Alt:   %d ft", (int)(altitude_m * 3.28084f));
  canvas->setCursor(51, 413);
  canvas->printf("QNH:   %.1f hPa", seaLevelPressure_hPa);

  canvas->setTextSize(2);
  canvas->setTextColor(COLOR_DIM);
  canvas->setCursor(51, 500);
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
  canvas->setCursor(51, 28);
  canvas->print("CALIBRATE");

  canvas->drawFastHLine(0, 100, 1024, canvas->color565(60, 60, 60));
  canvas->fillRoundRect(13, 110, 493, 93, 12, canvas->color565(50, 15, 15));
  canvas->drawRoundRect(13, 110, 493, 93, 12, RGB565_RED);
  canvas->fillRoundRect(519, 110, 493, 93, 12, canvas->color565(15, 50, 15));
  canvas->drawRoundRect(519, 110, 493, 93, 12, RGB565_GREEN);

  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(39, 128);
  canvas->print("QNH  -");
  canvas->setCursor(545, 128);
  canvas->print("QNH  +");

  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(358, 175);
  canvas->printf("%.1f hPa", seaLevelPressure_hPa);

  canvas->drawFastHLine(0, 225, 1024, canvas->color565(60, 60, 60));
  canvas->fillRoundRect(13, 235, 998, 118, 12, canvas->color565(20, 20, 60));
  canvas->drawRoundRect(13, 235, 998, 118, 12, RGB565_CYAN);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(39, 250);
  canvas->print("LEVEL HORIZON");
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(39, 310);
  canvas->printf("P: %.1f   R: %.1f", pitch - pitchOffset, roll - rollOffset);

  canvas->drawFastHLine(0, 375, 1024, canvas->color565(60, 60, 60));
  canvas->fillRoundRect(13, 385, 998, 118, 12, canvas->color565(20, 20, 60));
  canvas->drawRoundRect(13, 385, 998, 118, 12, RGB565_CYAN);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(39, 400);
  canvas->print("ZERO HEADING");
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(39, 460);
  canvas->printf("HDG: %.0f  %s", heading, magOk ? "(MAG)" : "(GYRO)");

  canvas->drawFastHLine(0, 519, 1024, canvas->color565(60, 60, 60));

  drawMenuButton();
}

// ==================== BOOT ====================
void showBoot(int row) {
  canvas->setCursor(10, 10 + row * 20);
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
  canvas->setCursor(256, 225);
  canvas->print("WRANGLER");
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(333, 300);
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
  canvas->setCursor(10, 10);
  if (muxAddr) canvas->printf("MUX: OK @ 0x%02X", muxAddr);
  else         canvas->print("MUX: NOT FOUND");
  row++;

  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(10, 10 + row * 20);
  canvas->print("I2C SCAN:");
  row++;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      canvas->setCursor(10, 10 + row * 20);
      canvas->printf("  0x%02X", addr);
      row++;
    }
  }

  initIMU();
  canvas->setCursor(10, 10 + row * 20); row++;
  canvas->setTextColor(imuOk ? RGB565_GREEN : RGB565_RED);
  canvas->printf("IMU: %s", imuOk ? "OK" : "FAIL");

  initMag();
  canvas->setCursor(10, 10 + row * 20); row++;
  canvas->setTextColor(magOk ? RGB565_GREEN : RGB565_RED);
  canvas->printf("MAG: %s (IST8310)", magOk ? "OK" : "FAIL");

  canvas->setCursor(10, 10 + row * 20); row++;
  canvas->setTextColor(RGB565_YELLOW);
  canvas->print("Calibrating gyro...");
  canvas->flush();
  calibrateGyro();
  canvas->setCursor(10, 10 + row * 20); row++;
  canvas->setTextColor(RGB565_GREEN);
  canvas->print("GYRO: OK");

  bmpOk = bmp.begin(0x77);
  if (!bmpOk) bmpOk = bmp.begin(0x76);
  canvas->setCursor(10, 10 + row * 20); row++;
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
  canvas->setCursor(10, 10 + row * 20); row++;
  canvas->setTextColor(gpsOk ? RGB565_GREEN : RGB565_RED);
  canvas->printf("GPS: %s (UART2)", gpsOk ? "OK" : "FAIL");

  canvas->setCursor(10, 10 + row * 20); row++;
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
