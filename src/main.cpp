#include <Arduino.h>
#include <Arduino_GFX.h>
#include <databus/Arduino_ESP32RGBPanel.h>
#include <display/Arduino_RGB_Display.h>
#include <canvas/Arduino_Canvas.h>
#include <Wire.h>
#include <TAMC_GT911.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>

extern "C" void initVariant() {}

// ==================== DISPLAY (Waveshare 4.3" RGB) ====================
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
  5, 3, 46, 7,
  1, 2, 42, 41, 40,
  39, 0, 45, 48, 47, 21,
  14, 38, 18, 17, 10,
  0, 8, 4, 8,
  0, 8, 4, 8,
  1, 16000000
);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(800, 480, bus, 0, true);
Arduino_Canvas *canvas = new Arduino_Canvas(800, 480, gfx, 0, 0);

#define COLOR_GROUND canvas->color565(90, 60, 30)
#define COLOR_SKY    canvas->color565(0, 100, 200)
#define COLOR_TAPE   canvas->color565(28, 28, 28)
#define COLOR_BORDER canvas->color565(80, 80, 80)
#define COLOR_DIM    canvas->color565(120, 120, 120)

// ==================== HARDWARE ====================
#define IMU_ADDR     0x68
#define MAG_ADDR     0x0C
#define GPS_I2C_ADDR 0x10
#define TOUCH_INT    4

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

uint8_t imuRead1(uint8_t reg) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)1);
  return Wire.read();
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

  uint8_t who = imuRead1(0x75);

  if (who == 0x71 || who == 0x73) {
    imuWrite(0x37, 0x02);
    delay(10);
    Wire.beginTransmission(MAG_ADDR);
    if (Wire.endTransmission() == 0) {
      Wire.beginTransmission(MAG_ADDR);
      Wire.write(0x0B); Wire.write(0x01);
      Wire.endTransmission();
      delay(10);
      Wire.beginTransmission(MAG_ADDR);
      Wire.write(0x0A); Wire.write(0x16);
      Wire.endTransmission();
      magOk = true;
    }
  }
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
  Wire.write(0x02);
  Wire.endTransmission(false);
  if (Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)1) != 1) return;
  if (!(Wire.read() & 0x01)) return;

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x03);
  Wire.endTransmission(false);
  if (Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)7) != 7) return;

  uint8_t mb[7];
  for (int i = 0; i < 7; i++) mb[i] = Wire.read();
  if (mb[6] & 0x08) return;

  float mx = (int16_t)(mb[1] << 8 | mb[0]) - magXOff;
  float my = (int16_t)(mb[3] << 8 | mb[2]) - magYOff;
  float mz = (int16_t)(mb[5] << 8 | mb[4]) - magZOff;

  float pr = (pitch - pitchOffset) * DEG_TO_RAD;
  float rr = (roll  - rollOffset)  * DEG_TO_RAD;
  float mx2 = mx * cos(pr) + mz * sin(pr);
  float my2 = mx * sin(rr) * sin(pr) + my * cos(rr) - mz * sin(rr) * cos(pr);

  heading = atan2(-my2, mx2) * RAD_TO_DEG;
  if (heading < 0) heading += 360.0f;
}

void readGPS() {
  if (!gpsOk) return;
  if (Wire.requestFrom((uint8_t)GPS_I2C_ADDR, (uint8_t)32) == 0) return;
  while (Wire.available()) {
    char c = Wire.read();
    if (c != 0x00 && c != 0xFF) gps.encode(c);
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

  if (currentScreen != 0 && tapX >= 620 && tapY <= 70) {
    currentScreen = 0;
    return;
  }

  if (currentScreen != 0 && tapX < 180 && tapY > 410) {
    menuActive = true;
    return;
  }

  if (currentScreen == 0 && tapX < 130 && tapY > 420) {
    currentScreen = 3;
    return;
  }

  if (currentScreen == 3) {
    if (tapY >= 80 && tapY < 170) {
      if (tapX < 400) seaLevelPressure_hPa -= 1.0f;
      else            seaLevelPressure_hPa += 1.0f;
      return;
    }
    if (tapY >= 180 && tapY < 290) {
      pitchOffset = pitch;
      rollOffset = roll;
      return;
    }
    if (tapY >= 300 && tapY < 410) {
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

// ==================== DRAW: HORIZON ====================
void drawHorizon() {
  const int W = 800, H = 420;
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
      canvas->setCursor(cx - len - 30, lO - 7);
      canvas->print(abs(pv));
      canvas->setCursor(cx + len + 6, rO - 7);
      canvas->print(abs(pv));
    }
  }

  canvas->drawFastHLine(cx - 80, cy, 50, RGB565_YELLOW);
  canvas->drawFastHLine(cx - 80, cy + 1, 50, RGB565_YELLOW);
  canvas->drawFastHLine(cx + 30, cy, 50, RGB565_YELLOW);
  canvas->drawFastHLine(cx + 30, cy + 1, 50, RGB565_YELLOW);
  canvas->drawFastVLine(cx - 30, cy, 8, RGB565_YELLOW);
  canvas->drawFastVLine(cx + 30, cy, 8, RGB565_YELLOW);
  canvas->fillRect(cx - 3, cy - 1, 6, 4, RGB565_YELLOW);
}

// ==================== DRAW: ROLL TAPE (left) ====================
void drawRollTape() {
  const int tapeW = 120, tapeH = 420;
  const int cy = tapeH / 2;
  const float scale = 5.0;

  canvas->fillRect(0, 0, tapeW, tapeH, COLOR_TAPE);
  canvas->drawFastVLine(tapeW, 0, tapeH, COLOR_BORDER);

  float r = roll - rollOffset;

  for (int deg = -90; deg <= 90; deg += 5) {
    int y = cy - (int)((deg - r) * scale);
    if (y < 0 || y >= tapeH) continue;

    bool major = (deg % 10 == 0);
    int tickLen = major ? 25 : 12;
    canvas->drawFastHLine(tapeW - tickLen, y, tickLen, COLOR_BORDER);

    if (major) {
      canvas->setTextSize(2);
      canvas->setTextColor(RGB565_WHITE);
      canvas->setCursor(tapeW - tickLen - 36, y - 7);
      canvas->print(deg);
    }
  }

  canvas->fillTriangle(tapeW + 1, cy, tapeW + 14, cy - 8, tapeW + 14, cy + 8, RGB565_WHITE);

  int rollVal = (int)round(r);
  canvas->fillRect(0, cy - 22, tapeW, 44, RGB565_BLACK);
  canvas->drawRect(0, cy - 22, tapeW + 1, 44, RGB565_WHITE);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_GREEN);
  int digits = (abs(rollVal) >= 10) ? 2 : 1;
  if (rollVal < 0) digits++;
  canvas->setCursor((tapeW - digits * 24) / 2, cy - 14);
  canvas->print(rollVal);

  canvas->setTextSize(1);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(tapeW / 2 - 12, cy - 32);
  canvas->print("ROLL");
}

// ==================== DRAW: ALTITUDE TAPE (right) ====================
void drawAltitudeTape() {
  const int tapeW = 120, tapeH = 420;
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
    int tickLen = major ? 25 : 12;
    canvas->drawFastHLine(tapeX, y, tickLen, COLOR_BORDER);

    if (major) {
      canvas->setTextSize(2);
      canvas->setTextColor(RGB565_WHITE);
      canvas->setCursor(tapeX + tickLen + 4, y - 7);
      canvas->print(ft);
    }
  }

  canvas->fillTriangle(tapeX - 2, cy, tapeX - 15, cy - 8, tapeX - 15, cy + 8, RGB565_WHITE);

  int altVal = (int)round(altitude_ft);
  canvas->fillRect(tapeX, cy - 22, tapeW, 44, RGB565_BLACK);
  canvas->drawRect(tapeX - 1, cy - 22, tapeW + 1, 44, RGB565_WHITE);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_GREEN);
  canvas->setCursor(tapeX + 8, cy - 14);
  canvas->print(altVal);

  canvas->setTextSize(1);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(tapeX + tapeW / 2 - 6, cy - 32);
  canvas->print("ALT");
}

// ==================== DRAW: HEADING BAR (bottom) ====================
void drawHeadingTape() {
  const int barY = 420, barH = 60;
  const int cx = 400;
  const float pxPerDeg = 4.0;

  canvas->fillRect(0, barY, 800, barH, COLOR_TAPE);
  canvas->drawFastHLine(0, barY, 800, COLOR_BORDER);

  for (int b = -60; b <= 60; b++) {
    int deg = (int)(heading + b + 360) % 360;
    int x = cx + (int)(b * pxPerDeg);
    if (x < 0 || x >= 800) continue;

    if (deg % 30 == 0) {
      canvas->drawFastVLine(x, barY + 2, 18, RGB565_WHITE);
      const char* label = "";
      switch (deg) {
        case 0:   label = "N"; break;
        case 90:  label = "E"; break;
        case 180: label = "S"; break;
        case 270: label = "W"; break;
      }
      if (label[0]) {
        canvas->setTextSize(2);
        canvas->setTextColor(RGB565_GREEN);
        canvas->setCursor(x - 6, barY + 22);
        canvas->print(label);
      } else {
        canvas->setTextSize(2);
        canvas->setTextColor(RGB565_WHITE);
        canvas->setCursor(x - 12, barY + 22);
        canvas->print(deg);
      }
    } else if (deg % 10 == 0) {
      canvas->drawFastVLine(x, barY + 2, 10, COLOR_BORDER);
    }
  }

  canvas->fillTriangle(cx, barY + 1, cx - 8, barY - 8, cx + 8, barY - 8, RGB565_WHITE);

  canvas->fillRect(cx - 30, barY + 38, 60, 22, RGB565_BLACK);
  canvas->drawRect(cx - 30, barY + 38, 60, 22, RGB565_WHITE);
  canvas->setTextSize(2);
  canvas->setTextColor(RGB565_GREEN);
  canvas->setCursor(cx - 18, barY + 41);
  canvas->printf("%03d", (int)heading % 360);
}

// ==================== DRAW: OVERLAY ====================
void drawOverlay() {
  if (abs(roll - rollOffset) > 35 || abs(pitch - pitchOffset) > 35) {
    canvas->setTextColor(RGB565_RED);
    canvas->setTextSize(4);
    canvas->setCursor(250, 10);
    canvas->print("! DANGER !");
  }

  canvas->fillRoundRect(30, 430, 90, 50, 6, canvas->color565(40, 40, 40));
  canvas->drawRoundRect(30, 430, 90, 50, 6, COLOR_BORDER);
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(42, 440);
  canvas->print("CAL");

  canvas->setTextSize(1);
  canvas->setTextColor(gpsFix ? RGB565_GREEN : COLOR_DIM);
  canvas->setCursor(750, 425);
  canvas->print(gpsFix ? "GPS" : "---");

  canvas->setTextColor(magOk ? RGB565_GREEN : COLOR_DIM);
  canvas->setCursor(710, 425);
  canvas->print(magOk ? "MAG" : "GYR");
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
  canvas->fillRoundRect(405, 88, 385, 74, 10, canvas->color565(15, 50, 15));
  canvas->drawRoundRect(405, 88, 385, 74, 10, RGB565_GREEN);

  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(30, 102);
  canvas->print("QNH  -");
  canvas->setCursor(425, 102);
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

  if (magOk) {
    canvas->setCursor(10, 10 + row * 20); row++;
    canvas->setTextColor(RGB565_GREEN);
    canvas->print("MAG: OK (AK8963)");
  }

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

  Wire.beginTransmission(GPS_I2C_ADDR);
  gpsOk = (Wire.endTransmission() == 0);
  canvas->setCursor(10, 10 + row * 20); row++;
  canvas->setTextColor(gpsOk ? RGB565_GREEN : RGB565_RED);
  canvas->printf("GPS: %s", gpsOk ? "OK" : "FAIL");

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
  delay(30);
}
