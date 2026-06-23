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

#define COLOR_BROWN canvas->color565(150, 75, 0)
#define COLOR_SKY   RGB565_BLUE

// ==================== HARDWARE ADDRESSES ====================
#define IMU_ADDR     0x68
#define MAG_ADDR     0x0C
#define GPS_I2C_ADDR 0x10
#define TOUCH_INT    4

// ==================== SENSOR OBJECTS ====================
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
TAMC_GT911 ts = TAMC_GT911(8, 9, TOUCH_INT, -1, 800, 480);

// ==================== SENSOR STATUS ====================
uint8_t muxAddr = 0;
bool imuOk  = false;
bool magOk  = false;
bool bmpOk  = false;
bool gpsOk  = false;

// ==================== FLIGHT STATE ====================
float pitch = 0, roll = 0, heading = 0;
float altitude_m  = 0;
float speed_mph   = 0;
bool  gpsFix      = false;
unsigned long prevTime = 0;

// ==================== CALIBRATION ====================
float pitchOffset = 0, rollOffset  = 0;
float magXOff = 0, magYOff = 0, magZOff = 0;
float seaLevelPressure_hPa = 1013.25;

// ==================== UI ====================
int  currentScreen = 0;   // 0=HUD 1=GPS 2=Info 3=Calibrate
bool menuActive    = false;
int  tapX = -1, tapY = -1;
bool touchDown = false, touchTap = false;

const char* menuItems[] = {"Horizon", "GPS", "Info", "Calibrate"};
const int   menuSize = 4;

// ==================== HELPERS ====================
void imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
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

// ==================== DISPLAY POWER ====================
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
  // reset
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B); Wire.write(0x80);
  imuOk = (Wire.endTransmission() == 0);
  if (!imuOk) return;
  delay(100);

  imuWrite(0x6B, 0x01);  // wake, PLL clock
  delay(10);
  imuWrite(0x1A, 0x03);  // DLPF 41 Hz
  imuWrite(0x1B, 0x00);  // gyro ±250°/s
  imuWrite(0x1C, 0x00);  // accel ±2g
  imuWrite(0x1D, 0x03);  // accel DLPF 41 Hz

  uint8_t who = imuRead1(0x75);
  Serial.printf("IMU WHO_AM_I: 0x%02X\n", who);

  // MPU-9250 (0x71) or MPU-9255 (0x73) have AK8963 magnetometer
  if (who == 0x71 || who == 0x73) {
    // enable I2C bypass so AK8963 appears on the bus at 0x0C
    imuWrite(0x37, 0x02);
    delay(10);

    Wire.beginTransmission(MAG_ADDR);
    if (Wire.endTransmission() == 0) {
      // reset AK8963
      Wire.beginTransmission(MAG_ADDR);
      Wire.write(0x0B); Wire.write(0x01);
      Wire.endTransmission();
      delay(10);
      // continuous mode 2 (100 Hz), 16-bit
      Wire.beginTransmission(MAG_ADDR);
      Wire.write(0x0A); Wire.write(0x16);
      Wire.endTransmission();
      magOk = true;
      Serial.println("MAG: AK8963 OK");
    }
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

  // accel angles
  float fax = ax / 16384.0f;
  float fay = ay / 16384.0f;
  float faz = az / 16384.0f;
  float accelPitch = atan2(fax, faz) * RAD_TO_DEG;
  float accelRoll  = atan2(fay, faz) * RAD_TO_DEG;

  // gyro rates (°/s at ±250 scale)
  float gxRate = gx / 131.0f;
  float gyRate = gy / 131.0f;
  float gzRate = gz / 131.0f;

  // complementary filter
  pitch = 0.96f * (pitch + gxRate * dt) + 0.04f * accelPitch;
  roll  = 0.96f * (roll  + gyRate * dt) + 0.04f * accelRoll;

  // heading: magnetometer if available, else gyro integration
  if (!magOk) {
    heading += gzRate * dt;
    heading = fmod(heading, 360.0f);
    if (heading < 0) heading += 360.0f;
  }
}

void readMag() {
  if (!magOk) return;

  // check data ready
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x02);
  Wire.endTransmission(false);
  if (Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)1) != 1) return;
  if (!(Wire.read() & 0x01)) return;

  // read 6 data bytes + ST2 (must read ST2 to complete cycle)
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x03);
  Wire.endTransmission(false);
  if (Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)7) != 7) return;

  uint8_t mb[7];
  for (int i = 0; i < 7; i++) mb[i] = Wire.read();

  if (mb[6] & 0x08) return; // overflow

  // AK8963 data is little-endian
  float mx = (int16_t)(mb[1] << 8 | mb[0]) - magXOff;
  float my = (int16_t)(mb[3] << 8 | mb[2]) - magYOff;
  float mz = (int16_t)(mb[5] << 8 | mb[4]) - magZOff;

  // tilt-compensated heading
  float pr = (pitch - pitchOffset) * DEG_TO_RAD;
  float rr = (roll  - rollOffset)  * DEG_TO_RAD;
  float mx2 =  mx * cos(pr) + mz * sin(pr);
  float my2 =  mx * sin(rr) * sin(pr) + my * cos(rr) - mz * sin(rr) * cos(pr);

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

  // BACK button (top-right) on any sub-screen -> HUD
  if (currentScreen != 0 && tapX >= 620 && tapY <= 70) {
    currentScreen = 0;
    return;
  }

  // MENU button (bottom-left) on non-horizon screens
  if (currentScreen != 0 && tapX < 180 && tapY > 410) {
    menuActive = true;
    return;
  }

  // Horizon: CAL button -> calibrate screen
  if (currentScreen == 0 && tapX < 140 && tapY > 420) {
    currentScreen = 3;
    return;
  }

  // Calibrate screen controls
  if (currentScreen == 3) {
    // QNH row (y 80..170)
    if (tapY >= 80 && tapY < 170) {
      if (tapX < 400) { seaLevelPressure_hPa -= 1.0f; return; }
      else            { seaLevelPressure_hPa += 1.0f; return; }
    }
    // LEVEL row (y 180..290)
    if (tapY >= 180 && tapY < 290) {
      pitchOffset = pitch;
      rollOffset = roll;
      return;
    }
    // HEADING row (y 300..410)
    if (tapY >= 300 && tapY < 410) {
      heading = 0;
      return;
    }
  }
}

// ==================== DRAW: MENU ====================
void drawMenuButton() {
  // MENU bottom-left
  canvas->fillRoundRect(0, 415, 170, 65, 10, canvas->color565(30, 30, 30));
  canvas->drawRoundRect(0, 415, 170, 65, 10, RGB565_WHITE);
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(30, 430);
  canvas->print("MENU");

  // BACK top-right
  canvas->fillRoundRect(620, 10, 170, 55, 10, canvas->color565(40, 40, 40));
  canvas->drawRoundRect(620, 10, 170, 55, 10, RGB565_WHITE);
  canvas->setTextColor(RGB565_WHITE);
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
  const int W = 800, H = 480;
  int cx = W / 2, cy = H / 2;

  float r = (roll - rollOffset) * DEG_TO_RAD;
  int   p = (pitch - pitchOffset) * 3;

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
      canvas->drawFastVLine(x, horizonY, H - horizonY, COLOR_BROWN);
  }
  canvas->drawLine(0, y1, W - 1, y2, RGB565_WHITE);

  // pitch ladder
  const float pxPerDeg = 6.0;
  const int halfLen = 60, gap = 20;

  for (int pv = -30; pv <= 30; pv += 10) {
    if (pv == 0) continue;
    int yAtCx = cy + p - (int)(pv * pxPerDeg);
    if (yAtCx < 0 || yAtCx > H) continue;

    int lOuter = yAtCx + (int)((-halfLen) * slope);
    int lInner = yAtCx + (int)((-gap) * slope);
    int rInner = yAtCx + (int)((gap) * slope);
    int rOuter = yAtCx + (int)((halfLen) * slope);

    canvas->drawLine(cx - halfLen, lOuter, cx - gap, lInner, RGB565_WHITE);
    canvas->drawLine(cx + gap, rInner, cx + halfLen, rOuter, RGB565_WHITE);

    canvas->setTextSize(4);
    canvas->setTextColor(RGB565_YELLOW);
    canvas->setCursor(cx - halfLen - 55, lOuter - 16);
    canvas->print(abs(pv));
    canvas->setCursor(cx + halfLen + 6, rOuter - 16);
    canvas->print(abs(pv));
  }

  // boresight
  canvas->drawFastHLine(cx - gap - 10, cy, 20, RGB565_YELLOW);
  canvas->drawFastHLine(cx + gap - 10, cy, 20, RGB565_YELLOW);
  canvas->drawFastHLine(cx - gap - 10, cy + 1, 20, RGB565_YELLOW);
  canvas->drawFastHLine(cx + gap - 10, cy + 1, 20, RGB565_YELLOW);
  canvas->drawFastVLine(cx, cy - gap - 4, 8, RGB565_YELLOW);
  canvas->drawCircle(cx, cy, 10, RGB565_YELLOW);
  canvas->fillCircle(cx, cy, 3, RGB565_YELLOW);
}

// ==================== DRAW: TILT TAPE (left) ====================
void drawRollTape() {
  const int top = 0, bottom = 480;
  const int cy = (top + bottom) / 2;
  const int halfH = (bottom - top) / 2;
  const float scale = 5.0;
  const int xEdge = 10, xMax = 70;
  const int d = xMax - xEdge;

  const float R = (float)(d * d + halfH * halfH) / (2.0f * d);
  const float cxC = xMax - R;

  canvas->fillRect(0, top, 170, bottom - top, RGB565_BLACK);

  // label
  canvas->fillRect(0, 0, 170, 70, RGB565_BLACK);
  for (int i = 0; i < 4; i++)
    canvas->drawRoundRect(i, i, 170 - 2 * i, 70 - 2 * i, 12, RGB565_WHITE);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(37, 19); canvas->print("TILT");
  canvas->setCursor(38, 19); canvas->print("TILT");

  float r = roll - rollOffset;

  for (int deg = -90; deg <= 90; deg += 10) {
    int y = cy - (int)((deg - r) * scale);
    if (y < 75 || y > bottom) continue;

    int curveX = (int)(cxC + sqrt(R * R - (float)(y - cy) * (y - cy)));
    bool major = (deg % 20 == 0);
    int tickLen = major ? 20 : 10;
    canvas->drawFastHLine(curveX, y, tickLen, RGB565_WHITE);

    if (major) {
      canvas->setTextSize(2);
      canvas->setTextColor(RGB565_YELLOW);
      canvas->setCursor(curveX + tickLen + 4, y - 7);
      canvas->print(deg);
    }
  }

  canvas->fillTriangle(xMax + 20, cy, xMax + 35, cy - 8, xMax + 35, cy + 8, RGB565_YELLOW);

  int rollVal = (int)round(r);
  canvas->fillRoundRect(52, cy - 30, 90, 60, 8, RGB565_BLACK);
  for (int i = 0; i < 3; i++)
    canvas->drawRoundRect(52 + i, cy - 30 + i, 90 - 2 * i, 60 - 2 * i, 8, RGB565_WHITE);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(61, cy - 16); canvas->print(rollVal);
  canvas->setCursor(62, cy - 16); canvas->print(rollVal);
}

// ==================== DRAW: ALTITUDE TAPE (right) ====================
void drawAltitudeTape() {
  const int top = 0, bottom = 480;
  const int cy = (top + bottom) / 2;
  const int halfH = (bottom - top) / 2;
  const float scale = 2.0;
  const int xEdge = 790, xMax = 730;
  const int d = xEdge - xMax;

  const float R = (float)(d * d + halfH * halfH) / (2.0f * d);
  const float cxC = xMax + R;

  canvas->fillRect(630, top, 170, bottom - top, RGB565_BLACK);

  // label
  canvas->fillRect(630, 0, 170, 70, RGB565_BLACK);
  for (int i = 0; i < 4; i++)
    canvas->drawRoundRect(630 + i, i, 170 - 2 * i, 70 - 2 * i, 12, RGB565_WHITE);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(679, 19); canvas->print("ALT");
  canvas->setCursor(680, 19); canvas->print("ALT");

  float altitude_ft = altitude_m * 3.28084f;
  int startFt = ((int)(altitude_ft / 20)) * 20 - 120;
  for (int ft = startFt; ft <= startFt + 240; ft += 20) {
    int y = cy - (int)((ft - altitude_ft) * scale);
    if (y < 75 || y > bottom) continue;

    int curveX = (int)(cxC - sqrt(R * R - (float)(y - cy) * (y - cy)));
    canvas->drawFastHLine(curveX - 20, y, 20, RGB565_WHITE);

    canvas->setTextSize(2);
    canvas->setTextColor(RGB565_YELLOW);
    canvas->setCursor(curveX - 64, y - 7);
    canvas->print(ft);
  }

  canvas->fillTriangle(xMax - 20, cy, xMax - 35, cy - 8, xMax - 35, cy + 8, RGB565_YELLOW);

  int altVal = (int)round(altitude_ft);
  canvas->fillRoundRect(657, cy - 30, 90, 60, 8, RGB565_BLACK);
  for (int i = 0; i < 3; i++)
    canvas->drawRoundRect(657 + i, cy - 30 + i, 90 - 2 * i, 60 - 2 * i, 8, RGB565_WHITE);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(666, cy - 16); canvas->print(altVal);
  canvas->setCursor(667, cy - 16); canvas->print(altVal);
}

// ==================== DRAW: COMPASS ROSE ====================
void drawHeadingTape() {
  const int cx = 400, cy = 240, R = 180;

  canvas->drawCircle(cx, cy, R, RGB565_CYAN);
  canvas->drawCircle(cx, cy, R - 1, RGB565_CYAN);
  canvas->drawCircle(cx, cy, R - 2, RGB565_CYAN);

  for (int b = 0; b < 360; b += 30) {
    float ang = (b - heading) * DEG_TO_RAD;
    int x1 = cx + (int)(sin(ang) * R);
    int y1 = cy - (int)(cos(ang) * R);
    int x2 = cx + (int)(sin(ang) * (R - 12));
    int y2 = cy - (int)(cos(ang) * (R - 12));
    canvas->drawLine(x1, y1, x2, y2, RGB565_CYAN);
    canvas->drawLine(x1 + 1, y1, x2 + 1, y2, RGB565_CYAN);
  }

  const char* dirs[4] = {"N", "E", "S", "W"};
  const int bearings[4] = {0, 90, 180, 270};
  for (int i = 0; i < 4; i++) {
    float ang = (bearings[i] - heading) * DEG_TO_RAD;
    int x = cx + (int)(sin(ang) * (R - 28));
    int y = cy - (int)(cos(ang) * (R - 28));
    canvas->setTextSize(3);
    canvas->setTextColor(RGB565_YELLOW);
    canvas->setCursor(x - 12, y - 12);
    canvas->print(dirs[i]);
  }

  canvas->fillTriangle(cx, cy - R - 32, cx - 22, cy - R + 4, cx + 22, cy - R + 4, RGB565_RED);
}

// ==================== DRAW: OVERLAY ====================
void drawOverlay() {
  if (abs(roll - rollOffset) > 35 || abs(pitch - pitchOffset) > 35) {
    canvas->setTextColor(RGB565_RED);
    canvas->setTextSize(4);
    canvas->setCursor(100, 20);
    canvas->print("DANGER");
  }

  // CAL button
  canvas->fillRoundRect(0, 420, 140, 60, 8, canvas->color565(30, 30, 30));
  canvas->drawRoundRect(0, 420, 140, 60, 8, RGB565_YELLOW);
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(30, 437);
  canvas->print("CAL");

  // speed readout bottom center
  canvas->setTextSize(2);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(350, 460);
  canvas->printf("%.0f mph", speed_mph);

  // heading readout top center
  canvas->fillRoundRect(355, 0, 90, 36, 6, RGB565_BLACK);
  canvas->drawRoundRect(355, 0, 90, 36, 6, RGB565_CYAN);
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_CYAN);
  canvas->setCursor(363, 6);
  canvas->printf("%03d", (int)heading % 360);

  // mag/gyro indicator
  canvas->setTextSize(1);
  canvas->setTextColor(magOk ? RGB565_GREEN : RGB565_YELLOW);
  canvas->setCursor(450, 14);
  canvas->print(magOk ? "MAG" : "GYR");

  // GPS indicator top right
  canvas->setTextSize(2);
  canvas->setTextColor(gpsFix ? RGB565_GREEN : RGB565_RED);
  canvas->setCursor(700, 80);
  canvas->print(gpsFix ? "GPS" : "---");

  // touch debug
  canvas->setTextSize(2);
  canvas->setTextColor(RGB565_RED);
  canvas->setCursor(250, 460);
  canvas->printf("X:%d Y:%d %s", tapX, tapY, touchDown ? "DN" : "  ");
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
  canvas->printf("SATS: %d  CHARS: %lu", gps.satellites.value(), gps.charsProcessed());

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
  canvas->setTextColor(canvas->color565(120, 120, 120));
  canvas->setCursor(40, 400);
  canvas->printf("IMU:%s  MAG:%s  BARO:%s  GPS:%s",
    imuOk ? "OK" : "--", magOk ? "OK" : "--",
    bmpOk ? "OK" : "--", gpsOk ? "OK" : "--");

  drawMenuButton();
}

// ==================== DRAW: CALIBRATE SCREEN ====================
void drawCalibrateScreen() {
  canvas->fillScreen(RGB565_BLACK);

  // title
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(40, 22);
  canvas->print("CALIBRATE");

  // BACK button top-right (shared)
  canvas->fillRoundRect(620, 10, 170, 55, 10, canvas->color565(40, 40, 40));
  canvas->drawRoundRect(620, 10, 170, 55, 10, RGB565_WHITE);
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(650, 22);
  canvas->print("< BACK");

  // ---- QNH row (y 80..170) ----
  canvas->drawFastHLine(0, 80, 800, canvas->color565(60, 60, 60));

  // left half = minus, right half = plus
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

  // current value centered
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(280, 140);
  canvas->printf("%.1f hPa", seaLevelPressure_hPa);

  // ---- LEVEL row (y 180..290) ----
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

  // ---- HEADING row (y 300..410) ----
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

  // touch debug
  canvas->setTextSize(2);
  canvas->setTextColor(RGB565_RED);
  canvas->setCursor(300, 450);
  canvas->printf("X:%d Y:%d %s", tapX, tapY, touchDown ? "DN" : "  ");

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
  canvas->print("rv.2");
  canvas->flush();
  delay(2000);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);

  // find I2C mux
  for (uint8_t a = 0x70; a <= 0x73; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { muxAddr = a; break; }
  }
  if (muxAddr) {
    selectMux();
    Serial.printf("MUX @ 0x%02X\n", muxAddr);
  }

  enableDisplay();
  gfx->begin();
  canvas->begin(GFX_SKIP_OUTPUT_BEGIN);

  // boot diagnostics
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
      Serial.printf("  I2C: 0x%02X\n", addr);
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
