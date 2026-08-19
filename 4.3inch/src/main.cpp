#include <Arduino.h>
#include <Arduino_GFX.h>
#include <databus/Arduino_ESP32RGBPanel.h>
#include <display/Arduino_RGB_Display.h>
#include <canvas/Arduino_Canvas.h>
#include <Wire.h>
#include <TAMC_GT911.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
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
#define MAG_ADDR     0x0C   // AK8963, built into the MPU-9250/9255, reached via I2C bypass
#define TOUCH_INT    4
#define I2C_SDA_PIN  8
#define I2C_SCL_PIN  9

// NEO-M8N GPS is UART-only (no I2C on this breakout). GPIO17 is already the
// display's B3 data line, so UART2 can't use the Arduino default (16/17) TX pin.
#define GPS_RX_PIN   16   // ESP32 RX <- GPS TX
#define GPS_TX_PIN   15   // ESP32 TX -> GPS RX

Adafruit_BMP280 bmp;
TinyGPSPlus gps;
TAMC_GT911 ts = TAMC_GT911(8, 9, TOUCH_INT, -1, 800, 480);

uint8_t muxAddr = 0;
bool imuOk = false, magOk = false, bmpOk = false, gpsOk = false;

// Consecutive failed IMU reads -- used as the canary for a wedged I2C bus
// (automotive electrical noise can leave a slave holding SDA low mid-
// transaction). The IMU is read every loop, making it the fastest signal.
uint16_t imuFailStreak = 0;
const uint16_t I2C_FAIL_THRESHOLD = 40;  // ~2s of consecutive misses at the ~50ms loop period

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

// Persisted to NVS so QNH / level-horizon calibration survives power cycles
// instead of resetting to defaults every time this gets power-cycled in the
// Jeep. Writes are debounced -- only flushed CAL_SAVE_DEBOUNCE_MS after the
// value stops changing, so mashing the QNH +/- buttons doesn't hammer flash.
Preferences prefs;
bool calDirty = false;
unsigned long calDirtyTime = 0;
const unsigned long CAL_SAVE_DEBOUNCE_MS = 1000;

void markCalDirty() {
  calDirty = true;
  calDirtyTime = millis();
}

void saveCalIfDirty() {
  if (!calDirty || millis() - calDirtyTime < CAL_SAVE_DEBOUNCE_MS) return;
  prefs.putFloat("pitchOff", pitchOffset);
  prefs.putFloat("rollOff", rollOffset);
  prefs.putFloat("qnh", seaLevelPressure_hPa);
  calDirty = false;
  Serial.println("Calibration saved to NVS");
}

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

// Classic I2C bus-recovery: if a slave got left holding SDA low mid-
// transaction (an automotive electrical glitch is a plausible cause here),
// the bus is wedged and every future transaction will silently fail. Drop
// down to bit-banging the pins, clock SCL until the slave releases SDA, then
// manufacture a STOP condition and hand the bus back to the Wire driver.
// This only clears the electrical wedge -- sensor register config (mag
// continuous mode, baro sampling, etc.) survives and doesn't need re-init.
void recoverI2CBus() {
  Wire.end();
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, OUTPUT);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(5);

  if (digitalRead(I2C_SDA_PIN) == HIGH) {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    return;  // bus wasn't actually stuck -- the failures had some other cause
  }

  Serial.println("I2C bus recovery: SDA stuck low, pulsing SCL...");
  for (int i = 0; i < 9 && digitalRead(I2C_SDA_PIN) == LOW; i++) {
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
  }

  // STOP condition: SDA rising while SCL is held high.
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(I2C_SDA_PIN, HIGH);
  delayMicroseconds(5);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.printf("I2C bus recovery done, SDA now %s\n",
                digitalRead(I2C_SDA_PIN) == HIGH ? "released" : "STILL STUCK");
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
  // CONFIG (gyro/temp DLPF) and ACCEL_CONFIG2 (accel DLPF): was DLPF_CFG=3
  // (~41Hz gyro / ~45Hz accel bandwidth), which let a lot of Jeep engine and
  // road vibration straight through -- felt "jerky"/twitchy on a test drive
  // even on smooth pavement. Tightened to DLPF_CFG=5 (~10Hz both), which
  // rejects most vibration content while still responding to real attitude
  // changes (which happen over seconds, not tens of Hz). Costs a bit of
  // added filter delay (~18ms gyro / ~36ms accel) -- negligible next to the
  // ~50-65ms display loop period. Bump to 6 (~5Hz) if still twitchy, or back
  // toward 4/3 if it starts feeling laggy on quick maneuvers.
  imuWrite(0x1A, 0x05);
  imuWrite(0x1B, 0x00);
  imuWrite(0x1C, 0x00);
  imuWrite(0x1D, 0x05);

  // Bypass the MPU-9250's I2C master so its built-in AK8963 magnetometer
  // is directly addressable on the main bus at 0x0C, instead of only being
  // reachable through the IMU's internal auxiliary I2C bus.
  imuWrite(0x6A, 0x00);   // USER_CTRL: disable I2C master mode
  imuWrite(0x37, 0x02);   // INT_PIN_CFG: BYPASS_EN
  delay(10);

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x75);  // WHO_AM_I
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)1);
  uint8_t whoami = Wire.read();
  Serial.printf("IMU WHO_AM_I=0x%02X (9250=0x71, 9255=0x73, 6500=0x70 -- 6500 has no AK8963)\n", whoami);
}

// AK8963 magnetometer built into the MPU-9250/9255 (the GY-91's own compass),
// reached through the I2C bypass initIMU() enables -- not a separate chip.
float magAsaX = 1, magAsaY = 1, magAsaZ = 1;  // factory sensitivity adjustment, from Fuse ROM

void initMag() {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x00);  // WIA (who I am)
  uint8_t ackErr = Wire.endTransmission(false);
  if (ackErr != 0) {
    Serial.printf("AK8963 probe: no ACK at 0x%02X (I2C error %d)\n", MAG_ADDR, ackErr);
    return;
  }
  if (Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)1) != 1) {
    Serial.println("AK8963 probe: WIA read failed");
    return;
  }
  uint8_t wia = Wire.read();
  Serial.printf("AK8963 WIA=0x%02X (expect 0x48)\n", wia);
  if (wia != 0x48) return;

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A); Wire.write(0x00);  // CNTL1: power-down (required before mode change)
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A); Wire.write(0x0F);  // CNTL1: fuse ROM access mode
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x10);  // ASAX (sensitivity adjustment values start here)
  Wire.endTransmission(false);
  if (Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)3) == 3) {
    uint8_t asax = Wire.read(), asay = Wire.read(), asaz = Wire.read();
    magAsaX = ((asax - 128) * 0.5f / 128.0f) + 1.0f;
    magAsaY = ((asay - 128) * 0.5f / 128.0f) + 1.0f;
    magAsaZ = ((asaz - 128) * 0.5f / 128.0f) + 1.0f;
  }

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A); Wire.write(0x00);  // CNTL1: power-down again before switching modes
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A); Wire.write(0x16);  // CNTL1: 16-bit output, continuous mode 2 (100 Hz)
  Wire.endTransmission();
  delay(10);

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
  if (Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)14) != 14) { imuFailStreak++; return; }

  uint8_t buf[14];
  for (int i = 0; i < 14; i++) buf[i] = Wire.read();

  int16_t ax = (buf[0]  << 8) | buf[1];
  int16_t ay = (buf[2]  << 8) | buf[3];
  int16_t az = (buf[4]  << 8) | buf[5];
  int16_t gx = (buf[8]  << 8) | buf[9];
  int16_t gy = (buf[10] << 8) | buf[11];
  int16_t gz = (buf[12] << 8) | buf[13];

  if (ax == 0 && ay == 0 && az == 0) { imuFailStreak++; return; }
  imuFailStreak = 0;

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0f;
  if (dt <= 0 || dt > 0.5f) dt = 0.01f;
  prevTime = now;

  // GY-91 mounted vertical, pins at top: Y=up, X=lateral, Z=fore/aft
  float fax = ax / 16384.0f;
  float fay = ay / 16384.0f;
  float faz = az / 16384.0f;
  float accelPitch = atan2(faz, fay) * RAD_TO_DEG + 130.5f;   // this GY-91's mount reads -130.5 deg at true level
  float accelRoll  = atan2(-fax, fay) * RAD_TO_DEG - 93.0f;   // this GY-91's mount is rotated 93 deg CCW from the reference orientation
  accelPitch = fmod(accelPitch + 180.0f, 360.0f);
  if (accelPitch < 0) accelPitch += 360.0f;
  accelPitch -= 180.0f;
  accelRoll = fmod(accelRoll + 180.0f, 360.0f);
  if (accelRoll < 0) accelRoll += 360.0f;
  accelRoll -= 180.0f;

  // Pitch axis reads backwards on this mount (nose-up shows as nose-down) --
  // flip both the accel and gyro contributions together so the complementary
  // filter's two inputs stay in agreement instead of fighting each other.
  // Negating post-normalization is safe: 0 (level) stays 0 either way.
  accelPitch = -accelPitch;

  float gxRate = gx / 131.0f - gxBias;
  float gyRate = gy / 131.0f - gyBias;
  float gzRate = gz / 131.0f - gzBias;

  // Sensor mount orientation in the HUD box: X=right, Y=up, Z=rear. So pitch
  // rotates about X, yaw about Y, and roll about Z -- gx=pitch (inverted, see
  // above), gy=yaw, gz=roll (inverted). The axis *assignment* here follows
  // directly from that orientation; which ones need sign-flipping doesn't
  // (that's this chip's own sign convention combined with mount handedness)
  // -- both inversions here were found empirically, not derived.
  pitch = 0.96f * (pitch - gxRate * dt) + 0.04f * accelPitch;
  roll  = 0.96f * (roll  - gzRate * dt) + 0.04f * accelRoll;

  if (!magOk) {
    heading += gyRate * dt;
    heading = fmod(heading, 360.0f);
    if (heading < 0) heading += 360.0f;
  }
}

void readMag() {
  if (!magOk) return;

  // AK8963 free-runs in continuous mode (set in initMag()) -- just check DRDY.
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x02);  // ST1
  Wire.endTransmission(false);
  if (Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)1) != 1) return;
  if (!(Wire.read() & 0x01)) return;  // data not ready yet

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x03);  // HXL
  Wire.endTransmission(false);
  // Read through ST2 (0x09) in the same transaction -- AK8963 requires ST2 to
  // be read to latch the sample and confirm no overflow; skipping it leaves
  // DRDY stuck and the next read stale.
  if (Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)7) != 7) return;

  uint8_t mb[7];
  for (int i = 0; i < 7; i++) mb[i] = Wire.read();
  if (mb[6] & 0x08) return;  // ST2 HOFL bit: magnetic sensor overflow, discard

  float mx = (int16_t)(mb[1] << 8 | mb[0]) * magAsaX - magXOff;
  float my = (int16_t)(mb[3] << 8 | mb[2]) * magAsaY - magYOff;
  float mz = (int16_t)(mb[5] << 8 | mb[4]) * magAsaZ - magZOff;

  float pr = (pitch - pitchOffset) * DEG_TO_RAD;
  float rr = (roll  - rollOffset)  * DEG_TO_RAD;
  float mx2 = mx * cos(pr) + mz * sin(pr);
  float my2 = mx * sin(rr) * sin(pr) + my * cos(rr) - mz * sin(rr) * cos(pr);

  heading = atan2(-my2, mx2) * RAD_TO_DEG;
  if (heading < 0) heading += 360.0f;

  // This N/S swap was tuned for the IST8310's mounting orientation. The AK8963
  // (inside the MPU-9250) may not need the same correction, or may need a
  // different one -- verify heading against a known reference (phone compass)
  // and adjust or remove this line once real readings are in hand.
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

// Home field's known elevation -- lets the Calibrate screen back-solve the
// exact QNH from today's raw pressure in one tap, instead of nudging QNH
// +-1 hPa by hand and eyeballing the altitude readout.
const float HOME_FIELD_ALT_FT = 52.0f;
const float HOME_FIELD_ALT_M  = HOME_FIELD_ALT_FT / 3.28084f;

void syncQnhToKnownAltitude() {
  if (!bmpOk) return;
  float pressure_hPa = bmp.readPressure() / 100.0f;
  seaLevelPressure_hPa = pressure_hPa / pow(1.0f - HOME_FIELD_ALT_M / 44330.0f, 1.0f / 0.1903f);
  Serial.printf("QNH synced to known field alt (%.0fft): pressure=%.1fhPa -> QNH=%.1fhPa\n",
                HOME_FIELD_ALT_FT, pressure_hPa, seaLevelPressure_hPa);
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
      markCalDirty();
      return;
    }
    if (tapY >= 180 && tapY < 300) {
      pitchOffset = pitch;
      rollOffset = roll;
      markCalDirty();
      return;
    }
    if (tapY >= 300 && tapY < 415) {
      heading = 0;
      return;
    }
    // Bottom strip right of the MENU button (which already claims x<180 via
    // the currentScreen!=0 && tapX<180 && tapY>415 check above this block).
    if (tapY > 415) {
      syncQnhToKnownAltitude();
      markCalDirty();
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

  // Bottom strip, right of the MENU button: one-tap QNH sync against the
  // known home-field elevation, instead of nudging QNH +-1 hPa by hand.
  canvas->fillRoundRect(180, 415, 620, 65, 10, canvas->color565(15, 40, 15));
  canvas->drawRoundRect(180, 415, 620, 65, 10, RGB565_GREEN);
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(200, 432);
  canvas->printf("SYNC ALT -> %.0f ft", HOME_FIELD_ALT_FT);

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

  prefs.begin("hud", false);
  // No "qnh" key yet means this is the very first boot with nothing saved --
  // seed it from the known home-field elevation once (below, after the baro
  // is up) instead of leaving it at the standard-atmosphere default. Every
  // boot after this trusts whatever's persisted (this seed, or a later
  // manual QNH +-/SYNC ALT) rather than re-syncing to 52ft every time, since
  // this unit won't always power on at home.
  bool firstBoot = !prefs.isKey("qnh");
  pitchOffset = prefs.getFloat("pitchOff", 0);
  rollOffset = prefs.getFloat("rollOff", 0);
  seaLevelPressure_hPa = prefs.getFloat("qnh", 1013.25f);
  Serial.printf("Calibration loaded from NVS: pitchOff=%.1f rollOff=%.1f qnh=%.1f%s\n",
                pitchOffset, rollOffset, seaLevelPressure_hPa,
                firstBoot ? " (first boot, will seed from home-field alt)" : "");

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
  canvas->printf("MAG: %s (AK8963)", magOk ? "OK" : "FAIL");

  canvas->setCursor(8, 8 + row * 16); row++;
  canvas->setTextColor(RGB565_YELLOW);
  canvas->print("Calibrating gyro...");
  canvas->flush();
  calibrateGyro();
  canvas->setCursor(8, 8 + row * 16); row++;
  canvas->setTextColor(RGB565_GREEN);
  canvas->print("GYRO: OK");

  bmpOk = bmp.begin(0x77);
  uint8_t bmpAddr = bmpOk ? 0x77 : 0;
  if (!bmpOk) { bmpOk = bmp.begin(0x76); if (bmpOk) bmpAddr = 0x76; }
  Serial.printf("BARO begin: %s at 0x%02X\n", bmpOk ? "OK" : "FAIL", bmpAddr);
  canvas->setCursor(8, 8 + row * 16); row++;
  canvas->setTextColor(bmpOk ? RGB565_GREEN : RGB565_RED);
  canvas->printf("BARO: %s", bmpOk ? "OK" : "FAIL");
  if (bmpOk) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X4,
                    Adafruit_BMP280::FILTER_X4,
                    Adafruit_BMP280::STANDBY_MS_125);
    delay(50);

    if (firstBoot) {
      syncQnhToKnownAltitude();
      markCalDirty();
      Serial.println("First boot: seeded QNH from home-field elevation");
    }

    Serial.printf("BARO first read: temp=%.2fC pressure=%.2fhPa altitude=%.1fm (%.0fft)\n",
                  bmp.readTemperature(), bmp.readPressure() / 100.0f,
                  bmp.readAltitude(seaLevelPressure_hPa),
                  bmp.readAltitude(seaLevelPressure_hPa) * 3.28084f);
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

  // Armed only after showBoot()'s up-to-60s "waiting for a tap" delay --
  // arming it earlier would panic-reboot the board mid-splash-screen every
  // time nobody's tapped it yet. From here on, loop() must feed it at least
  // once every WDT_TIMEOUT_S or the chip force-reboots, recovering from a
  // genuine hang (e.g. a display/DMA stall) instead of freezing forever.
  const uint32_t WDT_TIMEOUT_S = 8;
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
}

// This RGB panel has no double-buffer -- confirmed in the GFX library itself
// ("It uses a Single Frame Buffer in PSRAM"), and draw16bitRGBBitmap() writes
// straight into that single buffer with zero sync to the DMA scan-out. A
// full-frame flush() is therefore always racing the scan; this doesn't
// eliminate that race, but chunking the same write into smaller row-bands
// shrinks how long any single write is "live" and how much of the screen it
// can smear if it does land mid-scan. Bands must stay full-width (0..800):
// draw16bitRGBBitmap() has no stride parameter, so a sub-region only reads
// correctly out of the canvas buffer when it spans the entire row.
void flushBanded() {
  const int BAND_H = 60;  // 480 / 60 = 8 bands/frame
  uint16_t *fb = canvas->getFramebuffer();
  for (int y = 0; y < 480; y += BAND_H) {
    gfx->draw16bitRGBBitmap(0, y, fb + (size_t)y * 800, 800, BAND_H);
  }
}

// ==================== LOOP ====================
void loop() {
  esp_task_wdt_reset();

  readTouch();
  processTouch();
  saveCalIfDirty();

  selectMux();
  readIMU();
  readMag();
  readGPS();
  readBaro();

  // IMU canary for a wedged I2C bus (see recoverI2CBus() for why). Recovery
  // only clears the electrical wedge, so nothing downstream needs re-init.
  if (imuOk && imuFailStreak > I2C_FAIL_THRESHOLD) {
    recoverI2CBus();
    imuFailStreak = 0;
  }

  static unsigned long lastAttPrint = 0;
  if (millis() - lastAttPrint > 500) {
    lastAttPrint = millis();
    Serial.printf("pitch=%.1f roll=%.1f\n", pitch, roll);
  }

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

  flushBanded();
  // Single unbuffered PSRAM framebuffer (no bounce buffer in this toolchain) is
  // actively DMA-scanned while flush() writes it; keeping loop period >= panel's
  // ~52ms refresh reduces how often a write collides mid-scan (visible as a shift).
  delay(50);
}
