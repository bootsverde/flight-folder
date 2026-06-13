#include <Arduino.h>
#include <Arduino_GFX.h>
#include <databus/Arduino_ESP32RGBPanel.h>
#include <display/Arduino_RGB_Display.h>
#include <canvas/Arduino_Canvas.h>
#include <Wire.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_seesaw.h>
#include <TinyGPSPlus.h>

// Prevent esp32s3box BSP from initializing its own peripherals
// which conflict with the Waveshare RGB panel pin assignments
extern "C" void initVariant() {}

// ================= DISPLAY (Waveshare RGB panel) =================
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
  5, 3, 46, 7,             // DE, VSYNC, HSYNC, PCLK
  1, 2, 42, 41, 40,        // R0-R4
  39, 0, 45, 48, 47, 21,   // G0-G5
  14, 38, 18, 17, 10,      // B0-B4
  0, 8, 4, 8,              // hsync: polarity, front_porch, pulse_width, back_porch
  0, 8, 4, 8,              // vsync: polarity, front_porch, pulse_width, back_porch
  1, 16000000              // pclk_active_neg, prefer_speed
);

Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
  800, 480, bus, 0, true
);

// Full-screen offscreen buffer (avoids flicker from drawing directly
// to the live, continuously-scanned-out RGB panel framebuffer)
Arduino_Canvas *canvas = new Arduino_Canvas(800, 480, gfx, 0, 0);

// Extra colors not predefined by Arduino_GFX_Library
#define COLOR_BROWN     canvas->color565(150, 75, 0)
#define COLOR_DARKGREEN canvas->color565(0, 100, 0)

// ================= SENSORS =================
Adafruit_ISM330DHCX imu;
Adafruit_BMP3XX bmp;
bool bmpOk = false;
float seaLevelPressure_hPa = 1013.25;
TinyGPSPlus gps;
#define gpsSerial Serial1

// ================= ENCODER =================
// Adafruit seesaw-based STEMMA QT I2C rotary encoder
#define SEESAW_ADDR 0x36
#define SS_SWITCH   24

Adafruit_seesaw ss;
bool encOk = false;
int32_t encoderPosition = 0;
bool encButtonHeld = false;
unsigned long encButtonPressTime = 0;

// ================= CALIBRATE SCREEN =================
const char* calibItems[] = {
  "QNH (hPa)", "Zero Pitch/Roll", "Zero Heading"
};
const int calibSize = 3;
int calibIndex = 0;

// ================= GLOBAL =================
float pitch = 0, roll = 0, heading = 0;
float speed_kmh = 0;
float altitude_m = 0;
bool gpsFix = false;

// ================= MENU =================
bool menuActive = false;
int menuIndex = 0;
int currentScreen = 0;
int menuOffset = 0;

const char* menuItems[] = {
  "Horizon", "GPS", "Info", "Calibrate"
};
const int menuSize = 4;

// ================= CALIBRATION =================
float pitchOffset = 0, rollOffset = 0;

// ================= FORWARD DECLARATIONS =================
void enableDisplay();
void readEncoder();
void readIMU();
void readHeading();
void readGPS();
void readBaro();
void drawMenu();
void drawIcon(int x, int y, int type);
void handleMenu();
void screenTransition();
void drawHorizon();
void drawRollTape();
void drawAltitudeTape();
void drawHeadingTape();
void drawOverlay();
void drawGPSScreen();
void drawInfoScreen();
void drawCalibrateScreen();
void showBoot();

// ================= DISPLAY POWER =================
void enableDisplay() {
  Wire.begin(8, 9);

  // reg 0x02 = mode register: 0xFF = all 8 pins as outputs
  Wire.beginTransmission(0x24);
  Wire.write(0x02);
  Wire.write(0xFF);
  Wire.endTransmission();
  delay(10);

  // reg 0x03 = output register: 0xFF = all pins HIGH
  // IO2 = backlight ON, IO3 = LCD reset released HIGH
  Wire.beginTransmission(0x24);
  Wire.write(0x03);
  Wire.write(0xFF);
  Wire.endTransmission();
  delay(200);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  enableDisplay();

  Serial.println("Calling gfx->begin()...");
  bool gfxOk = gfx->begin();
  Serial.printf("gfx->begin() = %d\n", gfxOk);

  bool canvasOk = canvas->begin(GFX_SKIP_OUTPUT_BEGIN);
  Serial.printf("canvas->begin() = %d\n", canvasOk);

  Wire.begin();
  bool imuOk = imu.begin_I2C();
  Serial.printf("imu.begin_I2C() = %d\n", imuOk);

  bmpOk = bmp.begin_I2C();
  Serial.printf("bmp.begin_I2C() = %d\n", bmpOk);
  if (bmpOk) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  gpsSerial.begin(9600);

  encOk = ss.begin(SEESAW_ADDR);
  Serial.printf("ss.begin() = %d\n", encOk);
  if (encOk) {
    ss.pinMode(SS_SWITCH, INPUT_PULLUP);
    encoderPosition = ss.getEncoderPosition();
  }

  showBoot();
}

// ================= LOOP =================
void loop() {
  readEncoder();
  readIMU();
  readHeading();
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
      case 1: drawGPSScreen(); break;
      case 2: drawInfoScreen(); break;
      case 3: drawCalibrateScreen(); break;
    }
  }

  canvas->flush();

  delay(100);
}

// ================= IMU =================
void readIMU() {
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);

  float newPitch = atan2(accel.acceleration.x, accel.acceleration.z) * 57.3;
  float newRoll  = atan2(accel.acceleration.y, accel.acceleration.z) * 57.3;

  float alpha = 0.1;
  pitch = pitch*(1-alpha) + newPitch*alpha;
  roll  = roll*(1-alpha) + newRoll*alpha;
}

// ================= HEADING =================
void readHeading() {
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);

  heading += gyro.gyro.z;
  heading = fmod(heading, 360);
  if (heading < 0) heading += 360;
}

// ================= GPS =================
void readGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  gpsFix = gps.location.isValid();
  if (gpsFix) {
    speed_kmh = gps.speed.kmph();
    if (!bmpOk) altitude_m = gps.altitude.meters();
  }
}

// ================= BAROMETER =================
void readBaro() {
  if (!bmpOk) return;
  if (bmp.performReading()) {
    altitude_m = bmp.readAltitude(seaLevelPressure_hPa);
  }
}

// ================= ENCODER =================
void readEncoder() {
  if (!encOk) return;

  int32_t newPosition = ss.getEncoderPosition();
  int32_t delta = newPosition - encoderPosition;
  encoderPosition = newPosition;

  if (menuActive) {
    menuIndex += delta;
    menuIndex %= menuSize;
    if (menuIndex < 0) menuIndex += menuSize;
  } else if (currentScreen == 3 && delta != 0) {
    switch (calibIndex) {
      case 0: // QNH
        seaLevelPressure_hPa += delta * 0.5;
        if (seaLevelPressure_hPa < 950) seaLevelPressure_hPa = 950;
        if (seaLevelPressure_hPa > 1050) seaLevelPressure_hPa = 1050;
        break;
    }
  }

  bool pressed = !ss.digitalRead(SS_SWITCH);

  if (pressed && !encButtonHeld) {
    encButtonHeld = true;
    encButtonPressTime = millis();
  } else if (!pressed && encButtonHeld) {
    encButtonHeld = false;
    unsigned long heldFor = millis() - encButtonPressTime;

    if (menuActive) {
      handleMenu();
      menuActive = false;
    } else if (heldFor >= 600) {
      menuActive = true;
    } else if (currentScreen == 3) {
      switch (calibIndex) {
        case 1: // Zero Pitch/Roll
          pitchOffset = pitch;
          rollOffset = roll;
          break;
        case 2: // Zero Heading
          heading = 0;
          break;
      }
      calibIndex = (calibIndex + 1) % calibSize;
    } else {
      menuActive = true;
    }
  }
}

// ================= MENU =================
void drawMenu() {
  canvas->fillScreen(RGB565_BLACK);

  int target = menuIndex * 30;
  menuOffset += (target - menuOffset) * 0.3;

  for (int i = 0; i < menuSize; i++) {
    int y = 50 + (i*30) - menuOffset;

    if (i == menuIndex)
      canvas->setTextColor(RGB565_YELLOW);
    else
      canvas->setTextColor(RGB565_WHITE);

    drawIcon(40, y+10, i);

    canvas->setCursor(70, y);
    canvas->print(menuItems[i]);
  }
}

void drawIcon(int x,int y,int type){
  switch(type){
    case 0: canvas->drawLine(x-5,y,x+5,y,RGB565_WHITE); break;
    case 1: canvas->drawCircle(x,y,4,RGB565_GREEN); break;
    case 2: canvas->drawCircle(x,y,5,RGB565_WHITE); break;
    case 3: canvas->drawFastHLine(x-5,y,10,RGB565_YELLOW); break;
  }
}

void handleMenu(){
  screenTransition();

  switch(menuIndex){
    case 0: currentScreen=0; break;
    case 1: currentScreen=1; break;
    case 2: currentScreen=2; break;
    case 3: currentScreen=3; calibIndex=0; break;
  }
}

// ================= TRANSITION =================
void screenTransition(){
  for(int i=0;i<800;i+=20){
    canvas->fillScreen(RGB565_BLACK);
    canvas->fillRect(i,0,800,480,RGB565_WHITE);
    canvas->flush();
    delay(5);
  }
}

// ================= HORIZON =================
void drawHorizon(){
  const int W=800, H=480;
  int cx=W/2, cy=H/2;

  float r=(roll-rollOffset)*DEG_TO_RAD;
  int p=(pitch-pitchOffset)*3;

  float maxAngle=85*DEG_TO_RAD;
  if(r>maxAngle) r=maxAngle;
  if(r<-maxAngle) r=-maxAngle;
  float slope=tan(r);

  int y1=cy+p+(int)((0-cx)*slope);
  int y2=cy+p+(int)((W-cx)*slope);

  for(int x=0;x<W;x++){
    int horizonY=y1+(int)((long)(y2-y1)*x/W);
    if(horizonY<0) horizonY=0;
    if(horizonY>H) horizonY=H;

    if(horizonY>0)
      canvas->drawFastVLine(x,0,horizonY,RGB565_BLUE);
    if(horizonY<H)
      canvas->drawFastVLine(x,horizonY,H-horizonY,COLOR_BROWN);
  }

  canvas->drawLine(0,y1,W-1,y2,RGB565_WHITE);

  // ---- pitch ladder ----
  const float pxPerDeg = 6.0;
  const int halfLen = 60;  // outer extent of each rung from center
  const int gap = 20;      // gap around center for aircraft symbol

  for(int pv=-30; pv<=30; pv+=10){
    if(pv==0) continue;

    int yAtCx = cy + p - (int)(pv * pxPerDeg);
    if(yAtCx < 0 || yAtCx > H) continue;

    int leftOuterY  = yAtCx + (int)((-halfLen) * slope);
    int leftInnerY  = yAtCx + (int)((-gap) * slope);
    int rightInnerY = yAtCx + (int)((gap) * slope);
    int rightOuterY = yAtCx + (int)((halfLen) * slope);

    canvas->drawLine(cx-halfLen, leftOuterY, cx-gap, leftInnerY, RGB565_WHITE);
    canvas->drawLine(cx+gap, rightInnerY, cx+halfLen, rightOuterY, RGB565_WHITE);

    canvas->setTextSize(4);
    canvas->setTextColor(RGB565_YELLOW);
    canvas->setCursor(cx-halfLen-55, leftOuterY-16);
    canvas->print(abs(pv));
    canvas->setCursor(cx+halfLen+6, rightOuterY-16);
    canvas->print(abs(pv));
  }

  // fixed aircraft reference symbol (boresight)
  canvas->drawFastHLine(cx-gap-10, cy, 20, RGB565_YELLOW);
  canvas->drawFastHLine(cx+gap-10, cy, 20, RGB565_YELLOW);
  canvas->drawFastHLine(cx-gap-10, cy+1, 20, RGB565_YELLOW);
  canvas->drawFastHLine(cx+gap-10, cy+1, 20, RGB565_YELLOW);
  canvas->drawFastVLine(cx, cy-gap-4, 8, RGB565_YELLOW);
  canvas->drawCircle(cx, cy, 10, RGB565_YELLOW);
  canvas->fillCircle(cx, cy, 3, RGB565_YELLOW);
}

// ================= ROLL TAPE (left, curved) =================
void drawRollTape(){
  const int top=0, bottom=480;     // tape vertical extent (full screen height)
  const int cy=(top+bottom)/2;     // vertical center
  const int halfH=(bottom-top)/2;
  const float scale=5.0;   // px per degree
  const int xEdge=10;      // x of curve at top/bottom of tape
  const int xMax=70;       // x of curve at vertical center (bulge)
  const int d=xMax-xEdge;

  // circle that the tick marks lie along: passes through (xEdge,top),
  // (xMax,cy) and (xEdge,bottom)
  const float R = (float)(d*d + halfH*halfH) / (2.0f*d);
  const float cxCircle = xMax - R;

  canvas->fillRect(0,top,170,bottom-top,RGB565_BLACK);

  // label box on top of tape
  canvas->fillRect(0,0,170,70,RGB565_BLACK);
  for(int i=0;i<4;i++) canvas->drawRoundRect(i,i,170-2*i,70-2*i,12,RGB565_WHITE);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(37,19);
  canvas->print("TILT");
  canvas->setCursor(38,19);
  canvas->print("TILT");

  float r = roll - rollOffset;

  for(int deg=-90; deg<=90; deg+=10){
    int y = cy - (int)((deg - r) * scale);
    if(y < top || y > bottom) continue;
    if(y < 75) continue; // under the label box

    int curveX = (int)(cxCircle + sqrt(R*R - (float)(y-cy)*(y-cy)));

    bool major = (deg % 20 == 0);
    int tickLen = major ? 20 : 10;
    canvas->drawFastHLine(curveX, y, tickLen, RGB565_WHITE);

    if(major){
      canvas->setTextSize(2);
      canvas->setTextColor(RGB565_YELLOW);
      canvas->setCursor(curveX+tickLen+4, y-7);
      canvas->print(deg);
    }
  }

  // fixed index pointer at vertical center, pointing left into the curve
  canvas->fillTriangle(xMax+20, cy, xMax+35, cy-8, xMax+35, cy+8, RGB565_YELLOW);

  // magnifying window over the pointer showing current value
  int rollVal = (int)round(r);
  canvas->fillRoundRect(52,cy-30,90,60,8,RGB565_BLACK);
  for(int i=0;i<3;i++) canvas->drawRoundRect(52+i,cy-30+i,90-2*i,60-2*i,8,RGB565_WHITE);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(61,cy-16);
  canvas->print(rollVal);
  canvas->setCursor(62,cy-16);
  canvas->print(rollVal);
}

// ================= ALTITUDE TAPE (right, curved) =================
void drawAltitudeTape(){
  const int top=0, bottom=480;     // tape vertical extent (full screen height)
  const int cy=(top+bottom)/2;     // vertical center
  const int halfH=(bottom-top)/2;
  const float scale=4.0;   // px per meter
  const int xEdge=790;     // x of curve at top/bottom of tape
  const int xMax=730;      // x of curve at vertical center (bulge)
  const int d=xEdge-xMax;

  // circle that the tick marks lie along: passes through (xEdge,top),
  // (xMax,cy) and (xEdge,bottom)
  const float R = (float)(d*d + halfH*halfH) / (2.0f*d);
  const float cxCircle = xMax + R;

  canvas->fillRect(630,top,170,bottom-top,RGB565_BLACK);

  // label box on top of tape
  canvas->fillRect(630,0,170,70,RGB565_BLACK);
  for(int i=0;i<4;i++) canvas->drawRoundRect(630+i,i,170-2*i,70-2*i,12,RGB565_WHITE);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(679,19);
  canvas->print("ALT");
  canvas->setCursor(680,19);
  canvas->print("ALT");

  int startM = ((int)altitude_m/10)*10 - 60;
  for(int m=startM; m<=startM+120; m+=10){
    int y = cy - (int)((m - altitude_m) * scale);
    if(y < top || y > bottom) continue;
    if(y < 75) continue; // under the label box

    int curveX = (int)(cxCircle - sqrt(R*R - (float)(y-cy)*(y-cy)));

    int tickLen = 20;
    canvas->drawFastHLine(curveX-tickLen, y, tickLen, RGB565_WHITE);

    canvas->setTextSize(2);
    canvas->setTextColor(RGB565_YELLOW);
    canvas->setCursor(curveX-tickLen-44, y-7);
    canvas->print(m);
  }

  // fixed index pointer at vertical center, pointing right into the curve
  canvas->fillTriangle(xMax-20, cy, xMax-35, cy-8, xMax-35, cy+8, RGB565_YELLOW);

  // magnifying window over the pointer showing current value
  int altVal = (int)round(altitude_m);
  canvas->fillRoundRect(657,cy-30,90,60,8,RGB565_BLACK);
  for(int i=0;i<3;i++) canvas->drawRoundRect(657+i,cy-30+i,90-2*i,60-2*i,8,RGB565_WHITE);
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(666,cy-16);
  canvas->print(altVal);
  canvas->setCursor(667,cy-16);
  canvas->print(altVal);
}

// ================= COMPASS ROSE (bottom center, overlay) =================
void drawHeadingTape(){
  const int cx=400, cy=240, R=180;

  // no background fill - overlays directly on the horizon for a transparent look
  canvas->drawCircle(cx,cy,R,RGB565_CYAN);
  canvas->drawCircle(cx,cy,R-1,RGB565_CYAN);
  canvas->drawCircle(cx,cy,R-2,RGB565_CYAN);

  // tick marks every 30 degrees, rotating with heading
  for(int b=0;b<360;b+=30){
    float ang=(b-heading)*DEG_TO_RAD;
    int x1=cx+(int)(sin(ang)*R);
    int y1=cy-(int)(cos(ang)*R);
    int x2=cx+(int)(sin(ang)*(R-12));
    int y2=cy-(int)(cos(ang)*(R-12));
    canvas->drawLine(x1,y1,x2,y2,RGB565_CYAN);
    canvas->drawLine(x1+1,y1,x2+1,y2,RGB565_CYAN);
    canvas->drawLine(x1,y1+1,x2,y2+1,RGB565_CYAN);
  }

  // cardinal labels, rotating with heading
  const char* dirs[4] = {"N","E","S","W"};
  const int bearings[4] = {0,90,180,270};
  for(int i=0;i<4;i++){
    float ang=(bearings[i]-heading)*DEG_TO_RAD;
    int x=cx+(int)(sin(ang)*(R-28));
    int y=cy-(int)(cos(ang)*(R-28));
    canvas->setTextSize(3);
    canvas->setTextColor(RGB565_YELLOW);
    canvas->setCursor(x-12,y-12);
    canvas->print(dirs[i]);
  }

  // fixed pointer at top showing current heading
  canvas->fillTriangle(cx,cy-R-32, cx-22,cy-R+4, cx+22,cy-R+4, RGB565_RED);
}

// ================= OVERLAY =================
void drawOverlay(){
  if(abs(roll)>35||abs(pitch)>35){
    canvas->setTextColor(RGB565_RED);
    canvas->setTextSize(4);
    canvas->setCursor(100,20);
    canvas->print("DANGER");
  }
}

// ================= SCREENS =================
void drawGPSScreen(){
  canvas->fillScreen(RGB565_BLACK);

  // Speed readout enlarged from size 3 -> size 4
  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(40,150);
  canvas->print(speed_kmh);
}

void drawInfoScreen(){
  canvas->fillScreen(RGB565_BLACK);

  // Numbers enlarged from size 2 -> size 3
  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_WHITE);

  canvas->setCursor(40,120);
  canvas->print("Pitch:");
  canvas->print(pitch,1);

  canvas->setCursor(40,170);
  canvas->print("Roll:");
  canvas->print(roll,1);

  canvas->setCursor(40,220);
  canvas->print("Head:");
  canvas->print(heading);
}

void drawCalibrateScreen(){
  canvas->fillScreen(RGB565_BLACK);

  canvas->setTextSize(4);
  canvas->setTextColor(RGB565_YELLOW);
  canvas->setCursor(40,40);
  canvas->print("CALIBRATE");

  canvas->setTextSize(3);
  for(int i=0; i<calibSize; i++){
    int y = 130 + i*60;
    bool sel = (i == calibIndex);

    canvas->setTextColor(sel ? RGB565_GREEN : RGB565_WHITE);
    canvas->setCursor(40, y);
    canvas->print(sel ? "> " : "  ");
    canvas->print(calibItems[i]);

    if(i == 0){
      canvas->setCursor(450, y);
      canvas->print(seaLevelPressure_hPa, 1);
    }
  }

  canvas->setTextSize(2);
  canvas->setTextColor(RGB565_WHITE);
  canvas->setCursor(40, 130 + calibSize*60 + 40);
  canvas->print("Rotate: adjust   Press: next   Hold: menu");
}

// ================= BOOT =================
void showBoot(){
  canvas->fillScreen(RGB565_BLACK);

  canvas->setTextSize(3);
  canvas->setTextColor(RGB565_GREEN);

  canvas->setCursor(60,150);
  canvas->print("WRANGLER");

  canvas->setCursor(40,200);
  canvas->print("SYSTEM");

  canvas->flush();

  delay(2000);
}
