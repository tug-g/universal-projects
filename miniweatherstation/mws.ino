#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_HMC5883_U.h>
#include <ArduinoBLE.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>

#define SHT31_ADDR 0x44

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define OLED_ADDR 0x3C
#define SD_CS 10

// ble
BLEService envService("180A");
BLECharacteristic displayChar(
  "2A56",
  BLERead | BLENotify,
  120
);

RTC_DS3231 rtc;
// display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BMP280 bmp;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
const unsigned long DISPLAY_INTERVAL = 1000; // 1 second
unsigned long lastDisplayTime = 0;

// hall effect sensor
int hallSensorPin = 2;

// wind speed
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
const int magnets = 1;
const float radius = 0.175;      // meters (axis -> cup center)
const float calibration = 7.81;   // anemometer factor = (known wind speed / pulse frequency)   [ k = v / f ] 
const unsigned long POST_INTERVAL = 5000; // 5 seconds
unsigned long lastPostTime = 0;

// wind direction
float declinationAngle = 0.0;
float directionOffset = 0.0;

// magnetometer initial offsets
float magOffsetX = 0.0;
float magOffsetY = 0.0;
float magScaleX = 1.0;
float magScaleY = 1.0;

// north
bool calibratingNorth = true;
unsigned long northCalStart = 0;
float headingSum = 0.0;
int headingSamples = 0;

// nan
float tSHT = NAN, h = NAN;
float tBMP = NAN, p = NAN;
float headingDegrees = NAN;
float wind_ms = NAN;
float wind_kmh = NAN;

void hallISR() {
  unsigned long now = millis();
  if (now - lastPulseTime > 5) {
    pulseCount++;
    lastPulseTime = now;
  }
}


String directionName(float deg) {
  if (deg >= 337.5 || deg < 22.5) return "N";
  if (deg < 67.5)  return "NW";
  if (deg < 112.5) return "W";
  if (deg < 157.5) return "SW";
  if (deg < 202.5) return "S";
  if (deg < 247.5) return "SE";
  if (deg < 292.5) return "E";
  return "NE";
}

// initialization functions
void initCore() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  pinMode(hallSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallSensorPin), hallISR, FALLING);
}

void initDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED not found");
    while (1);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Sensors Ready");
  display.display();
}

void initSD() {
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI.begin();
  delay(100);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card failed");
  }
}

void initSensors() {
  if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
    Serial.println("BMP280 not found");
    while (1);
  }

  if (!mag.begin()) {
    Serial.println("HMC5883L not found");
    while (1);
  }
  
  if (!rtc.begin()) {
    Serial.println("RTC failed");
    while (1);
  }
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));   // first time setup
}

void initBLE() {
  if (!BLE.begin()) {
    Serial.println("BLE failed");
    while (1);
  }

  BLE.setLocalName("Nano33BLE-Sense");
  BLE.setDeviceName("Nano33BLE-Sense");
  BLE.setAdvertisedService(envService);

  envService.addCharacteristic(displayChar);
  BLE.addService(envService);
  BLE.advertise();
}

void startNorthCalibration() {
  northCalStart = millis();
  calibratingNorth = true;
  Serial.println("System ready");
  Serial.println("North calibration started (6s)");
}

// loop functions
void readSensors() {
  Wire.beginTransmission(SHT31_ADDR);
  Wire.write(0x24);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(15);

  Wire.requestFrom(SHT31_ADDR, 6);
  if (Wire.available() == 6) {
    uint16_t rawT = (Wire.read() << 8) | Wire.read();
    Wire.read();
    uint16_t rawH = (Wire.read() << 8) | Wire.read();
    Wire.read();

    tSHT = -45 + (175 * (rawT / 65535.0));
    h = 100 * (rawH / 65535.0);
  }

  tBMP = bmp.readTemperature();
  p = bmp.readPressure() / 100.0;  // hPa
}

void computeWind() {
  sensors_event_t event;
  mag.getEvent(&event);

  float mx = -(event.magnetic.y - magOffsetY) * magScaleY;
  float my = (event.magnetic.x - magOffsetX) * magScaleX;

  if (abs(mx) > 0.01 || abs(my) > 0.01) {
    float heading = atan2(my, mx);
    heading += declinationAngle;

    if (heading < 0) heading += 2 * PI;
    if (heading >= 2 * PI) heading -= 2 * PI;

    float rawHeadingDeg = heading * 180.0 / PI;

    if (calibratingNorth) {
      headingSum += rawHeadingDeg;
      headingSamples++;

      if (millis() - northCalStart >= 6000) {
        float avgHeading = headingSum / headingSamples;
        directionOffset = 360.0 - avgHeading;
        if (directionOffset >= 360) directionOffset -= 360;

        calibratingNorth = false;

        Serial.print("North calibrated. Offset = ");
        Serial.println(directionOffset, 1);
      }
    }

    headingDegrees = rawHeadingDeg + directionOffset;
    if (headingDegrees >= 360) headingDegrees -= 360;
  }
}

void computeWindSpeed() {
  noInterrupts();
  unsigned long pulses = pulseCount;
  pulseCount = 0;
  interrupts();

  float frequency = pulses / (5.0 * magnets);

  wind_ms = frequency * calibration;
  wind_kmh = wind_ms * 3.6;

  if (pulses == 0) {
    wind_ms = 0.0;
    wind_kmh = 0.0;
  }
}


void printSerial() {
  DateTime now = rtc.now();
  Serial.print("Time: ");
  Serial.print(now.year()); Serial.print("-");
  Serial.print(now.month()); Serial.print("-");
  Serial.print(now.day()); Serial.print(" ");
  Serial.print(now.hour()); Serial.print(":");
  Serial.print(now.minute()); Serial.print(":");
  Serial.println(now.second());
  Serial.print("SHT31 Temp: "); Serial.println(tSHT, 2);
  Serial.print("Humidity: "); Serial.println(h, 1);
  Serial.print("BMP280 Temp: "); Serial.println(tBMP, 2);
  Serial.print("Pressure: "); Serial.println(p, 1);
  Serial.print("Wind Dir: "); Serial.print(headingDegrees, 1);
  Serial.print(" ("); Serial.print(directionName(headingDegrees)); Serial.println(")");
  Serial.print("Wind Speed: "); Serial.println(wind_ms, 2);
}

void logSD() {
  DateTime now = rtc.now();

  File logFile = SD.open("weather.csv", FILE_WRITE);
  if (logFile) {
    logFile.print(now.year()); logFile.print("-");
    logFile.print(now.month()); logFile.print("-");
    logFile.print(now.day()); logFile.print(",");

    logFile.print(now.hour()); logFile.print(":");
    logFile.print(now.minute()); logFile.print(":");
    logFile.print(now.second()); logFile.print(",");

    logFile.print(tSHT, 1); logFile.print(",");
    logFile.print(h, 1); logFile.print(",");
    logFile.print(tBMP, 1); logFile.print(",");
    logFile.print(p, 1); logFile.print(",");
    logFile.print(headingDegrees, 1); logFile.print(",");
    logFile.println(wind_ms, 2);

    logFile.close();
  }
}


void updateBLE() {
  BLEDevice central = BLE.central();
  if (central && central.connected()) {

    float wd = isnan(headingDegrees) ? 0.0 : headingDegrees;
    float ws = isnan(wind_ms) ? 0.0 : wind_ms;

    char buf[120];
    sprintf(buf, "T=%.2f,H=%.1f,P=%.1f,WD=%.0f,WS=%.2f",
            tSHT, h, p, wd, ws);

    displayChar.writeValue(buf);
  }
}


void getRTCTime(char *buf) {
  DateTime now = rtc.now();
  sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
          now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());
}

void updateDisplay() {
  char timeBuf[20];
  getRTCTime(timeBuf);

  display.clearDisplay();

  display.setCursor(0, 0);
  display.print(timeBuf);

  const int col1 = 0;
  const int col2 = 64;

  display.setCursor(col1, 8);
  display.print("T1:"); display.print(tSHT, 1);

  display.setCursor(col2, 8);
  display.print("T2:"); display.print(tBMP, 1);

  display.setCursor(col1, 16);
  display.print("H:"); display.print((int)h); display.print("%");

  display.setCursor(col2, 16);
  display.print("P:"); display.print((int)p); display.print("hPa");

  display.setCursor(col1, 23);
  display.print(headingDegrees, 0);
  display.print((char)247);
  display.print(" ");
  display.print(directionName(headingDegrees));

  display.setCursor(col2, 23);
  display.print(wind_ms, 2);
  display.print("m/s");

  display.display();
}

// main
void setup() {
  initCore();
  initDisplay();
  initSD();
  initSensors();
  initBLE();
  startNorthCalibration();
}
void loop() {
  readSensors();
  computeWind();

  unsigned long now = millis();

  if (now - lastPostTime >= POST_INTERVAL) {
    lastPostTime = now;

    computeWindSpeed();
    printSerial();
    logSD();
    updateBLE();
  }

  if (now - lastDisplayTime >= DISPLAY_INTERVAL) {
    lastDisplayTime = now;
    updateDisplay();
  }
}
