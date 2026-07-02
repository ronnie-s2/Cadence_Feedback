#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// =========================
// SD CARD
// =========================
#define SD_CS 5
File logFile;
bool sdOK = false;

// =========================
// ADXL335 PINS
// =========================
const int X_PIN = 34;
const int Y_PIN = 35;
const int Z_PIN = 32;

// =========================
// MAX30102 REGISTERS
// =========================
#define MAX30102_ADDR     0x57
#define REG_FIFO_DATA     0x07
#define REG_MODE_CONFIG   0x09
#define REG_SPO2_CONFIG   0x0A
#define REG_LED1_PA       0x0C
#define REG_LED2_PA       0x0D
#define REG_FIFO_WR_PTR   0x04
#define REG_OVF_COUNTER   0x05
#define REG_FIFO_RD_PTR   0x06

// =========================
// Tunable parameters (unchanged)
// =========================
const int32_t  fingerThreshold     = 50000;
const float    minPeakHeight       = 300.0f;
const float    slopeThreshold      = 30.0f;
const uint32_t refractoryTime      = 333;
const float    outlierThreshold    = 25.0f;
const float    convergenceThreshold = 500.0f;
const uint32_t convergenceHoldMs   = 2000;

// =========================
// State (unchanged)
// =========================
bool     fingerPresent    = false;
bool     settled          = false;
bool     convergenceTimer = false;
uint32_t convergenceStart = 0;

const float alphaFast = 0.984f;
const float alphaSlow = 0.999f;
float alpha = alphaFast;

float dcLevel = 0;
float smoothed = 0;
float prevSmoothed = 0;
float derivative = 0;
float prevDerivative = 0;
bool rising = false;

uint32_t lastBeatTime = 0;
float beatAvg = 0;
int beatCount = 0;

uint32_t lastPrint = 0;

// =========================
// BPM storage for logging
// =========================
float lastBPM = 0;

// =========================
// I2C helpers (unchanged)
// =========================
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint32_t readIROnce() {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(REG_FIFO_DATA);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX30102_ADDR, 6);
  if (Wire.available() < 6) return 0;
  Wire.read(); Wire.read(); Wire.read();
  uint32_t ir =
      ((uint32_t)Wire.read() << 16) |
      ((uint32_t)Wire.read() << 8)  |
       Wire.read();
  return ir & 0x03FFFF;
}

uint32_t drainFIFO() {
  uint8_t wr = 0, rd = 0;

  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(REG_FIFO_WR_PTR);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX30102_ADDR, 2);
  wr = Wire.read();
  rd = Wire.read();

  int samples = (wr - rd + 32) % 32;
  if (samples == 0) return 0;

  uint32_t latest = 0;
  for (int i = 0; i < samples; i++) latest = readIROnce();
  return latest;
}

// =========================
// RESET
// =========================
void resetState() {
  settled = false;
  convergenceTimer = false;
  convergenceStart = 0;
  alpha = alphaFast;
  dcLevel = 0;
  smoothed = 0;
  prevSmoothed = 0;
  derivative = 0;
  prevDerivative = 0;
  rising = false;
  lastBeatTime = 0;
  beatAvg = 0;
  beatCount = 0;
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  analogReadResolution(12);

  // ---------- SD ----------
  SPI.begin(18, 19, 23, SD_CS);

  if (SD.begin(SD_CS)) {
    sdOK = true;
    logFile = SD.open("/run.csv", FILE_WRITE);
    if (logFile) {
      logFile.println("t,ir,dc,ac,bpm,ax,ay,az");
    }
  }

  // ---------- MAX30102 ----------
  writeReg(REG_MODE_CONFIG, 0x40);
  delay(100);
  writeReg(REG_FIFO_WR_PTR, 0x00);
  writeReg(REG_OVF_COUNTER, 0x00);
  writeReg(REG_FIFO_RD_PTR, 0x00);
  writeReg(REG_MODE_CONFIG, 0x03);
  writeReg(REG_SPO2_CONFIG, 0x27);
  writeReg(REG_LED1_PA, 0x2A);
  writeReg(REG_LED2_PA, 0x2A);

  Serial.println("System ready.");
}

// =========================
// PROCESS SAMPLE (your logic unchanged)
// =========================
void processSample(uint32_t irRaw, uint32_t now) {

  if (irRaw < fingerThreshold) {
    if (fingerPresent) {
      fingerPresent = false;
      resetState();
    }
    return;
  }

  if (!fingerPresent) {
    fingerPresent = true;
    resetState();
    dcLevel = irRaw;
    return;
  }

  dcLevel = alpha * dcLevel + (1 - alpha) * irRaw;
  float ac = irRaw - dcLevel;

  if (!settled) {
    smoothed = 0.8f * smoothed + 0.2f * ac;

    if (fabs(ac) < convergenceThreshold) {
      if (!convergenceTimer) {
        convergenceTimer = true;
        convergenceStart = now;
      } else if (now - convergenceStart > convergenceHoldMs) {
        settled = true;
        alpha = alphaSlow;
        smoothed = 0;
      }
    } else {
      convergenceTimer = false;
    }
    return;
  }

  smoothed = 0.8f * smoothed + 0.2f * ac;
  derivative = smoothed - prevSmoothed;

  if (derivative > slopeThreshold && smoothed > 0)
    rising = true;

  bool peak =
      rising &&
      derivative < 0 &&
      prevDerivative >= 0 &&
      smoothed > minPeakHeight &&
      (now - lastBeatTime > refractoryTime);

  if (peak) {
    rising = false;

    if (lastBeatTime > 0) {
      uint32_t interval = now - lastBeatTime;
      float bpm = 60000.0f / interval;

      if (beatCount > 3) lastBPM = beatAvg;

      beatAvg = (beatCount == 0)
        ? bpm
        : 0.75f * beatAvg + 0.25f * bpm;

      beatCount++;
    }

    lastBeatTime = now;
  }

  prevDerivative = derivative;
  prevSmoothed = smoothed;
}

// =========================
// LOOP
// =========================
void loop() {

  uint32_t now = millis();
  uint32_t ir = drainFIFO();

  if (ir == 0) {
    delay(5);
    return;
  }

  processSample(ir, now);

  int ax = analogRead(X_PIN);
  int ay = analogRead(Y_PIN);
  int az = analogRead(Z_PIN);

  float ac = ir - dcLevel;

  // ---------- LOG ----------
  if (sdOK && logFile) {
    logFile.printf("%lu,%lu,%.2f,%.2f,%.1f,%d,%d,%d\n",
      now,
      ir,
      dcLevel,
      ac,
      lastBPM,
      ax, ay, az
    );
  }

  // ---------- FLUSH ----------
  static uint32_t lastFlush = 0;
  if (now - lastFlush > 1000 && sdOK) {
    logFile.flush();
    lastFlush = now;
  }

  delay(8);
}