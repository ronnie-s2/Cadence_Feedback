#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

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
// Tunable parameters
// =========================
const int32_t  fingerThreshold      = 50000;
const float    minPeakHeight        = 300.0f;
const float    slopeThreshold       = 30.0f;
const uint32_t refractoryTime       = 333;
const float    outlierThreshold     = 25.0f;
const float    convergenceThreshold = 500.0f;
const uint32_t convergenceHoldMs    = 2000;

// =========================
// Finger / settling state
// =========================
bool     fingerPresent    = false;
bool     settled          = false;
bool     convergenceTimer = false;
uint32_t convergenceStart = 0;

// =========================
// Signal processing
// =========================
const float alphaFast = 0.984f;
const float alphaSlow = 0.999f;

float alpha = alphaFast;

float dcLevel        = 0;
float smoothed       = 0;
float prevSmoothed   = 0;
float derivative     = 0;
float prevDerivative = 0;

bool rising = false;

// =========================
// Beat / BPM state
// =========================
uint32_t lastBeatTime = 0;
float    beatAvg      = 0;
float    lastBPM      = 0;
int      beatCount    = 0;

// =========================
// Timing
// =========================
uint32_t lastPrint = 0;

// =========================
// I2C HELPERS
// =========================
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(MAX30102_ADDR, 1);

  return Wire.available() ? Wire.read() : 0;
}

uint32_t readIROnce() {

  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(REG_FIFO_DATA);
  Wire.endTransmission(false);

  Wire.requestFrom(MAX30102_ADDR, 6);

  if (Wire.available() < 6)
    return 0;

  // discard RED
  Wire.read();
  Wire.read();
  Wire.read();

  uint32_t ir =
      ((uint32_t)Wire.read() << 16) |
      ((uint32_t)Wire.read() << 8 ) |
       (uint32_t)Wire.read();

  return ir & 0x03FFFF;
}

uint32_t drainFIFO() {

  uint8_t wrPtr = readReg(REG_FIFO_WR_PTR);
  uint8_t rdPtr = readReg(REG_FIFO_RD_PTR);

  int samples = (wrPtr - rdPtr + 32) % 32;

  if (samples == 0)
    return 0;

  uint32_t latest = 0;

  for (int i = 0; i < samples; i++) {
    latest = readIROnce();
  }

  return latest;
}

// =========================
// RESET STATE
// =========================
void resetState() {

  settled          = false;
  convergenceTimer = false;
  convergenceStart = 0;

  alpha            = alphaFast;

  dcLevel          = 0;
  smoothed         = 0;
  prevSmoothed     = 0;
  derivative       = 0;
  prevDerivative   = 0;

  rising           = false;

  lastBeatTime     = 0;
  beatAvg          = 0;
  lastBPM          = 0;
  beatCount        = 0;
}

// =========================
// SETUP
// =========================
void setup() {

  Serial.begin(115200);

  delay(2000);

  Serial.println("Starting system...");

  // ---------- ADC ----------
  analogReadResolution(12);

  // ---------- I2C ----------
  Wire.begin(21, 22);

  // ---------- SPI / SD ----------
  SPI.begin(18, 19, 23, SD_CS);

  Serial.println("Initializing SD...");

  if (SD.begin(SD_CS)) {

    sdOK = true;

    Serial.println("SD initialization successful!");

    logFile = SD.open("/run.csv", FILE_WRITE);

    if (logFile) {

      logFile.println("t,ir,dc,ac,bpm,ax,ay,az");

      Serial.println("Opened run.csv");
    }
    else {
      Serial.println("Failed to open run.csv");
    }
  }
  else {
    Serial.println("SD initialization failed!");
  }

  // ---------- MAX30102 ----------
  writeReg(REG_MODE_CONFIG, 0x40);

  delay(100);

  writeReg(REG_FIFO_WR_PTR, 0x00);
  writeReg(REG_OVF_COUNTER, 0x00);
  writeReg(REG_FIFO_RD_PTR, 0x00);

  writeReg(REG_MODE_CONFIG, 0x03);

  // 100 SPS, 18-bit, 411us pulse
  writeReg(REG_SPO2_CONFIG, 0x27);

  // LED current
  writeReg(REG_LED1_PA, 0x2A);
  writeReg(REG_LED2_PA, 0x2A);

  Serial.println("MAX30102 ready.");
}

// =========================
// PROCESS SAMPLE
// =========================
void processSample(uint32_t irRaw, uint32_t now) {

  // ---------- Finger detection ----------
  if (irRaw < fingerThreshold) {

    if (fingerPresent) {

      fingerPresent = false;

      resetState();

      Serial.println("[STATE] Finger removed.");
    }

    return;
  }

  // ---------- Finger inserted ----------
  if (!fingerPresent) {

    fingerPresent = true;

    resetState();

    dcLevel = (float)irRaw;

    Serial.println("[STATE] Finger detected.");
    Serial.println("[STATE] Waiting for convergence...");

    return;
  }

  // ---------- DC removal ----------
  dcLevel = alpha * dcLevel +
            (1.0f - alpha) * (float)irRaw;

  float ac = (float)irRaw - dcLevel;

  // ---------- Convergence phase ----------
  if (!settled) {

    smoothed = 0.80f * smoothed +
               0.20f * ac;

    prevSmoothed = smoothed;

    if (fabsf(ac) < convergenceThreshold) {

      if (!convergenceTimer) {

        convergenceTimer = true;
        convergenceStart = now;
      }
      else if (now - convergenceStart >= convergenceHoldMs) {

        settled = true;

        alpha = alphaSlow;

        smoothed       = 0;
        prevSmoothed   = 0;
        prevDerivative = 0;
        rising         = false;
        lastBeatTime   = 0;

        Serial.println("[STATE] DC converged.");
        Serial.println("[STATE] Detecting beats.");
      }
    }
    else {

      convergenceTimer = false;
    }

    return;
  }

  // ---------- AC smoothing ----------
  smoothed =
      0.80f * smoothed +
      0.20f * ac;

  // ---------- Derivative ----------
  derivative = smoothed - prevSmoothed;

  // ---------- Rising edge ----------
  if (derivative > slopeThreshold &&
      smoothed > 0) {

    rising = true;
  }

  // ---------- Peak detection ----------
  bool peakDetected =
      rising &&
      derivative < 0 &&
      prevDerivative >= 0 &&
      smoothed > minPeakHeight &&
      (now - lastBeatTime > refractoryTime);

  if (peakDetected) {

    rising = false;

    if (lastBeatTime > 0) {

      uint32_t interval = now - lastBeatTime;

      if (interval > 333 &&
          interval < 2000) {

        float candidateBPM =
            60000.0f / interval;

        bool accept = true;

        // ---------- Outlier rejection ----------
        if (beatCount >= 4 &&
            fabsf(candidateBPM - beatAvg) >
            outlierThreshold) {

          accept = false;

          Serial.print("[REJECT] Outlier BPM: ");
          Serial.println(candidateBPM, 1);
        }

        if (accept) {

          beatAvg =
              (beatCount == 0)
              ? candidateBPM
              : 0.75f * beatAvg +
                0.25f * candidateBPM;

          lastBPM = beatAvg;

          beatCount++;
        }
      }
    }

    lastBeatTime = now;
  }

  // ---------- Update history ----------
  prevDerivative = derivative;
  prevSmoothed   = smoothed;
}

// =========================
// MAIN LOOP
// =========================
void loop() {

  uint32_t now = millis();

  // ---------- Read PPG ----------
  uint32_t irRaw = drainFIFO();

  if (irRaw == 0) {

    delay(5);

    return;
  }

  processSample(irRaw, now);

  // ---------- Read accelerometer ----------
  int ax = analogRead(X_PIN);
  int ay = analogRead(Y_PIN);
  int az = analogRead(Z_PIN);

  // ---------- Current AC ----------
  float ac = (float)irRaw - dcLevel;

  // =========================
  // SD LOGGING
  // =========================
  if (sdOK && logFile) {

    logFile.printf(
      "%lu,%lu,%.2f,%.2f,%.1f,%d,%d,%d\n",
      now,
      irRaw,
      dcLevel,
      ac,
      lastBPM,
      ax,
      ay,
      az
    );
  }

  // =========================
  // PERIODIC FLUSH
  // =========================
  static uint32_t lastFlush = 0;

  if (now - lastFlush > 1000 &&
      sdOK &&
      logFile) {

    logFile.flush();

    lastFlush = now;

    Serial.println("[SD] Flushed to card");
  }

  // =========================
  // STATUS PRINTS
  // =========================
  if (now - lastPrint >= 1000) {

    lastPrint = now;

    if (!fingerPresent) {

      Serial.println("[STATE] No finger detected");
    }
    else if (!settled) {

      Serial.print("[STATE] Converging... dc=");
      Serial.println(dcLevel, 0);
    }
    else if (beatCount < 4) {

      Serial.print("[STATE] Acquiring beats=");
      Serial.println(beatCount);
    }
    else {

      Serial.print("[BPM] ");
      Serial.println(beatAvg, 1);
    }

    // ---------- Signal info ----------
    Serial.print("IR=");
    Serial.print(irRaw);

    Serial.print(" AC=");
    Serial.print(ac, 1);

    Serial.print(" BPM=");
    Serial.print(lastBPM, 1);

    Serial.print(" AX=");
    Serial.print(ax);

    Serial.print(" AY=");
    Serial.print(ay);

    Serial.print(" AZ=");
    Serial.println(az);
  }

  // =========================
  // Optional waveform stream
  // Uncomment for Serial Plotter
  // =========================

  /*
  static int waveformCounter = 0;

  waveformCounter++;

  if (waveformCounter >= 20) {

    waveformCounter = 0;

    Serial.println(ac);
  }
  */

  delay(8);
}