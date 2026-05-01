#include <Wire.h>

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
const int32_t  fingerThreshold     = 50000;
const float    minPeakHeight       = 300.0f;  // smoothed AC must exceed this at peak
const float    slopeThreshold      = 30.0f;   // min derivative to arm rising flag
                                               // raise if false positives, lower if missing beats
const uint32_t refractoryTime      = 333;     // ms — max ~180 BPM
const float    outlierThreshold    = 25.0f;   // BPM deviation to reject
const float    convergenceThreshold = 500.0f; // |ac| must stay below this to be "settled"
const uint32_t convergenceHoldMs   = 2000;    // ms of stability required before detecting

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
const float alphaFast = 0.984f;  // fast DC tracking during convergence (τ ≈ 1s @ 125Hz)
const float alphaSlow = 0.999f;  // slow DC tracking during detection  (τ ≈ 8s @ 125Hz)
float       alpha     = alphaFast;

float dcLevel        = 0;
float smoothed       = 0;
float prevSmoothed   = 0;
float derivative     = 0;
float prevDerivative = 0;
bool  rising         = false;

// =========================
// Beat / BPM state
// =========================
uint32_t lastBeatTime = 0;
float    beatAvg      = 0;
int      beatCount    = 0;

// =========================
// Timing
// =========================
uint32_t lastPrint = 0;

// =========================
// I2C helpers
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
  if (Wire.available() < 6) return 0;
  Wire.read(); Wire.read(); Wire.read();  // discard Red channel
  uint32_t ir =
      ((uint32_t)Wire.read() << 16) |
      ((uint32_t)Wire.read() << 8)  |
       Wire.read();
  return ir & 0x03FFFF;
}

uint32_t drainFIFO() {
  uint8_t wrPtr   = readReg(REG_FIFO_WR_PTR);
  uint8_t rdPtr   = readReg(REG_FIFO_RD_PTR);
  int     samples = (wrPtr - rdPtr + 32) % 32;
  if (samples == 0) return 0;
  uint32_t latest = 0;
  for (int i = 0; i < samples; i++) latest = readIROnce();
  return latest;
}

// =========================
// Full state reset
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
  beatCount        = 0;
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  delay(1000);

  writeReg(REG_MODE_CONFIG, 0x40);  // reset chip
  delay(100);
  writeReg(REG_FIFO_WR_PTR,  0x00);
  writeReg(REG_OVF_COUNTER,  0x00);
  writeReg(REG_FIFO_RD_PTR,  0x00);
  writeReg(REG_MODE_CONFIG,  0x03); // SpO2 mode
  writeReg(REG_SPO2_CONFIG,  0x27); // 100 SPS, 18-bit, 411us pulse
  writeReg(REG_LED1_PA,      0x2A); // ~8mA — increase to 0x47 if signal weak
  writeReg(REG_LED2_PA,      0x2A);

  Serial.println("MAX30102 HR monitor ready.");
}

// =========================
// Sample processing
// =========================
void processSample(uint32_t irRaw, uint32_t now) {

  // ---- Finger detection ----
  if (irRaw < fingerThreshold) {
    if (fingerPresent) {
      fingerPresent = false;
      resetState();
      Serial.println("Finger removed.");
    }
    return;
  }

  if (!fingerPresent) {
    fingerPresent = true;
    resetState();
    dcLevel = (float)irRaw;  // seed DC to real value — minimises initial transient
    Serial.println("Finger detected — waiting for DC convergence...");
    return;
  }

  // ---- DC removal ----
  dcLevel      = alpha * dcLevel + (1.0f - alpha) * (float)irRaw;
  float ac     = (float)irRaw - dcLevel;

  // ---- Convergence detection (settling phase) ----
  if (!settled) {
    smoothed     = 0.80f * smoothed + 0.20f * ac;
    prevSmoothed = smoothed;  // keep primed but don't generate derivatives yet

    if (fabsf(ac) < convergenceThreshold) {
      if (!convergenceTimer) {
        convergenceTimer = true;
        convergenceStart = now;
      } else if (now - convergenceStart >= convergenceHoldMs) {
        settled        = true;
        alpha          = alphaSlow;
        smoothed       = 0;     // clean slate for detection
        prevSmoothed   = 0;
        prevDerivative = 0;
        rising         = false;
        lastBeatTime   = 0;
        Serial.println("DC converged — detecting beats.");
      }
    } else {
      convergenceTimer = false;  // ac spiked (finger moved) — restart hold
    }
    return;
  }

  // ---- AC smoothing ----
  smoothed   = 0.80f * smoothed + 0.20f * ac;

  // ---- Derivative — computed from previous smoothed value ----
  derivative = smoothed - prevSmoothed;

  // ---- Rising edge: only arm when slope is meaningfully positive ----
  // Also require smoothed > 0 so we don't arm on the descending baseline
  if (derivative > slopeThreshold && smoothed > 0) {
    rising = true;
  }

  // ---- Peak detection: zero-crossing of derivative while armed ----
  // prevDerivative here is genuinely one sample old (updated after this block)
  bool peakDetected =
      rising                  &&
      derivative    <  0      &&
      prevDerivative >= 0     &&
      smoothed      > minPeakHeight &&
      (now - lastBeatTime > refractoryTime);

  if (peakDetected) {
    rising = false;  // disarm until next meaningful upslope

    if (lastBeatTime > 0) {
      uint32_t interval = now - lastBeatTime;

      if (interval > 333 && interval < 2000) {
        float candidateBPM = 60000.0f / interval;
        bool  accept       = true;

        // Reject outliers once average is established
        if (beatCount >= 4 && fabsf(candidateBPM - beatAvg) > outlierThreshold) {
          accept = false;
          Serial.print("  [outlier rejected: ");
          Serial.print(candidateBPM, 1);
          Serial.println(" BPM]");
        }

        if (accept) {
          beatAvg = (beatCount == 0)
                    ? candidateBPM
                    : 0.75f * beatAvg + 0.25f * candidateBPM;
          beatCount++;
        }
      }
    }
    lastBeatTime = now;
  }

  // ---- Update history AFTER peak check so prevDerivative is truly t-1 ----
  prevDerivative = derivative;
  prevSmoothed   = smoothed;
}

// =========================
// Main loop
// =========================
void loop() {
  uint32_t now   = millis();
  uint32_t irRaw = drainFIFO();
  if (irRaw == 0) { delay(5); return; }

  processSample(irRaw, now);

  if (now - lastPrint >= 1000) {
    lastPrint = now;

    if (!fingerPresent) {
      Serial.println("No finger detected.");
    } else if (!settled) {
      Serial.print("Converging... dc=");
      Serial.println(dcLevel, 0);
    } else if (beatCount < 4) {
      Serial.print("Acquiring... beats=");
      Serial.println(beatCount);
    } else {
      Serial.print("BPM: ");
      Serial.println(beatAvg, 1);
    }
  }

  delay(8);
}