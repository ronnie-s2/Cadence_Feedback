// =========================================================
// PPG TEST BUILD
// Only the PPG subsystem is active.
// All other subsystems are stubbed to compile cleanly.
// Remove this file once PPG is validated; merge into main.ino
// =========================================================

// =========================================================
// LIBRARIES
// =========================================================

#include <Wire.h>

// Stubbed — not needed for PPG-only test
// #include <SPI.h>
// #include <SD.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>

// =========================================================
// PIN DEFINITIONS
// =========================================================

// ---------- I2C ----------
const int SDA_PIN  = 16;
const int SCL_PIN  = 17;

// ---------- SPI (unused in this build) ----------
// const int SD_CS    = 5;
// const int SPI_SCK  = 18;
// const int SPI_MISO = 19;
// const int SPI_MOSI = 23;

// ---------- Encoder (unused in this build) ----------
// const int ENC_CLK  = 25;
// const int ENC_DT   = 26;
// const int ENC_SW   = 27;

// ---------- Accelerometer (unused in this build) ----------
// const int ACCEL_X  = 34;
// const int ACCEL_Y  = 35;
// const int ACCEL_Z  = 32;

// ---------- Misc ----------
const int LED_PIN  = 2;

// =========================================================
// MAX30102 REGISTER MAP
// =========================================================

#define MAX30102_ADDR   0x57
#define REG_FIFO_DATA   0x07
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA     0x0C
#define REG_LED2_PA     0x0D
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06

// =========================================================
// PPG TUNABLE PARAMETERS
// =========================================================

const int32_t  fingerThreshold     = 50000;
const float    minPeakHeight       = 300.0f;
const float    slopeThreshold      = 30.0f;
const uint32_t refractoryTime      = 333;
const float    outlierThreshold    = 25.0f;
const float    convergenceThreshold = 500.0f;
const uint32_t convergenceHoldMs   = 2000;

// =========================================================
// SHARED SENSOR DATA
// =========================================================

float    currentBPM    = 0.0f;
bool     fingerPresent = false;
bool     signalSettled = false;

// =========================================================
// PPG INTERNAL STATE
// =========================================================

const float alphaFast  = 0.984f;
const float alphaSlow  = 0.999f;
float       ppgAlpha   = alphaFast;
float       dcLevel    = 0.0f;

float smoothed         = 0.0f;
float prevSmoothed     = 0.0f;
float derivative       = 0.0f;
float prevDerivative   = 0.0f;
bool  rising           = false;

bool     convergenceTimer = false;
uint32_t convergenceStart = 0;

uint32_t lastBeatTime  = 0;
float    beatAvg       = 0.0f;
int      beatCount     = 0;

// =========================================================
// TIMING
// =========================================================

uint32_t now;

const uint32_t PPG_INTERVAL = 8;   // ms (~125 Hz)
uint32_t lastPPGUpdate      = 0;
uint32_t lastPPGPrint       = 0;

// =========================================================
// FORWARD DECLARATIONS
// =========================================================

void     writeReg(uint8_t reg, uint8_t val);
uint8_t  readReg(uint8_t reg);
uint32_t readIROnce();
uint32_t drainFIFO();
void     processPPGSample(uint32_t irRaw, uint32_t now);
void     resetPPGState();

// =========================================================
// SETUP
// =========================================================

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);
    setupPPG();
}

// =========================================================
// LOOP
// =========================================================

void loop() {
    now = millis();
    updatePPG(now);
}

// =========================================================
// PPG SETUP
// =========================================================

void setupPPG() {
    delay(1000);

    writeReg(REG_MODE_CONFIG, 0x40);  // soft reset
    delay(100);

    writeReg(REG_FIFO_WR_PTR,  0x00);
    writeReg(REG_OVF_COUNTER,  0x00);
    writeReg(REG_FIFO_RD_PTR,  0x00);
    writeReg(REG_MODE_CONFIG,  0x03);  // SpO2 mode
    writeReg(REG_SPO2_CONFIG,  0x27);  // 100 SPS, 18-bit, 411 µs pulse
    writeReg(REG_LED1_PA,      0x2A);  // ~8 mA
    writeReg(REG_LED2_PA,      0x2A);

    Serial.println("MAX30102 ready.");
}

// =========================================================
// PPG UPDATE  (called every loop, self-throttled to 8 ms)
// =========================================================

void updatePPG(uint32_t now) {
    if (now - lastPPGUpdate < PPG_INTERVAL) return;
    lastPPGUpdate = now;

    uint32_t irRaw = drainFIFO();
    if (irRaw == 0) return;

    processPPGSample(irRaw, now);

    // 1 Hz serial debug
    if (now - lastPPGPrint >= 1000) {
        lastPPGPrint = now;
        if (!fingerPresent) {
            Serial.println("No finger detected.");
        } else if (!signalSettled) {
            Serial.print("Converging... dc=");
            Serial.println(dcLevel, 0);
        } else if (beatCount < 4) {
            Serial.print("Acquiring... beats=");
            Serial.println(beatCount);
        } else {
            Serial.print("BPM: ");
            Serial.println(currentBPM, 1);
        }
    }
}

// =========================================================
// PPG HELPERS — I2C
// =========================================================

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
    Wire.read(); Wire.read(); Wire.read();  // discard Red
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

// =========================================================
// PPG HELPERS — signal processing
// =========================================================

void processPPGSample(uint32_t irRaw, uint32_t now) {

    // ---- Finger detection ----
    if (irRaw < (uint32_t)fingerThreshold) {
        if (fingerPresent) {
            fingerPresent = false;
            resetPPGState();
            Serial.println("Finger removed.");
        }
        return;
    }

    if (!fingerPresent) {
        fingerPresent = true;
        resetPPGState();
        dcLevel = (float)irRaw;
        Serial.println("Finger detected — waiting for DC convergence...");
        return;
    }

    // ---- DC removal ----
    dcLevel  = ppgAlpha * dcLevel + (1.0f - ppgAlpha) * (float)irRaw;
    float ac = (float)irRaw - dcLevel;

    // ---- Convergence / settling ----
    if (!signalSettled) {
        smoothed     = 0.80f * smoothed + 0.20f * ac;
        prevSmoothed = smoothed;

        if (fabsf(ac) < convergenceThreshold) {
            if (!convergenceTimer) {
                convergenceTimer = true;
                convergenceStart = now;
            } else if (now - convergenceStart >= convergenceHoldMs) {
                signalSettled  = true;
                ppgAlpha       = alphaSlow;
                smoothed       = 0.0f;
                prevSmoothed   = 0.0f;
                prevDerivative = 0.0f;
                rising         = false;
                lastBeatTime   = 0;
                Serial.println("DC converged — detecting beats.");
            }
        } else {
            convergenceTimer = false;
        }
        return;
    }

    // ---- AC smoothing ----
    smoothed   = 0.80f * smoothed + 0.20f * ac;

    // ---- Derivative ----
    derivative = smoothed - prevSmoothed;

    // ---- Arm rising edge ----
    if (derivative > slopeThreshold && smoothed > 0) rising = true;

    // ---- Peak detection ----
    bool peakDetected =
        rising                         &&
        derivative     <  0            &&
        prevDerivative >= 0            &&
        smoothed       > minPeakHeight &&
        (now - lastBeatTime > refractoryTime);

    if (peakDetected) {
        rising = false;

        if (lastBeatTime > 0) {
            uint32_t interval = now - lastBeatTime;

            if (interval > 333 && interval < 2000) {
                float candidateBPM = 60000.0f / interval;
                bool  accept       = true;

                if (beatCount >= 4 && fabsf(candidateBPM - beatAvg) > outlierThreshold) {
                    accept = false;
                    Serial.print("  [outlier rejected: ");
                    Serial.print(candidateBPM, 1);
                    Serial.println(" BPM]");
                }

                if (accept) {
                    beatAvg    = (beatCount == 0)
                                 ? candidateBPM
                                 : 0.75f * beatAvg + 0.25f * candidateBPM;
                    beatCount++;
                    currentBPM = beatAvg;
                }
            }
        }
        lastBeatTime = now;
    }

    // ---- Update history ----
    prevDerivative = derivative;
    prevSmoothed   = smoothed;
}

void resetPPGState() {
    signalSettled    = false;
    convergenceTimer = false;
    convergenceStart = 0;
    ppgAlpha         = alphaFast;
    dcLevel          = 0.0f;
    smoothed         = 0.0f;
    prevSmoothed     = 0.0f;
    derivative       = 0.0f;
    prevDerivative   = 0.0f;
    rising           = false;
    lastBeatTime     = 0;
    beatAvg          = 0.0f;
    beatCount        = 0;
    currentBPM       = 0.0f;
}
