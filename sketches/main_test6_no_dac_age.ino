// =========================================================
// LIBRARIES
// =========================================================

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"

// =========================================================
// PIN DEFINITIONS
// =========================================================

// ---------- I2C bus 0 — MAX30102 ----------
const int SDA_PIN  = 16;
const int SCL_PIN  = 17;

// ---------- I2C bus 1 — OLED ----------
const int OLED_SDA = 21;
const int OLED_SCL = 13;

// ---------- SPI ----------
const int SD_CS    = 5;
const int SPI_SCK  = 18;
const int SPI_MISO = 19;
const int SPI_MOSI = 23;

// ---------- Encoder ----------
const int ENC_CLK  = 25;
const int ENC_DT   = 26;
const int ENC_SW   = 27;

// ---------- Accelerometer ----------
const int ACCEL_X  = 34;
const int ACCEL_Y  = 35;
const int ACCEL_Z  = 32;

// ---------- Misc ----------
const int LED_PIN  = 2;

// =========================================================
// OLED — separate TwoWire instance on bus 1
// =========================================================

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_ADDR    0x3C

TwoWire I2C_OLED = TwoWire(1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);

// =========================================================
// PPG — SparkFun MAX30105 library (init + FIFO only)
// =========================================================

MAX30105 particleSensor;

// =========================================================
// PPG SIGNAL PROCESSING STATE
// =========================================================

// DC tracking
// alphaFast: EMA coefficient during convergence. Lower = faster DC tracking.
// 0.984 ≈ τ 1s, 0.96 ≈ τ 0.3s @ 125Hz. Use lower value for faster settling
// across different users. Trade-off: too low strips the AC pulse signal.
const float alphaFast = 0.960f;
// alphaSlow: EMA during beat detection. Higher = more stable DC baseline.
// 0.998 gives slight DC adaptability to handle slow finger drift.
const float alphaSlow = 0.998f;
float       ppgAlpha  = alphaFast;
float       dcLevel   = 0.0f;

// AC / smoothing
float smoothed       = 0.0f;
float prevSmoothed   = 0.0f;
float derivative     = 0.0f;
float prevDerivative = 0.0f;
bool  rising         = false;

// Convergence
bool     convergenceTimer = false;
uint32_t convergenceStart = 0;

// Beat tracking
uint32_t lastBeatTime = 0;
float    beatAvg      = 0.0f;
int      beatCount    = 0;

// ---------- Tunable parameters ----------

// --- Finger detection ---
const int32_t fingerThreshold       = 40000;
const int32_t fingerRemoveThreshold = 30000;

// motionCeiling: if |smoothed| exceeds this, sample is a motion artifact.
const float   motionCeiling         = 1100.0f;

// --- Beat detection ---
const uint32_t refractoryPeriod     = 500;
const float    minPeakHeight        = 400.0f;
const float    slopeThreshold       = 30.0f;
const float    outlierThreshold     = 40.0f;

// --- Convergence ---
const float    convergenceThreshold = 2000.0f;
const uint32_t convergenceHoldMs    = 500;

// =========================================================
// APPLICATION STATE
// =========================================================

enum SetupState {
    SET_AGE,        // new first step
    SET_HR1,
    SET_DUR1,
    SET_HR2,
    SET_DUR2,
    SET_CONFIRMED
};

enum WorkoutPhase {
    PHASE_IDLE,
    PHASE_1,
    PHASE_2,
    PHASE_DONE
};

SetupState   setupState   = SET_HR1;
WorkoutPhase workoutPhase = PHASE_IDLE;

// =========================================================
// WORKOUT PARAMETERS
// =========================================================

int userAge   = 30;   // set during setup wizard
int maxHR     = 190;  // computed as 220 - userAge
int targetHR1 = 60;
int targetHR2 = 60;
int duration1 = 90;
int duration2 = 90;

// =========================================================
// SHARED SENSOR DATA
// =========================================================

float    currentBPM    = 0.0f;
uint32_t lastIR        = 0;
bool     fingerPresent = false;
bool     signalSettled = false;

int accelX = 0;
int accelY = 0;
int accelZ = 0;

// =========================================================
// ENCODER / BUTTON STATE
// =========================================================

int  lastCLKState    = HIGH;
bool lastButtonState = HIGH;

uint32_t lastButtonTime = 0;
uint32_t lastMoveTime   = 0;

const uint32_t BUTTON_DEBOUNCE_MS  = 250;
const uint32_t ENCODER_DEBOUNCE_MS = 5;

// =========================================================
// DISPLAY STATE
// =========================================================

bool     showColon    = true;
uint32_t lastBlink    = 0;
bool     displayDirty = true;

// =========================================================
// WORKOUT TIMING
// =========================================================

uint32_t phaseStartMillis = 0;
uint32_t phaseDurationMs  = 0;

// =========================================================
// SD
// =========================================================

File logFile;
bool sdAvailable = false;

// =========================================================
// TASK TIMING
// =========================================================

uint32_t now;

const uint32_t PPG_INTERVAL    =    8;
const uint32_t ACCEL_INTERVAL  =   20;
const uint32_t OLED_INTERVAL   =  100;
const uint32_t BLINK_INTERVAL  =  500;
const uint32_t LOGGER_INTERVAL = 1000;
const uint32_t FLUSH_INTERVAL  = 5000;

uint32_t lastPPGUpdate    = 0;
uint32_t lastAccelUpdate  = 0;
uint32_t lastOLEDUpdate   = 0;
uint32_t lastLoggerUpdate = 0;
uint32_t lastFlushUpdate  = 0;
uint32_t lastPPGPrint     = 0;

// =========================================================
// FORWARD DECLARATIONS
// =========================================================

void drawSetupScreen();
const char* hrZoneLabel(int bpm, int maxhr);
void drawWorkoutScreen();
void drawCompleteScreen();
void startWorkout();
void resetApplication();
int  getRemainingTime();
void writeCSVRow();
void resetPPGState();

// =========================================================
// SETUP
// =========================================================

void setup() {
    setupSerial();
    setupPins();
    setupI2C();
    setupSPI();
    setupDisplay();
    setupSD();
    setupPPG();
    setupAccelerometer();
    setupEncoder();
    setupApplication();
    drawInitialScreen();
}

// =========================================================
// LOOP
// =========================================================

void loop() {

    now = millis();

    // HIGH PRIORITY
    updatePPG(now);
    updateEncoder(now);

    // MEDIUM PRIORITY
    updateWorkout(now);
    updateAccelerometer(now);

    // LOW PRIORITY
    updateDisplay(now);
    updateLogger(now);
    flushSD(now);
}

// =========================================================
// SETUP FUNCTIONS
// =========================================================

void setupSerial() {
    Serial.begin(115200);
    Serial.println("[BOOT] Serial OK");
}

void setupPins() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    pinMode(ENC_CLK, INPUT_PULLUP);
    pinMode(ENC_DT,  INPUT_PULLUP);
    pinMode(ENC_SW,  INPUT_PULLUP);
}

void setupI2C() {
    // Bus 0 — MAX30102
    Wire.begin(SDA_PIN, SCL_PIN);

    // Bus 1 — OLED
    I2C_OLED.begin(OLED_SDA, OLED_SCL);

    delay(100);

    // Scan bus 0
    Serial.println("[I2C] Scanning bus 0 (MAX30102)...");
    int found0 = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("[I2C] Bus 0 device at 0x");
            Serial.println(addr, HEX);
            found0++;
        }
    }
    if (found0 == 0) Serial.println("[I2C] Bus 0: no devices found.");

    // Scan bus 1
    Serial.println("[I2C] Scanning bus 1 (OLED)...");
    int found1 = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        I2C_OLED.beginTransmission(addr);
        if (I2C_OLED.endTransmission() == 0) {
            Serial.print("[I2C] Bus 1 device at 0x");
            Serial.println(addr, HEX);
            found1++;
        }
    }
    if (found1 == 0) Serial.println("[I2C] Bus 1: no devices found.");
}

void setupSPI() {
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);
}

void setupDisplay() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("[OLED] Init failed — halting.");
        while (true);
    }
    display.clearDisplay();
    display.display();
    Serial.println("[BOOT] OLED OK");
}

void setupSD() {
    if (!SD.begin(SD_CS)) {
        Serial.println("[SD] Init failed — continuing without logging.");
        sdAvailable = false;
        return;
    }
    logFile = SD.open("/run.csv", FILE_WRITE);
    if (!logFile) {
        Serial.println("[SD] Could not open run.csv.");
        sdAvailable = false;
        return;
    }
    logFile.println("time_ms,ir,bpm,ax,ay,az,phase");
    sdAvailable = true;
    Serial.println("[SD] Ready — logging to /run.csv");
}

void setupPPG() {
    Serial.println("[PPG] Initialising MAX30105...");
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("[PPG] Not found — check wiring.");
        return;
    }
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
    Serial.println("[PPG] MAX30105 ready.");
}

void setupAccelerometer() {
    analogReadResolution(12);
}

void setupEncoder() {
    lastCLKState = digitalRead(ENC_CLK);
}

void setupApplication() {
    setupState   = SET_AGE;
    workoutPhase = PHASE_IDLE;
    displayDirty = true;
}

void drawInitialScreen() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Initialising...");
    display.display();
    delay(500);
    displayDirty = true;
}

// =========================================================
// PPG SUBSYSTEM
// =========================================================

void updatePPG(uint32_t now) {
    if (now - lastPPGUpdate < PPG_INTERVAL) return;
    lastPPGUpdate = now;

    uint32_t irRaw = (uint32_t)particleSensor.getIR();
    lastIR = irRaw;

    // ---- Finger detection with hysteresis ----
    if (!fingerPresent && irRaw < (uint32_t)fingerThreshold) return;

    if (fingerPresent && irRaw < (uint32_t)fingerRemoveThreshold) {
        fingerPresent = false;
        resetPPGState();
        Serial.println("[PPG] Finger removed.");
        return;
    }

    if (!fingerPresent) {
        fingerPresent = true;
        resetPPGState();
        dcLevel = (float)irRaw;
        Serial.println("[PPG] Finger detected — waiting for convergence...");
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
                Serial.println("[PPG] DC converged — detecting beats.");
            }
        } else {
            convergenceTimer = false;
        }

        if (now - lastPPGPrint >= 1000) {
            lastPPGPrint = now;
            Serial.print("[PPG] Converging... dc=");
            Serial.print(dcLevel, 0);
            Serial.print(" ac=");
            Serial.print(fabsf(ac), 1);
            Serial.print(" threshold=");
            Serial.println(convergenceThreshold, 0);
        }
        return;
    }

    // ---- AC smoothing ----
    smoothed = 0.80f * smoothed + 0.20f * ac;

    // ---- Motion artifact rejection ----
    if (fabsf(smoothed) > motionCeiling) {
        prevSmoothed   = smoothed;
        prevDerivative = 0.0f;
        rising         = false;
        return;
    }

    // ---- Derivative + peak detection ----
    derivative = smoothed - prevSmoothed;

    if (derivative > slopeThreshold && smoothed > 0) rising = true;

    bool peakDetected =
        rising                         &&
        derivative     <  0            &&
        prevDerivative >= 0            &&
        smoothed       > minPeakHeight &&
        (now - lastBeatTime > refractoryPeriod);

    if (peakDetected) {
        rising = false;

        if (lastBeatTime > 0) {
            uint32_t interval = now - lastBeatTime;
            if (interval > 333 && interval < 2000) {
                float candidateBPM = 60000.0f / interval;
                bool  accept       = true;

                if (beatCount >= 4 &&
                    fabsf(candidateBPM - beatAvg) > outlierThreshold) {
                    accept = false;
                    Serial.print("[PPG] Outlier rejected: ");
                    Serial.println(candidateBPM, 1);
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

    prevDerivative = derivative;
    prevSmoothed   = smoothed;

    // ---- 1 Hz serial status ----
    if (now - lastPPGPrint >= 1000) {
        lastPPGPrint = now;
        if (beatCount < 2) {
            Serial.print("[PPG] Acquiring... beats=");
            Serial.print(beatCount);
            Serial.print(" smoothed=");
            Serial.print(smoothed, 1);
            Serial.print(" minPeak=");
            Serial.println(minPeakHeight, 0);
        } else {
            Serial.print("[PPG] BPM=");
            Serial.println(currentBPM, 1);
        }
    }
}

// =========================================================
// ENCODER SUBSYSTEM
// =========================================================

void updateEncoder(uint32_t now) {

    if (workoutPhase == PHASE_IDLE && setupState != SET_CONFIRMED) {
        int currentCLK = digitalRead(ENC_CLK);

        if (currentCLK != lastCLKState && currentCLK == LOW) {
            if (now - lastMoveTime > ENCODER_DEBOUNCE_MS) {
                lastMoveTime = now;
                int dir = (digitalRead(ENC_DT) != currentCLK) ? 1 : -1;

                switch (setupState) {
                    case SET_AGE:
                        userAge = constrain(userAge + dir * 5, 10, 90);
                        break;
                    case SET_HR1:
                        targetHR1 = constrain(targetHR1 + dir * 10,
                                              (int)(maxHR * 0.5f), maxHR);
                        break;
                    case SET_DUR1:
                        duration1 = constrain(duration1 + dir * 30, 90, 600);
                        break;
                    case SET_HR2:
                        targetHR2 = constrain(targetHR2 + dir * 10,
                                              (int)(maxHR * 0.5f), maxHR);
                        break;
                    case SET_DUR2:
                        duration2 = constrain(duration2 + dir * 30, 90, 600);
                        break;
                    default: break;
                }
                displayDirty = true;
            }
        }
        lastCLKState = currentCLK;
    }

    bool buttonState = digitalRead(ENC_SW);

    if (lastButtonState == HIGH && buttonState == LOW) {
        if (now - lastButtonTime > BUTTON_DEBOUNCE_MS) {
            lastButtonTime = now;

            if (workoutPhase == PHASE_IDLE) {
                switch (setupState) {
                    case SET_AGE:
                        maxHR     = 220 - userAge;
                        // Clamp existing HR targets to new valid range
                        targetHR1 = constrain(targetHR1, (int)(maxHR * 0.5f), maxHR);
                        targetHR2 = constrain(targetHR2, (int)(maxHR * 0.5f), maxHR);
                        setupState = SET_HR1;
                        break;
                    case SET_HR1:  setupState = SET_DUR1; break;
                    case SET_DUR1: setupState = SET_HR2;  break;
                    case SET_HR2:  setupState = SET_DUR2; break;
                    case SET_DUR2:
                        setupState = SET_CONFIRMED;
                        startWorkout();
                        break;
                    default: break;
                }
            } else if (workoutPhase == PHASE_1) {
                // Skip to phase 2
                workoutPhase     = PHASE_2;
                phaseStartMillis = millis();
                phaseDurationMs  = duration2 * 1000UL;
                Serial.println("[WORKOUT] Phase 1 skipped — Phase 2 started.");
                displayDirty = true;
            } else if (workoutPhase == PHASE_2) {
                // Skip to complete
                workoutPhase = PHASE_DONE;
                digitalWrite(LED_PIN, LOW);
                if (sdAvailable && logFile) {
                    logFile.close();
                    sdAvailable = false;
                    Serial.println("[SD] Log closed. Safe to remove card.");
                }
                Serial.println("[WORKOUT] Phase 2 skipped — complete.");
                displayDirty = true;
            } else if (workoutPhase == PHASE_DONE) {
                resetApplication();
            }
            displayDirty = true;
        }
    }
    lastButtonState = buttonState;
}

// =========================================================
// WORKOUT SUBSYSTEM
// =========================================================

void updateWorkout(uint32_t now) {

    if (workoutPhase == PHASE_1) {
        digitalWrite(LED_PIN, (now / 300) % 2);

        if (now - phaseStartMillis >= phaseDurationMs) {
            workoutPhase     = PHASE_2;
            phaseStartMillis = now;
            phaseDurationMs  = duration2 * 1000UL;
            Serial.println("[WORKOUT] Phase 2 started.");
            displayDirty = true;
        }
    }
    else if (workoutPhase == PHASE_2) {
        digitalWrite(LED_PIN, HIGH);

        if (now - phaseStartMillis >= phaseDurationMs) {
            workoutPhase = PHASE_DONE;
            digitalWrite(LED_PIN, LOW);

            if (sdAvailable && logFile) {
                logFile.close();
                sdAvailable = false;
                Serial.println("[SD] Log closed. Safe to remove card.");
            }

            Serial.println("[WORKOUT] Complete.");
            displayDirty = true;
        }
    }
}

// =========================================================
// ACCELEROMETER SUBSYSTEM
// =========================================================

void updateAccelerometer(uint32_t now) {
    if (now - lastAccelUpdate < ACCEL_INTERVAL) return;
    lastAccelUpdate = now;

    accelX = analogRead(ACCEL_X);
    accelY = analogRead(ACCEL_Y);
    accelZ = analogRead(ACCEL_Z);
}

// =========================================================
// DISPLAY SUBSYSTEM
// =========================================================

void updateDisplay(uint32_t now) {
    if (now - lastOLEDUpdate < OLED_INTERVAL) return;
    lastOLEDUpdate = now;

    if (workoutPhase == PHASE_IDLE && now - lastBlink >= BLINK_INTERVAL) {
        lastBlink    = now;
        showColon    = !showColon;
        displayDirty = true;
    }

    if (workoutPhase == PHASE_1 || workoutPhase == PHASE_2) displayDirty = true;

    if (!displayDirty) return;
    displayDirty = false;

    switch (workoutPhase) {
        case PHASE_IDLE: drawSetupScreen();    break;
        case PHASE_1:    drawWorkoutScreen();  break;
        case PHASE_2:    drawWorkoutScreen();  break;
        case PHASE_DONE: drawCompleteScreen(); break;
    }
}

// =========================================================
// LOGGER SUBSYSTEM
// =========================================================

void updateLogger(uint32_t now) {
    if (now - lastLoggerUpdate < LOGGER_INTERVAL) return;
    lastLoggerUpdate = now;
    writeCSVRow();
}

// =========================================================
// SD MAINTENANCE
// =========================================================

void flushSD(uint32_t now) {
    if (!sdAvailable) return;
    if (now - lastFlushUpdate < FLUSH_INTERVAL) return;
    lastFlushUpdate = now;
    logFile.flush();
}

// =========================================================
// DISPLAY HELPERS
// =========================================================

void drawSetupScreen() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);

    // ---- Header ----
    switch (setupState) {
        case SET_AGE:       display.print("Your age");     break;
        case SET_HR1:       display.print("Select HR 1");  break;
        case SET_DUR1:      display.print("Select Dur 1"); break;
        case SET_HR2:       display.print("Select HR 2");  break;
        case SET_DUR2:      display.print("Select Dur 2"); break;
        case SET_CONFIRMED: display.print("Starting...");  break;
    }
    if (setupState != SET_CONFIRMED && showColon) display.print(":");

    // ---- Large value ----
    display.setTextSize(3);
    display.setCursor(10, 20);

    if (setupState == SET_AGE) {
        display.print(userAge);
        display.setTextSize(1);
        // Show calculated max HR below
        display.setCursor(0, 52);
        display.print("Max HR: ");
        display.print(220 - userAge);
        display.print(" bpm");
    }

    if (setupState == SET_HR1 || setupState == SET_HR2) {
        int hr = (setupState == SET_HR1) ? targetHR1 : targetHR2;
        display.print(hr);

        // Unit label
        display.setTextSize(1);
        display.setCursor(90, 22);
        display.print("BPM");

        // Zone label bottom line
        display.setCursor(0, 52);
        display.print(hrZoneLabel(hr, maxHR));
    }

    if (setupState == SET_DUR1 || setupState == SET_DUR2) {
        float mins = (setupState == SET_DUR1 ? duration1 : duration2) / 60.0f;
        display.print(mins, 1);
        display.setTextSize(1);
        display.print(" min");
    }

    display.display();
}

// Returns a short zone label for the given BPM and max HR.
// Zones follow the standard 5-zone model as % of max HR.
const char* hrZoneLabel(int bpm, int maxhr) {
    float pct = (float)bpm / (float)maxhr;
    if (pct < 0.60f) return "Zone 1: Recovery";
    if (pct < 0.70f) return "Zone 2: Fat burn";
    if (pct < 0.80f) return "Zone 3: Aerobic";
    if (pct < 0.90f) return "Zone 4: Threshold";
    return                   "Zone 5: Max";
}

void drawWorkoutScreen() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(workoutPhase == PHASE_1 ? "Phase 1" : "Phase 2");

    display.setCursor(70, 0);
    display.print("Tgt:");
    display.print(workoutPhase == PHASE_1 ? targetHR1 : targetHR2);
    display.print("bpm");

    display.setTextSize(3);
    display.setCursor(10, 18);
    display.print(getRemainingTime());
    display.print("s");

    display.setTextSize(1);
    display.setCursor(0, 50);
    if (signalSettled && beatCount >= 2) {
        display.print("HR:");
        display.print((int)currentBPM);
    } else if (fingerPresent) {
        display.print(signalSettled ? "Acquiring..." : "Converging...");
    } else {
        display.print("No finger");
    }

    display.display();
}

void drawCompleteScreen() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(2);
    display.setCursor(10, 10);
    display.print("Workout");
    display.setCursor(10, 30);
    display.print("Complete");

    display.setTextSize(1);
    display.setCursor(0, 55);
    display.print("Press to restart");

    display.display();
}

// =========================================================
// PPG HELPERS
// =========================================================

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
    lastIR           = 0;
}

// =========================================================
// WORKOUT HELPERS
// =========================================================

void startWorkout() {
    workoutPhase     = PHASE_1;
    phaseStartMillis = millis();
    phaseDurationMs  = duration1 * 1000UL;
    lastButtonTime   = millis();
    Serial.println("[WORKOUT] Started — Phase 1.");
}

void resetApplication() {
    setupState   = SET_AGE;
    workoutPhase = PHASE_IDLE;
    userAge      = 30;
    maxHR        = 190;
    targetHR1    = 95;   // ~50% of maxHR for default age 30
    targetHR2    = 95;
    duration1    = 90;
    duration2    = 90;
    digitalWrite(LED_PIN, LOW);
    displayDirty = true;

    if (!sdAvailable && SD.begin(SD_CS)) {
        logFile = SD.open("/run.csv", FILE_APPEND);
        if (logFile) {
            sdAvailable = true;
            Serial.println("[SD] Log reopened for next workout.");
        }
    }

    Serial.println("[WORKOUT] Reset.");
}

int getRemainingTime() {
    uint32_t elapsed = millis() - phaseStartMillis;
    if (elapsed >= phaseDurationMs) return 0;
    return (int)((phaseDurationMs - elapsed) / 1000UL);
}

// =========================================================
// LOGGER HELPERS
// =========================================================

void writeCSVRow() {
    if (!sdAvailable) return;

    logFile.printf(
        "%lu,%lu,%.1f,%d,%d,%d,%d\n",
        (unsigned long)millis(),
        (unsigned long)lastIR,
        currentBPM,
        accelX,
        accelY,
        accelZ,
        (int)workoutPhase
    );
}
