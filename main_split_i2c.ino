// =========================================================
// LIBRARIES
// =========================================================

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =========================================================
// PIN DEFINITIONS
// =========================================================

// ---------- I2C bus 0 — MAX30102 only ----------
const int SDA_PIN  = 16;
const int SCL_PIN  = 17;

// ---------- I2C bus 1 — OLED only ----------
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
// OLED — separate TwoWire instance on bus 1
// =========================================================

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_ADDR    0x3C

TwoWire I2C_OLED = TwoWire(1);  // I2C peripheral 1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);

// =========================================================
// PPG TUNABLE PARAMETERS
// =========================================================

const int32_t  fingerThreshold      = 50000;
const float    minPeakHeight        = 300.0f;
const float    slopeThreshold       = 30.0f;
const uint32_t refractoryTime       = 333;
const float    outlierThreshold     = 25.0f;
const float    convergenceThreshold = 500.0f;
const uint32_t convergenceHoldMs    = 2000;

// =========================================================
// APPLICATION STATE
// =========================================================

enum SetupState {
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
// PPG INTERNAL STATE
// =========================================================

const float alphaFast = 0.984f;
const float alphaSlow = 0.999f;
float       ppgAlpha  = alphaFast;
float       dcLevel   = 0.0f;

float smoothed       = 0.0f;
float prevSmoothed   = 0.0f;
float derivative     = 0.0f;
float prevDerivative = 0.0f;
bool  rising         = false;

bool     convergenceTimer = false;
uint32_t convergenceStart = 0;

uint32_t lastBeatTime = 0;
float    beatAvg      = 0.0f;
int      beatCount    = 0;

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

void     writeReg(uint8_t reg, uint8_t val);
uint8_t  readReg(uint8_t reg);
uint32_t readIROnce();
uint32_t drainFIFO();
void     processPPGSample(uint32_t irRaw, uint32_t now);
void     resetPPGState();
void     drawSetupScreen();
void     drawWorkoutScreen();
void     drawCompleteScreen();
void     startWorkout();
void     resetApplication();
int      getRemainingTime();
void     writeCSVRow();

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

    updatePPG(now);
    updateEncoder(now);

    updateWorkout(now);
    updateAccelerometer(now);

    updateDisplay(now);
    updateLogger(now);
    flushSD(now);
}

// =========================================================
// SETUP FUNCTIONS
// =========================================================

void setupSerial() {
    Serial.begin(115200);
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

    // Scan both buses
    Serial.println("[I2C] Scanning bus 0 (MAX30102)...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("[I2C] Bus 0 device at 0x");
            Serial.println(addr, HEX);
        }
    }

    Serial.println("[I2C] Scanning bus 1 (OLED)...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        I2C_OLED.beginTransmission(addr);
        if (I2C_OLED.endTransmission() == 0) {
            Serial.print("[I2C] Bus 1 device at 0x");
            Serial.println(addr, HEX);
        }
    }
}

void setupSPI() {
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);
}

void setupDisplay() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("SSD1306 init failed — halting.");
        while (true);
    }
    display.clearDisplay();
    display.display();
}

void setupSD() {
    if (!SD.begin(SD_CS)) {
        Serial.println("SD init failed — continuing without logging.");
        sdAvailable = false;
        return;
    }
    logFile = SD.open("/run.csv", FILE_WRITE);
    if (!logFile) {
        Serial.println("Failed to open log file — continuing without logging.");
        sdAvailable = false;
        return;
    }
    logFile.println("time_ms,ir,bpm,ax,ay,az,phase");
    sdAvailable = true;
    Serial.println("SD ready — logging to /run.csv");
}

void setupPPG() {
    delay(100);
    writeReg(REG_MODE_CONFIG, 0x40);  // soft reset
    delay(100);
    writeReg(REG_SPO2_CONFIG,  0x27);
    writeReg(REG_LED1_PA,      0x2A);
    writeReg(REG_LED2_PA,      0x2A);
    writeReg(REG_MODE_CONFIG,  0x03);
    delay(100);
    // Flush FIFO: set RD_PTR = WR_PTR
    uint8_t wr = readReg(REG_FIFO_WR_PTR);
    writeReg(REG_FIFO_RD_PTR, wr);
    Serial.println("MAX30102 ready.");
}

void setupAccelerometer() {
    analogReadResolution(12);
}

void setupEncoder() {
    lastCLKState = digitalRead(ENC_CLK);
}


void setupApplication() {
    setupState   = SET_HR1;
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

    // --- FIFO overflow recovery ---
    uint8_t ov = readReg(REG_OVF_COUNTER);
    if (ov > 0) {
        uint8_t wr = readReg(REG_FIFO_WR_PTR);
        writeReg(REG_FIFO_RD_PTR, wr);
        Serial.print("[PPG] FIFO overflow (OVF=");
        Serial.print(ov);
        Serial.println(") — cleared.");
        return;
    }

    // --- FIFO pointer debug every 2s ---
    static uint32_t lastFIFODebug = 0;
    if (now - lastFIFODebug >= 2000) {
        lastFIFODebug = now;
        uint8_t wr = readReg(REG_FIFO_WR_PTR);
        uint8_t rd = readReg(REG_FIFO_RD_PTR);
        Serial.print("[FIFO] WR="); Serial.print(wr);
        Serial.print(" RD="); Serial.print(rd);
        Serial.print(" samples="); Serial.println((wr - rd + 32) % 32);
    }

    uint32_t irRaw = drainFIFO();

    // --- Raw IR debug for first 10s ---
    static bool earlyDebug = true;
    if (earlyDebug) {
        Serial.print("[PPG] irRaw="); Serial.println(irRaw);
        if (now > 10000) {
            earlyDebug = false;
            Serial.println("[PPG] Early debug done.");
        }
    }

    if (irRaw == 0) return;

    lastIR = irRaw;
    processPPGSample(irRaw, now);

    // --- 1 Hz status ---
    if (now - lastPPGPrint >= 1000) {
        lastPPGPrint = now;
        if (!fingerPresent) {
            Serial.println("[PPG] No finger.");
        } else if (!signalSettled) {
            Serial.print("[PPG] Converging... dc=");
            Serial.println(dcLevel, 0);
        } else if (beatCount < 4) {
            Serial.print("[PPG] Acquiring... beats=");
            Serial.println(beatCount);
        } else {
            Serial.print("[PPG] BPM=");
            Serial.println(currentBPM, 1);
        }
    }
}

void updateEncoder(uint32_t now) {

    if (workoutPhase == PHASE_IDLE && setupState != SET_CONFIRMED) {
        int currentCLK = digitalRead(ENC_CLK);

        if (currentCLK != lastCLKState && currentCLK == LOW) {
            if (now - lastMoveTime > ENCODER_DEBOUNCE_MS) {
                lastMoveTime = now;
                int dir = (digitalRead(ENC_DT) != currentCLK) ? 1 : -1;

                switch (setupState) {
                    case SET_HR1:  targetHR1 = constrain(targetHR1 + dir * 10, 60, 170); break;
                    case SET_DUR1: duration1 = constrain(duration1 + dir * 30, 90, 600); break;
                    case SET_HR2:  targetHR2 = constrain(targetHR2 + dir * 10, 60, 170); break;
                    case SET_DUR2: duration2 = constrain(duration2 + dir * 30, 90, 600); break;
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
                    case SET_HR1:  setupState = SET_DUR1; break;
                    case SET_DUR1: setupState = SET_HR2;  break;
                    case SET_HR2:  setupState = SET_DUR2; break;
                    case SET_DUR2:
                        setupState = SET_CONFIRMED;
                        startWorkout();
                        break;
                    default: break;
                }
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
            Serial.println("Phase 2 started.");
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
                Serial.println("Log file closed. SD card safe to remove.");
            }

            Serial.println("Workout complete.");
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

    switch (setupState) {
        case SET_HR1:       display.print("Select HR 1");  break;
        case SET_DUR1:      display.print("Select Dur 1"); break;
        case SET_HR2:       display.print("Select HR 2");  break;
        case SET_DUR2:      display.print("Select Dur 2"); break;
        case SET_CONFIRMED: display.print("Starting...");  break;
    }
    if (setupState != SET_CONFIRMED && showColon) display.print(":");

    display.setTextSize(3);
    display.setCursor(10, 25);

    if (setupState == SET_HR1 || setupState == SET_HR2) {
        display.print(setupState == SET_HR1 ? targetHR1 : targetHR2);
        display.setTextSize(1);
        display.setCursor(90, 45);
        display.print("BPM");
    }
    if (setupState == SET_DUR1 || setupState == SET_DUR2) {
        float mins = (setupState == SET_DUR1 ? duration1 : duration2) / 60.0f;
        display.print(mins, 1);
        display.setTextSize(1);
        display.print(" min");
    }

    display.display();
}

void drawWorkoutScreen() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(workoutPhase == PHASE_1 ? "Phase 1" : "Phase 2");

    display.setCursor(72, 0);
    display.print("Tgt:");
    display.print("bpm");

    display.setTextSize(3);
    display.setCursor(10, 18);
    display.print(getRemainingTime());
    display.print("s");

    display.setTextSize(1);
    display.setCursor(0, 50);
    if (signalSettled && beatCount >= 4) {
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
// PPG HELPERS — I2C (always uses Wire / bus 0)
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

    if (irRaw < (uint32_t)fingerThreshold) {
        if (fingerPresent) {
            fingerPresent = false;
            resetPPGState();
            Serial.println("[PPG] Finger removed.");
        }
        return;
    }

    if (!fingerPresent) {
        fingerPresent = true;
        resetPPGState();
        dcLevel = (float)irRaw;
        Serial.println("[PPG] Finger detected — waiting for DC convergence...");
        return;
    }

    dcLevel  = ppgAlpha * dcLevel + (1.0f - ppgAlpha) * (float)irRaw;
    float ac = (float)irRaw - dcLevel;

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
        return;
    }

    smoothed   = 0.80f * smoothed + 0.20f * ac;
    derivative = smoothed - prevSmoothed;

    if (derivative > slopeThreshold && smoothed > 0) rising = true;

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
                    Serial.print("[PPG] outlier rejected: ");
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
    lastIR           = 0;
}

// =========================================================
// WORKOUT HELPERS
// =========================================================

void startWorkout() {
    workoutPhase     = PHASE_1;
    phaseStartMillis = millis();
    phaseDurationMs  = duration1 * 1000UL;
    Serial.println("Workout started — Phase 1.");
}

void resetApplication() {
    setupState   = SET_HR1;
    workoutPhase = PHASE_IDLE;
    targetHR1    = 60;
    targetHR2    = 60;
    duration1    = 90;
    duration2    = 90;
    digitalWrite(LED_PIN, LOW);
    displayDirty = true;

    if (!sdAvailable && SD.begin(SD_CS)) {
        logFile = SD.open("/run.csv", FILE_APPEND);
        if (logFile) {
            sdAvailable = true;
            Serial.println("Log file reopened for next workout.");
        }
    }

    Serial.println("Reset.");
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
