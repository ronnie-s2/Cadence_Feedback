// =========================================================
// LIBRARIES
// =========================================================

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include <driver/i2s.h>
#include <math.h>

// =========================================================
// PIN DEFINITIONS
// =========================================================

// ---------- I2C ----------
const int SDA_PIN  = 16;
const int SCL_PIN  = 17;

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

// ---------- I2S / PCM5102 ----------
const int I2S_BCK  = 33;
const int I2S_WS   = 14;
const int I2S_DO   = 22;

// ---------- Misc ----------
const int LED_PIN  = 2;

// =========================================================
// OLED
// =========================================================

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_ADDR    0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =========================================================
// PPG — SparkFun MAX30105 library (init + FIFO only)
// =========================================================

MAX30105 particleSensor;

// =========================================================
// PPG SIGNAL PROCESSING STATE
// =========================================================

const float alphaFast = 0.960f;
const float alphaSlow = 0.998f;
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

// ---------- PPG tunable parameters ----------
const int32_t  fingerThreshold       = 40000;
const int32_t  fingerRemoveThreshold = 30000;
const float    motionCeiling         = 1100.0f;
const uint32_t refractoryPeriod      = 500;
const float    minPeakHeight         = 400.0f;
const float    slopeThreshold        = 30.0f;
const float    outlierThreshold      = 40.0f;
const float    convergenceThreshold  = 2000.0f;
const uint32_t convergenceHoldMs     = 500;

// =========================================================
// METRONOME PARAMETERS
// =========================================================

const float    METRO_SPM_MIN       = 120.0f;
const float    METRO_SPM_MAX       = 200.0f;
const float    METRO_SPM_DEFAULT   = 150.0f;
const float    METRO_GAIN          = 0.8f;
const float    METRO_MAX_STEP      = 5.0f;
const uint32_t METRO_CTRL_INTERVAL = 10000;  // ms between controller updates
const int      METRO_ACQUIRE_BEATS = 4;      // beats before control loop closes

// ---------- I2S / click generation ----------
const int   SAMPLE_RATE    = 44100;
const float CLICK_FREQ_HZ  = 1200.0f;
const int   CLICK_MS       = 30;
const int   BUFFER_SAMPLES = 256;

// =========================================================
// METRONOME STATE
// =========================================================

float    metronomeSPM      = METRO_SPM_DEFAULT;
uint32_t metronomeInterval = 0;   // ms between clicks
uint32_t lastClickTime     = 0;
uint32_t lastCtrlUpdate    = 0;
bool     clickLeft         = true; // alternate L/R channels each beat

int16_t audioBuffer[BUFFER_SAMPLES * 2];  // stereo interleaved

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
void drawWorkoutScreen();
void drawCompleteScreen();
void startWorkout();
void resetApplication();
int  getRemainingTime();
void writeCSVRow();
void resetPPGState();
void generateClick(bool leftChannel);
void updateMetronomeInterval();
int  activeTargetHR();

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
    setupI2S();
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
    updateMetronome(now);
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
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);

    Serial.println("[I2C] Scanning...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("[I2C] Device at 0x");
            Serial.println(addr, HEX);
            found++;
        }
    }
    if (found == 0) Serial.println("[I2C] No devices found.");
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
    logFile.println("time_ms,ir,bpm,ax,ay,az,phase,spm");
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

void setupI2S() {
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = 0,
        .dma_buf_count        = 8,
        .dma_buf_len          = 256,
        .use_apll             = false,
        .tx_desc_auto_clear   = true,
        .fixed_mclk           = 0
    };

    i2s_pin_config_t pins = {
        .bck_io_num   = I2S_BCK,
        .ws_io_num    = I2S_WS,
        .data_out_num = I2S_DO,
        .data_in_num  = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pins);

    // Pre-fill DMA with silence so DAC doesn't output noise at rest
    memset(audioBuffer, 0, sizeof(audioBuffer));
    size_t bw;
    for (int i = 0; i < 8; i++)
        i2s_write(I2S_NUM_0, audioBuffer,
                  BUFFER_SAMPLES * 2 * sizeof(int16_t), &bw, portMAX_DELAY);

    updateMetronomeInterval();
    Serial.println("[I2S] Ready.");
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
            Serial.print(currentBPM, 1);
            Serial.print(" SPM=");
            Serial.println(metronomeSPM, 1);
        }
    }
}

// =========================================================
// METRONOME SUBSYSTEM
// =========================================================

void updateMetronome(uint32_t now) {

    // Only active during workout phases
    if (workoutPhase != PHASE_1 && workoutPhase != PHASE_2) return;
    if (metronomeInterval == 0) return;

    // ---- Click tick ----
    if (now - lastClickTime >= metronomeInterval) {
        lastClickTime = now;
        generateClick(clickLeft);
        clickLeft = !clickLeft;  // alternate L/R each beat
    }

    // ---- Proportional cadence controller ----
    if (signalSettled && beatCount >= METRO_ACQUIRE_BEATS) {
        if (now - lastCtrlUpdate >= METRO_CTRL_INTERVAL) {
            lastCtrlUpdate = now;

            float error      = (float)activeTargetHR() - currentBPM;
            float adjustment = constrain(error * METRO_GAIN,
                                         -METRO_MAX_STEP, METRO_MAX_STEP);
            metronomeSPM     = constrain(metronomeSPM + adjustment,
                                         METRO_SPM_MIN, METRO_SPM_MAX);
            updateMetronomeInterval();

            Serial.print("[METRO] error="); Serial.print(error, 1);
            Serial.print(" adj=");          Serial.print(adjustment, 1);
            Serial.print(" SPM=");          Serial.println(metronomeSPM, 1);
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
            lastCtrlUpdate   = now;
            metronomeSPM     = METRO_SPM_DEFAULT;
            updateMetronomeInterval();
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
    display.print(activeTargetHR());
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
        display.print(" SPM:");
        display.print((int)metronomeSPM);
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
// I2S / METRONOME HELPERS
// =========================================================

// Non-blocking click: generates and writes audio directly to DMA.
// i2s_write with a short timeout returns immediately once DMA accepts data.
// At 44100Hz a 30ms click is 1323 samples — well within 8x256 DMA buffers.
void generateClick(bool leftChannel) {
    const int clickSamples = (SAMPLE_RATE * CLICK_MS) / 1000;
    float phase    = 0.0f;
    float phaseInc = 2.0f * PI * CLICK_FREQ_HZ / SAMPLE_RATE;

    int remaining = clickSamples;
    while (remaining > 0) {
        int chunk = min(remaining, BUFFER_SAMPLES);

        for (int i = 0; i < chunk; i++) {
            float env    = expf(-5.0f * (float)i / clickSamples);
            float s      = sinf(phase) * env;
            int16_t samp = (int16_t)(s * 12000);
            phase += phaseInc;

            audioBuffer[i * 2]     = leftChannel ? samp : 0;  // L
            audioBuffer[i * 2 + 1] = leftChannel ? 0 : samp;  // R
        }

        size_t bw;
        i2s_write(I2S_NUM_0, audioBuffer,
                  chunk * 2 * sizeof(int16_t),
                  &bw, pdMS_TO_TICKS(10));  // 10ms timeout — non-blocking

        remaining -= chunk;
    }
}

void updateMetronomeInterval() {
    if (metronomeSPM > 0)
        metronomeInterval = (uint32_t)(60000.0f / metronomeSPM);
}

int activeTargetHR() {
    return (workoutPhase == PHASE_1) ? targetHR1 : targetHR2;
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
    metronomeSPM     = METRO_SPM_DEFAULT;
    lastCtrlUpdate   = millis();
    updateMetronomeInterval();
    Serial.println("[WORKOUT] Started — Phase 1.");
}

void resetApplication() {
    setupState   = SET_HR1;
    workoutPhase = PHASE_IDLE;
    targetHR1    = 60;
    targetHR2    = 60;
    duration1    = 90;
    duration2    = 90;
    metronomeSPM = METRO_SPM_DEFAULT;
    updateMetronomeInterval();
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
        "%lu,%lu,%.1f,%d,%d,%d,%d,%.1f\n",
        (unsigned long)millis(),
        (unsigned long)lastIR,
        currentBPM,
        accelX,
        accelY,
        accelZ,
        (int)workoutPhase,
        metronomeSPM
    );
}
