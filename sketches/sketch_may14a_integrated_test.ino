#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "MAX30105.h"

MAX30105 ppg;

// ---------- SD CARD ----------
const int SD_CS = 5;
File logFile;

// ---------- ADXL335 ----------
const int X_PIN = 34;
const int Y_PIN = 35;
const int Z_PIN = 32;

// ---------- TIMING ----------
unsigned long lastFlush = 0;

void setup() {
  Serial.begin(115200);

  delay(1000);

  Serial.println("Starting logger...");

  // ---------- ADC ----------
  analogReadResolution(12); // 0-4095

  // ---------- I2C ----------
  Wire.begin(21, 22);

  // ---------- SD CARD ----------
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }

  Serial.println("SD card initialized.");

  // Create/open file
  logFile = SD.open("/run.csv", FILE_WRITE);

  if (!logFile) {
    Serial.println("Failed to open file!");
    while (1);
  }

  // CSV header
  logFile.println("time_ms,ir,red,ax,ay,az");

  // ---------- MAX30102 ----------
  if (!ppg.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found!");
    while (1);
  }

  Serial.println("MAX30102 initialized.");

  // Sensor configuration
  ppg.setup(
    50,   // LED brightness (0-255)
    4,    // sample averaging
    2,    // LED mode (Red + IR)
    100,  // sample rate Hz
    411,  // pulse width
    4096  // ADC range
  );

  Serial.println("Logging started.");
}

void loop() {

  // ---------- READ PPG ----------
  long ir = ppg.getIR();
  long red = ppg.getRed();

  // ---------- READ ACCEL ----------
  int ax = analogRead(X_PIN);
  int ay = analogRead(Y_PIN);
  int az = analogRead(Z_PIN);

  // ---------- TIMESTAMP ----------
  unsigned long t = millis();

  // ---------- WRITE CSV ----------
  logFile.printf(
    "%lu,%ld,%ld,%d,%d,%d\n",
    t,
    ir,
    red,
    ax,
    ay,
    az
  );

  // ---------- PERIODIC FLUSH ----------
  if (millis() - lastFlush > 1000) {
    logFile.flush();
    lastFlush = millis();
    Serial.println("Flushed to SD");
  }

  // ---------- SAMPLE RATE ----------
  delay(10); // ~100 Hz
}