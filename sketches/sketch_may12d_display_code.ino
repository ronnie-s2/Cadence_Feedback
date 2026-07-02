#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =====================
// PINS
// =====================
const int CLK = 18;
const int DT  = 19;
const int SW  = 23;
const int LED_PIN = 2;

// =====================
// SETUP STATE MACHINE
// =====================
enum State {
  SET_HR1,
  SET_DUR1,
  SET_HR2,
  SET_DUR2,
  CONFIRMED
};

State state = SET_HR1;

// =====================
// EXECUTION MODE
// =====================
enum Mode {
  SETUP,
  RUN_TIMER1,
  RUN_TIMER2,
  COMPLETE
};

Mode mode = SETUP;

// =====================
// VALUES
// =====================
int hr1 = 60;
int hr2 = 60;

// duration stored in SECONDS
int dur1 = 90; // 1.5 min default
int dur2 = 90;

// =====================
// TIMING
// =====================
unsigned long timerStartMillis = 0;
unsigned long currentDuration = 0;

// =====================
// ENCODER
// =====================
int lastCLK;

// =====================
// BUTTON
// =====================
bool lastButton = HIGH;
unsigned long lastButtonTime = 0;
const int buttonDebounceMs = 250;

// =====================
// UI TIMING
// =====================
unsigned long lastMoveTime = 0;
const int encoderDelay = 5;

unsigned long lastBlink = 0;
bool showColon = true;

unsigned long lastUIUpdate = 0;

// =========================================================
// SETUP
// =========================================================
void setup() {

  Serial.begin(115200);

  Wire.begin(21, 22);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (true);
  }

  lastCLK = digitalRead(CLK);

  drawScreen();
}

// =========================================================
// LOOP
// =========================================================
void loop() {

  if (mode == SETUP) {

    handleButton();
    handleEncoder();
    handleBlink();

  } else {

    runTimers();
    handleCompleteButton();
  }
}

// =========================================================
// BUTTON (setup)
// =========================================================
void handleButton() {

  bool buttonState = digitalRead(SW);

  if (lastButton == HIGH && buttonState == LOW) {

    if (millis() - lastButtonTime > buttonDebounceMs) {

      lastButtonTime = millis();
      advanceState();
    }
  }

  lastButton = buttonState;
}

// =========================================================
// STATE ADVANCE
// =========================================================
void advanceState() {

  switch (state) {

    case SET_HR1: state = SET_DUR1; break;
    case SET_DUR1: state = SET_HR2; break;
    case SET_HR2: state = SET_DUR2; break;

    case SET_DUR2:
      state = CONFIRMED;
      startWorkout();
      break;

    default: break;
  }

  drawScreen();
}

// =========================================================
// START WORKOUT
// =========================================================
void startWorkout() {

  mode = RUN_TIMER1;

  timerStartMillis = millis();
  currentDuration = dur1 * 1000UL;

  Serial.println("Workout started");
}

// =========================================================
// TIMER LOOP
// =========================================================
void runTimers() {

  unsigned long now = millis();

  // =====================
  // PHASE 1
  // =====================
  if (mode == RUN_TIMER1) {

    digitalWrite(LED_PIN, (millis() / 300) % 2);

    if (now - timerStartMillis >= currentDuration) {

      mode = RUN_TIMER2;

      timerStartMillis = now;
      currentDuration = dur2 * 1000UL;

      drawScreen();
    }
  }

  // =====================
  // PHASE 2
  // =====================
  else if (mode == RUN_TIMER2) {

    digitalWrite(LED_PIN, HIGH);

    if (now - timerStartMillis >= currentDuration) {

      mode = COMPLETE;

      digitalWrite(LED_PIN, LOW);

      drawScreen();
    }
  }

  // UI refresh throttle (important for countdown)
  if (millis() - lastUIUpdate > 200) {
    drawScreen();
    lastUIUpdate = millis();
  }
}

// =========================================================
// COMPLETE SCREEN BUTTON
// =========================================================
void handleCompleteButton() {

  bool buttonState = digitalRead(SW);

  if (lastButton == HIGH && buttonState == LOW) {

    if (millis() - lastButtonTime > buttonDebounceMs) {

      lastButtonTime = millis();
      resetAll();
    }
  }

  lastButton = buttonState;
}

// =========================================================
// RESET
// =========================================================
void resetAll() {

  state = SET_HR1;
  mode = SETUP;

  hr1 = 60;
  hr2 = 60;
  dur1 = 90;
  dur2 = 90;

  Serial.println("Reset");

  drawScreen();
}

// =========================================================
// ENCODER
// =========================================================
void handleEncoder() {

  if (state == CONFIRMED) return;

  int currentCLK = digitalRead(CLK);

  if (currentCLK != lastCLK && currentCLK == LOW) {

    if (millis() - lastMoveTime > encoderDelay) {

      int dir = (digitalRead(DT) != currentCLK) ? 1 : -1;

      switch (state) {

        case SET_HR1:
          hr1 += dir * 10;
          hr1 = constrain(hr1, 60, 170);
          break;

        case SET_DUR1:
          dur1 += dir * 30;
          dur1 = constrain(dur1, 90, 600);
          break;

        case SET_HR2:
          hr2 += dir * 10;
          hr2 = constrain(hr2, 60, 170);
          break;

        case SET_DUR2:
          dur2 += dir * 30;
          dur2 = constrain(dur2, 90, 600);
          break;

        default:
          break;
      }

      lastMoveTime = millis();
      drawScreen();
    }
  }

  lastCLK = currentCLK;
}

// =========================================================
// BLINK
// =========================================================
void handleBlink() {

  if (millis() - lastBlink > 500) {
    showColon = !showColon;
    lastBlink = millis();
    drawScreen();
  }
}

// =========================================================
// HELPER: remaining time
// =========================================================
int getRemainingSeconds() {

  unsigned long elapsed = millis() - timerStartMillis;

  if (elapsed >= currentDuration) return 0;

  return (currentDuration - elapsed) / 1000;
}

// =========================================================
// DISPLAY
// =========================================================
void drawScreen() {

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // =====================
  // SETUP MODE
  // =====================
  if (mode == SETUP) {

    display.setTextSize(1);
    display.setCursor(0, 0);

    switch (state) {

      case SET_HR1:
        display.print("Select HR 1");
        if (showColon) display.print(":");
        break;

      case SET_DUR1:
        display.print("Select Dur 1");
        if (showColon) display.print(":");
        break;

      case SET_HR2:
        display.print("Select HR 2");
        if (showColon) display.print(":");
        break;

      case SET_DUR2:
        display.print("Select Dur 2");
        if (showColon) display.print(":");
        break;

      case CONFIRMED:
        display.print("Starting...");
        break;
    }

    display.setTextSize(3);
display.setCursor(10, 25);

if (state == SET_HR1) {
  display.print(hr1);
  display.setTextSize(1);
  display.setCursor(90, 45);
  display.print("BPM");
}

if (state == SET_HR2) {
  display.print(hr2);
  display.setTextSize(1);
  display.setCursor(90, 45);
  display.print("BPM");
}

    if (state == SET_DUR1 || state == SET_DUR2) {

      float mins = (state == SET_DUR1 ? dur1 : dur2) / 60.0;
      display.print(mins, 1);
      display.setTextSize(1);
      display.print(" min");
    }
  }

  // =====================
  // TIMER 1
  // =====================
  else if (mode == RUN_TIMER1) {

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Timer 1 running");

    display.setTextSize(3);
    display.setCursor(20, 25);
    display.print(getRemainingSeconds());
    display.print("s");
  }

  // =====================
  // TIMER 2
  // =====================
  else if (mode == RUN_TIMER2) {

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Timer 2 running");

    display.setTextSize(3);
    display.setCursor(20, 25);
    display.print(getRemainingSeconds());
    display.print("s");
  }

  // =====================
  // COMPLETE
  // =====================
  else if (mode == COMPLETE) {

    display.setTextSize(2);
    display.setCursor(10, 10);
    display.print("Workout");

    display.setCursor(10, 30);
    display.print("Complete");

    display.setTextSize(1);
    display.setCursor(0, 55);
    display.print("Press to restart");
  }

  display.display();
}