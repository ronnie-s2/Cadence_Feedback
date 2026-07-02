#include <Wire.h>
#include <U8g2lib.h>

// =====================
// OLED (SH1106 128x64 I2C common clone)
// =====================
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// =====================
// PINS
// =====================
const int CLK = 18;
const int DT  = 19;
const int SW  = 23;
const int LED_PIN = 2;

// =====================
// STATES (setup flow)
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
// MODES (execution flow)
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

int dur1 = 90;
int dur2 = 90;

// =====================
// TIMERS
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
const int debounceMs = 250;

// =====================
// UI TIMING
// =====================
unsigned long lastMoveTime = 0;
unsigned long lastBlink = 0;
bool showColon = true;

// =========================================================
// SETUP
// =========================================================
void setup() {

  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  Wire.begin(21, 22);

  u8g2.begin();

  lastCLK = digitalRead(CLK);
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

  draw();
}

// =========================================================
// BUTTON
// =========================================================
void handleButton() {

  bool b = digitalRead(SW);

  if (lastButton == HIGH && b == LOW) {

    if (millis() - lastButtonTime > debounceMs) {
      lastButtonTime = millis();
      advanceState();
    }
  }

  lastButton = b;
}

// =========================================================
// STATE MACHINE
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
}

// =========================================================
// START WORKOUT
// =========================================================
void startWorkout() {

  mode = RUN_TIMER1;
  timerStartMillis = millis();
  currentDuration = dur1 * 1000UL;
}

// =========================================================
// TIMER LOGIC
// =========================================================
void runTimers() {

  unsigned long now = millis();

  if (mode == RUN_TIMER1) {

    digitalWrite(LED_PIN, (now / 300) % 2);

    if (now - timerStartMillis >= currentDuration) {
      mode = RUN_TIMER2;
      timerStartMillis = now;
      currentDuration = dur2 * 1000UL;
    }
  }

  else if (mode == RUN_TIMER2) {

    digitalWrite(LED_PIN, HIGH);

    if (now - timerStartMillis >= currentDuration) {
      mode = COMPLETE;
      digitalWrite(LED_PIN, LOW);
    }
  }
}

// =========================================================
// RESET
// =========================================================
void handleCompleteButton() {

  bool b = digitalRead(SW);

  if (lastButton == HIGH && b == LOW) {

    if (millis() - lastButtonTime > debounceMs) {
      lastButtonTime = millis();
      resetAll();
    }
  }

  lastButton = b;
}

void resetAll() {

  state = SET_HR1;
  mode = SETUP;

  hr1 = 60;
  hr2 = 60;
  dur1 = 90;
  dur2 = 90;
}

// =========================================================
// ENCODER
// =========================================================
void applyEncoder(int dir) {

  if (state == SET_HR1) hr1 = constrain(hr1 + dir * 10, 60, 170);
  if (state == SET_HR2) hr2 = constrain(hr2 + dir * 10, 60, 170);

  if (state == SET_DUR1) dur1 = constrain(dur1 + dir * 30, 90, 600);
  if (state == SET_DUR2) dur2 = constrain(dur2 + dir * 30, 90, 600);
}

void handleEncoder() {

  static uint8_t lastState = 0;

  uint8_t clkState = digitalRead(CLK);
  uint8_t dtState  = digitalRead(DT);

  uint8_t currentState = (clkState << 1) | dtState;

  if (currentState == lastState) return;

  if ((lastState == 0b00 && currentState == 0b01) ||
      (lastState == 0b01 && currentState == 0b11) ||
      (lastState == 0b11 && currentState == 0b10) ||
      (lastState == 0b10 && currentState == 0b00)) {
    applyEncoder(1);
  }

  if ((lastState == 0b00 && currentState == 0b10) ||
      (lastState == 0b10 && currentState == 0b11) ||
      (lastState == 0b11 && currentState == 0b01) ||
      (lastState == 0b01 && currentState == 0b00)) {
    applyEncoder(-1);
  }

  lastState = currentState;
}

// =========================================================
// BLINK
// =========================================================
void handleBlink() {
  if (millis() - lastBlink > 500) {
    showColon = !showColon;
    lastBlink = millis();
  }
}

// =========================================================
// TIMER HELPERS
// =========================================================
int getRemaining() {

  unsigned long elapsed = millis() - timerStartMillis;

  if (elapsed >= currentDuration) return 0;

  return (currentDuration - elapsed) / 1000;
}

// =========================================================
// DRAW
// =========================================================
void draw() {

  u8g2.firstPage();
  do {

    // =====================
    // SETUP SCREEN
    // =====================
    if (mode == SETUP) {

      u8g2.setFont(u8g2_font_6x10_tf);

      const char* title =
        (state == SET_HR1) ? "Select HR 1" :
        (state == SET_DUR1) ? "Select Dur 1" :
        (state == SET_HR2) ? "Select HR 2" :
        (state == SET_DUR2) ? "Select Dur 2" :
        "Starting";

      u8g2.drawStr(0, 10, title);

      char buf[20];

      // =====================
      // HR DISPLAY (DYNAMIC FONT FIX)
      // =====================
      if (state == SET_HR1 || state == SET_HR2) {

        int hr = (state == SET_HR1) ? hr1 : hr2;

        sprintf(buf, "%d BPM", hr);

        if (hr >= 100) {
          u8g2.setFont(u8g2_font_logisoso24_tf);
          u8g2.drawStr(10, 55, buf);
        } else {
          u8g2.setFont(u8g2_font_logisoso32_tf);
          u8g2.drawStr(0, 55, buf);
        }
      }

      // =====================
      // DURATION DISPLAY (DYNAMIC FONT FIX)
      // =====================
      else {

        float mins = ((state == SET_DUR1 ? dur1 : dur2) / 60.0);

        char buf[20];
        sprintf(buf, "%.1f min", mins);

        int w;

        // Try large font first
        u8g2.setFont(u8g2_font_logisoso32_tf);
        w = u8g2.getStrWidth(buf);

        if (w <= 128) {
          // Fits → use big font
          u8g2.drawStr((128 - w) / 2, 55, buf);
        } else {
          // Too wide → shrink font
          u8g2.setFont(u8g2_font_logisoso24_tf);
          w = u8g2.getStrWidth(buf);
          u8g2.drawStr((128 - w) / 2, 55, buf);
        }
      }
    }

    // =====================
    // TIMER 1
    // =====================
    else if (mode == RUN_TIMER1) {

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 15, "Timer 1");

      char buf[10];
      sprintf(buf, "%ds", getRemaining());

      u8g2.setFont(u8g2_font_logisoso32_tf);
      u8g2.drawStr(0, 55, buf);
    }

    // =====================
    // TIMER 2
    // =====================
    else if (mode == RUN_TIMER2) {

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(0, 15, "Timer 2");

      char buf[10];
      sprintf(buf, "%ds", getRemaining());

      u8g2.setFont(u8g2_font_logisoso32_tf);
      u8g2.drawStr(0, 55, buf);
    }

    // =====================
    // COMPLETE
    // =====================
    else if (mode == COMPLETE) {

      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawStr(20, 25, "Workout");
      u8g2.drawStr(20, 45, "Complete");

      u8g2.setFont(u8g2_font_5x7_tf);
      u8g2.drawStr(10, 62, "Press to restart");
    }

  } while (u8g2.nextPage());
}