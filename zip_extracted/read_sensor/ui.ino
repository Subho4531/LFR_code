/*
 * ui.ino  — Buttons + OLED display
 *
 * Button behaviour:
 *   BTN_CAL  short press  → start calibration sweep (5 s)
 *   BTN_CAL  long press   → start motor test
 *   BTN_START short press → toggle run / stop
 *   BTN_START long press  → (reserved for maze mode toggle)
 */

// ─────────────────────────────────────────────
//  BUTTON HANDLING  (software debounce + long-press)
// ─────────────────────────────────────────────
void handleButtons() {
  static unsigned long calPressStart   = 0;
  static bool          calLongFired    = false;
  static unsigned long startPressStart = 0;
  static bool          startLongFired  = false;

  // ── BTN_CAL ───────────────────────────────
  bool calDown = (digitalRead(BTN_CAL) == LOW);

  if (calDown) {
    if (calPressStart == 0) calPressStart = millis();
    if (!calLongFired && (millis() - calPressStart > 1000)) {
      calLongFired = true;
      if (robotState == STATE_IDLE || robotState == STATE_RUNNING) {
        robotState = STATE_IDLE;
        stopMotors();
        startMotorTest();
      }
    }
  } else {
    if (calPressStart > 0) {
      unsigned long held = millis() - calPressStart;
      if (!calLongFired && held > 50) {             // short press
        if (robotState == STATE_IDLE) {
          startCalibration();
        }
      }
      calPressStart = 0;
      calLongFired  = false;
    }
  }

  // ── BTN_START ─────────────────────────────
  bool startDown = (digitalRead(BTN_START) == LOW);

  if (startDown) {
    if (startPressStart == 0) startPressStart = millis();
    if (!startLongFired && (millis() - startPressStart > 800)) {
      startLongFired = true;
      // Long press: reserved (maze mode toggle placeholder)
    }
  } else {
    if (startPressStart > 0) {
      unsigned long held = millis() - startPressStart;
      if (!startLongFired && held > 50) {           // short press
        if (robotState == STATE_IDLE && isCalibrated) {
          robotState = STATE_RUNNING;
          lastDirection = STRAIGHT;
          previous_error = 0;
          readSensors();
        } else if (robotState == STATE_RUNNING) {
          robotState = STATE_IDLE;
          stopMotors();
        }
      }
      startPressStart = 0;
      startLongFired  = false;
    }
  }
}

// ─────────────────────────────────────────────
//  DISPLAY
// ─────────────────────────────────────────────
void showStartScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 4);  display.print("KAIROS  v2");
  display.setCursor(0,  20); display.print("CAL:  short=sweep");
  display.setCursor(0,  32); display.print("CAL:  long =motor");
  display.setCursor(0,  44); display.print("START:short=run");
  display.display();
}

void updateDisplay() {
  if (millis() - lastDisplayUpdate < DISPLAY_INTERVAL_MS) return;
  lastDisplayUpdate = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // State label
  display.setCursor(0, 0);
  switch (robotState) {
    case STATE_RUNNING:    display.print("RUN  "); break;
    case STATE_CALIBRATING:display.print("CAL  "); break;
    case STATE_MOTOR_TEST: display.print("TEST "); break;
    default:               display.print("IDLE "); break;
  }
  display.print(isCalibrated ? "OK" : "--");

  // Sensor bitmap bar (16 squares across top)
  for (int i = 0; i < NUM_SENSORS; i++) {
    int x = i * 8;
    if (sensorDigital[i]) display.fillRect(x, 14, 6, 6, SSD1306_WHITE);
    else                  display.drawRect(x, 14, 6, 6, SSD1306_WHITE);
  }

  // Error + direction
  display.setCursor(0, 26);
  display.print("err:");
  display.print((int)error);
  display.print("  ");
  switch (lastDirection) {
    case LEFT:     display.print("<L"); break;
    case RIGHT:    display.print("R>"); break;
    default:       display.print("--"); break;
  }

  // Motor speeds
  display.setCursor(0, 38);
  display.print("L:");  display.print(motorL);
  display.print(" R:"); display.print(motorR);

  // Position bar
  int xPos = map((int)line_position, WeightValue[0], WeightValue[15], 0, 127);
  xPos = constrain(xPos, 0, 127);
  display.drawLine(0,    56, 127, 56, SSD1306_WHITE);   // baseline
  display.drawLine(63,   56, 63,  63, SSD1306_WHITE);   // center tick
  display.drawLine(xPos, 54, xPos, 63, SSD1306_WHITE);  // position cursor

  // Calibration progress bar
  if (robotState == STATE_CALIBRATING) {
    display.setCursor(0, 50);
    display.print("CAL ");
    int pct = (int)((millis() - calStartTime) * 100UL / CAL_DURATION_MS);
    pct = constrain(pct, 0, 100);
    display.print(pct);
    display.print("%  SWEEP NOW");
  }

  display.display();
}
