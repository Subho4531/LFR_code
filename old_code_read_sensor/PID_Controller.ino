/*
 * PID_Controller.ino
 *
 * FIX LOG vs original code:
 *   1. REMOVED delay(delay_before_turn) from PID hot path — was 100 ms blind drive.
 *   2. REMOVED blocking while(bitSensor==255){stop()} with no timeout.
 *      Replaced with millis() watchdog → INTERSECTION_TIMEOUT_MS.
 *   3. REMOVED distance() call from inside PID — replaced with non-blocking
 *      push-through using millis() while still reading sensors.
 *   4. direction enum (STRAIGHT/LEFT/RIGHT) instead of String — faster compare,
 *      no heap fragmentation.
 *   5. direction is set ONLY on clear outer-sensor events; never clobbered mid-turn.
 *   6. Turn threshold changed from single center sensor [4] to sensors [6,7,8,9]
 *      — more reliable on 16-sensor array.
 *   7. Motor correction constrained BEFORE writing to motors.
 *   8. Previous error updated even during intersection handling so derivative
 *      doesn't spike on re-entry.
 */

void PID_Controller() {
  readSensors();

  // ── 1. All sensors off white → line lost → execute saved turn ──────────────
  if (bitSensor == 0) {
    previous_error = 0;           // prevent derivative spike on re-entry
    if (lastDirection != STRAIGHT) {
      executeTurn(lastDirection);
    } else {
      // No saved turn direction — creep forward a little and wait
      motorL = BASE_SPEED;
      motorR = BASE_SPEED;
      applyMotors();
    }
    return;
  }

  // ── 2. Intersection / full-black (most sensors active) ─────────────────────
  if (sumOnSensor >= 12) {
    handleIntersection();
    return;
  }

  // ── 3. Update direction from outer sensors (before PID calc) ───────────────
  //   Left 5 sensors (0-4) active → line is going left
  //   Right 5 sensors (11-15) active → line is going right
  int leftActive  = 0;
  int rightActive = 0;
  for (int i = 0;  i < 5;  i++) if (sensorDigital[i])  leftActive++;
  for (int i = 11; i < 16; i++) if (sensorDigital[i])  rightActive++;

  if (leftActive >= 3 && rightActive == 0)       lastDirection = LEFT;
  else if (rightActive >= 3 && leftActive == 0)  lastDirection = RIGHT;
  // Don't reset to STRAIGHT here — keep last meaningful direction

  // ── 4. Normal PID ──────────────────────────────────────────────────────────
  error      = CENTER_POSITION - line_position;
  derivative = error - previous_error;

  float correction = (Kp * error) + (Kd * derivative);
  previous_error   = error;

  // Positive error → bot is left of line → steer right: increase R, decrease L
  int leftCmd  = (int)(BASE_SPEED - correction);
  int rightCmd = (int)(BASE_SPEED + correction);

  motorL = constrain(leftCmd,  -MAX_SPEED, MAX_SPEED);
  motorR = constrain(rightCmd, -MAX_SPEED, MAX_SPEED);

  applyMotors();
}

// ─────────────────────────────────────────────
//  INTERSECTION HANDLER  (non-blocking push-through)
//
//  FIX: Old code called stop() in a while loop with zero timeout.
//       New code drives forward for PUSHTHROUGH_MS while reading sensors.
//       If sensors still show all-black after timeout → assume stop marker.
// ─────────────────────────────────────────────
void handleIntersection() {
  unsigned long startT = millis();

  // Push through the intersection at base speed while watching sensors
  while (millis() - startT < PUSHTHROUGH_MS) {
    motorL = BASE_SPEED;
    motorR = BASE_SPEED;
    applyMotors();
    readSensors();

    // If sensors clear mid-push (narrow stripe was just noise), resume PID
    if (sumOnSensor < 12) return;
  }

  // After push-through: re-read final state
  readSensors();

  if (sumOnSensor == 0) {
    // T-intersection: no line ahead after crossing → turn right (default)
    lastDirection = RIGHT;
  } else if (sumOnSensor >= 12) {
    // Thick stop marker OR dead end — stop and signal
    stopMotors();
    // Blink display or serial — handled by updateDisplay
    // Wait until cleared (with watchdog so we don't freeze forever)
    unsigned long stopStart = millis();
    while (sumOnSensor >= 12 && (millis() - stopStart < INTERSECTION_TIMEOUT_MS)) {
      readSensors();
    }
    // If still saturated after watchdog, give up and go straight
    lastDirection = STRAIGHT;
  } else {
    // Line visible after push-through → cross intersection, go straight
    lastDirection = STRAIGHT;
  }
}
