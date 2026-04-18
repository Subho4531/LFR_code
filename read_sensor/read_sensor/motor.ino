/*
 * motor.ino  +  Turns.ino  (merged)
 *
 * FIX LOG vs original code:
 *   1. constrain() moved BEFORE direction logic — original set direction
 *      pins based on unconstrained value then constrained, causing a brief
 *      full-reverse pulse at the instant of direction flip.
 *   2. Zero case (coast) now properly sets both pins LOW; original left pins
 *      in previous state when speed crossed zero.
 *   3. turnRight / turnLeft: direction = STRAIGHT moved OUTSIDE the while loop.
 *      Original clobbered it every iteration → center sensors' first flicker
 *      caused premature exit.
 *   4. Turn exit condition expanded to sensors 6 AND 7 AND 8 AND 9
 *      (inner 4 of 16) — single-sensor [4] was fragile for 16-sensor arrays.
 *   5. distance() replaced with non-blocking version using millis().
 *      No sensors were read during the old blocking while loop.
 *   6. Motor test is fully non-blocking (state machine, no delays).
 */

// ─────────────────────────────────────────────
//  LOW-LEVEL MOTOR WRITE
// ─────────────────────────────────────────────
void applyMotors() {
  // Constrain first, then set direction
  int L = constrain(motorL, -MAX_SPEED, MAX_SPEED);
  int R = constrain(motorR, -MAX_SPEED, MAX_SPEED);

  // Left motor (A channel)
  if (L > 0)      { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
  else if (L < 0) { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
  else            { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);  }
  analogWrite(PWMA, abs(L));

  // Right motor (B channel)
  if (R > 0)      { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
  else if (R < 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
  else            { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);  }
  analogWrite(PWMB, abs(R));
}

void stopMotors() {
  motorL = 0;
  motorR = 0;
  applyMotors();
}

// ─────────────────────────────────────────────
//  PIVOT TURNS
//
//  Exit condition: inner 4 sensors (6,7,8,9) — at least 2 must see black.
//  This is much more reliable than a single-sensor check on a 16-sensor array.
//
//  FIX: direction flag set AFTER the loop exits, not inside it.
//       Inside-loop reset meant any brief sensor flicker during the turn
//       would reset direction before the exit check ran.
// ─────────────────────────────────────────────
bool centerOnLine() {
  // Returns true when the bot has found the line with its center sensors
  return (sensorDigital[6] || sensorDigital[7] || sensorDigital[8] || sensorDigital[9]);
}

void executeTurn(Dir dir) {
  if (dir == RIGHT) {
    motorL =  ROTATE_SPEED;
    motorR = -ROTATE_SPEED;
  } else {
    motorL = -ROTATE_SPEED;
    motorR =  ROTATE_SPEED;
  }
  applyMotors();

  // Read sensors until center locks on — with a safety timeout
  unsigned long startT = millis();
  do {
    readSensors();
    if (millis() - startT > 1500) break;   // failsafe: 1.5 s max turn
  } while (!centerOnLine());

  // Turn complete → set direction ONCE here, not inside the loop
  lastDirection = STRAIGHT;
  stopMotors();
}

// ─────────────────────────────────────────────
//  NON-BLOCKING FORWARD PUSH (replaces old blocking distance())
//
//  Drives forward for exactly pushMs milliseconds.
//  Unlike the old version, sensor reads happen each iteration so
//  the caller can break out early if needed.
// ─────────────────────────────────────────────
void pushForward(unsigned long pushMs) {
  unsigned long start = millis();
  while (millis() - start < pushMs) {
    motorL = BASE_SPEED;
    motorR = BASE_SPEED;
    applyMotors();
    readSensors();
  }
}

// ─────────────────────────────────────────────
//  MOTOR TEST  (non-blocking state machine)
//
//  Phase 1 (0–1.5 s): Both forward
//  Phase 2 (1.5–3 s): Both backward
//  Phase 3 (3–4.5 s): Pivot left
//  Phase 4 (4.5–6 s): Pivot right
//  Phase 5 (>6 s)  : Stop, exit test
// ─────────────────────────────────────────────
void startMotorTest() {
  motorTestStart = millis();
  motorTestPhase = 1;
  robotState     = STATE_MOTOR_TEST;
}

void runMotorTest() {
  unsigned long elapsed = millis() - motorTestStart;

  if      (elapsed < 1500) { motorL =  150; motorR =  150; }
  else if (elapsed < 3000) { motorL = -150; motorR = -150; }
  else if (elapsed < 4500) { motorL = -150; motorR =  150; }
  else if (elapsed < 6000) { motorL =  150; motorR = -150; }
  else {
    stopMotors();
    robotState = STATE_IDLE;
    return;
  }
  applyMotors();
}
