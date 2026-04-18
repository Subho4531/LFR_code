/*
 * read_sensor.ino
 * Handles MUX read, calibration sweep, and sensor digitization.
 *
 * FIX LOG vs original code:
 *   1. MUX settle time raised to MUX_SETTLE_US (100 µs default) to avoid ghost
 *      readings from long sensor wires.
 *   2. Per-sensor calibrated threshold replaces single global threshold.
 *      Each sensor's midpoint = (min+max)/2 after sweep.
 *   3. Hardcoded 200/300 clamp removed — map() + constrain() handles it correctly.
 *   4. sensorNumber → NUM_SENSORS (16, not 8).
 *   5. WeightValue weights are now symmetric around 0 (defined in main file).
 *   6. bitSensor uses uint16_t and correct bit packing for 16 sensors.
 *   7. line_position only updated when 1–8 sensors active to avoid intersection
 *      noise.  Threshold raised from 2→8 (out of 16).
 */

// ─────────────────────────────────────────────
//  MUX READ  (single channel, analog)
// ─────────────────────────────────────────────
int readMux(int ch) {
  // Standard binary decode — no bit-reversal needed if wiring matches
  digitalWrite(S0, (ch >> 0) & 1);
  digitalWrite(S1, (ch >> 1) & 1);
  digitalWrite(S2, (ch >> 2) & 1);
  digitalWrite(S3, (ch >> 3) & 1);
  delayMicroseconds(MUX_SETTLE_US);   // let MUX + RC settle
  return analogRead(MUX_SIG);
}

// ─────────────────────────────────────────────
//  READ ALL SENSORS → fill global stats
// ─────────────────────────────────────────────
void readSensors() {
  sumOnSensor = 0;
  sensorWight = 0;
  bitSensor   = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorRaw[i] = readMux(i);

    // Map raw ADC to 0–100 using per-sensor calibrated range
    int mapped = map(sensorRaw[i], sensorMin[i], sensorMax[i], 0, 100);
    mapped = constrain(mapped, 0, 100);

    // Digital: 1 = black line detected
    bool isBlack = isCalibrated
                     ? (mapped >= 50)
                     : (sensorRaw[i] >= sensorThreshold[i]);

    sensorDigital[i] = isBlack;

    if (isBlack) {
      sumOnSensor++;
      sensorWight += WeightValue[i];
      bitSensor   |= (1U << i);
    }
  }

  // Update line position ONLY on a clean narrow line (1–8 sensors active).
  // Freeze on intersections / wide coverage so PID uses last good position.
  if (sumOnSensor >= 1 && sumOnSensor <= 8) {
    line_position = (float)sensorWight / sumOnSensor;
  }
  // If sumOnSensor == 0 OR >= 9: keep last line_position (intentional freeze)
}

// ─────────────────────────────────────────────
//  CALIBRATION SWEEP  (non-blocking, called from loop)
// ─────────────────────────────────────────────
void startCalibration() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }
  calStartTime  = millis();
  isCalibrated  = false;
  robotState    = STATE_CALIBRATING;
}

void runCalibration() {
  // Keep sweeping min/max for every sensor
  for (int i = 0; i < NUM_SENSORS; i++) {
    int v = readMux(i);
    if (v < sensorMin[i]) sensorMin[i] = v;
    if (v > sensorMax[i]) sensorMax[i] = v;
  }

  if (millis() - calStartTime >= CAL_DURATION_MS) {
    // Compute per-sensor thresholds
    for (int i = 0; i < NUM_SENSORS; i++) {
      // Guard against a sensor that never moved (flat environment)
      if (sensorMax[i] - sensorMin[i] < 50) {
        sensorThreshold[i] = 512;   // fallback
      } else {
        sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
      }
    }
    isCalibrated = true;
    robotState   = STATE_IDLE;
  }
}

// ─────────────────────────────────────────────
//  SERIAL DEBUG HELPERS
// ─────────────────────────────────────────────
void printSensorBar() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorDigital[i] ? "X" : "_");
  }
  Serial.print("  pos:");
  Serial.print(line_position, 1);
  Serial.print("  err:");
  Serial.println(error, 1);
}
