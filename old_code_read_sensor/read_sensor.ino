/*
 * ============================================================
 *  KAIROS — 16-Sensor MUX Line Follower  (Arduino Nano)
 *  Corrected & production-ready build
 *
 *  Hardware:
 *    Motor Driver : TB6612FNG
 *    Sensor Array : HD16 Analog (CD74HC4067 MUX)
 *    Display      : SSD1306 0.96" OLED (I2C)
 *    Buttons      : BTN_CAL (A1), BTN_START (A2)
 *    MCU          : Arduino Nano (ATmega328P)
 *
 *  Pin Map (matches your hardware doc exactly):
 *    PWMA  → D10    PWMB  → D11
 *    AIN1  → D2     AIN2  → D4
 *    BIN1  → D3     BIN2  → D5
 *    MUX_SIG → A0   S0→A3  S1→D12  S2→D8  S3→D7
 *    BTN_CAL → A1   BTN_START → A2
 *    OLED SDA → A4  OLED SCL → A5
 * ============================================================
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ─────────────────────────────────────────────
//  HARDWARE PINS
// ─────────────────────────────────────────────
#define PWMA        10
#define PWMB        11
#define AIN1         2
#define AIN2         4
#define BIN1         3
#define BIN2         5

#define MUX_SIG     A0
#define S0          A3
#define S1          12
#define S2           8
#define S3           7

#define BTN_CAL     A1
#define BTN_START   A2

// ─────────────────────────────────────────────
//  OLED
// ─────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ─────────────────────────────────────────────
//  TUNING  — adjust these on the track
// ─────────────────────────────────────────────
float Kp            = 6.0;
float Kd            = 40.0;          // safe default; won't overflow EEPROM if stored as float
int   BASE_SPEED    = 110;
int   ROTATE_SPEED  =  90;
int   MAX_SPEED     = 255;

// Intersection push-through timeout (ms).
// Tune with distance_calibration() before competition.
#define PUSHTHROUGH_MS   120          // ~4 cm @ 0.35 m/s — measure & replace

// MUX settle time (µs) — increase if ghost readings appear on long sensor wires
#define MUX_SETTLE_US    100

// Intersection watchdog — exit deadlock after this many ms
#define INTERSECTION_TIMEOUT_MS  300

// ─────────────────────────────────────────────
//  SENSOR ARRAYS  (16 sensors)
// ─────────────────────────────────────────────
#define NUM_SENSORS 16

// Position weights:  sensor 0 = leftmost (-75), sensor 15 = rightmost (+75)
// Center = 0.  Negative = left of centre, positive = right.
const int WeightValue[NUM_SENSORS] = {
  -75, -65, -55, -45, -35, -25, -15, -5,
    5,  15,  25,  35,  45,  55,  65,  75
};

// center_position = 0 (because weights are symmetric around zero)
const float CENTER_POSITION = 0.0f;

int  sensorRaw[NUM_SENSORS];
int  sensorMin[NUM_SENSORS];
int  sensorMax[NUM_SENSORS];
bool sensorDigital[NUM_SENSORS];

// Calibrated threshold: per-sensor mid-point
int  sensorThreshold[NUM_SENSORS];

// ─────────────────────────────────────────────
//  PID STATE
// ─────────────────────────────────────────────
float line_position   = CENTER_POSITION;
float error           = 0;
float previous_error  = 0;
float derivative      = 0;

int   motorL = 0;
int   motorR = 0;

// ─────────────────────────────────────────────
//  DIRECTION / TURN STATE
// ─────────────────────────────────────────────
enum Dir { STRAIGHT, LEFT, RIGHT };
Dir lastDirection = STRAIGHT;

// ─────────────────────────────────────────────
//  DERIVED SENSOR STATS (filled by readSensors)
// ─────────────────────────────────────────────
int      sumOnSensor = 0;   // number of active (black) sensors
long     sensorWight = 0;   // weighted sum for position calculation
uint16_t bitSensor   = 0;   // bitmask of active sensors (bit 0 = sensor 0)

// ─────────────────────────────────────────────
//  ROBOT STATE MACHINE
// ─────────────────────────────────────────────
enum RobotState { STATE_IDLE, STATE_CALIBRATING, STATE_RUNNING, STATE_MOTOR_TEST };
RobotState robotState = STATE_IDLE;

bool isCalibrated = false;

// Calibration sweep timer
unsigned long calStartTime = 0;
#define CAL_DURATION_MS 5000

// Motor test state
unsigned long motorTestStart = 0;
int           motorTestPhase = 0;

// ─────────────────────────────────────────────
//  DISPLAY REFRESH RATE
// ─────────────────────────────────────────────
unsigned long lastDisplayUpdate = 0;
#define DISPLAY_INTERVAL_MS 150

// ─────────────────────────────────────────────
//  FUNCTION PROTOTYPES
// ─────────────────────────────────────────────
void showStartScreen();
void handleButtons();
void updateDisplay();
void readSensors();
void startCalibration();
void runCalibration();
void PID_Controller();
void stopMotors();
void startMotorTest();
void runMotorTest();
void applyMotors();
void executeTurn(Dir dir);
bool centerOnLine();
void pushForward(unsigned long pushMs);
void handleIntersection();
void printSensorBar();

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Motor driver pins
  pinMode(PWMA,  OUTPUT); pinMode(PWMB,  OUTPUT);
  pinMode(AIN1,  OUTPUT); pinMode(AIN2,  OUTPUT);
  pinMode(BIN1,  OUTPUT); pinMode(BIN2,  OUTPUT);
  stopMotors();

  // MUX select pins
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);

  // Buttons
  pinMode(BTN_CAL,   INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);

  // Sensor defaults
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i]       = 1023;
    sensorMax[i]       = 0;
    sensorThreshold[i] = 512;   // sane default before calibration
  }

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // If OLED missing, continue anyway — don't hang
    while (1) { delay(100); }
  }
  showStartScreen();
}

// ─────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────
void loop() {
  handleButtons();

  switch (robotState) {

    case STATE_CALIBRATING:
      runCalibration();
      break;

    case STATE_RUNNING:
      PID_Controller();
      break;

    case STATE_MOTOR_TEST:
      runMotorTest();
      break;

    case STATE_IDLE:
    default:
      stopMotors();
      break;
  }

  updateDisplay();
}
