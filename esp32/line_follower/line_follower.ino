/*
============================================================================
  LINE FOLLOWER v2  —  ESP32 (Bluetooth) — proportional + tunable center
============================================================================
  FIXES from v1 (based on testing):
    1. TRUE PROPORTIONAL: correction now scales smoothly with how far off
       the robot is. Far from center = strong steer, near center = gentle.
       (v1 felt "constant" because error was too quantized.)
    2. CENTER OFFSET: if it parks off to one side, nudge the true-center
       with 'o' / 'p' keys until it sits where you want.
    3. SMOOTHER: error is low-pass filtered so steering isn't jerky.

  YOUR ROBOT'S CALIBRATION:
    Servo neutral=86, MAX_LEFT=98, MAX_RIGHT=74
    Sensors S1=34 S2=35 S3=36 S4=39 (left to right)

  COMMANDS:
    g  GO        x STOP        f flip polarity      m sensor readings
    + / -  steering strength (gain)
    o / p  shift center LEFT / RIGHT (fixes parking off to one side)
    < / >  less / more smoothing
    [ / ]  slower / faster
    ?  status
============================================================================
*/

#include <ESP32Servo.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ===== SERVO + LIMITS =====
const int SERVO_PIN  = 4;
const int NEUTRAL    = 86;
const int MAX_LEFT   = 98;   // higher angle = left
const int MAX_RIGHT  = 74;   // lower angle = right
const int PULSE_MIN_US = 500;
const int PULSE_MAX_US = 2500;
Servo steering;
int lastServoWrite = -1;

// ===== LINE SENSORS =====
const int S_PIN[4] = {34, 35, 36, 39};
bool lineIsHigh = true;      // toggle with 'f' if it steers wrong way

// ===== MOTORS =====
#define ENA 25
#define ENB 26
#define IN1 16
#define IN2 17
#define IN3 22
#define IN4 23
const int pwmFreq = 1000;
const int pwmResolution = 8;
int dcSpeed = 130;

// ===== CONTROL =====
bool   following   = false;
float  steerGain   = 8.0;    // proportional gain (how hard to correct)
float  centerOffset = 0.0;   // shift "true center" if it parks off-side
float  smoothing   = 0.4;    // 0=no smoothing(jerky) ... 0.9=very smooth(laggy)
float  filteredError = 0.0;  // low-pass filtered error
float  lastError   = 0.0;
int    lostCounter = 0;
const int LOST_LIMIT = 25;

// Sensor position weights (left negative, right positive)
const float WEIGHT[4] = {-3.0, -1.0, +1.0, +3.0};

void stopDC() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0); ledcWrite(ENB, 0);
}
void driveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(ENA, dcSpeed); ledcWrite(ENB, dcSpeed);
}

void writeServo(int a) {
  a = constrain(a, MAX_RIGHT, MAX_LEFT);
  if (a != lastServoWrite) { steering.write(a); lastServoWrite = a; }
}

// Returns true if line seen; sets *error to position (with center offset applied)
bool readLine(float *error) {
  int onCount = 0;
  float sum = 0;
  for (int i = 0; i < 4; i++) {
    int raw = digitalRead(S_PIN[i]);
    bool onLine = lineIsHigh ? (raw == 1) : (raw == 0);
    if (onLine) { sum += WEIGHT[i]; onCount++; }
  }
  if (onCount == 0) return false;
  *error = (sum / onCount) - centerOffset;   // apply center correction
  return true;
}

void printStatus() {
  SerialBT.print("[status] follow="); SerialBT.print(following ? "YES" : "no");
  SerialBT.print(" pol="); SerialBT.print(lineIsHigh ? "1" : "0");
  SerialBT.print(" gain="); SerialBT.print(steerGain);
  SerialBT.print(" center="); SerialBT.print(centerOffset);
  SerialBT.print(" smooth="); SerialBT.print(smoothing);
  SerialBT.print(" speed="); SerialBT.println(dcSpeed);
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("RobotTuner");

  ESP32PWM::allocateTimer(0);
  steering.setPeriodHertz(50);
  steering.attach(SERVO_PIN, PULSE_MIN_US, PULSE_MAX_US);
  writeServo(NEUTRAL);

  for (int i = 0; i < 4; i++) pinMode(S_PIN[i], INPUT);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcAttach(ENA, pwmFreq, pwmResolution);
  ledcAttach(ENB, pwmFreq, pwmResolution);
  stopDC();

  SerialBT.println("=== LINE FOLLOWER v2 ===");
  SerialBT.println("g go x stop f flip m sensors | +/- gain o/p center </> smooth [ ] speed");
  printStatus();
}

void loop() {
  if (SerialBT.available()) {
    char c = SerialBT.read();
    switch (c) {
      case 'g': following = true;  filteredError = 0; SerialBT.println(">> GO"); break;
      case 'x': following = false; stopDC(); writeServo(NEUTRAL);
                SerialBT.println(">> STOP"); break;
      case 'f': lineIsHigh = !lineIsHigh;
                SerialBT.print(">> polarity="); SerialBT.println(lineIsHigh?"1":"0"); break;
      case '+': steerGain += 1.0; printStatus(); break;
      case '-': steerGain = max(0.0f, steerGain - 1.0f); printStatus(); break;
      case 'o': centerOffset -= 0.25; printStatus(); break;   // shift center left
      case 'p': centerOffset += 0.25; printStatus(); break;   // shift center right
      case '<': smoothing = max(0.0f, smoothing - 0.1f); printStatus(); break;
      case '>': smoothing = min(0.9f, smoothing + 0.1f); printStatus(); break;
      case '[': dcSpeed = max(0, dcSpeed - 10);
                SerialBT.print(">> speed="); SerialBT.println(dcSpeed); break;
      case ']': dcSpeed = min(255, dcSpeed + 10);
                SerialBT.print(">> speed="); SerialBT.println(dcSpeed); break;
      case '?': printStatus(); break;
      case 'm': {
        SerialBT.print("sensors: ");
        for (int i = 0; i < 4; i++) { SerialBT.print(digitalRead(S_PIN[i])); SerialBT.print(" "); }
        float e; bool s = readLine(&e);
        SerialBT.print("  error="); SerialBT.print(s ? String(e) : "none");
        SerialBT.println();
        break;
      }
      default: break;
    }
  }

  if (following) {
    float error;
    bool seen = readLine(&error);

    if (seen) {
      lostCounter = 0;
      lastError = error;
    } else {
      lostCounter++;
      error = lastError;
      if (lostCounter > LOST_LIMIT) {
        stopDC(); writeServo(NEUTRAL);
        SerialBT.println(">> LINE LOST - stopped");
        following = false;
        return;
      }
    }

    // Low-pass filter: smooth the error so steering isn't jerky.
    // filteredError moves toward error; 'smoothing' controls how slowly.
    filteredError = smoothing * filteredError + (1.0 - smoothing) * error;

    // TRUE PROPORTIONAL: angle change scales with how far off we are.
    // Bigger filteredError -> bigger correction. Near center -> gentle.
    int angle = NEUTRAL - (int)(steerGain * filteredError);
    writeServo(angle);

    driveForward();
  }
}
