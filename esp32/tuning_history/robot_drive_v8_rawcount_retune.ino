/*
============================================================================
  ROBOT DRIVE v8 — RAW ENCODER DISTANCE RETUNE
============================================================================
  Purpose of this version:
    - Use the SAME encoder-reading style as the working measurement sketch.
    - Remove the confusing estX / turn-loss distance layer while retuning.
    - Default to STRAIGHT distance drive: servo neutral, no line-lost stop.
    - Distance = abs(encoder_count) / PULSES_PER_CM.

  This is the version to use now to finish the distance step.

  WIRING:
    Encoder Yellow(A)->GPIO32   Red->3.3V   Black->GND
    Servo->4 | L298N ENA25 ENB26 IN1 16 IN2 17 IN3 22 IN4 23
    Line sensors optional: S1=34 S2=35 S3=36 S4=39

  COMMANDS:
    DRIVE: 1=10cm 2=20cm 3=30cm ... 9=90cm, g=drive target, x=stop
    DIST : d/D target -/+10cm
    SPEED: [/] cruise -/+5, k/K kick power -/+5, n/N kick cm -/+0.5
    RAMP : ,/. ramp-down cm -/+2
    END  : h/H crawl -/+5, a/A minMove -/+5, s/S stopEarly -/+0.1
    ENCODER: e/E ppcm -/+0.05, z/Z ppcm -/+0.50, r=reset, c=count
    LINE : l toggle line-follow mode. DEFAULT OFF for distance tuning.
    INFO : m live measure, ? status
============================================================================
*/

#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include "driver/pulse_cnt.h"

BluetoothSerial SerialBT;

// ===== ENCODER: copied style from your working measurement code =====
const int ENC_PIN   = 32;
const int GLITCH_NS = 200;
pcnt_unit_handle_t    pcnt_unit = NULL;
pcnt_channel_handle_t pcnt_chan = NULL;

int readCount() {
  int c = 0;
  pcnt_unit_get_count(pcnt_unit, &c);
  return c;
}

void resetCount() {
  pcnt_unit_clear_count(pcnt_unit);
}

long absCount() {
  long c = (long)readCount();
  return (c < 0) ? -c : c;
}

// ===== CALIBRATION =====
float PULSES_PER_CM = 8.60;   // retune after encoder filter = 200
int   targetDistCm  = 30;

// Drive profile
int   cruisePower   = 155;
int   kickPower     = 165;
float kickDistCm    = 1.5;
int   rampDownCm    = 6;
int   crawlPower    = 120;
int   minMovePower  = 110;
float stopEarlyCm   = 1.50;

// Safety / anti-stall
int   unstickPower  = 185;
int   stallAfterMs  = 700;
int   stallKickMs   = 220;
int   brakeMs       = 0;

// ===== SERVO =====
const int SERVO_PIN = 4;
const int NEUTRAL   = 86;
const int MAX_LEFT  = 98;
const int MAX_RIGHT = 74;
Servo steering;
int lastServo = -1;

// ===== OPTIONAL LINE FOLLOWING =====
const int S_PIN[4] = {34, 35, 36, 39};
const float WEIGHT[4] = {-3, -1, +1, +3};
bool lineFollowEnabled = false;   // IMPORTANT: OFF by default for distance tuning
bool lineIsHigh = false;
float steerGain = 12.0;
float smoothing = 0.30;
float centerOffset = 0.0;
float filtErr = 0.0;
float lastErr = 0.0;
int lostCount = 0;
const int LOST_LIMIT = 25;

// ===== MOTORS =====
#define ENA 25
#define ENB 26
#define IN1 16
#define IN2 17
#define IN3 22
#define IN4 23
const int pwmFreq = 1000;
const int pwmRes  = 8;

// ===== STATE =====
bool driving = false;
float cutMotorAtCm = 0.0;
long stallWatchPulses = 0;
unsigned long stallWatchMs = 0;
unsigned long unstickUntilMs = 0;

float maxf2(float a, float b) { return (a > b) ? a : b; }
float minf2(float a, float b) { return (a < b) ? a : b; }

int effectiveCruisePower() {
  return constrain(max(cruisePower, minMovePower), 0, 255);
}

int effectiveApproachPower() {
  int c = effectiveCruisePower();
  int p = max(crawlPower, minMovePower);
  if (p > c) p = c;
  return constrain(p, 0, 255);
}

float effectiveRampDownCm() {
  return minf2((float)rampDownCm, maxf2(2.0, cutMotorAtCm * 0.45));
}

float rawDistanceCm() {
  return ((float)absCount()) / PULSES_PER_CM;
}

void writeServo(int a) {
  a = constrain(a, MAX_RIGHT, MAX_LEFT);
  if (a != lastServo) {
    steering.write(a);
    lastServo = a;
  }
}

void motorsRaw(int power) {
  power = constrain(power, 0, 255);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(ENA, power);
  ledcWrite(ENB, power);
}

void motorsOff() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

void motorsBrakeThenOff() {
  if (brakeMs > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
    ledcWrite(ENA, 255);
    ledcWrite(ENB, 255);
    delay(brakeMs);
  }
  motorsOff();
}

bool readLine(float *err) {
  int on = 0;
  float sum = 0.0;
  for (int i = 0; i < 4; i++) {
    int raw = digitalRead(S_PIN[i]);
    bool onLine = lineIsHigh ? (raw == 1) : (raw == 0);
    if (onLine) {
      sum += WEIGHT[i];
      on++;
    }
  }
  if (on == 0) return false;
  *err = (sum / on) - centerOffset;
  return true;
}

void updateSteering() {
  if (!lineFollowEnabled) {
    writeServo(NEUTRAL);
    return;
  }

  float err;
  bool seen = readLine(&err);
  if (seen) {
    lostCount = 0;
    lastErr = err;
  } else {
    // Do NOT stop the distance run just because line is missing.
    // Hold last error shortly, then go neutral.
    lostCount++;
    err = (lostCount > LOST_LIMIT) ? 0.0 : lastErr;
  }

  filtErr = smoothing * filtErr + (1.0 - smoothing) * err;
  int servoCmd = NEUTRAL - (int)(steerGain * filtErr);
  writeServo(servoCmd);
}

int computeDrivePower(float doneCm, float remCm) {
  int baseCruise = effectiveCruisePower();
  int approach = effectiveApproachPower();
  float rampCm = effectiveRampDownCm();

  int power;
  if (doneCm < kickDistCm) {
    float t = (kickDistCm > 0.0) ? (doneCm / kickDistCm) : 1.0;
    power = kickPower + (int)((baseCruise - kickPower) * t);
  } else if (remCm < rampCm) {
    float t = (rampCm > 0.0) ? (remCm / rampCm) : 0.0;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    power = approach + (int)((baseCruise - approach) * t);
  } else {
    power = baseCruise;
  }

  if (millis() < unstickUntilMs) power = max(power, unstickPower);
  return constrain(power, 0, 255);
}

void stopDriveAndReport(const char *reason) {
  float rawCm = rawDistanceCm();
  long pulses = absCount();
  driving = false;
  motorsBrakeThenOff();
  writeServo(NEUTRAL);

  SerialBT.print(">> "); SerialBT.print(reason);
  SerialBT.print(" raw="); SerialBT.print(rawCm, 2);
  SerialBT.print("cm cutAt="); SerialBT.print(cutMotorAtCm, 2);
  SerialBT.print("cm pulses="); SerialBT.println(pulses);
}

void startDrive(int cm) {
  targetDistCm = cm;

  // IMPORTANT FIX: use the working measurement style.
  // Reset the PCNT counter at the start, then distance is simply count / ppcm.
  resetCount();

  cutMotorAtCm = maxf2(0.0, ((float)targetDistCm) - stopEarlyCm);
  filtErr = 0.0;
  lastErr = 0.0;
  lostCount = 0;
  stallWatchPulses = 0;
  stallWatchMs = millis();
  unstickUntilMs = 0;

  writeServo(NEUTRAL);
  driving = true;

  SerialBT.print(">> DRIVE "); SerialBT.print(targetDistCm);
  SerialBT.print("cm | cut motors at raw="); SerialBT.print(cutMotorAtCm, 2);
  SerialBT.print("cm | stopEarly="); SerialBT.print(stopEarlyCm, 2);
  SerialBT.print("cm | ppcm="); SerialBT.print(PULSES_PER_CM, 2);
  SerialBT.print(" | effCruise="); SerialBT.print(effectiveCruisePower());
  SerialBT.print(" | approach="); SerialBT.print(effectiveApproachPower());
  SerialBT.print(" | line="); SerialBT.println(lineFollowEnabled ? "ON" : "OFF");
}

void printStatus() {
  SerialBT.print("[status] V8_RAW filtNs="); SerialBT.print(GLITCH_NS);
  SerialBT.print(" dist="); SerialBT.print(targetDistCm);
  SerialBT.print(" cruise="); SerialBT.print(cruisePower);
  SerialBT.print(" kickP="); SerialBT.print(kickPower);
  SerialBT.print(" kickCm="); SerialBT.print(kickDistCm, 1);
  SerialBT.print(" rampDn="); SerialBT.print(rampDownCm);
  SerialBT.print(" crawl="); SerialBT.print(crawlPower);
  SerialBT.print(" minMove="); SerialBT.print(minMovePower);
  SerialBT.print(" effCruise="); SerialBT.print(effectiveCruisePower());
  SerialBT.print(" approach="); SerialBT.print(effectiveApproachPower());
  SerialBT.print(" unstick="); SerialBT.print(unstickPower);
  SerialBT.print(" ppcm="); SerialBT.print(PULSES_PER_CM, 2);
  SerialBT.print(" stopEarly="); SerialBT.print(stopEarlyCm, 2);
  SerialBT.print(" brakeMs="); SerialBT.print(brakeMs);
  SerialBT.print(" line="); SerialBT.print(lineFollowEnabled ? "ON" : "OFF");
  SerialBT.print(" count="); SerialBT.print(readCount());
  SerialBT.print(" raw="); SerialBT.println(rawDistanceCm(), 2);
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("RobotTuner");

  // PCNT encoder: same structure as measure_params_v2.
  pcnt_unit_config_t uc = { .low_limit = -30000, .high_limit = 30000 };
  pcnt_new_unit(&uc, &pcnt_unit);
  pcnt_glitch_filter_config_t fc = { .max_glitch_ns = GLITCH_NS };
  pcnt_unit_set_glitch_filter(pcnt_unit, &fc);
  pcnt_chan_config_t cc = { .edge_gpio_num = ENC_PIN, .level_gpio_num = -1 };
  pcnt_new_channel(pcnt_unit, &cc, &pcnt_chan);
  pcnt_channel_set_edge_action(pcnt_chan,
      PCNT_CHANNEL_EDGE_ACTION_INCREASE,
      PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_unit_enable(pcnt_unit);
  pcnt_unit_clear_count(pcnt_unit);
  pcnt_unit_start(pcnt_unit);

  ESP32PWM::allocateTimer(0);
  steering.setPeriodHertz(50);
  steering.attach(SERVO_PIN, 500, 2500);
  writeServo(NEUTRAL);

  for (int i = 0; i < 4; i++) pinMode(S_PIN[i], INPUT);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcAttach(ENA, pwmFreq, pwmRes);
  ledcAttach(ENB, pwmFreq, pwmRes);
  motorsOff();

  SerialBT.println("=== ROBOT DRIVE v8 RAW-COUNT RETUNE ===");
  SerialBT.println("1=10 2=20 3=30 | g drive | x stop | ? status | l line toggle | e/E,z/Z ppcm");
  printStatus();
}

void handleCommand(char c) {
  if (c >= '1' && c <= '9') {
    startDrive((c - '0') * 10);
    return;
  }

  switch (c) {
    case 'g': case 'G': startDrive(targetDistCm); break;

    case 'x': case 'X':
      driving = false;
      motorsOff();
      writeServo(NEUTRAL);
      SerialBT.print(">> STOP count="); SerialBT.print(readCount());
      SerialBT.print(" raw="); SerialBT.println(rawDistanceCm(), 2);
      break;

    case 'r': resetCount(); SerialBT.println(">> count reset"); break;
    case 'c': SerialBT.print(">> count="); SerialBT.print(readCount());
              SerialBT.print(" raw="); SerialBT.println(rawDistanceCm(), 2); break;

    case 'd': targetDistCm = max(0, targetDistCm - 10); printStatus(); break;
    case 'D': targetDistCm += 10; printStatus(); break;

    case '[': cruisePower = max(0, cruisePower - 5); printStatus(); break;
    case ']': cruisePower = min(255, cruisePower + 5); printStatus(); break;
    case 'k': kickPower = max(0, kickPower - 5); printStatus(); break;
    case 'K': kickPower = min(255, kickPower + 5); printStatus(); break;
    case 'n': kickDistCm -= 0.5; if (kickDistCm < 0) kickDistCm = 0; printStatus(); break;
    case 'N': kickDistCm += 0.5; printStatus(); break;

    case ',': rampDownCm = max(2, rampDownCm - 2); printStatus(); break;
    case '.': rampDownCm = min(40, rampDownCm + 2); printStatus(); break;
    case 'h': crawlPower = max(0, crawlPower - 5); printStatus(); break;
    case 'H': crawlPower = min(255, crawlPower + 5); printStatus(); break;
    case 'a': minMovePower = max(0, minMovePower - 5); printStatus(); break;
    case 'A': minMovePower = min(255, minMovePower + 5); printStatus(); break;

    case 's': stopEarlyCm -= 0.10; if (stopEarlyCm < 0) stopEarlyCm = 0; printStatus(); break;
    case 'S': stopEarlyCm += 0.10; printStatus(); break;

    case 'y': unstickPower = max(0, unstickPower - 5); printStatus(); break;
    case 'Y': unstickPower = min(255, unstickPower + 5); printStatus(); break;
    case 'b': brakeMs = max(0, brakeMs - 10); printStatus(); break;
    case 'B': brakeMs = min(250, brakeMs + 10); printStatus(); break;

    case 'e': PULSES_PER_CM -= 0.05; if (PULSES_PER_CM < 1.0) PULSES_PER_CM = 1.0; printStatus(); break;
    case 'E': PULSES_PER_CM += 0.05; printStatus(); break;
    case 'z': PULSES_PER_CM -= 0.50; if (PULSES_PER_CM < 1.0) PULSES_PER_CM = 1.0; printStatus(); break;
    case 'Z': PULSES_PER_CM += 0.50; printStatus(); break;

    case 'l': case 'L': lineFollowEnabled = !lineFollowEnabled; printStatus(); break;
    case 'f': lineIsHigh = !lineIsHigh; printStatus(); break;
    case '+': steerGain += 1; printStatus(); break;
    case '-': steerGain -= 1; if (steerGain < 0) steerGain = 0; printStatus(); break;
    case 'o': centerOffset -= 0.25; printStatus(); break;
    case 'p': centerOffset += 0.25; printStatus(); break;

    case 'm':
      SerialBT.print(">> count="); SerialBT.print(readCount());
      SerialBT.print(" abs="); SerialBT.print(absCount());
      SerialBT.print(" raw="); SerialBT.print(rawDistanceCm(), 2);
      SerialBT.print(" line="); SerialBT.println(lineFollowEnabled ? "ON" : "OFF");
      break;

    case '?': printStatus(); break;
    default: break;
  }
}

void loop() {
  if (SerialBT.available()) {
    char c = SerialBT.read();
    handleCommand(c);
  }

  if (!driving) return;

  float rawCm = rawDistanceCm();
  long pulses = absCount();
  float remCm = cutMotorAtCm - rawCm;

  if (rawCm >= cutMotorAtCm) {
    stopDriveAndReport("CUT");
    return;
  }

  // Safety: if the scale is very wrong, never run forever.
  if (rawCm > ((float)targetDistCm + 15.0)) {
    stopDriveAndReport("SAFETY");
    return;
  }

  // Anti-stall based directly on raw encoder pulses.
  unsigned long nowMs = millis();
  if (pulses > stallWatchPulses + 2) {
    stallWatchPulses = pulses;
    stallWatchMs = nowMs;
  } else if ((nowMs - stallWatchMs > (unsigned long)stallAfterMs) && (rawCm < cutMotorAtCm - 1.0)) {
    unstickUntilMs = nowMs + (unsigned long)stallKickMs;
    stallWatchMs = nowMs;
    SerialBT.print(">> ANTI-STALL raw="); SerialBT.print(rawCm, 1);
    SerialBT.print(" pwr="); SerialBT.println(unstickPower);
  }

  updateSteering();
  int drivePower = computeDrivePower(rawCm, remCm);
  motorsRaw(drivePower);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 250) {
    SerialBT.print("   count="); SerialBT.print(readCount());
    SerialBT.print(" raw="); SerialBT.print(rawCm, 1);
    SerialBT.print("/"); SerialBT.print(targetDistCm);
    SerialBT.print(" cut="); SerialBT.print(cutMotorAtCm, 1);
    SerialBT.print(" pwr="); SerialBT.print(drivePower);
    SerialBT.print(" line="); SerialBT.println(lineFollowEnabled ? "ON" : "OFF");
    lastPrint = millis();
  }
}
