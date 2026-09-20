/*
============================================================================
  ROBOT DRIVE v10 — V6 good logic + encoder filter 200
============================================================================
  Keeps the V6 good anti-stall/line-distance logic and only fixes the encoder filter.

  Fixes the V5 heavy/stalling problem by keeping final approach power
  above the real minimum moving power and adding anti-stall boost.

  Still separates TWO different errors:

    1) PULSES_PER_CM = scale calibration of the encoder.
       Do NOT tune this just because the robot coasts after stop.

    2) stopEarlyCm = final coast / inertia compensation.
       Robot cuts the motors before the target and lets the remaining coast land
       it near the requested distance.

  Also adds:
    - PCNT glitch filter lowered to 200 ns, matching the working measurement sketch
    - speed-aware stop-early compensation
    - steering-loss compensation, because encoder wheel travel during turns is
      not equal to straight x-axis progress
    - optional active brake time, default 0 ms so behavior starts familiar
    - anti-stall boost if encoder distance stops increasing before target

  Values below are the measured results of the tuning runs:
    current old PPCM = 8.6
    10 cm -> 11.45 cm, 20 cm -> 20.95 cm, 30 cm -> 30 cm
    => true PULSES_PER_CM ≈ 9.27 and coast/stopEarly ≈ 2.25 cm

  WIRING:
    Encoder Yellow(A)->GPIO32   Red->3.3V   Black->GND
    Servo->4 | L298N ENA25 ENB26 IN1 16 IN2 17 IN3 22 IN4 23
    Line S1=34 S2=35 S3=36 S4=39

  COMMANDS:
    DRIVE: 1-9 = drive Nx10cm     g = drive set distance     x = stop
    DIST : d/D distance -/+10cm
    SPEED: [ ] cruise             k/K kick power             n/N kick cm
           a/A minMove            y/Y unstick power
    RAMP : , . ramp-down cm
    FIX  : s/S stopEarly -/+0.1   v/V speedCoast -/+0.005
           u/U turnLoss -/+0.02   b/B brakeMs -/+10   h/H crawl -/+5
    LINE : +/- gain               f flip polarity            o/p center
    ENCODER: e/E PPCM -/+0.05
    INFO : m sensors+count        c count        w manual-drive  ? status
============================================================================
*/

#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include <ArduinoJson.h>
#include "driver/pulse_cnt.h"

BluetoothSerial SerialBT;

// ===== Z AXIS STEPPER (A4988) =====
const int STEP_PIN = 18;
const int DIR_PIN  = 19;
const int Z_ENA_PIN = 21;   // active LOW, set to -1 if tied to GND
const int STEP_PULSE_US = 800;
const bool DIR_UP_LEVEL = HIGH;
const int MAX_STEPS_PER_CMD = 5000;

String jsonLine = "";
int lastCmdId = -1;


// ===== ENCODER (single channel, PCNT + glitch filter) =====
const int ENC_PIN = 32;
const int GLITCH_NS = 200;        // MATCH the working measure_params_v2 encoder filter
pcnt_unit_handle_t    pcnt_unit = NULL;
pcnt_channel_handle_t pcnt_chan = NULL;

long getCount() {
  int c = 0;
  pcnt_unit_get_count(pcnt_unit, &c);
  return (long)c;
}
void resetCount() { pcnt_unit_clear_count(pcnt_unit); }

// ===== CALIBRATION (tunable) =====
float PULSES_PER_CM = 7.52;       // LOCKED tuned value
int   kickPower     = 175;        // short start boost
float kickDistCm    = 4.0;
int   cruisePower   = 140;        // lower effective power caused stalling under load
int   rampDownCm    = 6;          // V5 slowed too early; 12cm was too long for a 20cm move
int   crawlPower    = 125;        // final approach power; must stay high enough to move
int   targetDistCm  = 30;
int   testPower     = 130;        // manual measurement drive power for w/x encoder check

// ===== DISTANCE FIX PARAMETERS =====
float stopEarlyCm       = 1.00;   // after fixing ramp-down, start lower and tune with s/S
float speedCoastPerPWM  = 0.010;  // extra cm per PWM above crawlPower=80
float turnLossGain      = 0.25;   // compensates encoder over-counting during steering
int   brakeMs           = 0;      // optional active brake. Start at 0, increase only if needed.
int   minMovePower      = 120;    // never command below this while still trying to move
int   unstickPower      = 185;    // short boost when encoder count stops before target
int   stallAfterMs      = 700;    // if pulses do not increase for this long, boost
int   stallKickMs       = 250;    // boost duration
float backPowerScale    = 1.40;   // LOCKED (backward needs more push to MOVE)
// ---- separate BACKWARD handling (front sensors trail when reversing) ----
int   backCruisePower   = 110;    // backward CRUISE (slower than fwd so trailing sensors keep up)
float backSteerGain     = 16.0;   // backward steering gain (stronger: drift detected late)
int   backSteerSign     = +1;     // backward steering sign: flip with 'i' if it veers off

// ===== SERVO =====
const int SERVO_PIN = 4;
const int NEUTRAL = 86, MAX_LEFT = 98, MAX_RIGHT = 74;
Servo steering;
int lastServo = -1;
float currentSteerNorm = 0.0;     // 0 = straight, 1 = full steering

// ===== LINE SENSORS =====
const int S_PIN[4] = {34, 35, 36, 39};
bool  lineIsHigh = false;
float steerGain = 12.0, smoothing = 0.30, centerOffset = 0.0;
float filtErr = 0.0, lastErr = 0.0;
int   lostCount = 0;
const int LOST_LIMIT = 25;
const float WEIGHT[4] = {-3, -1, +1, +3};

// ===== MOTORS =====
#define ENA 25
#define ENB 26
#define IN1 16
#define IN2 17
#define IN3 22
#define IN4 23
const int pwmFreq = 1000, pwmRes = 8;

// ===== DRIVE STATE =====
bool driving = false;
int  driveDir = +1;               // +1 = forward, -1 = backward
int  pendingDir = +1;             // direction applied to next 1-9 digit
long startCount = 0;
long lastAbsPulses = 0;
float estForwardCm = 0.0;         // steering-compensated x-axis progress estimate
float cutMotorAtCm = 0.0;         // targetDistCm - stopEarly
long stallWatchPulses = 0;
unsigned long stallWatchMs = 0;
unsigned long unstickUntilMs = 0;
bool driveHadError = false;
const char *lastDriveReason = "idle";
float lastDriveCmDone = 0.0;
float lastDriveRawCm = 0.0;
long  lastDrivePulses = 0;

void startDriveDir(int cm, int dir);   // fwd decl
float maxf2(float a, float b) { return (a > b) ? a : b; }
float minf2(float a, float b) { return (a < b) ? a : b; }

int effectiveCruisePower() {
  // Backward cruises slower (front sensors trail when reversing).
  int base = (driveDir < 0) ? backCruisePower : cruisePower;
  return constrain(max(base, minMovePower), 0, 255);
}

int effectiveApproachPower() {
  int p = max(crawlPower, minMovePower);
  int c = effectiveCruisePower();
  if (p > c) p = c;
  return constrain(p, 0, 255);
}

float effectiveRampDownCm() {
  // For short distances, do not spend most of the travel crawling.
  // Example: 20cm target, cut 19cm, rampDown=6cm => slowdown starts near 13cm.
  return minf2((float)rampDownCm, maxf2(2.0, cutMotorAtCm * 0.45));
}

void motorsRaw(int power) {
  // Backward needs more push than forward: scale the power up when reversing.
  if (driveDir < 0) power = (int)(power * backPowerScale);
  power = constrain(power, 0, 255);
  if (driveDir >= 0) {
    // FORWARD
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    // BACKWARD (both motor directions reversed)
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  }
  ledcWrite(ENA, power); ledcWrite(ENB, power);
}

void motorsOff() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0); ledcWrite(ENB, 0);
}

void motorsBrakeThenOff() {
  if (brakeMs > 0) {
    // L298N dynamic braking: both motor pins same while enable is on.
    digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
    ledcWrite(ENA, 255); ledcWrite(ENB, 255);
    delay(brakeMs);
  }
  motorsOff();
}

void writeServo(int a) {
  a = constrain(a, MAX_RIGHT, MAX_LEFT);
  int span = max(1, max(abs(MAX_LEFT - NEUTRAL), abs(NEUTRAL - MAX_RIGHT)));
  currentSteerNorm = minf2(1.0, ((float)abs(a - NEUTRAL)) / ((float)span));
  if (a != lastServo) { steering.write(a); lastServo = a; }
}

bool readLine(float *err) {
  int on = 0; float sum = 0;
  for (int i = 0; i < 4; i++) {
    int raw = digitalRead(S_PIN[i]);
    bool onLine = lineIsHigh ? (raw == 1) : (raw == 0);
    if (onLine) { sum += WEIGHT[i]; on++; }
  }
  if (on == 0) return false;
  *err = (sum / on) - centerOffset;
  return true;
}

float dynamicStopEarlyCm() {
  float extra = 0.0;
  if (crawlPower > 80) extra = (crawlPower - 80) * speedCoastPerPWM;
  return maxf2(0.0, stopEarlyCm + extra);
}

void stopDriveAndReport(const char *reason, long absPulses, float rawCm, bool isError=false) {
  // Cut motors, then wait briefly so the encoder can count the final coast.
  motorsBrakeThenOff();
  delay(180);
  long finalDelta = getCount() - startCount;
  long finalAbs = (finalDelta < 0) ? -finalDelta : finalDelta;
  updateDistanceEstimate(finalAbs);

  lastDrivePulses = finalAbs;
  lastDriveRawCm = ((float)finalAbs) / PULSES_PER_CM;
  lastDriveCmDone = estForwardCm;
  if (lastDriveCmDone < 0) lastDriveCmDone = 0;
  lastDriveReason = reason;
  driveHadError = isError;

  driving = false;
  writeServo(NEUTRAL);
  SerialBT.print(">> "); SerialBT.print(reason);
  SerialBT.print(" raw="); SerialBT.print(lastDriveRawCm, 2);
  SerialBT.print("cm estX="); SerialBT.print(lastDriveCmDone, 2);
  SerialBT.print("cm cutAt="); SerialBT.print(cutMotorAtCm, 2);
  SerialBT.print("cm pulses="); SerialBT.println(finalAbs);
}

void startDrive(int cm) { startDriveDir(cm, +1); }

void startDriveDir(int cm, int dir) {
  driveDir = (dir < 0) ? -1 : +1;
  targetDistCm = cm;
  startCount = getCount();
  lastAbsPulses = 0;
  estForwardCm = 0.0;
  filtErr = 0; lastErr = 0; lostCount = 0;
  currentSteerNorm = 0.0;
  stallWatchPulses = 0;
  stallWatchMs = millis();
  unstickUntilMs = 0;
  driveHadError = false;
  lastDriveReason = "running";
  lastDriveCmDone = 0.0;
  lastDriveRawCm = 0.0;
  lastDrivePulses = 0;

  float early = dynamicStopEarlyCm();
  cutMotorAtCm = maxf2(0.0, ((float)targetDistCm) - early);

  driving = true;
  SerialBT.print(">> DRIVE "); SerialBT.print(driveDir < 0 ? "BACK " : "FWD "); SerialBT.print(targetDistCm);
  SerialBT.print("cm | cut motors at estX="); SerialBT.print(cutMotorAtCm, 2);
  SerialBT.print("cm | stopEarly="); SerialBT.print(early, 2);
  SerialBT.print("cm | ppcm="); SerialBT.print(PULSES_PER_CM, 2);
  SerialBT.print(" | effCruise="); SerialBT.print(effectiveCruisePower());
  SerialBT.print(" | approach="); SerialBT.println(effectiveApproachPower());
}

void printStatus() {
  SerialBT.print("[status] dist="); SerialBT.print(targetDistCm);
  SerialBT.print(" cruise="); SerialBT.print(cruisePower);
  SerialBT.print(" kickP="); SerialBT.print(kickPower);
  SerialBT.print(" kickCm="); SerialBT.print(kickDistCm, 1);
  SerialBT.print(" rampDn="); SerialBT.print(rampDownCm);
  SerialBT.print(" crawl="); SerialBT.print(crawlPower);
  SerialBT.print(" minMove="); SerialBT.print(minMovePower);
  SerialBT.print(" effCruise="); SerialBT.print(effectiveCruisePower());
  SerialBT.print(" approach="); SerialBT.print(effectiveApproachPower());
  SerialBT.print(" unstick="); SerialBT.print(unstickPower);
  SerialBT.print(" manualP="); SerialBT.print(testPower);
  SerialBT.print(" gain="); SerialBT.print(steerGain, 1);
  SerialBT.print(" smooth="); SerialBT.print(smoothing, 2);
  SerialBT.print(" pol="); SerialBT.print(lineIsHigh ? "1" : "0");
  SerialBT.print(" ppcm="); SerialBT.print(PULSES_PER_CM, 2);
  SerialBT.print(" stopEarly="); SerialBT.print(stopEarlyCm, 2);
  SerialBT.print(" speedCoast="); SerialBT.print(speedCoastPerPWM, 3);
  SerialBT.print(" turnLoss="); SerialBT.print(turnLossGain, 2);
  SerialBT.print(" backScale="); SerialBT.print(backPowerScale, 2);
  SerialBT.print(" backCruise="); SerialBT.print(backCruisePower);
  SerialBT.print(" backGain="); SerialBT.print(backSteerGain, 0);
  SerialBT.print(" backSign="); SerialBT.print(backSteerSign);
  SerialBT.print(" brakeMs="); SerialBT.println(brakeMs);
}



// ===== JSON RESPONSE HELPERS =====
void sendJsonDoc(JsonDocument &doc) {
  serializeJson(doc, SerialBT);
  SerialBT.print('\n');
}

void sendDone(int id, const char *what) {
  StaticJsonDocument<160> doc;
  doc["type"] = "DONE";
  doc["id"] = id;
  doc["what"] = what;
  sendJsonDoc(doc);
}

void sendDriveDone(int id) {
  StaticJsonDocument<256> doc;
  doc["type"] = "DONE";
  doc["id"] = id;
  doc["what"] = "drive_cm";
  doc["cm_done"] = lastDriveCmDone;
  doc["raw_cm"] = lastDriveRawCm;
  doc["pulses"] = lastDrivePulses;
  doc["reason"] = lastDriveReason;
  sendJsonDoc(doc);
}

void sendDriveError(int id, const char *reason) {
  StaticJsonDocument<256> doc;
  doc["type"] = "ERROR";
  doc["id"] = id;
  doc["reason"] = reason;
  doc["cm_done"] = lastDriveCmDone;
  doc["raw_cm"] = lastDriveRawCm;
  doc["pulses"] = lastDrivePulses;
  sendJsonDoc(doc);
}

void sendError(int id, const char *reason) {
  StaticJsonDocument<160> doc;
  doc["type"] = "ERROR";
  doc["id"] = id;
  doc["reason"] = reason;
  sendJsonDoc(doc);
}

int doZSteps(int steps) {
  bool up = steps >= 0;
  int n = abs(steps);
  if (n > MAX_STEPS_PER_CMD) return -1;
  if (Z_ENA_PIN >= 0) digitalWrite(Z_ENA_PIN, LOW);
  digitalWrite(DIR_PIN, up ? DIR_UP_LEVEL : !DIR_UP_LEVEL);
  delayMicroseconds(50);
  for (int i = 0; i < n; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_PULSE_US);
  }
  return n;
}

int computeDrivePower(float doneCm, float remCm) {
  int baseCruise = effectiveCruisePower();
  int approach   = effectiveApproachPower();
  float rampCm   = effectiveRampDownCm();

  int power;
  if (doneCm < kickDistCm) {
    float t = (kickDistCm > 0) ? (doneCm / kickDistCm) : 1.0;
    power = kickPower + (int)((baseCruise - kickPower) * t);
  } else if (remCm < rampCm) {
    float t = (rampCm > 0) ? (remCm / rampCm) : 0.0;
    // True slowdown, but never below the power that can actually move the robot.
    power = approach + (int)((baseCruise - approach) * t);
  } else {
    power = baseCruise;
  }

  // If the encoder has stopped increasing before the target, give a short kick.
  if (millis() < unstickUntilMs) power = max(power, unstickPower);

  return constrain(power, 0, 255);
}

void updateDistanceEstimate(long absPulses) {
  long stepPulses = absPulses - lastAbsPulses;
  if (stepPulses < 0) stepPulses = 0;
  lastAbsPulses = absPulses;

  float rawStepCm = ((float)stepPulses) / PULSES_PER_CM;

  // When steering, wheel path is longer than straight x-axis progress.
  // Loss grows mainly when steering is large, so square the normalized steering.
  float loss = turnLossGain * currentSteerNorm * currentSteerNorm;
  loss = constrain(loss, 0.0, 0.50);        // never discount more than 50%
  estForwardCm += rawStepCm * (1.0 - loss);
}



bool updateDriveOnce() {
  if (!driving) return true;

  long deltaPulses = getCount() - startCount;
  long absPulses   = (deltaPulses < 0) ? -deltaPulses : deltaPulses;
  float rawCm      = ((float)absPulses) / PULSES_PER_CM;

  updateDistanceEstimate(absPulses);
  float remCm = cutMotorAtCm - estForwardCm;

  if (estForwardCm >= cutMotorAtCm) {
    stopDriveAndReport("CUT+COAST", absPulses, rawCm);
    return true;
  }

  float safetyCm = ((float)targetDistCm) + dynamicStopEarlyCm() + 15.0;
  if (rawCm > safetyCm) {
    stopDriveAndReport("SAFETY STOP", absPulses, rawCm, true);
    return true;
  }

  unsigned long nowMs = millis();
  if (absPulses > stallWatchPulses + 2) {
    stallWatchPulses = absPulses;
    stallWatchMs = nowMs;
  } else if ((nowMs - stallWatchMs > (unsigned long)stallAfterMs) && (estForwardCm < cutMotorAtCm - 1.0)) {
    unstickUntilMs = nowMs + (unsigned long)stallKickMs;
    stallWatchMs = nowMs;
    SerialBT.print(">> ANTI-STALL boost pwr="); SerialBT.print(unstickPower);
    SerialBT.print(" at estX="); SerialBT.println(estForwardCm, 1);
  }

  float err;
  bool seen = readLine(&err);
  if (seen) { lostCount = 0; lastErr = err; }
  else {
    lostCount++;
    err = lastErr;
    if (lostCount > LOST_LIMIT) {
      driving = false; motorsOff(); writeServo(NEUTRAL);
      driveHadError = true;
      lastDriveReason = "line_lost";
      lastDrivePulses = absPulses;
      lastDriveRawCm = rawCm;
      lastDriveCmDone = estForwardCm;
      SerialBT.println(">> LINE LOST - stopped");
      return true;
    }
  }

  filtErr = smoothing * filtErr + (1.0 - smoothing) * err;
  int servoCmd;
  if (driveDir < 0) servoCmd = NEUTRAL - backSteerSign * (int)(backSteerGain * filtErr);
  else servoCmd = NEUTRAL - (int)(steerGain * filtErr);
  writeServo(servoCmd);

  int drivePower = computeDrivePower(estForwardCm, remCm);
  motorsRaw(drivePower);
  return false;
}

bool driveBlocking(float cm, int dir) {
  startDriveDir((int)round(cm), dir);
  while (driving) {
    updateDriveOnce();
    delay(5);
    // Emergency STOP while moving
    if (SerialBT.available()) {
      char c = SerialBT.peek();
      if (c == 'x' || c == 'X') {
        SerialBT.read();
        driving = false; motorsOff(); writeServo(NEUTRAL);
        driveHadError = true;
        lastDriveReason = "stopped";
        SerialBT.println(">> STOP during DRIVE_CM");
        return false;
      }
    }
  }
  return !driveHadError;
}

void handleJson(const String &line) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) return;

  const char *cmd = doc["cmd"] | "";
  int id = doc["id"] | 0;

  if (id != 0 && id == lastCmdId && strcmp(cmd, "PING") != 0) {
    sendDone(id, "duplicate_ack");
    return;
  }

  if (strcmp(cmd, "PING") == 0) {
    StaticJsonDocument<100> resp;
    resp["type"] = "PONG";
    resp["id"] = id;
    sendJsonDoc(resp);
  }
  else if (strcmp(cmd, "STOP") == 0) {
    driving = false; motorsOff(); writeServo(NEUTRAL);
    lastCmdId = id;
    sendDone(id, "stop");
  }
  else if (strcmp(cmd, "ELEVATE") == 0) {
    int steps = doc["steps"] | 0;
    if (abs(steps) > MAX_STEPS_PER_CMD) { sendError(id, "too_many_steps"); return; }
    int done = doZSteps(steps);
    if (done < 0) sendError(id, "z_failed");
    else { lastCmdId = id; sendDone(id, "elevate"); }
  }
  else if (strcmp(cmd, "DRIVE_CM") == 0) {
    float cm = doc["cm"] | 0.0;
    const char *dirS = doc["dir"] | "FWD";
    int dir = (strcmp(dirS, "BACK") == 0) ? -1 : +1;
    bool ok = driveBlocking(cm, dir);
    lastCmdId = id;
    if (ok) sendDriveDone(id);
    else sendDriveError(id, lastDriveReason);
  }
  else {
    sendError(id, "unknown_cmd");
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("BookFinderESP32");

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  if (Z_ENA_PIN >= 0) { pinMode(Z_ENA_PIN, OUTPUT); digitalWrite(Z_ENA_PIN, LOW); }

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

  SerialBT.println("=== BookFinder FINAL: Z + X drive + ALWAYS line-following ===");
  printStatus();
}

void handleManualChar(char c) {
  if (c >= '1' && c <= '9') { driveBlocking((c - '0') * 10, pendingDir); pendingDir = +1; }
  else if (c == 'R') { pendingDir = -1; SerialBT.println(">> next digit = BACKWARD"); }
  else if (c == 'F') { pendingDir = +1; SerialBT.println(">> next digit = FORWARD"); }
  else if (c == 'x' || c == 'X') { driving = false; motorsOff(); writeServo(NEUTRAL); SerialBT.println(">> STOP"); }
  else if (c == '?') printStatus();
  else if (c == 'm') {
    SerialBT.print(">> sensors: ");
    for (int i = 0; i < 4; i++) { SerialBT.print(digitalRead(S_PIN[i])); SerialBT.print(" "); }
    SerialBT.print(" count="); SerialBT.println(getCount());
  }
}

void loop() {
  while (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\n') {
      jsonLine.trim();
      if (jsonLine.length() > 0) handleJson(jsonLine);
      jsonLine = "";
    } else if (c == '{' || jsonLine.length() > 0) {
      jsonLine += c;
      if (jsonLine.length() > 240) jsonLine = "";
    } else {
      handleManualChar(c);
    }
  }
  updateDriveOnce();
}
