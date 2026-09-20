/*
============================================================================
  ROBOT DRIVE v5  —  encoder distance fix + line follow
============================================================================
  Fixes the V4 distance problem by separating TWO different errors:

    1) PULSES_PER_CM = scale calibration of the encoder.
       Do NOT tune this just because the robot coasts after stop.

    2) stopEarlyCm = final coast / inertia compensation.
       Robot cuts the motors before the target and lets the remaining coast land
       it near the requested distance.

  Also adds:
    - same stronger PCNT glitch filter used in measure_params_v2.ino
    - speed-aware stop-early compensation
    - steering-loss compensation, because encoder wheel travel during turns is
      not equal to straight x-axis progress
    - optional active brake time, default 0 ms so behavior starts familiar

  Suggested starting values based on your tests:
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
    RAMP : , . ramp-down cm
    FIX  : s/S stopEarly -/+0.1   v/V speedCoast -/+0.005
           u/U turnLoss -/+0.02   b/B brakeMs -/+10   h/H crawl -/+5
    LINE : +/- gain               f flip polarity            o/p center
    ENCODER: e/E PPCM -/+0.05
    INFO : m sensors+count        c count                    ? status
============================================================================
*/

#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include "driver/pulse_cnt.h"

BluetoothSerial SerialBT;

// ===== ENCODER (single channel, PCNT + glitch filter) =====
const int ENC_PIN = 32;
const int GLITCH_NS = 1000;       // IMPORTANT: same stronger filter as measure_params_v2
pcnt_unit_handle_t    pcnt_unit = NULL;
pcnt_channel_handle_t pcnt_chan = NULL;

long getCount() {
  int c = 0;
  pcnt_unit_get_count(pcnt_unit, &c);
  return (long)c;
}
void resetCount() { pcnt_unit_clear_count(pcnt_unit); }

// ===== CALIBRATION (tunable) =====
float PULSES_PER_CM = 9.27;       // true scale estimate; do not use this to hide coast
int   kickPower     = 170;        // your latest value
float kickDistCm    = 4.0;
int   cruisePower   = 100;
int   rampDownCm    = 12;
int   crawlPower    = 80;         // final approach power; must be lower than cruise
int   targetDistCm  = 30;

// ===== DISTANCE FIX PARAMETERS =====
float stopEarlyCm       = 1.00;   // after fixing ramp-down, start lower and tune with s/S
float speedCoastPerPWM  = 0.010;  // extra cm per PWM above crawlPower=80
float turnLossGain      = 0.25;   // compensates encoder over-counting during steering
int   brakeMs           = 0;      // optional active brake. Start at 0, increase only if needed.

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
long startCount = 0;
long lastAbsPulses = 0;
float estForwardCm = 0.0;         // steering-compensated x-axis progress estimate
float cutMotorAtCm = 0.0;         // targetDistCm - stopEarly

float maxf2(float a, float b) { return (a > b) ? a : b; }
float minf2(float a, float b) { return (a < b) ? a : b; }

void motorsRaw(int power) {
  power = constrain(power, 0, 255);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
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

void stopDriveAndReport(const char *reason, long absPulses, float rawCm) {
  driving = false;
  motorsBrakeThenOff();
  writeServo(NEUTRAL);
  SerialBT.print(">> "); SerialBT.print(reason);
  SerialBT.print(" raw="); SerialBT.print(rawCm, 2);
  SerialBT.print("cm estX="); SerialBT.print(estForwardCm, 2);
  SerialBT.print("cm cutAt="); SerialBT.print(cutMotorAtCm, 2);
  SerialBT.print("cm pulses="); SerialBT.println(absPulses);
}

void startDrive(int cm) {
  targetDistCm = cm;
  startCount = getCount();
  lastAbsPulses = 0;
  estForwardCm = 0.0;
  filtErr = 0; lastErr = 0; lostCount = 0;
  currentSteerNorm = 0.0;

  float early = dynamicStopEarlyCm();
  cutMotorAtCm = maxf2(0.0, ((float)targetDistCm) - early);

  driving = true;
  SerialBT.print(">> DRIVE "); SerialBT.print(targetDistCm);
  SerialBT.print("cm | cut motors at estX="); SerialBT.print(cutMotorAtCm, 2);
  SerialBT.print("cm | stopEarly="); SerialBT.print(early, 2);
  SerialBT.print("cm | ppcm="); SerialBT.println(PULSES_PER_CM, 2);
}

void printStatus() {
  SerialBT.print("[status] dist="); SerialBT.print(targetDistCm);
  SerialBT.print(" cruise="); SerialBT.print(cruisePower);
  SerialBT.print(" kickP="); SerialBT.print(kickPower);
  SerialBT.print(" kickCm="); SerialBT.print(kickDistCm, 1);
  SerialBT.print(" rampDn="); SerialBT.print(rampDownCm);
  SerialBT.print(" crawl="); SerialBT.print(crawlPower);
  SerialBT.print(" gain="); SerialBT.print(steerGain, 1);
  SerialBT.print(" smooth="); SerialBT.print(smoothing, 2);
  SerialBT.print(" pol="); SerialBT.print(lineIsHigh ? "1" : "0");
  SerialBT.print(" ppcm="); SerialBT.print(PULSES_PER_CM, 2);
  SerialBT.print(" stopEarly="); SerialBT.print(stopEarlyCm, 2);
  SerialBT.print(" speedCoast="); SerialBT.print(speedCoastPerPWM, 3);
  SerialBT.print(" turnLoss="); SerialBT.print(turnLossGain, 2);
  SerialBT.print(" brakeMs="); SerialBT.println(brakeMs);
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("RobotTuner");

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

  SerialBT.println("=== ROBOT DRIVE v5 distance fix ===");
  SerialBT.println("1-9 drive | g set-dist | x stop | ? status | s/S stopEarly | u/U turnLoss | h/H crawl");
  printStatus();
}

int computeDrivePower(float doneCm, float remCm) {
  int power;
  if (doneCm < kickDistCm) {
    float t = (kickDistCm > 0) ? (doneCm / kickDistCm) : 1.0;
    power = kickPower + (int)((cruisePower - kickPower) * t);
  } else if (remCm < rampDownCm) {
    float t = (rampDownCm > 0) ? (remCm / rampDownCm) : 0.0;
    // IMPORTANT: V4 used kickPower - 30 here. With kickPower=170 that made
    // the robot speed UP to 140 near the target. That was not ramp-down.
    power = crawlPower + (int)((cruisePower - crawlPower) * t);
  } else {
    power = cruisePower;
  }
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

void loop() {
  if (SerialBT.available()) {
    char c = SerialBT.read();
    if (c >= '1' && c <= '9') { startDrive((c - '0') * 10); }
    else switch (c) {
      case 'g': case 'G': startDrive(targetDistCm); break;
      case 'x': case 'X': driving = false; motorsOff(); writeServo(NEUTRAL);
                SerialBT.println(">> STOP"); break;
      case 'r': resetCount(); SerialBT.println(">> count reset"); break;
      case 'd': targetDistCm = max(0, targetDistCm - 10); printStatus(); break;
      case 'D': targetDistCm += 10; printStatus(); break;
      case '[': cruisePower = max(0, cruisePower - 5);
                SerialBT.print(">> cruise="); SerialBT.println(cruisePower); break;
      case ']': cruisePower = min(255, cruisePower + 5);
                SerialBT.print(">> cruise="); SerialBT.println(cruisePower); break;
      case 'k': kickPower = max(0, kickPower - 5);
                SerialBT.print(">> kickP="); SerialBT.println(kickPower); break;
      case 'K': kickPower = min(255, kickPower + 5);
                SerialBT.print(">> kickP="); SerialBT.println(kickPower); break;
      case 'n': kickDistCm -= 0.5; if (kickDistCm < 0) kickDistCm = 0;
                SerialBT.print(">> kickCm="); SerialBT.println(kickDistCm, 1); break;
      case 'N': kickDistCm += 0.5;
                SerialBT.print(">> kickCm="); SerialBT.println(kickDistCm, 1); break;
      case ',': rampDownCm = max(2, rampDownCm - 2);
                SerialBT.print(">> rampDn="); SerialBT.println(rampDownCm); break;
      case '.': rampDownCm = min(50, rampDownCm + 2);
                SerialBT.print(">> rampDn="); SerialBT.println(rampDownCm); break;
      case 'h': crawlPower = max(0, crawlPower - 5);
                SerialBT.print(">> crawl="); SerialBT.println(crawlPower); break;
      case 'H': crawlPower = min(255, crawlPower + 5);
                SerialBT.print(">> crawl="); SerialBT.println(crawlPower); break;
      case 's': stopEarlyCm -= 0.10; if (stopEarlyCm < 0) stopEarlyCm = 0; printStatus(); break;
      case 'S': stopEarlyCm += 0.10; printStatus(); break;
      case 'v': speedCoastPerPWM -= 0.005; if (speedCoastPerPWM < 0) speedCoastPerPWM = 0; printStatus(); break;
      case 'V': speedCoastPerPWM += 0.005; printStatus(); break;
      case 'u': turnLossGain -= 0.02; if (turnLossGain < 0) turnLossGain = 0; printStatus(); break;
      case 'U': turnLossGain += 0.02; if (turnLossGain > 1.0) turnLossGain = 1.0; printStatus(); break;
      case 'b': brakeMs = max(0, brakeMs - 10); printStatus(); break;
      case 'B': brakeMs = min(300, brakeMs + 10); printStatus(); break;
      case '+': steerGain += 1; printStatus(); break;
      case '-': steerGain -= 1; if (steerGain < 0) steerGain = 0; printStatus(); break;
      case 'f': lineIsHigh = !lineIsHigh;
                SerialBT.print(">> pol="); SerialBT.println(lineIsHigh ? "1" : "0"); break;
      case 'o': centerOffset -= 0.25; printStatus(); break;
      case 'p': centerOffset += 0.25; printStatus(); break;
      case 'c': SerialBT.print(">> count="); SerialBT.println(getCount()); break;
      case 'e': PULSES_PER_CM -= 0.05; if (PULSES_PER_CM < 1.0) PULSES_PER_CM = 1.0; printStatus(); break;
      case 'E': PULSES_PER_CM += 0.05; printStatus(); break;
      case 'm': {
        SerialBT.print(">> sensors: ");
        for (int i = 0; i < 4; i++) { SerialBT.print(digitalRead(S_PIN[i])); SerialBT.print(" "); }
        long absP = labs(getCount() - startCount);
        SerialBT.print(" count="); SerialBT.print(getCount());
        SerialBT.print(" raw="); SerialBT.print(((float)absP) / PULSES_PER_CM, 2);
        SerialBT.print(" estX="); SerialBT.println(estForwardCm, 2);
        break;
      }
      case '?': printStatus(); break;
      default: break;
    }
  }

  if (driving) {
    long deltaPulses = getCount() - startCount;
    long absPulses   = (deltaPulses < 0) ? -deltaPulses : deltaPulses;
    float rawCm      = ((float)absPulses) / PULSES_PER_CM;

    updateDistanceEstimate(absPulses);
    float remCm = cutMotorAtCm - estForwardCm;

    if (estForwardCm >= cutMotorAtCm) {
      stopDriveAndReport("CUT+COAST", absPulses, rawCm);
      return;
    }

    // Safety only: never let it run forever if line correction over-discounts distance.
    float safetyCm = ((float)targetDistCm) + dynamicStopEarlyCm() + 15.0;
    if (rawCm > safetyCm) {
      stopDriveAndReport("SAFETY STOP", absPulses, rawCm);
      return;
    }

    float err;
    bool seen = readLine(&err);
    if (seen) { lostCount = 0; lastErr = err; }
    else {
      lostCount++;
      err = lastErr;
      if (lostCount > LOST_LIMIT) {
        driving = false; motorsOff(); writeServo(NEUTRAL);
        SerialBT.println(">> LINE LOST - stopped");
        return;
      }
    }

    filtErr = smoothing * filtErr + (1.0 - smoothing) * err;
    int servoCmd = NEUTRAL - (int)(steerGain * filtErr);
    writeServo(servoCmd);

    motorsRaw(computeDrivePower(estForwardCm, remCm));

    static unsigned long lastP = 0;
    if (millis() - lastP > 300) {
      SerialBT.print("   raw="); SerialBT.print(rawCm, 1);
      SerialBT.print(" estX="); SerialBT.print(estForwardCm, 1);
      SerialBT.print("/"); SerialBT.print(targetDistCm);
      SerialBT.print(" cut="); SerialBT.print(cutMotorAtCm, 1);
      SerialBT.print(" steer="); SerialBT.println(currentSteerNorm, 2);
      lastP = millis();
    }
  }
}
