/*
============================================================================
  ROBOT DRIVE v4  —  SINGLE-CHANNEL encoder (reliable) + line follow + distance
============================================================================
  WHY v3: quadrature needs BOTH channels, but the BLUE wire keeps dying.
  The measure_params firmware that reliably counts uses SINGLE channel
  (yellow on GPIO 32) via the PCNT hardware counter + glitch filter.
  So v3 uses that same reliable method - no dependency on the fragile blue.

  Noise handling: PCNT hardware glitch filter + the caps you installed.
  (Your PULSES_PER_CM=10.4 was measured with this same single-channel method,
   so it's the correct calibration for this firmware.)

  === CALIBRATION ===
    PULSES_PER_CM=10.4  kick=120  cruise=100(tunable)
    Servo neutral=86 maxL=98 maxR=74
    Line gain=12 smooth=0.4 center=0 polarity line=0

  === WIRING ===
    Encoder Yellow(A)->GPIO32   Red->3.3V   Black->GND
      (BLUE not needed - single channel!)
    Servo->4 | L298N ENA25 ENB26 IN1 16 IN2 17 IN3 22 IN4 23
    Line S1=34 S2=35 S3=36 S4=39

  === COMMANDS ===
   DRIVE: 1-9 = drive Nx10cm (3=30cm)   g=set-dist   x=STOP
   DIST:  d/D  distance -10/+10 cm
   SPEED: [ ]  cruise    k/K kick power    n/N kick distance(cm)
   RAMP:  , .  ramp-down distance
   LINE:  +/-  gain    f flip polarity    o/p center
   INFO:  m sensors+count   c count   r reset count   ? status
============================================================================
*/

#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include "driver/pulse_cnt.h"
BluetoothSerial SerialBT;

// ===== ENCODER (single channel, PCNT + glitch filter) =====
const int ENC_PIN = 32;          // yellow only
const int GLITCH_NS = 200;
pcnt_unit_handle_t    pcnt_unit = NULL;
pcnt_channel_handle_t pcnt_chan = NULL;

long getCount() {
  int c = 0;
  pcnt_unit_get_count(pcnt_unit, &c);
  return (long)c;
}
void resetCount() { pcnt_unit_clear_count(pcnt_unit); }

// ===== CALIBRATION (tunable) =====
float PULSES_PER_CM = 8.6;   // corrected from 10.4 (was overshooting)
int   kickPower     = 120;
float kickDistCm    = 4.0;
int   cruisePower   = 100;
int   rampDownCm    = 12;
int   targetDistCm  = 30;

// ===== SERVO =====
const int SERVO_PIN = 4;
const int NEUTRAL = 86, MAX_LEFT = 98, MAX_RIGHT = 74;
Servo steering;
int lastServo = -1;

// ===== LINE SENSORS =====
const int S_PIN[4] = {34, 35, 36, 39};
bool  lineIsHigh = false;
float steerGain = 12.0, smoothing = 0.4, centerOffset = 0.0;
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
long targetPulses = 0;

void motorsRaw(int power) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(ENA, power); ledcWrite(ENB, power);
}
void motorsStop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0); ledcWrite(ENB, 0);
}
void writeServo(int a) {
  a = constrain(a, MAX_RIGHT, MAX_LEFT);
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

void startDrive(int cm) {
  targetDistCm = cm;
  targetPulses = (long)(cm * PULSES_PER_CM);
  startCount   = getCount();
  filtErr = 0; lastErr = 0; lostCount = 0;
  driving = true;
  SerialBT.print(">> DRIVE "); SerialBT.print(cm);
  SerialBT.print("cm (need "); SerialBT.print(targetPulses); SerialBT.println(" pulses)");
}

void printStatus() {
  SerialBT.print("[status] dist="); SerialBT.print(targetDistCm);
  SerialBT.print(" cruise="); SerialBT.print(cruisePower);
  SerialBT.print(" kickP="); SerialBT.print(kickPower);
  SerialBT.print(" kickCm="); SerialBT.print(kickDistCm);
  SerialBT.print(" rampDn="); SerialBT.print(rampDownCm);
  SerialBT.print(" gain="); SerialBT.print(steerGain);
  SerialBT.print(" pol="); SerialBT.print(lineIsHigh ? "1" : "0");
  SerialBT.print(" ppcm="); SerialBT.println(PULSES_PER_CM);
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("RobotTuner");

  // PCNT single-channel encoder (same reliable method as measure_params)
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
  motorsStop();

  SerialBT.println("=== ROBOT DRIVE v4 (single-channel) ===");
  SerialBT.println("1-9 driveNx10cm  x stop  m sensors  c count  e/E ppcm-/+  ? status");
  printStatus();
}

int computeDrivePower(float doneCm, float remCm) {
  int power;
  if (doneCm < kickDistCm) {
    float t = (kickDistCm > 0) ? (doneCm / kickDistCm) : 1.0;
    power = kickPower + (int)((cruisePower - kickPower) * t);
  } else if (remCm < rampDownCm) {
    float t = (rampDownCm > 0) ? (remCm / rampDownCm) : 0.0;
    int crawl = max(0, kickPower - 30);
    power = crawl + (int)((cruisePower - crawl) * t);
  } else {
    power = cruisePower;
  }
  return constrain(power, 0, 255);
}

void loop() {
  if (SerialBT.available()) {
    char c = SerialBT.read();
    if (c >= '1' && c <= '9') { startDrive((c - '0') * 10); }
    else switch (c) {
      case 'g': case 'G': startDrive(targetDistCm); break;
      case 'x': case 'X': driving = false; motorsStop(); writeServo(NEUTRAL);
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
      case 'n': kickDistCm = max(0.0f, kickDistCm - 1.0f);
                SerialBT.print(">> kickCm="); SerialBT.println(kickDistCm); break;
      case 'N': kickDistCm += 1.0f;
                SerialBT.print(">> kickCm="); SerialBT.println(kickDistCm); break;
      case ',': rampDownCm = max(2, rampDownCm - 2);
                SerialBT.print(">> rampDn="); SerialBT.println(rampDownCm); break;
      case '.': rampDownCm = min(50, rampDownCm + 2);
                SerialBT.print(">> rampDn="); SerialBT.println(rampDownCm); break;
      case '+': steerGain += 1; printStatus(); break;
      case '-': steerGain = max(0.0f, steerGain - 1); printStatus(); break;
      case 'f': lineIsHigh = !lineIsHigh;
                SerialBT.print(">> pol="); SerialBT.println(lineIsHigh ? "1" : "0"); break;
      case 'o': centerOffset -= 0.25; printStatus(); break;
      case 'p': centerOffset += 0.25; printStatus(); break;
      case 'c': SerialBT.print(">> count="); SerialBT.println(getCount()); break;
      case 'e': PULSES_PER_CM = max(1.0f, PULSES_PER_CM - 0.2f);
                SerialBT.print(">> ppcm="); SerialBT.println(PULSES_PER_CM); break;
      case 'E': PULSES_PER_CM += 0.2f;
                SerialBT.print(">> ppcm="); SerialBT.println(PULSES_PER_CM); break;
      case 'm': {
        SerialBT.print(">> sensors: ");
        for (int i = 0; i < 4; i++) { SerialBT.print(digitalRead(S_PIN[i])); SerialBT.print(" "); }
        SerialBT.print("  count="); SerialBT.println(getCount());
        break;
      }
      case '?': printStatus(); break;
      default: break;
    }
  }

  if (driving) {
    long deltaPulses = getCount() - startCount;
    long absPulses   = (deltaPulses < 0) ? -deltaPulses : deltaPulses;
    float doneCm = absPulses / PULSES_PER_CM;
    float remCm  = (targetPulses - absPulses) / PULSES_PER_CM;

    if (absPulses >= targetPulses) {
      driving = false;
      motorsStop();
      writeServo(NEUTRAL);
      SerialBT.print(">> ARRIVED. pulses="); SerialBT.print(absPulses);
      SerialBT.print("  ~"); SerialBT.print(doneCm, 1); SerialBT.println("cm");
      return;
    }

    float err;
    bool seen = readLine(&err);
    if (seen) { lostCount = 0; lastErr = err; }
    else {
      lostCount++;
      err = lastErr;
      if (lostCount > LOST_LIMIT) {
        driving = false; motorsStop(); writeServo(NEUTRAL);
        SerialBT.println(">> LINE LOST - stopped");
        return;
      }
    }
    filtErr = smoothing * filtErr + (1.0 - smoothing) * err;
    writeServo(NEUTRAL - (int)(steerGain * filtErr));

    motorsRaw(computeDrivePower(doneCm, remCm));

    static unsigned long lastP = 0;
    if (millis() - lastP > 300) {
      SerialBT.print("   ..."); SerialBT.print(doneCm, 1);
      SerialBT.print("/"); SerialBT.print(targetDistCm); SerialBT.println("cm");
      lastP = millis();
    }
  }
}
