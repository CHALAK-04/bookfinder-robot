/*
==============================================================================
  ELEVATION CONTROLLER  (runs on the ESP32)  -  Phase 1: Z Calibration
==============================================================================

  Target board: NodeMCU ESP-32S V1.1
  Built-in LED: GPIO 2 (blue)

  STATUS LED BEHAVIOUR:
    Blinking (500 ms on / 500 ms off)  ->  waiting for Bluetooth connection
    Solid ON                           ->  Pi is connected

  This sketch makes the ESP32 a Bluetooth Serial peripheral that the Pi
  can connect to. It listens for JSON commands and drives a stepper motor
  to elevate the camera mast.

  THIS SKETCH ONLY HANDLES Z (stepper / mast). The wheel motors (X axis)
  are NOT controlled here. We'll add them in Phase 2.

  PROTOCOL (same JSON+newline style as Pi<->Laptop):
    Pi -> ESP32:   {"cmd":"ELEVATE","steps":100}
    ESP32 -> Pi:   {"type":"DONE","steps_done":100}
                or {"type":"ERROR","reason":"..."}

    Pi -> ESP32:   {"cmd":"PING"}
    ESP32 -> Pi:   {"type":"PONG"}

    Pi -> ESP32:   {"cmd":"STOP"}
    ESP32 -> Pi:   {"type":"DONE"}

  WIRING:
    Stepper driver (A4988):
      STEP -> ESP32 GPIO 18
      DIR  -> ESP32 GPIO 19
      ENA  -> ESP32 GPIO 21    (active LOW)
    Motor supply: separate 12V to driver - NOT from ESP32.
    Common ground between A4988 and ESP32 is REQUIRED.

    L298N (DC wheel motors) pins are documented in the sketch for Phase 2.
    They are NOT wired in or used by this sketch.

  BOARD SETUP (Arduino IDE):
    - Tools -> Board -> ESP32 Arduino -> NodeMCU-32S   (or ESP32 Dev Module)
    - Tools -> Port  -> select your USB port

  DEPENDENCIES (Arduino IDE -> Library Manager):
    - ArduinoJson (by Benoit Blanchon, v6.x or v7.x)
    - BluetoothSerial (comes with the ESP32 board package)

  PAIRING ON THE PI (one-time):
    sudo bluetoothctl
      scan on
      pair <MAC of BookFinderESP32>
      trust <MAC>
      exit
    sudo rfcomm bind 0 <MAC> 1

  Then on the Pi side, /dev/rfcomm0 is the serial link.
==============================================================================
*/

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ArduinoJson.h>

// =========================================================================
// PIN CONFIG  --  matches your existing robot wiring
// =========================================================================
// Stepper (A4988) -- Z axis (mast elevation):
const int STEP_PIN = 18;        // STEP pulse to A4988
const int DIR_PIN  = 19;        // DIR pin to A4988
const int ENA_PIN  = 21;        // ENABLE pin to A4988 (active LOW). Set
                                // to -1 if you have it tied to GND instead.

// FOR REFERENCE ONLY -- NOT USED IN PHASE 1
// L298N pins for the DC drive motors (X axis wheels). Phase 2 will use these.
//   ENA = 25, ENB = 26     (PWM speed pins)
//   IN1 = 16, IN2 = 17     (left motor direction)
//   IN3 = 22, IN4 = 23     (right motor direction)

// =========================================================================
// LED CONFIG  --  NodeMCU ESP-32S V1.1 built-in blue LED
// =========================================================================
// The built-in blue LED on the NodeMCU ESP-32S V1.1 is on GPIO 2.
// On this board HIGH = LED ON. If your LED behaves backwards (on when it
// should be off), swap LED_ON and LED_OFF below.
const int LED_PIN   = 2;
const int LED_ON    = HIGH;
const int LED_OFF   = LOW;

// How fast the LED blinks while waiting for a Bluetooth connection.
// 500 ms on / 500 ms off = a calm, slow blink.
// Lower the value for a faster blink (e.g. 200 for an urgent-looking flash).
const unsigned long BLINK_INTERVAL_MS = 500;

// =========================================================================
// MOTION CONFIG  --  TUNE FOR YOUR STEPPER + DRIVER MICROSTEP SETTING
// =========================================================================
const int   STEP_PULSE_US      = 800;   // microseconds per STEP pulse half-cycle.
                                        // Smaller = faster. Too small = missed steps.
                                        // 800us is a safe starting point.
const bool  DIR_UP_LEVEL       = HIGH;  // HIGH or LOW = "up" direction.
                                        // Flip if your mast goes the wrong way.
const int   MAX_STEPS_PER_CMD  = 5000;  // safety cap on single ELEVATE command.

// =========================================================================
// GLOBALS
// =========================================================================
BluetoothSerial SerialBT;
String inputLine = "";              // accumulates bytes until '\n'

// Tracks the id of the last COMPLETED elevate command. Used to make
// retries idempotent: if the Pi re-sends a command (because a response
// was lost over Bluetooth), we recognise the duplicate id and re-send
// the DONE acknowledgement WITHOUT moving the motor a second time.
int lastElevateId = -1;

// LED blink state (only used when no BT client is connected)
unsigned long lastBlinkTime = 0;
bool          ledState      = false;

// =========================================================================
// LED HELPER
// =========================================================================

// Call this every loop iteration.
// - No BT client  -> blink every BLINK_INTERVAL_MS milliseconds
// - BT connected  -> solid ON
//
// Uses millis() so it never blocks, even during long stepper moves the LED
// state will be correct as soon as the motor finishes and loop() resumes.
void updateLED() {
  if (SerialBT.hasClient()) {
    // Pi is connected -> solid blue
    digitalWrite(LED_PIN, LED_ON);
    ledState = true;            // keep state in sync so first blink after
    lastBlinkTime = millis();   // disconnect starts from now
  } else {
    // No connection -> slow blink
    unsigned long now = millis();
    if (now - lastBlinkTime >= BLINK_INTERVAL_MS) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? LED_ON : LED_OFF);
      lastBlinkTime = now;
    }
  }
}

// =========================================================================
// HELPERS
// =========================================================================

// Step the motor `steps` times. Direction is set BEFORE this is called.
// Returns the number of steps actually executed (could be less if interrupted
// in the future - for now always equals `steps`).
int doSteps(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_PULSE_US);
  }
  return steps;
}

// Send a JSON response back to the Pi.
void sendResponse(const JsonDocument& doc) {
  serializeJson(doc, SerialBT);
  SerialBT.print('\n');
}

// Send a simple {"type": ..., (optional reason)} response.
void sendType(const char* type, const char* reason = nullptr) {
  StaticJsonDocument<128> doc;
  doc["type"] = type;
  if (reason != nullptr) doc["reason"] = reason;
  sendResponse(doc);
}

// =========================================================================
// COMMAND HANDLERS
// =========================================================================

void handleElevate(const JsonDocument& cmd) {
  // Get the requested number of steps. Can be positive (up) or negative (down).
  int steps = cmd["steps"] | 0;
  int id    = cmd["id"]    | -1;   // command id from the Pi (-1 if absent)

  // ---- IDEMPOTENCY GUARD ----
  // If this id matches the last command we already completed, the Pi is
  // re-sending because our previous DONE got lost/corrupted over Bluetooth.
  // DO NOT move the motor again - just re-acknowledge so the Pi can proceed.
  if (id >= 0 && id == lastElevateId) {
    StaticJsonDocument<96> resp;
    resp["type"] = "DONE";
    resp["steps_done"] = (steps >= 0 ? steps : -steps);
    resp["id"]  = id;
    resp["dup"] = true;   // tells the Pi this was a duplicate (debug aid)
    sendResponse(resp);
    return;
  }

  if (steps == 0) {
    StaticJsonDocument<96> resp;
    resp["type"] = "DONE";
    resp["steps_done"] = 0;
    resp["id"] = id;
    sendResponse(resp);
    lastElevateId = id;
    return;
  }

  // Safety: cap absurd requests.
  if (abs(steps) > MAX_STEPS_PER_CMD) {
    sendType("ERROR", "too_many_steps");
    return;
  }

  // Set direction
  if (steps > 0) {
    digitalWrite(DIR_PIN, DIR_UP_LEVEL);
  } else {
    digitalWrite(DIR_PIN, !DIR_UP_LEVEL);
    steps = -steps;
  }

  // Enable driver (active LOW on A4988/DRV8825)
  if (ENA_PIN >= 0) digitalWrite(ENA_PIN, LOW);

  int done = doSteps(steps);

  // Mark this command complete BEFORE sending the response, so that if the
  // response is lost and the Pi retries, the idempotency guard above catches it.
  lastElevateId = id;

  StaticJsonDocument<96> resp;
  resp["type"] = "DONE";
  resp["steps_done"] = done;
  resp["id"] = id;
  sendResponse(resp);
}

void handlePing(const JsonDocument& cmd) {
  int id = cmd["id"] | -1;
  StaticJsonDocument<64> resp;
  resp["type"] = "PONG";
  if (id >= 0) resp["id"] = id;
  sendResponse(resp);
}

void handleStop() {
  // For Phase 1 we don't have continuous motion, so STOP just acknowledges.
  // In Phase 2 (wheel movement), STOP will cut motor power immediately.
  sendType("DONE");
}

// =========================================================================
// PARSING + DISPATCH
// =========================================================================

void processCommandLine(const String& line) {
  StaticJsonDocument<256> cmd;
  DeserializationError err = deserializeJson(cmd, line);
  if (err) {
    sendType("ERROR", "bad_json");
    return;
  }

  const char* cmdName = cmd["cmd"] | "";

  if (strcmp(cmdName, "ELEVATE") == 0) {
    handleElevate(cmd);
  } else if (strcmp(cmdName, "PING") == 0) {
    handlePing(cmd);
  } else if (strcmp(cmdName, "STOP") == 0) {
    handleStop();
  } else {
    sendType("ERROR", "unknown_cmd");
  }
}

// =========================================================================
// SETUP + LOOP
// =========================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("Booting elevation_controller...");

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN,  LOW);
  if (ENA_PIN >= 0) {
    pinMode(ENA_PIN, OUTPUT);
    digitalWrite(ENA_PIN, LOW);  // enable driver (active LOW)
  }

  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);   // start OFF; blink begins in loop()

  // Start Bluetooth Serial with a friendly name. The Pi will see this name
  // when scanning for nearby Bluetooth devices.
  SerialBT.begin("BookFinderESP32");
  Serial.println("Bluetooth ready. Device name: BookFinderESP32");

  inputLine.reserve(256);
}

void loop() {
  // Update the status LED on every iteration.
  // Blinks while waiting for BT connection; solid once Pi connects.
  updateLED();

  // Read any available bytes from the Pi.
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n') {
      if (inputLine.length() > 0) {
        processCommandLine(inputLine);
      }
      inputLine = "";
    } else if (c != '\r') {  // ignore carriage returns
      inputLine += c;
      // Safety: discard absurdly long input (avoid memory issues)
      if (inputLine.length() > 240) {
        inputLine = "";
        sendType("ERROR", "line_too_long");
      }
    }
  }
}
