
#include <Arduino.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

// ======================================================
// Hardware configuration
// ======================================================

const uint8_t DIR_PIN = 5;
const uint8_t STEP_PIN = 6;

const uint16_t MOTOR_FULL_STEPS = 200;

// Must match the actual TMC2209 hardware configuration.
// Adafruit TMC2209 default = 1/8 microstepping.
const uint8_t MICROSTEPS = 8;

// Conservative initial speed settings, in driver pulses.
// Increase only after verifying stable operation.
const float MAX_SPEED = 1600.0f;
const float ACCELERATION = 800.0f;

const uint32_t CONFIG_MAGIC = 0x4D433031UL;

// ======================================================
// Micrometer configuration
// ======================================================

struct MicrometerConfig {
  uint32_t magic;

  float minMM;
  float maxMM;
  float divisionMM;

  // Axial displacement per micrometer spindle revolution
  float pitchMM;

  // Motor revolutions per micrometer spindle revolution
  // Direct coupling = 1.0
  float gearRatio;

  float jogMM;

  // 0 = normal, 1 = reverse positive direction
  uint8_t reverseDirection;
};

MicrometerConfig cfg;
MicrometerConfig draft;

AccelStepper motor(
  AccelStepper::DRIVER,
  STEP_PIN,
  DIR_PIN
);

// ======================================================
// Runtime state
// ======================================================

bool configReady = false;
bool calibrated = false;
bool wizardActive = false;
bool motionActive = false;
bool stopRequested = false;

uint8_t wizardField = 0;

// Absolute micrometer reading corresponding to
// motor.currentPosition() == 0 after calibration.
float originMM = 0.0f;

char inputBuffer[64];
uint8_t inputLength = 0;
bool discardLine = false;

// ======================================================
// Default parameters
// ======================================================

MicrometerConfig defaults() {
  MicrometerConfig c;

  c.magic = CONFIG_MAGIC;

  c.minMM = 0.000f;
  c.maxMM = 25.000f;

  c.divisionMM = 0.001f;
  c.pitchMM = 0.050f;
  c.gearRatio = 1.0f;

  c.jogMM = 0.010f;
  c.reverseDirection = 0;

  return c;
}

// ======================================================
// Validation and conversion
// ======================================================

bool validConfig(const MicrometerConfig &c) {
  if (c.magic != CONFIG_MAGIC) return false;

  if (!(c.minMM > -1000.0f &&
        c.minMM < 1000.0f)) return false;

  if (!(c.maxMM > c.minMM &&
        c.maxMM < 1000.0f)) return false;

  if (!(c.divisionMM > 0.0f &&
        c.divisionMM <= 1.0f)) return false;

  if (!(c.pitchMM > 0.0f &&
        c.pitchMM <= 10.0f)) return false;

  if (!(c.gearRatio >= 0.01f &&
        c.gearRatio <= 1000.0f)) return false;

  if (!(c.jogMM > 0.0f &&
        c.jogMM <= c.maxMM - c.minMM)) return false;

  if (c.reverseDirection > 1) return false;

  float requiredPulses =
    (c.maxMM - c.minMM) *
    MOTOR_FULL_STEPS *
    MICROSTEPS *
    c.gearRatio /
    c.pitchMM;

  // Keep positions safely within a signed 32-bit counter.
  if (!(requiredPulses > 0.0f &&
        requiredPulses < 1000000000.0f)) {
    return false;
  }

  return true;
}

float mmPerPulse() {
  return cfg.pitchMM /
    (MOTOR_FULL_STEPS * MICROSTEPS * cfg.gearRatio);
}

float currentMM() {
  return originMM +
    motor.currentPosition() * mmPerPulse();
}

uint8_t displayDigits() {
  float v = cfg.divisionMM;

  for (uint8_t d = 0; d <= 6; d++) {
    if (fabs(v - round(v)) < 0.0001f) {
      return d;
    }
    v *= 10.0f;
  }

  return 6;
}

bool parseNumber(const char *s, float &out) {
  if (s == NULL || *s == '\0') return false;

  char *endPtr;
  double value = strtod(s, &endPtr);

  if (endPtr == s) return false;

  while (*endPtr == ' ' || *endPtr == '\t') {
    endPtr++;
  }

  if (*endPtr != '\0') return false;

  // Also rejects NaN and infinity.
  if (!(value > -1000000.0 &&
        value < 1000000.0)) return false;

  out = (float)value;
  return true;
}

// ======================================================
// Display
// ======================================================

void showPosition() {
  if (!calibrated) {
    Serial.println(F("Position unknown. Use C <reading>."));
    return;
  }

  Serial.print(F("Estimated position: "));
  Serial.print(currentMM(), displayDigits());
  Serial.println(F(" mm"));

  if (motionActive) {
    Serial.println(F("State: MOVING"));
  } else {
    Serial.println(F("State: HOLDING"));
  }
}

void showHelp() {
  Serial.println();
  Serial.println(F("=== MICROMETER CONTROLLER ==="));
  Serial.println(F("All distances are in mm."));
  Serial.println(F("G 1.250   Absolute target"));
  Serial.println(F("M 0.050   Move +0.050 mm"));
  Serial.println(F("M -0.020  Move -0.020 mm"));
  Serial.println(F("+          Jog positive"));
  Serial.println(F("-          Jog negative"));
  Serial.println(F("J 0.005   Set jog distance"));
  Serial.println(F("C 1.248   Calibrate actual reading"));
  Serial.println(F("P          Show estimated position"));
  Serial.println(F("STOP       Decelerate and stop"));
  Serial.println(F("SETUP      Configure micrometer"));
  Serial.println(F("HELP       Show commands"));
  Serial.println();
}

// ======================================================
// Configuration wizard
// ======================================================

void showWizardPrompt() {
  Serial.println();

  switch (wizardField) {
    case 0:
      Serial.print(F("1/7 Minimum reading (mm) ["));
      Serial.print(draft.minMM, 3);
      break;

    case 1:
      Serial.print(F("2/7 Maximum reading (mm) ["));
      Serial.print(draft.maxMM, 3);
      break;

    case 2:
      Serial.print(F("3/7 Graduation (mm) ["));
      Serial.print(draft.divisionMM, 6);
      break;

    case 3:
      Serial.print(F("4/7 Spindle advance (mm/rev) ["));
      Serial.print(draft.pitchMM, 6);
      break;

    case 4:
      Serial.print(F("5/7 Motor/spindle ratio ["));
      Serial.print(draft.gearRatio, 3);
      break;

    case 5:
      Serial.print(F("6/7 Default jog (mm) ["));
      Serial.print(draft.jogMM, 6);
      break;

    case 6:
      Serial.print(F("7/7 Reverse direction (0/1) ["));
      Serial.print(draft.reverseDirection);
      break;
  }

  Serial.println(F("]:"));
  Serial.println(F("Enter value, or Enter to keep default."));
}

void startWizard() {
  if (motionActive) {
    Serial.println(F("Stop motion before SETUP."));
    return;
  }

  draft = configReady ? cfg : defaults();

  wizardActive = true;
  wizardField = 0;

  Serial.println();
  Serial.println(F("=== MICROMETER SETUP ==="));
  Serial.println(F("Enter parameters in order."));
  showWizardPrompt();
}

void acceptWizardInput(char *text) {
  if (*text != '\0') {
    float v;

    if (!parseNumber(text, v)) {
      Serial.println(F("Invalid number. Try again."));
      return;
    }

    switch (wizardField) {
      case 0: draft.minMM = v; break;
      case 1: draft.maxMM = v; break;
      case 2: draft.divisionMM = v; break;
      case 3: draft.pitchMM = v; break;
      case 4: draft.gearRatio = v; break;
      case 5: draft.jogMM = v; break;

      case 6:
        if (!(v == 0.0f || v == 1.0f)) {
          Serial.println(F("Direction must be 0 or 1."));
          return;
        }
        draft.reverseDirection = (uint8_t)v;
        break;
    }
  }

  wizardField++;

  if (wizardField < 7) {
    showWizardPrompt();
    return;
  }

  draft.magic = CONFIG_MAGIC;

  if (!validConfig(draft)) {
    Serial.println(F("Invalid parameter combination."));
    Serial.println(F("Please re-enter the setup."));
    wizardField = 0;
    showWizardPrompt();
    return;
  }

  cfg = draft;
  EEPROM.put(0, cfg);

  motor.setPinsInverted(
    cfg.reverseDirection != 0,
    false,
    false
  );

  motor.setCurrentPosition(0);

  originMM = 0.0f;
  calibrated = false;
  configReady = true;
  wizardActive = false;

  Serial.println();
  Serial.println(F("Configuration saved to EEPROM."));
  Serial.println(F("Now use C <actual reading> to calibrate."));
  showHelp();
}

// ======================================================
// Motion control
// ======================================================

void gotoMM(float targetMM) {
  if (!calibrated) {
    Serial.println(F("ERROR: Calibrate first with C."));
    return;
  }

  if (motionActive) {
    Serial.println(F("ERROR: Already moving. Use STOP."));
    return;
  }

  if (targetMM < cfg.minMM || targetMM > cfg.maxMM) {
    Serial.println(F("ERROR: Target outside configured range."));
    return;
  }

  float desiredPulses =
    (targetMM - originMM) / mmPerPulse();

  if (!(desiredPulses > -1000000000.0f &&
        desiredPulses < 1000000000.0f)) {
    Serial.println(F("ERROR: Position counter overflow."));
    return;
  }

  long targetSteps = lroundf(desiredPulses);

  if (targetSteps == motor.currentPosition()) {
    Serial.println(F("Already at nearest motor position."));
    showPosition();
    return;
  }

  motor.moveTo(targetSteps);

  motionActive = true;
  stopRequested = false;

  Serial.print(F("Moving to "));
  Serial.print(targetMM, displayDigits());
  Serial.println(F(" mm"));
}

void moveRelativeMM(float deltaMM) {
  if (!calibrated) {
    Serial.println(F("Calibrate first with C."));
    return;
  }

  gotoMM(currentMM() + deltaMM);
}

void calibrateMM(float actualMM) {
  if (motionActive) {
    Serial.println(F("Stop and wait before calibration."));
    return;
  }

  if (actualMM < cfg.minMM || actualMM > cfg.maxMM) {
    Serial.println(F("ERROR: Calibration outside range."));
    return;
  }

  // No motor movement here.
  // The observed mechanical position becomes our reference.
  motor.setCurrentPosition(0);
  originMM = actualMM;
  calibrated = true;

  Serial.println(F("Calibration accepted."));
  showPosition();
}

void requestStop() {
  if (!motionActive) {
    Serial.println(F("Motor is already stationary."));
    return;
  }

  // Controlled deceleration. NOT an emergency power cut.
  motor.stop();
  stopRequested = true;

  Serial.println(F("Decelerating to stop..."));
}

// ======================================================
// Command parser
// ======================================================

void handleCommand(char *line) {
  // Trim leading whitespace.
  while (*line == ' ' || *line == '\t') {
    line++;
  }

  // Trim trailing whitespace.
  size_t n = strlen(line);
  while (n > 0 &&
         (line[n - 1] == ' ' || line[n - 1] == '\t')) {
    line[--n] = '\0';
  }

  if (wizardActive) {
    acceptWizardInput(line);
    return;
  }

  if (*line == '\0') return;

  char *cmd = strtok(line, " \t");
  char *arg = strtok(NULL, " \t");
  char *extra = strtok(NULL, " \t");

  if (extra != NULL) {
    Serial.println(F("ERROR: Too many arguments."));
    return;
  }

  for (char *p = cmd; *p; p++) {
    *p = (char)toupper((unsigned char)*p);
  }

  if (strcmp(cmd, "STOP") == 0) {
    requestStop();
    return;
  }

  if (strcmp(cmd, "HELP") == 0) {
    showHelp();
    return;
  }

  if (strcmp(cmd, "SETUP") == 0) {
    startWizard();
    return;
  }

  if (!configReady) {
    Serial.println(F("Complete SETUP first."));
    return;
  }

  if (strcmp(cmd, "P") == 0) {
    showPosition();
    return;
  }

  if (strcmp(cmd, "+") == 0) {
    moveRelativeMM(cfg.jogMM);
    return;
  }

  if (strcmp(cmd, "-") == 0) {
    moveRelativeMM(-cfg.jogMM);
    return;
  }

  float value;

  if (!parseNumber(arg, value)) {
    Serial.println(F("ERROR: Missing or invalid mm value."));
    return;
  }

  if (strcmp(cmd, "C") == 0) {
    calibrateMM(value);
  }

  else if (strcmp(cmd, "G") == 0) {
    gotoMM(value);
  }

  else if (strcmp(cmd, "M") == 0) {
    moveRelativeMM(value);
  }

  else if (strcmp(cmd, "J") == 0) {
    if (motionActive) {
      Serial.println(F("Change jog after motion stops."));
      return;
    }

    if (!(value > 0.0f &&
          value <= cfg.maxMM - cfg.minMM)) {
      Serial.println(F("ERROR: Invalid jog distance."));
      return;
    }

    cfg.jogMM = value;
    EEPROM.put(0, cfg);

    Serial.print(F("Jog distance = "));
    Serial.print(cfg.jogMM, 6);
    Serial.println(F(" mm"));
  }

  else {
    Serial.println(F("Unknown command. Type HELP."));
  }
}

// ======================================================
// Non-blocking serial input
// ======================================================

void pollSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (!discardLine && inputLength > 0) {
        inputBuffer[inputLength] = '\0';
        handleCommand(inputBuffer);
      }

      inputLength = 0;
      discardLine = false;
    }

    else if (!discardLine) {
      if (inputLength < sizeof(inputBuffer) - 1) {
        inputBuffer[inputLength++] = c;
      } else {
        inputLength = 0;
        discardLine = true;
        Serial.println(F("ERROR: Input line too long."));
      }
    }
  }
}

// ======================================================
// Arduino setup and loop
// ======================================================

void setup() {
  Serial.begin(115200);

  motor.setMaxSpeed(MAX_SPEED);
  motor.setAcceleration(ACCELERATION);
  motor.setMinPulseWidth(5);

  EEPROM.get(0, cfg);

  Serial.println();
  Serial.println(F("MICROMETER CONTROLLER STARTED"));

  if (validConfig(cfg)) {
    configReady = true;

    motor.setPinsInverted(
      cfg.reverseDirection != 0,
      false,
      false
    );

    Serial.println(F("Saved configuration loaded."));
    Serial.println(F("Actual position not yet calibrated."));

    showHelp();
    Serial.println(F("Use C <actual reading> before movement."));
  }

  else {
    configReady = false;
    Serial.println(F("No valid saved configuration."));
    startWizard();
  }
}

void loop() {
  // Non-blocking motor updates.
  motor.run();

  // Process user commands while motor is running.
  pollSerial();

  motor.run();

  // Check motion completion.
  if (motionActive && motor.distanceToGo() == 0) {
    motionActive = false;

    if (stopRequested) {
      Serial.println(F("Motion stopped."));
    } else {
      Serial.println(F("Target command completed."));
    }

    stopRequested = false;

    // Driver remains enabled, holding the position.
    showPosition();
  }
}
