#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "arm.h"

static Arm arm;

// strtof happily accepts "nan", "inf" and trailing garbage like "90abc";
// none of those should ever reach a servo.
static bool parseAngle(const char* s, float& out) {
  if (!s) return false;
  char* end;
  out = strtof(s, &end);
  return end != s && *end == '\0' && isfinite(out);
}

static bool parseDuration(const char* s, uint32_t& out) {
  if (!s) { out = DEFAULT_MOVE_MS; return true; }
  if (*s == '-') return false;  // strtoul silently wraps negatives
  char* end;
  unsigned long v = strtoul(s, &end, 10);
  if (end == s || *end != '\0' || v > MAX_MOVE_MS) return false;
  out = v;
  return true;
}

static void printPose(const char* label, const float p[NUM_JOINTS]) {
  Serial.print(label);
  for (int j = 0; j < NUM_JOINTS; j++) {
    Serial.print(' ');
    Serial.print(p[j], 1);
  }
  Serial.println();
}

static void printStatus() {
  float p[NUM_JOINTS], t[NUM_JOINTS];
  arm.getPose(p);
  arm.getTarget(t);
  printPose("pose  ", p);
  printPose("target", t);
  Serial.printf("moving=%d ready=%d calib=%d\n", arm.isMoving(), arm.ready(), arm.calibrating());
}

// Answers "is the PCA9685 even on the bus, and at which address?" without a scope.
static void scanI2C() {
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("i2c device at 0x%02X\n", addr);
      found++;
    }
  }
  Serial.printf("scan done, %d device(s)\n", found);
}

static void printHelp() {
  Serial.println("commands (pose order: base shoulder elbow wrist gripper):");
  Serial.println("  pose <b> <s> <e> <w> <g> [ms]");
  Serial.println("  joint <0-4> <deg> [ms]");
  Serial.println("  home [ms]");
  Serial.println("  stop");
  Serial.println("  status");
  Serial.println("  scan   (list I2C devices)");
  Serial.println("  calib on|off   (on = ignore joint limits, full 0-180; serial only)");
  Serial.printf("  ms: 0-%u, default %u (may be stretched by the speed limit)\n",
                (unsigned)MAX_MOVE_MS, (unsigned)DEFAULT_MOVE_MS);
}

static void reportMove(uint32_t ms) {
  float t[NUM_JOINTS];
  arm.getTarget(t);
  Serial.printf("ok duration=%u ", (unsigned)ms);
  printPose("target", t);  // shows any clamping that happened
}

static void handleLine(char* line) {
  char* cmd = strtok(line, " \t");
  if (!cmd) return;
  char* args[NUM_JOINTS + 2];
  int n = 0;
  char* tok;
  while ((tok = strtok(nullptr, " \t")) != nullptr) {
    if (n == (int)(sizeof(args) / sizeof(args[0]))) { Serial.println("error: too many arguments"); return; }
    args[n++] = tok;
  }

  if (!strcmp(cmd, "stop")) {
    arm.stop();
    Serial.println("ok stopped");
  } else if (!strcmp(cmd, "status")) {
    printStatus();
  } else if (!strcmp(cmd, "help")) {
    printHelp();
  } else if (!strcmp(cmd, "scan")) {
    scanI2C();
  } else if (!arm.ready()) {
    Serial.println("error: PCA9685 not initialised");
  } else if (!strcmp(cmd, "calib")) {
    if (n == 1 && !strcmp(args[0], "on")) {
      arm.setCalibration(true);
      Serial.println("ok CALIBRATION MODE: joint limits off (0-180). Move in small steps.");
    } else if (n == 1 && !strcmp(args[0], "off")) {
      arm.setCalibration(false);
      Serial.println("ok calibration off, easing back inside joint limits");
    } else {
      Serial.println("error: usage: calib on|off");
    }
  } else if (!strcmp(cmd, "home")) {
    uint32_t ms;
    if (n > 1 || !parseDuration(n ? args[0] : nullptr, ms)) { Serial.println("error: usage: home [ms]"); return; }
    reportMove(arm.home(ms));
  } else if (!strcmp(cmd, "pose")) {
    float p[NUM_JOINTS];
    uint32_t ms;
    if (n < NUM_JOINTS || n > NUM_JOINTS + 1) { Serial.println("error: pose needs 5 angles [+ ms]"); return; }
    for (int j = 0; j < NUM_JOINTS; j++)
      if (!parseAngle(args[j], p[j])) { Serial.printf("error: bad angle '%s'\n", args[j]); return; }
    if (!parseDuration(n > NUM_JOINTS ? args[NUM_JOINTS] : nullptr, ms)) { Serial.println("error: bad duration"); return; }
    reportMove(arm.moveTo(p, ms));
  } else if (!strcmp(cmd, "joint")) {
    float deg;
    uint32_t ms;
    char* end;
    long idx = n >= 2 ? strtol(args[0], &end, 10) : -1;
    if (n < 2 || n > 3 || *end != '\0' || idx < 0 || idx >= NUM_JOINTS) { Serial.println("error: usage: joint <0-4> <deg> [ms]"); return; }
    if (!parseAngle(args[1], deg)) { Serial.println("error: bad angle"); return; }
    if (!parseDuration(n == 3 ? args[2] : nullptr, ms)) { Serial.println("error: bad duration"); return; }
    reportMove(arm.moveJoint((int)idx, deg, ms));
  } else {
    Serial.printf("error: unknown command '%s' (try 'help')\n", cmd);
  }
}

// Accumulates characters without blocking so motion keeps updating while typing.
static void pollSerial() {
  static char buf[128];
  static size_t len = 0;
  static bool overflow = false;
  while (Serial.available()) {
    char c = (char)Serial.read();
    // Terminals send keystrokes raw: honour backspace, and drop other control
    // or non-ASCII bytes (e.g. pasted non-breaking spaces) that would make a
    // command that looks right on screen fail to match.
    if (c == '\b' || c == 0x7f) { if (len) len--; continue; }
    if (c == '\t') c = ' ';
    if (c != '\n' && (c < 0x20 || c > 0x7e)) continue;
    if (c == '\n') {
      if (overflow) Serial.println("error: line too long");
      else { buf[len] = '\0'; handleLine(buf); }
      len = 0;
      overflow = false;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      overflow = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nESP32 arm starting");
  if (!arm.begin()) {
    Serial.printf("ERROR: no PCA9685 at 0x%02X on SDA=%d SCL=%d. Check wiring/power.\n",
                  PCA9685_ADDR, I2C_SDA_PIN, I2C_SCL_PIN);
  }
  printHelp();
  printStatus();
}

void loop() {
  pollSerial();
  arm.update();
}
