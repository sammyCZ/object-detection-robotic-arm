#pragma once
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

// Drives the joints through the PCA9685. Motion is non-blocking: moveTo()
// only sets up a trajectory and update() advances it, so loop() stays free.
// Hobby servos give no feedback, so "pose" is always the commanded pose.
class Arm {
 public:
  bool begin();
  bool ready() const { return _ready; }

  // Starts a move from the current pose (works mid-motion too). Targets are
  // clamped to joint limits; all joints arrive together. Returns the duration
  // actually used, which may be longer than requested because of the speed limit.
  // Returns 0 and leaves motion untouched if any target is NaN/inf.
  uint32_t moveTo(const float target[NUM_JOINTS], uint32_t durationMs);
  uint32_t moveJoint(int joint, float deg, uint32_t durationMs);
  uint32_t home(uint32_t durationMs);

  // Freezes at the current interpolated pose immediately.
  void stop();

  // Call every loop(); internally rate-limited to MOTION_UPDATE_MS.
  void update();

  bool isMoving() const { return _moving; }
  // Calibration mode widens every joint to 0-180 so real limits can be found.
  // Turning it off eases any out-of-limit joint back inside its limits.
  void setCalibration(bool on);
  bool calibrating() const { return _calib; }
  void getPose(float out[NUM_JOINTS]) const;
  void getTarget(float out[NUM_JOINTS]) const;

  float clampJoint(int joint, float deg) const;

 private:
  void sample(uint32_t nowMs);
  void writeJoint(int joint, float deg);

  Adafruit_PWMServoDriver _pwm{PCA9685_ADDR, Wire};
  float _start[NUM_JOINTS] = {};
  float _target[NUM_JOINTS] = {};
  float _pose[NUM_JOINTS] = {};
  uint16_t _lastTicks[NUM_JOINTS];
  float _ticksPerUs = 0;
  uint32_t _t0 = 0;
  uint32_t _duration = 0;
  uint32_t _lastUpdate = 0;
  bool _moving = false;
  bool _ready = false;
  bool _calib = false;
};
