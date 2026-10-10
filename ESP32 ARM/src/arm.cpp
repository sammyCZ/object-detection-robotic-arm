#include "arm.h"
#include <Wire.h>
#include <math.h>

bool Arm::begin() {
  // Wire must be started on our pins first: pwm.begin() calls Wire.begin()
  // with no arguments, which on the ESP32 core is a no-op once the bus is up,
  // so our pins and clock survive.
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  if (!_pwm.begin()) return false;  // false = no ACK at PCA9685_ADDR

  _pwm.setOscillatorFrequency(PCA9685_OSC_HZ);
  _pwm.setPWMFreq(SERVO_FREQ_HZ);

  // The prescaler is an integer, so the real frame rate isn't exactly 50 Hz.
  // Derive ticks/us from the prescale the chip actually holds:
  // one tick = (prescale + 1) / f_osc seconds.
  _ticksPerUs = PCA9685_OSC_HZ / (1e6f * (_pwm.readPrescale() + 1));

  for (int j = 0; j < NUM_JOINTS; j++) {
    _lastTicks[j] = 0xFFFF;  // force the first write
    _pose[j] = _start[j] = _target[j] = JOINTS[j].homeDeg;
  }
  // The servos' real position at power-up is unknown, so there is nothing to
  // ease from: the first write snaps them to home. Power up near home.
  for (int j = 0; j < NUM_JOINTS; j++) writeJoint(j, _pose[j]);

  _lastUpdate = millis();
  _ready = true;
  return true;
}

float Arm::clampJoint(int joint, float deg) const {
  float lo = _calib ? 0.0f : JOINTS[joint].minDeg;
  float hi = _calib ? 180.0f : JOINTS[joint].maxDeg;
  return deg < lo ? lo : (deg > hi ? hi : deg);
}

void Arm::setCalibration(bool on) {
  if (on == _calib) return;
  _calib = on;
  if (!on) {
    float target[NUM_JOINTS];
    getTarget(target);
    moveTo(target, DEFAULT_MOVE_MS);  // moveTo clamps back into the soft limits
  }
}

uint32_t Arm::moveTo(const float target[NUM_JOINTS], uint32_t durationMs) {
  if (!_ready) return 0;
  // NaN slips through min/max comparisons, so refuse the whole move.
  for (int j = 0; j < NUM_JOINTS; j++)
    if (!isfinite(target[j])) return 0;
  uint32_t now = millis();
  // Retarget from where the arm is right now, not from the old start point.
  if (_moving) sample(now);

  float maxDelta = 0;
  for (int j = 0; j < NUM_JOINTS; j++) {
    _start[j] = _pose[j];
    _target[j] = clampJoint(j, target[j]);
    maxDelta = fmaxf(maxDelta, fabsf(_target[j] - _start[j]));
  }
  if (maxDelta < 0.01f) {
    _moving = false;
    return 0;
  }

  // Cosine easing peaks at (pi/2) * delta / T, so this is the shortest T
  // that keeps the fastest joint under MAX_JOINT_SPEED_DPS.
  uint32_t minMs = (uint32_t)ceilf(1000.0f * (float)M_PI_2 * maxDelta / MAX_JOINT_SPEED_DPS);
  if (durationMs > MAX_MOVE_MS) durationMs = MAX_MOVE_MS;
  _duration = durationMs > minMs ? durationMs : minMs;
  _t0 = now;
  _moving = true;
  return _duration;
}

uint32_t Arm::moveJoint(int joint, float deg, uint32_t durationMs) {
  if (joint < 0 || joint >= NUM_JOINTS) return 0;
  float target[NUM_JOINTS];
  getTarget(target);  // other joints keep heading where they were going
  target[joint] = deg;
  return moveTo(target, durationMs);
}

uint32_t Arm::home(uint32_t durationMs) {
  float target[NUM_JOINTS];
  for (int j = 0; j < NUM_JOINTS; j++) target[j] = JOINTS[j].homeDeg;
  return moveTo(target, durationMs);
}

void Arm::stop() {
  if (!_moving) return;
  sample(millis());
  _moving = false;
  for (int j = 0; j < NUM_JOINTS; j++) {
    _target[j] = _pose[j];
    writeJoint(j, _pose[j]);  // don't wait for the next update tick
  }
}

void Arm::update() {
  if (!_ready) return;
  uint32_t now = millis();
  if (now - _lastUpdate < MOTION_UPDATE_MS) return;
  _lastUpdate = now;
  if (!_moving) return;
  sample(now);
  for (int j = 0; j < NUM_JOINTS; j++) writeJoint(j, _pose[j]);
}

void Arm::sample(uint32_t nowMs) {
  uint32_t elapsed = nowMs - _t0;  // unsigned subtraction survives millis() rollover
  if (elapsed >= _duration) {
    for (int j = 0; j < NUM_JOINTS; j++) _pose[j] = _target[j];
    _moving = false;
    return;
  }
  float s = 0.5f - 0.5f * cosf((float)M_PI * elapsed / _duration);
  for (int j = 0; j < NUM_JOINTS; j++) _pose[j] = _start[j] + (_target[j] - _start[j]) * s;
}

void Arm::writeJoint(int joint, float deg) {
  const JointConfig& c = JOINTS[joint];
  // Soft limits are enforced on targets in moveTo(), and every interpolated
  // pose lies between start and target. Clamping to the soft limits here too
  // would make an arm that's outside them (after calibration mode) snap back
  // instead of easing, so only the servo's absolute range is enforced here.
  deg = deg < 0 ? 0 : (deg > 180 ? 180 : deg);
  float us = c.us0 + (float(c.us180) - float(c.us0)) * deg / 180.0f;
  long ticks = lroundf(us * _ticksPerUs);
  if (ticks < 0) ticks = 0;
  if (ticks > 4095) ticks = 4095;
  // Skip unchanged channels to keep I2C traffic down while holding still.
  if ((uint16_t)ticks == _lastTicks[joint]) return;
  _pwm.setPWM(c.channel, 0, (uint16_t)ticks);
  _lastTicks[joint] = (uint16_t)ticks;
}

void Arm::getPose(float out[NUM_JOINTS]) const {
  for (int j = 0; j < NUM_JOINTS; j++) out[j] = _pose[j];
}

void Arm::getTarget(float out[NUM_JOINTS]) const {
  for (int j = 0; j < NUM_JOINTS; j++) out[j] = _target[j];
}
