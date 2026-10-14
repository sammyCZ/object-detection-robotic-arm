#pragma once
#include <stdint.h>

// ---- I2C / PCA9685 ----
constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;
constexpr uint32_t I2C_CLOCK_HZ = 400000;
// 0x41, not the usual 0x40: this board has its A0 address pad bridged (found via `scan`).
constexpr uint8_t PCA9685_ADDR = 0x41;

// The PCA9685's internal oscillator is nominally 25 MHz but varies per chip
// (roughly 23-27 MHz). If pulse widths measure off on a scope, fix it here
// rather than fudging every joint's pulse range.
constexpr uint32_t PCA9685_OSC_HZ = 25000000;
constexpr float SERVO_FREQ_HZ = 50.0f;

// ---- Joints ----
// Pose order is ALWAYS [base, shoulder, elbow, wrist, gripper].
// This table is the only place that knows about PCA9685 channels.
constexpr int NUM_JOINTS = 5;

struct JointConfig {
  const char* name;
  uint8_t channel;
  float minDeg;     // soft limits: every target is clamped into [minDeg, maxDeg]
  float maxDeg;
  float homeDeg;
  uint16_t us0;     // pulse width at 0 deg
  uint16_t us180;   // pulse width at 180 deg (may be < us0 for a reversed servo)
};

// PLACEHOLDER limits: deliberately narrow around 90 so an uncalibrated arm
// can't drive into its end stops. Widen after calibration.
constexpr JointConfig JOINTS[NUM_JOINTS] = {
  // name        ch  min  max  home  us0   us180
  {"base",       0,  45,  135,  90,   500, 2500},
  {"shoulder",   1,  45,  135,  120,  500, 2500},
  {"elbow",      2,  45,  130,  110,  500, 2500},
  {"wrist",      3,  45,  180,  150,  500, 2500},
  {"gripper",    4,  60,  120,  90,   500, 2500},
};

// ---- Motion ----
constexpr uint32_t MOTION_UPDATE_MS = 20;    // 50 Hz, matches the servo frame rate
constexpr uint32_t DEFAULT_MOVE_MS = 1000;
constexpr uint32_t MAX_MOVE_MS = 10000; 
// Peak joint speed. Short requested durations are stretched so no joint
// exceeds this; protects the gearboxes from "move 90 deg in 0 ms".
constexpr float MAX_JOINT_SPEED_DPS = 120.0f;

// Catch typos in the table at compile time instead of on the hardware.
constexpr bool jointsValid(int i = 0) {
  return i >= NUM_JOINTS ||
         (JOINTS[i].channel < 16 &&
          JOINTS[i].minDeg >= 0 && JOINTS[i].maxDeg <= 180 &&
          JOINTS[i].minDeg < JOINTS[i].maxDeg &&
          JOINTS[i].homeDeg >= JOINTS[i].minDeg &&
          JOINTS[i].homeDeg <= JOINTS[i].maxDeg &&
          jointsValid(i + 1));
}
static_assert(jointsValid(), "JOINTS table has an invalid channel, limit or home angle");
