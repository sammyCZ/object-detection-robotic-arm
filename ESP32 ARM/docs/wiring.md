# Wiring: ESP32 + PCA9685 + 5 servos

Source of truth for pins and channels is [`include/config.h`](../include/config.h).
If you change wiring, update both.

## Diagram

```
                         ┌──────────────────────────┐
   USB (laptop / 5V) ───▶│        ESP32 DEVKIT      │
                         │                          │
                         │  3V3  ●──────────────────┼───────┐
                         │  GND  ●──────────────────┼─────┐ │
                         │ GPIO21●──── SDA ─────────┼───┐ │ │
                         │ GPIO22●──── SCL ─────────┼─┐ │ │ │
                         └──────────────────────────┘ │ │ │ │
                                                      │ │ │ │
   ┌──────────────────────────────────────────────────┼─┼─┼─┼────┐
   │  PCA9685 (SunFounder)        side header:        │ │ │ │    │
   │                              GND  ●──────────────┼─┼─┘ │    │
   │                              OE   ●  (leave NC)  │ │   │    │
   │                              SCL  ●──────────────┘ │   │    │
   │                              SDA  ●────────────────┘   │    │
   │                              VCC  ●────────────────────┘    │
   │                              V+   ●  (leave NC!)            │
   │                                                             │
   │   V+ terminal ● ◀── +5V ──┐        servo headers:           │
   │   GND terminal● ◀── GND ──┼──┐      ch: 0  1  2  3  4  ...  │
   │      [ 470-1000µF cap ]   │  │  PWM    ●  ●  ●  ●  ●  (sig) │
   │      across V+ / GND      │  │  V+     ●  ●  ●  ●  ●  (red) │
   │                           │  │  GND    ●  ●  ●  ●  ●  (brn) │
   └───────────────────────────┼──┼──────────┼──┼──┼──┼──┼───────┘
                               │  │          │  │  │  │  │
                     ┌─────────┴──┴──┐       │  │  │  │  └─ base     (pose 0)
                     │  5V PSU  ≥5A  │       │  │  │  └──── shoulder (pose 1)
                     └───────────────┘       │  │  └─────── elbow    (pose 2)
                                             │  └────────── wrist    (pose 3)
                                             └───────────── gripper  (pose 4)
```

## Connections

| From          | To                            | Notes |
|---------------|-------------------------------|-------|
| ESP32 3V3     | PCA9685 VCC                   | Logic power only. Keeps I2C pull-ups at 3.3 V, which is safe for the ESP32. |
| ESP32 GND     | PCA9685 GND (side header)     | Common ground. Nothing works without it. |
| ESP32 GPIO21  | PCA9685 SDA                   | |
| ESP32 GPIO22  | PCA9685 SCL                   | |
| PSU +5V       | PCA9685 V+ **screw terminal** | Servo power. |
| PSU GND       | PCA9685 GND **screw terminal**| Tied on-board to the header GND, completing the common ground. |

## Servo channels

Pose order everywhere (serial, JSON, Python, AI) is
`[base, shoulder, elbow, wrist, gripper]`. Only `config.h` maps pose index to channel.

| Pose index | Joint    | PCA9685 channel |
|-----------:|----------|----------------:|
| 0          | base     | 4 |
| 1          | shoulder | 3 |
| 2          | elbow    | 2 |
| 3          | wrist    | 1 |
| 4          | gripper  | 0 |

## Gotchas

- **Side-header V+: leave it unconnected.** It's tied directly to the servo 5 V
  rail. Connecting it to the ESP32 routes servo current through the devkit or
  back-feeds 5 V into it.
- **Don't power VCC from 5 V.** It pulls the I2C lines to 5 V, and ESP32 pins
  aren't rated for that.
- **Servo plug orientation:** brown/black wire on the GND row (board edge).
  Check the silkscreen, because SunFounder header order can differ from Adafruit's.
- **OE** can float. The board pulls it low, which enables the outputs. Wiring it to a
  spare GPIO is the way to avoid the power-up jump later.
- **Bulk capacitor** (470–1000 µF) across the V+/GND screw terminal absorbs
  current spikes when several servos start at once.
- **PSU sizing:** each servo can pull 1–2.5 A when stalled. Use ≥5 A at 5 V. A sagging
  supply shows up as jitter or brown-out resets.
- **I2C address** on this board is **0x41** (A0 pad bridged), not the default
  0x40 (`PCA9685_ADDR`). Run `scan` on the serial console to check. 0x70 also
  shows up: that is the PCA9685 all-call address, not a second device.
- **Power-up:** servos have no position feedback, so the firmware snaps to the
  home pose at boot. Park the arm near home before powering on.

## First power-up checklist

1. Logic only (ESP32 USB + PCA9685 VCC/GND/SDA/SCL), no servo PSU: flash, open
   the serial monitor, confirm no "no PCA9685" error, run `status`.
2. Connect the servo PSU with the arm parked near home.
3. Test one joint at a time with small moves, e.g. `joint 0 100 2000`.
4. Calibrate real limits and pulse ranges before widening `config.h`.
