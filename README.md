# Temporary solution: run on a macbook 

# YOLO11 + ESP32 robotic arm (Mac setup)

A camera on the gripper watches for an object (YOLO11, COCO classes). The Mac
steers the arm toward it by sending serial commands to the ESP32, which drives
5 servos through a PCA9685.

```
arm_tracker.py      vision + control loop (runs on the Mac)
yolo11n.pt          YOLO11 nano model weights
requirements.txt    Python packages
ESP32 ARM/          PlatformIO firmware for the ESP32 + wiring and calibration docs
```

## 1. Python (one time)

Needs Python 3.10–3.12. In Terminal, from this folder:

```
python3 -m venv yolo-env
source yolo-env/bin/activate
pip install -r requirements.txt
```

On an M1/M2 Mac the script runs YOLO on the Apple GPU ("mps") automatically.

## 2. Camera permission

The first run asks for camera access. If you missed the prompt, enable your
terminal app (Terminal, iTerm, VS Code) under System Settings > Privacy &
Security > Camera, then quit and reopen it.

Camera 0 is usually the MacBook's built-in camera. A USB webcam on the arm is
probably `--camera 1`.

## 3. Firmware (only if the ESP32 needs flashing)

The ESP32 keeps its firmware, so if it already works on the Jetson you can skip this.
To (re)flash: install VS Code + the PlatformIO extension, open the `ESP32 ARM`
folder, and click Upload. macOS has the CP2102 USB driver built in.

## 4. Run

Plug in the ESP32 (with a data cable, not a charge-only one) and connect the
5 V servo supply to the PCA9685 screw terminals (see `ESP32 ARM/docs/wiring.md`).

```
source yolo-env/bin/activate
python3 arm_tracker.py --dry-run --target bottle               # vision only, no arm
python3 arm_tracker.py --target bottle --reach-gain 0          # steer only
python3 arm_tracker.py --target bottle                         # steer + reach
```

The ESP32 shows up as `/dev/cu.usbserial-XXXX` and is found automatically. If not,
list ports with `ls /dev/cu.*` and pass `--port /dev/cu.usbserial-XXXX`.
Close any PlatformIO/Arduino serial monitor first, since only one program can use the port.

- `--target` is any COCO class name (`cup`, `cell phone`, `sports ball`...). Leave it
  out to follow any object.
- If an axis moves away from the object, flip the sign of its gain:
  `--base-gain -6`, `--tilt-gain -4`, `--reach-gain -2`.
- Keys: space = pause, h = home, q = quit (returns home).

## Manual control

```
python3 -m serial.tools.miniterm /dev/cu.usbserial-XXXX 115200
```

Type `help`, `status`, `home`, `joint 0 100 2000`... Exit with Ctrl+].

## Notes

- Joint limits in `ESP32 ARM/include/config.h` are narrow placeholders (45–135°).
  After calibrating (`ESP32 ARM/docs/calibration.md`), update `LIMITS` at the top of
  `arm_tracker.py` to match.
- If the camera sees the object but the arm doesn't move, check the 5 V servo supply:
  the ESP32 and PCA9685 run from USB, but the servos need their own power.
