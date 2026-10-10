"""
YOLO11 + ESP32 arm: steer a camera-on-the-gripper arm toward a detected object.

The camera rides on the arm (eye-in-hand), so no calibration is needed. Each frame:
  - object left/right of centre  -> rotate the base
  - object above/below centre    -> tilt the wrist
  - object smaller than wanted   -> reach forward (shoulder + elbow)
until the object is centred and fills --stop-area of the frame.

The ESP32 runs the firmware in "ESP32 ARM/" unchanged; this script just sends it
serial commands like "pose 90 90 90 90 90 150".

Usage (inside yolo-env, from the folder with yolo11n.pt):
    python3 arm_tracker.py --dry-run --target bottle   # camera + YOLO only, prints commands
    python3 arm_tracker.py --target bottle             # drive the arm (auto-detects the port)
    python3 arm_tracker.py --target cup --port /dev/ttyUSB0 --base-gain -8
    python3 arm_tracker.py --target cup --port /dev/cu.usbserial-0001          # macOS

If an axis moves the wrong way, flip the sign of its gain (--base-gain, --tilt-gain,
--reach-gain). Start with --reach-gain 0 to tune steering before letting it reach.

Keys:  space = pause/resume motion   h = home   q = quit (returns home)
"""

import argparse
import glob
import sys
import threading
import time

import cv2
import torch
from ultralytics import YOLO

# Pose order is ALWAYS [base, shoulder, elbow, wrist, gripper], same as the firmware.
BASE, SHOULDER, ELBOW, WRIST, GRIPPER = range(5)
# Mirror of the soft limits and home in include/config.h. The firmware clamps too,
# but clamping here stops the controller from "winding up" past a limit.
LIMITS = [(45, 135), (45, 135), (45, 135), (45, 135), (60, 120)]
HOME = [90.0, 90.0, 90.0, 90.0, 90.0]


class ArmLink:
    """Sends pose commands to the ESP32 over USB serial (or just prints them)."""

    def __init__(self, port, dry_run):
        self.ser = None
        if dry_run:
            print("[dry-run] no serial port; commands are printed instead")
            return
        import serial

        port = port or self._find_port()
        try:
            # Opening the port resets the ESP32, so give it time to boot.
            self.ser = serial.Serial(port, 115200, timeout=0.1)
        except serial.SerialException as e:
            raise SystemExit(f"Could not open {port}: {e}\n"
                             "On Linux, if it says 'Permission denied', run:  sudo usermod -aG dialout $USER"
                             "  then reboot. On a Mac, close any other program using the port"
                             " (Arduino/PlatformIO serial monitor).")
        print(f"Connected to {port}, waiting for ESP32 to boot...")
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        # Print the firmware's replies (errors, clamping) in the background.
        threading.Thread(target=self._reader, daemon=True).start()

    @staticmethod
    def _find_port():
        # Linux: ttyUSB/ttyACM. macOS: cu.usbserial-* (CP2102/CH340) or cu.usbmodem*.
        # On a Mac use the cu.* device, not tty.*, which waits for a modem carrier.
        patterns = ["/dev/ttyUSB*", "/dev/ttyACM*", "/dev/cu.usbserial*",
                    "/dev/cu.SLAB_USBtoUART*", "/dev/cu.wchusbserial*", "/dev/cu.usbmodem*"]
        ports = sorted(p for pat in patterns for p in glob.glob(pat))
        if not ports:
            raise SystemExit("No ESP32 found on USB. Plug it in (with a data cable), pass --port, "
                             "or use --dry-run.")
        return ports[0]

    def _reader(self):
        while True:
            try:
                line = self.ser.readline().decode(errors="replace").strip()
            except Exception:
                return
            if line.startswith("error") or line.startswith("ERROR"):
                print("ESP32:", line)

    def send(self, cmd):
        if self.ser is None:
            print("->", cmd)
        else:
            self.ser.write((cmd + "\n").encode())

    def pose(self, p, ms):
        self.send("pose " + " ".join(f"{a:.1f}" for a in p) + f" {int(ms)}")


def pick_device():
    """NVIDIA GPU (Jetson/PC), Apple-silicon GPU (Mac), or CPU, in that order."""
    if torch.cuda.is_available():
        return 0
    if torch.backends.mps.is_available():
        return "mps"
    return "cpu"


def clamp_pose(p):
    return [min(max(a, lo), hi) for a, (lo, hi) in zip(p, LIMITS)]


def pick_target(result, target, names):
    """Return (box_xyxy, label, conf) of the best matching detection, or None."""
    best, best_score = None, 0.0
    for box, cls, conf in zip(result.boxes.xyxy.tolist(), result.boxes.cls.tolist(),
                              result.boxes.conf.tolist()):
        label = names[int(cls)]
        if target and label != target:
            continue
        x1, y1, x2, y2 = box
        score = conf * (x2 - x1) * (y2 - y1)  # favour big, confident detections
        if score > best_score:
            best, best_score = (box, label, conf), score
    return best


def main():
    ap = argparse.ArgumentParser(description="Steer the ESP32 arm toward a YOLO detection")
    ap.add_argument("--model", default="yolo11n.pt")
    ap.add_argument("--camera", type=int, default=0, help="camera index (/dev/videoN)")
    ap.add_argument("--target", default=None,
                    help="class to follow, e.g. bottle, cup, sports ball (default: any)")
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--port", default=None, help="serial port (default: auto-detect)")
    ap.add_argument("--dry-run", action="store_true", help="don't open serial, print commands")
    # Degrees per step at full-frame error. Negative = reverse that axis.
    ap.add_argument("--base-gain", type=float, default=6.0)
    ap.add_argument("--tilt-gain", type=float, default=4.0)
    ap.add_argument("--reach-gain", type=float, default=2.0)
    ap.add_argument("--elbow-ratio", type=float, default=1.0,
                    help="elbow degrees moved per shoulder degree while reaching")
    ap.add_argument("--deadband", type=float, default=0.08,
                    help="ignore centring errors smaller than this fraction of the frame")
    ap.add_argument("--stop-area", type=float, default=0.25,
                    help="stop reaching when the box covers this fraction of the frame")
    ap.add_argument("--rate", type=float, default=8.0, help="commands per second")
    ap.add_argument("--grab", type=float, default=None,
                    help="gripper angle to close to once the object is reached (default: don't grab)")
    args = ap.parse_args()

    model = YOLO(args.model)
    if args.target and args.target not in model.names.values():
        raise SystemExit(f"'{args.target}' is not a class of {args.model}. "
                         f"Options: {', '.join(sorted(model.names.values()))}")

    # V4L2 is Linux-only; elsewhere let OpenCV pick (AVFoundation on a Mac).
    backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else cv2.CAP_ANY
    cap = cv2.VideoCapture(args.camera, backend)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera {args.camera}. On a Mac, allow camera access for "
                         "your terminal app in System Settings > Privacy & Security > Camera.")

    device = pick_device()
    print(f"Running YOLO on: {device}")

    arm = ArmLink(args.port, args.dry_run)
    pose = list(HOME)
    arm.pose(pose, 1500)

    period = 1.0 / args.rate
    last_cmd = 0.0
    paused = False
    reached = False
    lost_frames = 0
    print("Running. space = pause/resume, h = home, q = quit")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Camera read failed")
            break
        h, w = frame.shape[:2]
        result = model.predict(frame, conf=args.conf, device=device, verbose=False)[0]
        det = pick_target(result, args.target, model.names)
        status = "searching"

        # Everything YOLO sees, in grey, so you can tell "wrong label" from "nothing seen".
        for box, cls, conf in zip(result.boxes.xyxy.tolist(), result.boxes.cls.tolist(),
                                  result.boxes.conf.tolist()):
            bx1, by1, bx2, by2 = map(int, box)
            cv2.rectangle(frame, (bx1, by1), (bx2, by2), (160, 160, 160), 1)
            cv2.putText(frame, f"{model.names[int(cls)]} {conf:.2f}", (bx1, max(by1 - 4, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)

        if det:
            lost_frames = 0
            (x1, y1, x2, y2), label, conf = det
            # Errors in [-1, 1]: + means the object is right of / below the centre.
            ex = ((x1 + x2) / 2 - w / 2) / (w / 2)
            ey = ((y1 + y2) / 2 - h / 2) / (h / 2)
            area = (x2 - x1) * (y2 - y1) / (w * h)
            centred = abs(ex) < args.deadband and abs(ey) < args.deadband
            reached = reached or (centred and area >= args.stop_area)
            status = f"{label} {conf:.2f}  ex={ex:+.2f} ey={ey:+.2f} area={area:.2f}"

            now = time.time()
            if not paused and not reached and now - last_cmd >= period:
                new = list(pose)
                if abs(ex) > args.deadband:
                    new[BASE] -= args.base_gain * ex
                if abs(ey) > args.deadband:
                    new[WRIST] += args.tilt_gain * ey
                # Only reach once roughly centred, so it doesn't lunge off to the side.
                if abs(ex) < 2 * args.deadband and area < args.stop_area:
                    step = args.reach_gain * (1 - area / args.stop_area)
                    new[SHOULDER] += step
                    new[ELBOW] += step * args.elbow_ratio
                new = clamp_pose(new)
                if any(abs(a - b) > 0.05 for a, b in zip(new, pose)):
                    pose = new
                    # Duration a bit longer than the send period so motion blends smoothly.
                    arm.pose(pose, period * 1000 * 1.5)
                last_cmd = now

            if reached and args.grab is not None and pose[GRIPPER] != args.grab:
                pose[GRIPPER] = args.grab
                pose = clamp_pose(pose)
                arm.pose(pose, 800)

            color = (0, 255, 0) if reached else (0, 200, 255)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        else:
            lost_frames += 1
            if lost_frames == 5 and not args.dry_run:
                arm.send("stop")  # freeze rather than drift on stale commands

        if reached:
            status = "REACHED  " + status
        if paused:
            status = "PAUSED  " + status
        cv2.drawMarker(frame, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("arm tracker", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" "):
            paused = not paused
            if paused:
                arm.send("stop")
        if key == ord("h"):
            pose, reached = list(HOME), False
            arm.pose(pose, 1500)

    arm.pose(HOME, 1500)
    time.sleep(0.2)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
