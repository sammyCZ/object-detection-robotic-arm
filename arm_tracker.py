"""
YOLO11 + ESP32 arm: steer a camera-on-the-gripper arm toward a detected object.

The camera rides on the arm (eye-in-hand), so no calibration is needed. Each frame:
  - object left/right of center  -> rotate the base
  - object above/below center    -> tilt the wrist
  - object smaller than wanted   -> reach forward (shoulder + elbow)
until the object is centered and fills --stop-area of the frame.

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

import argparse #command-line flags
import glob #serial pot (/dev/ttyUSB0)
import sys #mac or linux- check platform for camera
import threading #run background thread to print what ESP-32 says
import time #sleep() and timestamps
import queue

import cv2
import torch #check if GPU is available
from ultralytics import YOLO #model/object detector
from mediapipe_helpers import *
import collections

# Pose order is: base = 0, shoulder = 1, elbow = 2, wrist = 3, gripper = 4 on hardware
# so we make five integer constants 
BASE, SHOULDER, ELBOW, WRIST, GRIPPER = range(5)
# Mirror the firmware's safety clamps in the include/config.h.
# The firmware clamps too, but clamping here stops the controller from "winding up" past a limit.
LIMITS = [(45, 135), (45, 135), (45, 130), (45, 170), (60, 120)]
# Netural resting position
HOME = [90.0, 120.0, 110.0, 150.0, 90.0]
# DETECT = [90.0, 120.0, 110.0, 150.0, 90.0]
# GraspState = ["APPROACHING", "CLOSING", "VERIFYING", "GRASPED", "FAILED"]

HAND_MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
                  "hand_landmarker/float16/latest/hand_landmarker.task")
HAND_MODEL_PATH = ("hand_landmarker.tasks")
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
]

INDEX_FINGERTIP = 8


class ArmLink:
    """Sends pose commands to the ESP32 over USB serial (or just prints them)."""
    # just prints a note, but doesn't open any real connection
    def __init__(self, port, dry_run):
        self.ser = None
        if dry_run:
            print("[dry-run] no serial port; commands are printed instead")
            return
        import serial

        port = port or self._find_port()
        try:
            # Opening the port resets the ESP32
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

def evaluate_grasp(samples, pre_area):
    seen_count = sum(1 for seen, _ in samples if seen)
    if seen_count == 0:
            return "GRASPED"

    areas = [a for seen, a in samples if seen]
    avg_area = sum(areas) / len(areas)
    if pre_area and avg_area < pre_area * 0.5:
        return "FAILED"
    return "FAILED"


def get_box(result):
    boxes = []
    for box in result.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        name = result.names[int(box.cls[0])]
        boxes.append((x1, y1, x2, y2, name, float(box.conf[0])))
    return boxes

# Loops to get user input 
def stdin_target_listener(q: queue.Queue):
    while True: 
        try: 
            text = input("target> ").strip()
            if text:
                q.put(text)
        except(KeyboardInterrupt, EOFError):
            break

def main():
    ap = argparse.ArgumentParser(description="Steer the ESP32 arm toward a YOLO detection")
    ap.add_argument("--model", default="yolo11n.pt")
    ap.add_argument("--camera", type=int, default=0, help="camera index (/dev/videoN)")
    ap.add_argument("--target", default=None,
                    help="class to follow, e.g. bottle, cup, sports ball (default: any)")
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--hands", type=int, default=2, help="max hands to track")
    ap.add_argument("--port", default=None, help="serial port (default: auto-detect)")
    ap.add_argument("--dry-run", action="store_true", help="don't open serial, print commands")
    # Degrees per step at full-frame error. Negative = reverse that axis.
    ap.add_argument("--base-gain", type=float, default=6.0)
    ap.add_argument("--tilt-gain", type=float, default=4.0)
    ap.add_argument("--reach-gain", type=float, default=2.0)
    ap.add_argument("--elbow-ratio", type=float, default=1.0,
                    help="elbow degrees moved per shoulder degree while reaching")
    ap.add_argument("--deadband", type=float, default=0.08,
                    help="ignore centering errors smaller than this fraction of the frame")
    ap.add_argument("--stop-area", type=float, default=0.25,
                    help="stop reaching when the box covers this fraction of the frame")
    ap.add_argument("--rate", type=float, default=8.0, help="commands per second")
    ap.add_argument("--grab", type=float, default=None,
                    help="gripper angle to close to once the object is reached (default: don't grab)")
    ap.add_argument("--grasp-settle-ms", type=float, default=800)
    ap.add_argument("--verify-frames", type=int, default=6)
    ap.add_argument("--open-angle", type=float, default=90.0)
    args = ap.parse_args()

    model = YOLO(args.model)
    if args.target and args.target not in model.names.values():
        raise SystemExit(f"'{args.target}' is not a class of {args.model}. "
                         f"Options: {', '.join(sorted(model.names.values()))}")

    # load  hand model
    landmarker = load_hand_model(args.hands, HAND_MODEL_PATH, HAND_MODEL_URL)

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
    use_hands = True
    last_cmd = 0.0
    paused = False
    reached = False
    mode = "HOME"
    lost_frames = 0
    start = prev = time.time()
    print("Running. space = pause/resume, h = home, m = mediapipe, q = quit")
    wave_step = 'idle'


    state = "APPROACHING"
    state_t0 = 0.0
    pre_grab_area = None
    verify_samples = collections.deque(maxlen=20)

    # Threads yayyy
    target_queue = queue.Queue()
    current_target = args.target
    threading.Thread(target=stdin_target_listener, args=(target_queue,), daemon=True).start()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Camera read failed")
            break

        while not target_queue.empty():
            candidate = target_queue.get_nowait()
            if candidate in model.names.values():
                current_target = candidate
                reached = False
                lost_frames = 0
                # state = "APPROACHING"
            else:
                print(f"'{candidate}' is not an object that this model detects")

        h, w = frame.shape[:2]
        result = model.predict(frame, conf=args.conf, device=device, verbose=False)[0]
        det = pick_target(result, current_target, model.names)

        status = "HOME: H for Home, Press y for YOLO, m for MediaPipe, q to Quit"
        pointing_at = []
        # status = "searching"

        if mode == 'MEDIAPIPE':
            timestamp_ms = int( (time.time() - start) * 1000 )
            hands = detect_hand_postion(landmarker, frame, timestamp_ms)
            # get box from yolo model
            boxes = get_box(result)
            pointing_at = draw(frame, boxes, hands, HAND_CONNECTIONS, INDEX_FINGERTIP)
            state = None
            if time.monotonic() >= home_ready_time and wave_step == 'idle' and detect_waving(hands, state, timestamp_ms):
                pose[BASE] = 70
                arm.pose(pose, 1000)
                wave_step = "moving to 70"
                next_move_time = time.monotonic() + 1
            elif wave_step == "moving to 70" and time.monotonic() >= next_move_time:
                pose[BASE] = 110
                arm.pose(pose, 1000)
                wave_step = "moving to 90"
                next_move_time = time.monotonic() + 1
            elif wave_step == "moving to 90" and time.monotonic() >= next_move_time:
                pose[BASE] = 90
                arm.pose(pose, 1000)
                wave_step = "return home"
                next_move_time = time.monotonic() + 1
            elif wave_step == "return home" and time.monotonic() >= next_move_time:
                wave_step = "cooldown"
                next_move_time = time.monotonic() + 2
            elif wave_step == "cooldown" and time.monotonic() >= next_move_time:
                state.clear()
                wave_step = "idle"



        # result = model.predict(frame, conf=args.conf, device=device, verbose=False)[0]
        # det = pick_target(result, args.target, model.names)
        # status = "HOME: Press y for YOLO, m for MediaPipe, q to Quit"

        if mode == "YOLO":
            result = model.predict(frame, conf=args.conf, device=device, verbose=False)[0]
            det = pick_target(result, current_target, model.names)
            status = f"searching for {current_target}"

            # Everything YOLO sees, in grey, so you can tell "wrong label" from "nothing seen".
            for box, cls, conf in zip(result.boxes.xyxy.tolist(), result.boxes.cls.tolist(),
                                    result.boxes.conf.tolist()):
                bx1, by1, bx2, by2 = map(int, box)
                cv2.rectangle(frame, (bx1, by1), (bx2, by2), (160, 160, 160), 1)
                cv2.putText(frame, f"{model.names[int(cls)]} {conf:.2f}", (bx1, max(by1 - 4, 12)),
                            # if you found the target in this frame
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)

            if det:
                # reset the counter 
                lost_frames = 0 
                (x1, y1, x2, y2), label, conf = det
                # how far the box's center is from the frame's horizontal center
                # normalized to [-1, 1]: + means the object is right of / below the center.
                ex = ((x1 + x2) / 2 - w / 2) / (w / 2)
                # how far the box's center is from the frame's vertical center
                ey = ((y1 + y2) / 2 - h / 2) / (h / 2)
                # calculates how close it is (bigger box means it's closer to the camera)
                area = (x2 - x1) * (y2 - y1) / (w * h)
                centered = abs(ex) < args.deadband and abs(ey) < args.deadband
                # when an item is reached
                reached = reached or (centered and area >= args.stop_area)
                status = f"{label} {conf:.2f}  ex={ex:+.2f} ey={ey:+.2f} area={area:.2f}"

                now = time.time()
                if not paused and not reached and now - last_cmd >= period:
                    new = list(pose)
                    if abs(ex) > args.deadband:
                        new[BASE] -= args.base_gain * ex
                    if abs(ey) > args.deadband:
                        new[WRIST] += args.tilt_gain * ey
                    # Only reach once roughly centered, so it doesn't lunge off to the side.
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

                # if reached and args.grab is not None and pose[GRIPPER] != args.grab:
                #     pose[GRIPPER] = args.grab
                #     pose = clamp_pose(pose)
                #     arm.pose(pose, 800)



                color = (0, 255, 0) if reached else (0, 200, 255)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            else:
                lost_frames += 1
                if lost_frames == 5 and not args.dry_run:
                    arm.send("stop")  # freeze rather than drift on stale commands

            if state == "APPROACHING":
                # if det and centered and area >= args.stop_area:
                #     pre_grab_area = area
                #     pose[GRIPPER] = args.grab
                #     pose = clamp_pose(pose)
                #     arm.pose(pose, 800)
                #     state, state_t0 = "CLOSING", now
                if det and centered and area >= args.stop_area:
                    if args.grab is None:
                        reached = True   # nothing to grasp with; just hold here
                    else:
                        pre_grab_area = area
                        pose[GRIPPER] = args.grab
                        pose = clamp_pose(pose)
                        arm.pose(pose, 800)
                        state, state_t0 = "CLOSING", now
            elif state == "CLOSING":
                if now - state_t0 >= args.grasp_settle_ms / 1000:
                    verify_samples.clear()
                    state, state_t0 = "VERIFYING", now
            elif state == "VERIFYING":
                verify_samples.append((det is not None, area if det else 0.0))
                if len(verify_samples) >= args.verify_frames:
                    result = evaluate_grasp(verify_samples, pre_grab_area)
                    if result == "GRASPED":
                        state = "GRASPED"
                    else:
                        pose[GRIPPER] = args.open_angle
                        pose = clamp_pose(pose)
                        arm.pose(pose, 500)
                        state = "APPROACHING"
                        reached = False
            elif state == "GRASPED":
                pass 

            if reached:
                status = "REACHED  " + status
            if paused:
                status = "PAUSED  " + status

        # Make camera pop up 
        cv2.drawMarker(frame, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        for i, text in enumerate(pointing_at):
            cv2.putText(frame, text, (10, 55 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("arm tracker", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" "):
            paused = not paused
            if paused:
                arm.send("stop")
        if key == ord("h"):
            # pose, reached = list(HOME), False
            # arm.pose(pose, 1500)
            mode = "HOME"
            pose, reached = list(HOME), False
            arm.pose(pose, 1500)
        if key == ord("m"):
            mode = "MEDIAPIPE"
            pose = list(HOME)
            arm.pose(pose, 1500)
            home_ready_time = time.monotonic() + 1.5
            wave_step = "idle"
            state.clear()
        if key == ord("y"):
            mode = "YOLO"
            # pose, reached = list(DETECT), False
            arm.pose(pose, 1500)

    arm.pose(HOME, 1500)
    time.sleep(0.2)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
