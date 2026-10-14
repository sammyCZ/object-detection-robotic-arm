import os
import cv2
import math
import urllib.request
import mediapipe as mp
from mediapipe.tasks.python import vision
from mediapipe.tasks import python as mp_tasks



def load_hand_model(num_hands, hand_model_path, hand_model_url):
    if not os.path.exists(hand_model_path):
        print("Downloading hand model...")
        urllib.request.urlretrieve(hand_model_url, hand_model_path)

    options = vision.HandLandmarkerOptions(
        base_options=mp_tasks.BaseOptions(model_asset_path=hand_model_path),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=num_hands,
    )

    return vision.HandLandmarker.create_from_options(options)


def detect_hand_postion(landmarker, frame, timestamp_ms):
    # convert from opencv fortmat to mp format
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

    # camera frame to run detection on
    result = landmarker.detect_for_video(mp_image, timestamp_ms)

    # get label for hand postion
    h,w = frame.shape[:2]
    hands = []
    hand_points = result.hand_landmarks
    for i, hand in enumerate(hand_points):
        pixel_postion = [ ( int(lm.x * w), int(lm.y * h) ) for lm in hand ]
        label = result.handedness[i][0].category_name
        hands.append((label, pixel_postion))

    return hands


def draw(frame, boxes, hands, hand_connections, index_fingertip):

    for x1, y1, x2, y2, name, conf in boxes:
        top_left_corner = (x1, y1)
        bot_right_corner = (x2, y2)
        color = (255, 128, 0)
        thickness = 2
        text_pos = x1, max(y1 - 6, 12)
        cv2.rectangle(frame, top_left_corner, bot_right_corner, color, thickness)
        cv2.putText(frame, f"{name} {conf:.2f}", (text_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness) 


    pointing_at = []
    for label, pixel_postion in hands:
        for a, b in hand_connections:
            cv2.line(frame, pixel_postion[a], pixel_postion[b], (255, 255, 255), 2)
        for p in pixel_postion:
            cv2.circle(frame, p, 3, (0, 128, 255), 2)
        tx, ty = pixel_postion[index_fingertip]

        for x1, y1, x2, y2, name, _ in boxes:
            if name != "person" and x1 <= tx <= x2 and y1 <= ty <= y2:
                pointing_at.append(f"{label} hand -> {name}")

    return pointing_at


def detect_waving(hands, wave_state, timestamp_ms):
    # remove history for hands no longer detected
    visible_labels = {label for label, _ in hands}
    for label in list(wave_state):
        if label not in visible_labels:
            del wave_state[label]
    
    for label, landmark_postion in hands:

        if not is_hand_open(landmark_postion):
            wave_state.pop(label, None)
            continue

        wrist = landmark_postion[0]

        if label not in wave_state:
            wave_state[label] = {
                "previous_wrist": wrist,
                "last_seen_ms": timestamp_ms,
                "direction": None,
                "reversals": 0,
                "wave_start_ms": timestamp_ms,
            }
        else:
            # positve is rightward movement, negative is leftward movement
            movement = wrist[0] - wave_state[label]["previous_wrist"][0]
            history = wave_state[label]

            if movement > 5:
                direction = "right"
            elif movement < -5:
                direction = "left"
            else: 
                direction = None

            if direction is not None:
                # is the hand going right and left
                if history["direction"] is not None and direction != history["direction"]:
                    history["reversals"] += 1
                history["direction"] = direction

            history["previous_wrist"] = wrist
            history["last_seen_ms"] = timestamp_ms
            elapsed_ms = timestamp_ms - history["wave_start_ms"]

            # reset wave attempt 
            if elapsed_ms > 2000:
                history["reversals"] = 0
                history["direction"] = None
                history["wave_start_ms"] = timestamp_ms
            elif history["reversals"] >= 2:
                del wave_state[label]
                return True

    return False


def is_hand_open(points):
    for base, joint, tip in [
        (5, 6, 8),      #index
        (9, 10, 12),    #middle
        (13, 14, 16),   #ring
        (17, 18, 20),   #pinku
    ]:
        # get finger postions
        ax = points[base][0] - points[joint][0]
        ay = points[base][1] - points[joint][1]
        bx = points[tip][0] - points[joint][0]
        by = points[tip][1] - points[joint][1]

        # check if finger postions are straight or bent by getting angle of finger 
        length = math.hypot(ax, ay) * math.hypot(bx, by)
        if length == 0:
            return False
        cosine = max(-1.0, min(1.0, (ax * bx + ay * by) / length))
        angle = math.degrees(math.acos(cosine))

        # all fingers must be straightish...
        if angle < 160:
            return False

    return True
            











