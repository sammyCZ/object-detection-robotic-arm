import os
import cv2
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
    hand_points = result.hand_landmarkers
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


def detect_waving(hands, state, timestamp_ms):

    #forget hands that dissapear

    # get wrist postion

    #start tracking new hand or reset after a pause
    None







