import cv2
import mediapipe as mp
import serial
import time
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


base_options = python.BaseOptions(model_asset_path='hand_landmarker.task')
options = vision.HandLandmarkerOptions(
    base_options=base_options,
    running_mode=vision.RunningMode.VIDEO,  
    num_hands=1,
    min_hand_detection_confidence=0.7,
    min_hand_presence_confidence=0.7,
    min_tracking_confidence=0.7
)
detector = vision.HandLandmarker.create_from_options(options)

# Helper variables to manually draw lines since legacy mp_drawing is gone
# Hand skeleton connections pairings: (Start landmark ID, End landmark ID)
HAND_CONNECTIONS = [
    (0,1), (1,2), (2,3), (3,4),       # Thumb
    (0,5), (5,6), (6,7), (7,8),       # Index
    (9,10), (10,11), (11,12),         # Middle
    (13,14), (14,15), (15,16),        # Ring
    (0,17), (17,18), (18,19), (19,20),# Pinky
    (5,9), (9,13), (13,17)            # Palm
]




try:
    esp32 = serial.Serial(port='COM3', baudrate=115200, timeout=0.05)
    time.sleep(2)
    print("[GESTURE MODULE]: Connected to ESP32 on COM3.")
except Exception as e:
    print(f"[GESTURE ERROR]: Could not connect to Serial: {e}")
    esp32 = None

def classify_hand_gesture(landmarks):
    """
    Counts extended fingers using modern normalized landmarks object.
    """
    fingers_extended = []

    # 1. Thumb (horizontal comparison)
    if landmarks[4].x < landmarks[3].x:  # Assuming right hand facing camera
        fingers_extended.append(1)
    else:
        fingers_extended.append(0)

    # 2. Four Fingers (Vertical comparison)
    finger_tips = [8, 12, 16, 20]
    finger_pips = [6, 10, 14, 18]

    for tip, pip in zip(finger_tips, finger_pips):
        if landmarks[tip].y < landmarks[pip].y:  # Y decreases upward in OpenCV
            fingers_extended.append(1)
        else:
            fingers_extended.append(0)

    total_extended = sum(fingers_extended)

    # Map finger count to commands
    if total_extended == 0:
        return "IDLE"
    elif total_extended == 1 and fingers_extended[1] == 1:
        return "COLLECT"
    elif total_extended >= 4:
        return "DROP"
    
    return "UNKNOWN"

def draw_landmarks_on_frame(frame, landmarks):
    """Replaces legacy mp_drawing utility using standard OpenCV operations"""
    h, w, _ = frame.shape
    # Draw connection lines
    for connection in HAND_CONNECTIONS:
        start_idx, end_idx = connection
        pt1 = (int(landmarks[start_idx].x * w), int(landmarks[start_idx].y * h))
        pt2 = (int(landmarks[end_idx].x * w), int(landmarks[end_idx].y * h))
        cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
    # Draw landmark dots
    for lm in landmarks:
        cx, cy = int(lm.x * w), int(lm.y * h)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), cv2.FILLED)


cap = cv2.VideoCapture(0)
last_gesture = None
gesture_hold_counter = 0
CONFIRMATION_FRAMES = 5

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Modern format conversion for MediaPipe Tasks
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
    
    # VIDEO mode requires an increasing millisecond timestamp 
    frame_timestamp_ms = int(time.time() * 1000)
    results = detector.detect_for_video(mp_image, frame_timestamp_ms)

    current_gesture = "NO_HAND"

    # In modern Tasks API, hand landmarks are nested inside results.hand_landmarks
    if results.hand_landmarks:
        for hand_landmarks in results.hand_landmarks:
            draw_landmarks_on_frame(frame, hand_landmarks)
            current_gesture = classify_hand_gesture(hand_landmarks)

    # --- De-bounce & Send Command ---
    if current_gesture in ["IDLE", "COLLECT", "DROP"]:
        if current_gesture == last_gesture:
            gesture_hold_counter += 1
        else:
            gesture_hold_counter = 0
            last_gesture = current_gesture

        if gesture_hold_counter == CONFIRMATION_FRAMES:
            command_str = f"GESTURE,{current_gesture}\n"
            print(f"[CMD SENT]: {command_str.strip()}")
            if esp32 and esp32.is_open:
                esp32.write(command_str.encode('utf-8'))

    # Display HUD status
    cv2.putText(frame, f"Gesture: {current_gesture}", (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.imshow("Gesture Control Module", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up resources safely
detector.close()
cap.release()
cv2.destroyAllWindows()
