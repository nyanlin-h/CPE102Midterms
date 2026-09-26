import cv2
import mediapipe as mp
import serial
import time
import os

# ==========================================
# 1. SERIAL COMMUNICATION SETUP
# ==========================================
try:
    esp32 = serial.Serial(port='COM3', baudrate=115200, timeout=0.05)
    pop32 = serial.Serial(port='COM4', baudrate=115200, timeout=0.05)
    time.sleep(2)
    print("[SERIAL]: Connected to ESP32 and POP32 successfully.")
except Exception as e:
    print(f"[SERIAL WARNING]: Could not open COM ports ({e}). Running in simulation mode.")
    esp32 = None
    pop32 = None

# Central Pile Coordinate Target
CENTER_PILE = (230.0, 240.0)

# ==========================================
# 2. MEDIAPIPE TASKS API INITIALIZATION
# ==========================================
model_path = 'hand_landmarker.task'
if not os.path.exists(model_path):
    raise FileNotFoundError(f"Missing '{model_path}'. Please download it into this folder.")

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.IMAGE,
    num_hands=1,
    min_hand_detection_confidence=0.7,
    min_hand_presence_confidence=0.7,
    min_tracking_confidence=0.7
)

# State management to avoid spamming serial commands
current_state = "IDLE"

def classify_gesture(landmarks):
    """
    Classifies gestures based on hand landmarks using the new Tasks API output.
    Landmark IDs: Index Tip (8), Middle Tip (12), Ring Tip (16), Pinky Tip (20).
    PIP Joints: Index (6), Middle (10), Ring (14), Pinky (18).
    """
    index_open = landmarks[8].y < landmarks[6].y
    middle_open = landmarks[12].y < landmarks[10].y
    ring_open = landmarks[16].y < landmarks[14].y
    pinky_open = landmarks[20].y < landmarks[18].y

    # 1. Open Palm (All 4 fingers extended) -> COLLECT
    if index_open and middle_open and ring_open and pinky_open:
        return "COLLECT"

    # 2. Peace Sign (Index & Middle extended, Ring & Pinky closed) -> DROP
    elif index_open and middle_open and not ring_open and not pinky_open:
        return "DROP"

    # 3. Closed Fist (All 4 fingers folded) -> IDLE
    elif not index_open and not middle_open and not ring_open and not pinky_open:
        return "IDLE"

    return None

# ==========================================
# 3. MAIN CAMERA & PROCESSING LOOP
# ==========================================
cap = cv2.VideoCapture(0)

# Instantiate the HandLandmarker task
with HandLandmarker.create_from_options(options) as landmarker:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Convert OpenCV BGR frame to MediaPipe Image format
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        # Process image using the new Tasks API
        detection_result = landmarker.detect(mp_image)

        detected_gesture = None

        if detection_result.hand_landmarks:
            hand_landmarks = detection_result.hand_landmarks[0] # Single hand
            
            # Extract landmarks and classify
            detected_gesture = classify_gesture(hand_landmarks)

            # Draw visual landmarks manually (or visualization utils)
            h, w, _ = frame.shape
            for lm in hand_landmarks:
                cx, cy = int(lm.x * w), int(lm.y * h)
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

        # Handle State Changes & Serial Commands
        if detected_gesture and detected_gesture != current_state:
            current_state = detected_gesture
            print(f"[STATE CHANGE]: New Mode -> {current_state}")

            if current_state == "IDLE":
                if pop32: pop32.write(b"STOP_INTAKE\n")
                if esp32: esp32.write(b"SET_GOAL,0.0,0.0\n")

            elif current_state == "COLLECT":
                if pop32: pop32.write(b"START_INTAKE\n")
                if esp32: esp32.write(f"SET_GOAL,{CENTER_PILE[0]:.1f},{CENTER_PILE[1]:.1f}\n".encode('utf-8'))

            elif current_state == "DROP":
                if pop32: 
                    pop32.write(b"STOP_INTAKE\n")
                    time.sleep(0.1)
                    pop32.write(b"RELEASE_STONE\n")

        # HUD Output
        cv2.putText(frame, f"State: {current_state}", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0) if current_state != "IDLE" else (0, 0, 255), 3)
        
        cv2.imshow("MediaPipe Tasks API Gesture Controller", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()