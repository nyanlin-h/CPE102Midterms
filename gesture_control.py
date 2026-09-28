import cv2
import mediapipe as mp
import serial
import time

# ==========================================
# 1. SETUP MEDIAPIPE & SERIAL LINK
# ==========================================
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

# Serial connection to ESP32 (adjust COM port if needed)
try:
    esp32 = serial.Serial(port='COM3', baudrate=115200, timeout=0.05)
    time.sleep(2)
    print("[GESTURE MODULE]: Connected to ESP32 on COM3.")
except Exception as e:
    print(f"[GESTURE ERROR]: Could not connect to Serial: {e}")
    esp32 = None

# ==========================================
# 2. GESTURE RECOGNITION HELPER
# ==========================================
def classify_hand_gesture(hand_landmarks):
    """
    Counts extended fingers using landmark vertical positions (Y-coordinates).
    Landmark IDs:
    - Thumb: Tip=4, IP=3
    - Index: Tip=8, PIP=6
    - Middle: Tip=12, PIP=10
    - Ring: Tip=16, PIP=14
    - Pinky: Tip=20, PIP=18
    Note: OpenCV Y decreases upwards (Tip Y < PIP Y means finger is EXTENDED).
    """
    lm = hand_landmarks.landmark
    fingers_extended = []

    # 1. Thumb (horizontal comparison for flexibility)
    if lm[4].x < lm[3].x:  # Assuming right hand facing camera
        fingers_extended.append(1)
    else:
        fingers_extended.append(0)

    # 2. Four Fingers (Vertical comparison: Tip higher than PIP joint)
    finger_tips = [8, 12, 16, 20]
    finger_pips = [6, 10, 14, 18]

    for tip, pip in zip(finger_tips, finger_pips):
        if lm[tip].y < lm[pip].y:  # Tip is higher in frame than joint
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

# ==========================================
# 3. MAIN LOOP
# ==========================================
cap = cv2.VideoCapture(1) # Camera index for gesture detection
last_gesture = None
gesture_hold_counter = 0
CONFIRMATION_FRAMES = 5  # De-bounce filter: must hold gesture for 5 frames

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Flip image horizontally for intuitive selfie-view
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)

    current_gesture = "NO_HAND"

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            current_gesture = classify_hand_gesture(hand_landmarks)

    # --- De-bounce & Send Command ---
    if current_gesture in ["IDLE", "COLLECT", "DROP"]:
        if current_gesture == last_gesture:
            gesture_hold_counter += 1
        else:
            gesture_hold_counter = 0
            last_gesture = current_gesture

        # Send command once gesture is held steady
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

cap.release()
cv2.destroyAllWindows()