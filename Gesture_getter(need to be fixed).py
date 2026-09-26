import cv2
import socket
import mediapipe as mp
from mediapipe.python.solutions import hands as mp_hands
from mediapipe.python.solutions import drawing_utils as mp_draw


# UDP Setup (Sends commands to local machine on port 5005)
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


hands = mp_hands.Hands(
    max_num_hands=1, 
    min_detection_confidence=0.7, 
    min_tracking_confidence=0.7
)

# In the loop where drawing happens:
mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

def detect_gesture(hand_landmarks):
    landmarks = hand_landmarks.landmark
    tips = [8, 12, 16, 20]
    pips = [6, 10, 14, 18]
    extended_fingers = 0
    
    for tip, pip in zip(tips, pips):
        if landmarks[tip].y < landmarks[pip].y:
            extended_fingers += 1
            
    if abs(landmarks[4].x - landmarks[0].x) > 0.1:
        extended_fingers += 1

    if extended_fingers <= 1:
        return "COLLECT"
    elif extended_fingers >= 4:
        return "DROP"
    return "NONE"

cap = cv2.VideoCapture(0) # Laptop camera for operator gestures
current_gesture = "NONE"
gesture_hold_counter = 0

print(f"--- Gesture Detector Active (Transmitting on UDP {UDP_PORT}) ---")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)
    detected_g = "NONE"
    
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            detected_g = detect_gesture(hand_landmarks)

    if detected_g == current_gesture and detected_g != "NONE":
        gesture_hold_counter += 1
    else:
        current_gesture = detected_g
        gesture_hold_counter = 0

    # Hold for ~10 frames (~0.3 sec) to confirm gesture
    if gesture_hold_counter == 11:
        print(f"[UDP OUT]: Transmitting trigger -> {current_gesture}")
        sock.sendto(current_gesture.encode(), (UDP_IP, UDP_PORT))

    cv2.putText(frame, f"Gesture: {detected_g} ({gesture_hold_counter}/10)", (20, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow("Gesture Controller Window", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()