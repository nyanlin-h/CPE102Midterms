import cv2
import numpy as np
import math
import serial
import time
import os
import mediapipe as mp

# ==========================================
# 1. FIELD & ROBOT CONFIGURATION
# ==========================================

CENTER_PILE = (230.0, 240.0) 
AVOID_RADIUS = 175.0 
RIGHT_EXIT_OFFSET_PX = 24.0 

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# ==========================================
# 2. CALIBRATION & SERIAL SETUP
# ==========================================

drop_off_targets = {}
if os.path.exists("map_drop_off.txt"):
    with open("map_drop_off.txt", "r") as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) == 3:
                drop_off_targets[parts[0].strip()] = (float(parts[1]), float(parts[2]))

esp32 = serial.Serial(port='COM3', baudrate=115200, timeout=0.05) # Drivetrain + TCS3200
pop32 = serial.Serial(port='COM4', baudrate=115200, timeout=0.05) # Intake & Exit Gate
time.sleep(2)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

# Front (Red) and Back (Blue) Marker HSV Color Bounds
lower_front_red, upper_front_red = np.array([0, 120, 120]), np.array([10, 255, 255])
lower_back_blue, upper_back_blue = np.array([100, 150, 100]), np.array([130, 255, 255])

# ==========================================
# 3. MEDIAPIPE GESTURE RECOGNITION SETUP
# ==========================================

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

def detect_gesture(hand_landmarks):
    """
    Detects basic gestures based on extended finger count:
    - FIST / THUMBS UP (0 or 1 finger): "COLLECT"
    - OPEN PALM (4 or 5 fingers): "DROP"
    """
    landmarks = hand_landmarks.landmark
    
    # Landmark IDs for finger tips and PIP joints
    tips = [8, 12, 16, 20] # Index, Middle, Ring, Pinky
    pips = [6, 10, 14, 18]
    
    extended_fingers = 0
    
    # Check four main fingers
    for tip, pip in zip(tips, pips):
        if landmarks[tip].y < landmarks[pip].y:
            extended_fingers += 1
            
    # Check thumb extension horizontal vs wrist
    if abs(landmarks[4].x - landmarks[0].x) > 0.1:
        extended_fingers += 1

    if extended_fingers <= 1:
        return "GESTURE_COLLECT"  # Closed fist or single thumb
    elif extended_fingers >= 4:
        return "GESTURE_DROP"     # Open hand / 4-5 extended fingers
    
    return "NONE"

# ==========================================
# 4. HELPER FUNCTIONS
# ==========================================

def get_marker_center(hsv_frame, lower_bound, upper_bound):
    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 100:
            M = cv2.moments(largest)
            if M["m00"] != 0:
                return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    return None

def calculate_arc_waypoint(robot_pos, target_pos, center_pos, radius):
    angle_robot = math.atan2(robot_pos[1] - center_pos[1], robot_pos[0] - center_pos[0])
    angle_target = math.atan2(target_pos[1] - center_pos[1], target_pos[0] - center_pos[0])
    mid_angle = (angle_robot + angle_target) / 2.0
    arc_x = center_pos[0] + radius * math.cos(mid_angle)
    arc_y = center_pos[1] + radius * math.sin(mid_angle)
    return (arc_x, arc_y)

def get_side_drop_target(drop_target, robot_heading_rad):
    tx, ty = drop_target
    target_x = tx + RIGHT_EXIT_OFFSET_PX * math.sin(robot_heading_rad)
    target_y = ty - RIGHT_EXIT_OFFSET_PX * math.cos(robot_heading_rad)
    return (target_x, target_y)

# ==========================================
# 5. MAIN STATE MACHINE EXECUTION
# ==========================================

# System States: WAITING_FOR_COLLECT_GESTURE, SEARCHING_PILE, COLLECTING, WAITING_FOR_DROP_GESTURE, MOVING_TO_ARC, MOVING_TO_DROP
nav_state = "WAITING_FOR_COLLECT_GESTURE"
final_target = None
detected_color = None
heading_rad = 0.0

gesture_hold_counter = 0
CURRENT_GESTURE = "NONE"

while True:
    ret, frame = cap.read()
    if not ret:
        break

    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 1. Process Hand Gestures
    results = hands.process(rgb_frame)
    detected_g = "NONE"
    
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            detected_g = detect_gesture(hand_landmarks)

    # Debounce gesture input to avoid false triggers
    if detected_g == CURRENT_GESTURE and detected_g != "NONE":
        gesture_hold_counter += 1
    else:
        CURRENT_GESTURE = detected_g
        gesture_hold_counter = 0

    # Gesture confirmed if held for ~10 frames (~0.3 sec)
    GESTURE_CONFIRMED = CURRENT_GESTURE if gesture_hold_counter > 10 else "NONE"

    # 2. Track Robot Position & Stream Telemetry
    front_pt = get_marker_center(hsv, lower_front_red, upper_front_red)
    back_pt = get_marker_center(hsv, lower_back_blue, upper_back_blue)

    if front_pt and back_pt:
        dx, dy = front_pt[0] - back_pt[0], front_pt[1] - back_pt[1]
        heading_rad = math.atan2(-dy, dx)
        esp32.write(f"POS,{back_pt[0]:.1f},{back_pt[1]:.1f},{heading_rad:.3f}\n".encode('utf-8'))

        cv2.line(frame, back_pt, front_pt, (0, 255, 0), 2)
        cv2.circle(frame, back_pt, 5, (255, 0, 0), -1)

    # 3. Read Serial Detections from ESP32
    if esp32.in_waiting > 0:
        msg = esp32.readline().decode('utf-8', errors='ignore').strip()

        if msg.startswith("DETECTED_COLOR,") and nav_state == "COLLECTING":
            detected_color = msg.split(",")[1]
            print(f"[STONE READ]: {detected_color}")
            pop32.write(b"STOP_INTAKE\n")
            nav_state = "WAITING_FOR_DROP_GESTURE"

        elif msg == "WAYPOINT_REACHED":
            if nav_state == "SEARCHING_PILE":
                print("[ROBOT]: Arrived at center pile. Starting intake feeder...")
                pop32.write(b"START_INTAKE\n")
                nav_state = "COLLECTING"

            elif nav_state == "MOVING_TO_ARC" and final_target:
                print("[ROBOT]: Cleared pile arc. Driving to drop zone...")
                esp32.write(f"SET_GOAL,{final_target[0]:.1f},{final_target[1]:.1f}\n".encode('utf-8'))
                nav_state = "MOVING_TO_DROP"

            elif nav_state == "MOVING_TO_DROP":
                print("[ROBOT]: At drop circle. Releasing stone...")
                pop32.write(b"RELEASE_STONE\n")
                time.sleep(1.0)
                nav_state = "WAITING_FOR_COLLECT_GESTURE"

    # 4. Handle State Transitions via Gestures
    if nav_state == "WAITING_FOR_COLLECT_GESTURE":
        if GESTURE_CONFIRMED == "GESTURE_COLLECT":
            print("[ACTION TRIGGERED]: Collect command received! Driving to center...")
            esp32.write(f"SET_GOAL,{CENTER_PILE[0]:.1f},{CENTER_PILE[1]:.1f}\n".encode('utf-8'))
            nav_state = "SEARCHING_PILE"
            gesture_hold_counter = 0

    elif nav_state == "WAITING_FOR_DROP_GESTURE":
        if GESTURE_CONFIRMED == "GESTURE_DROP":
            if detected_color in drop_off_targets and back_pt:
                print("[ACTION TRIGGERED]: Drop command received! Calculating path...")
                raw_target = drop_off_targets[detected_color]
                final_target = get_side_drop_target(raw_target, heading_rad)
                arc_pt = calculate_arc_waypoint(back_pt, final_target, CENTER_PILE, AVOID_RADIUS)

                esp32.write(f"SET_GOAL,{arc_pt[0]:.1f},{arc_pt[1]:.1f}\n".encode('utf-8'))
                nav_state = "MOVING_TO_ARC"
                gesture_hold_counter = 0

    # 5. On-Screen GUI Overlay
    cv2.putText(frame, f"State: {nav_state}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(frame, f"Gesture: {GESTURE_CONFIRMED}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    
    cv2.imshow("Gesture Controlled Tracker", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()