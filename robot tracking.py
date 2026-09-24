import cv2
import numpy as np
import math
import serial
import time
import os

# --- Load Drop-off Targets ---
drop_off_targets = {}
if os.path.exists("map_drop_off.txt"):
    with open("map_drop_off.txt", "r") as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) == 3:
                drop_off_targets[parts[0].strip()] = (float(parts[1]), float(parts[2]))

# Central pile center coordinates & safe avoidance radius (in pixels/cm)
CENTER_PILE = (320.0, 240.0)  # Adjust after getting the values from the other code
AVOID_RADIUS = 120.0          # Safe distance around pile to prevent collisions

esp32 = serial.Serial(port='COM3', baudrate=115200, timeout=0.05)   #change port later
time.sleep(2)
cap = cv2.VideoCapture(0)

# Color bounds for robot head/tail markers
lower_front_red, upper_front_red = np.array([2, 88, 144]), np.array([22, 168, 224])
lower_back_blue, upper_back_blue = np.array([8, 247, 196]), np.array([28, 71, 20])

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
    """Calculates an intermediate arc point around the center pile."""
    angle_robot = math.atan2(robot_pos[1] - center_pos[1], robot_pos[0] - center_pos[0])
    angle_target = math.atan2(target_pos[1] - center_pos[1], target_pos[0] - center_pos[0])
    
    mid_angle = (angle_robot + angle_target) / 2.0
    arc_x = center_pos[0] + radius * math.cos(mid_angle)
    arc_y = center_pos[1] + radius * math.sin(mid_angle)
    return (arc_x, arc_y)

current_target = None

while True:
    ret, frame = cap.read()
    if not ret:
        break
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Check for incoming Serial messages from ESP32 (Color Sensor outputs)
    if esp32.in_waiting > 0:
        msg = esp32.readline().decode('utf-8').strip()
        if msg.startswith("DETECTED_COLOR,"):
            detected_color = msg.split(",")[1]
            if detected_color in drop_off_targets:
                target_coords = drop_off_targets[detected_color]
                # Dynamic Arc Calculation
                front_pt = get_marker_center(hsv, lower_front_red, upper_front_red)
                if front_pt:
                    arc_pt = calculate_arc_waypoint(front_pt, target_coords, CENTER_PILE, AVOID_RADIUS)
                    # Send Arc Waypoint first to route around center pile
                    esp32.write(f"SET_GOAL,{arc_pt[0]:.1f},{arc_pt[1]:.1f}\n".encode('utf-8'))
                    current_target = target_coords

    # Track Robot Position and stream telemetry
    front_pt = get_marker_center(hsv, lower_front_red, upper_front_red)
    back_pt = get_marker_center(hsv, lower_back_blue, upper_back_blue)

    if front_pt and back_pt:
        dx, dy = front_pt[0] - back_pt[0], front_pt[1] - back_pt[1]
        heading_rad = math.atan2(-dy, dx)
        esp32.write(f"POS,{back_pt[0]:.1f},{back_pt[1]:.1f},{heading_rad:.3f}\n".encode('utf-8'))

    cv2.imshow("Master Tracker", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()