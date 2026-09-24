import cv2
import numpy as np
import math
import serial
import time
import os

# --- Load Target Locations from Calibration Map ---
drop_off_targets = {}
if os.path.exists("map_drop_off.txt"):
    with open("map_drop_off.txt", "r") as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) == 3:
                drop_off_targets[parts[0].strip()] = (float(parts[1]), float(parts[2]))

# Central pile avoidance parameters (in pixel coordinates)
CENTER_PILE = (320.0, 240.0) 
AVOID_RADIUS = 120.0          

# Serial Ports (Adjust COM ports to match Device Manager)
esp32 = serial.Serial(port='COM3', baudrate=115200, timeout=0.05) # Drivetrain ESP32
pop32 = serial.Serial(port='COM4', baudrate=115200, timeout=0.05) # Mechanism POP32
time.sleep(2)

cap = cv2.VideoCapture(0)

# Corrected HSV Bounds for Markers
lower_front_red, upper_front_red = np.array([0, 120, 120]), np.array([10, 255, 255])
lower_back_blue, upper_back_blue = np.array([100, 150, 100]), np.array([130, 255, 255])

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

# System States: IDLE, MOVING_TO_ARC, MOVING_TO_DROP
nav_state = "IDLE"
final_target = None

while True:
    ret, frame = cap.read()
    if not ret:
        break
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    front_pt = get_marker_center(hsv, lower_front_red, upper_front_red)
    back_pt = get_marker_center(hsv, lower_back_blue, upper_back_blue)

    # 1. Stream Overhead Position Telemetry to ESP32
    if front_pt and back_pt:
        dx, dy = front_pt[0] - back_pt[0], front_pt[1] - back_pt[1]
        heading_rad = math.atan2(-dy, dx)
        esp32.write(f"POS,{back_pt[0]:.1f},{back_pt[1]:.1f},{heading_rad:.3f}\n".encode('utf-8'))

    # 2. Read Color Detections from POP32 Board
    if pop32.in_waiting > 0 and nav_state == "IDLE":
        msg = pop32.readline().decode('utf-8').strip()
        if msg.startswith("DETECTED_COLOR,"):
            detected_color = msg.split(",")[1]
            if detected_color in drop_off_targets and back_pt:
                final_target = drop_off_targets[detected_color]
                arc_pt = calculate_arc_waypoint(back_pt, final_target, CENTER_PILE, AVOID_RADIUS)
                
                # Send intermediate ARC target to ESP32
                esp32.write(f"SET_GOAL,{arc_pt[0]:.1f},{arc_pt[1]:.1f}\n".encode('utf-8'))
                nav_state = "MOVING_TO_ARC"

    # 3. Handle Navigation Progress Events from ESP32
    if esp32.in_waiting > 0:
        msg = esp32.readline().decode('utf-8').strip()
        if msg == "WAYPOINT_REACHED":
            if nav_state == "MOVING_TO_ARC" and final_target:
                # Arc cleared! Send final drop-off target to ESP32
                esp32.write(f"SET_GOAL,{final_target[0]:.1f},{final_target[1]:.1f}\n".encode('utf-8'))
                nav_state = "MOVING_TO_DROP"
                
            elif nav_state == "MOVING_TO_DROP":
                # Arrived at drop-off circle! Signal POP32 to open exit gate
                pop32.write(b"RELEASE_STONE\n")
                nav_state = "IDLE"
                final_target = None

    cv2.imshow("Master Tracker", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()