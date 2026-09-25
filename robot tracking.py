import cv2
import numpy as np
import math
import serial
import time
import os


# Center pile coordinates in camera pixel space (Adjusted for left-shifted pile), but check if i need to adjust more
CENTER_PILE = (230.0, 240.0) 

# Avoidance radius in pixels (25 cm clearance)
AVOID_RADIUS = 175.0 

# Right-side drop-off offset (change this after the specifc scale is done using the other 2 codes)
RIGHT_EXIT_OFFSET_PX = 24.0 

# Frame dimensions (Default USB Webcam)
FRAME_WIDTH = 640
FRAME_HEIGHT = 480


# Load Drop-off Zone Targets from map file
drop_off_targets = {}
if os.path.exists("map_drop_off.txt"):
    with open("map_drop_off.txt", "r") as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) == 3:
                drop_off_targets[parts[0].strip()] = (float(parts[1]), float(parts[2]))

# Serial Ports (Adjust COM ports later
esp32 = serial.Serial(port='COM3', baudrate=115200, timeout=0.05) # color sensor and movement
pop32 = serial.Serial(port='COM4', baudrate=115200, timeout=0.05) # Intake & Exit Gate
time.sleep(2)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

# Front (Red) and Back (Blue) Marker HSV Color Bounds   (change colours later)
lower_front_red, upper_front_red = np.array([0, 120, 120]), np.array([10, 255, 255])
lower_back_blue, upper_back_blue = np.array([100, 150, 100]), np.array([130, 255, 255])



def get_marker_center(hsv_frame, lower_bound, upper_bound):
    """Detects tracking marker centroid in HSV frame."""
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
    """Calculates intermediate waypoint around center pile to avoid collisions."""
    angle_robot = math.atan2(robot_pos[1] - center_pos[1], robot_pos[0] - center_pos[0])
    angle_target = math.atan2(target_pos[1] - center_pos[1], target_pos[0] - center_pos[0])
    
    mid_angle = (angle_robot + angle_target) / 2.0
    arc_x = center_pos[0] + radius * math.cos(mid_angle)
    arc_y = center_pos[1] + radius * math.sin(mid_angle)
    return (arc_x, arc_y)

def get_side_drop_target(drop_target, robot_heading_rad):
    """Offsets target point so right-side exit port lands directly over drop zone."""
    tx, ty = drop_target
    target_x = tx + RIGHT_EXIT_OFFSET_PX * math.sin(robot_heading_rad)
    target_y = ty - RIGHT_EXIT_OFFSET_PX * math.cos(robot_heading_rad)
    return (target_x, target_y)


# States: SEARCHING_PILE, COLLECTING, MOVING_TO_ARC, MOVING_TO_DROP
nav_state = "SEARCHING_PILE"
final_target = None
heading_rad = 0.0

# Command initial drive to center collection area
esp32.write(f"SET_GOAL,{CENTER_PILE[0]:.1f},{CENTER_PILE[1]:.1f}\n".encode('utf-8'))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    front_pt = get_marker_center(hsv, lower_front_red, upper_front_red)
    back_pt = get_marker_center(hsv, lower_back_blue, upper_back_blue)

    # 1. Stream Overhead  to ESP32 
    if front_pt and back_pt:
        dx, dy = front_pt[0] - back_pt[0], front_pt[1] - back_pt[1]
        heading_rad = math.atan2(-dy, dx)
        esp32.write(f"POS,{back_pt[0]:.1f},{back_pt[1]:.1f},{heading_rad:.3f}\n".encode('utf-8'))

        # Draw vision tracking debug overlays
        cv2.line(frame, back_pt, front_pt, (0, 255, 0), 2)
        cv2.circle(frame, back_pt, 5, (255, 0, 0), -1)

    # 2. Read Serial Data from ESP32 (Color Sensor + Navigation Feedback)
    if esp32.in_waiting > 0:
        msg = esp32.readline().decode('utf-8', errors='ignore').strip()

        # Handle Color Sensing
        if msg.startswith("DETECTED_COLOR,") and nav_state == "COLLECTING":
            detected_color = msg.split(",")[1]
            print(f"[STONE DETECTED]: {detected_color}")

            if detected_color in drop_off_targets and back_pt:
                raw_target = drop_off_targets[detected_color]
                
                # Offset target for right-side exit port alignment
                final_target = get_side_drop_target(raw_target, heading_rad)
                
                # Calculate clearance arc around center pile
                arc_pt = calculate_arc_waypoint(back_pt, final_target, CENTER_PILE, AVOID_RADIUS)

                # Pause front intake rollers while moving to drop zone
                pop32.write(b"STOP_INTAKE\n")

                # Command intermediate arc target to ESP32
                esp32.write(f"SET_GOAL,{arc_pt[0]:.1f},{arc_pt[1]:.1f}\n".encode('utf-8'))
                nav_state = "MOVING_TO_ARC"

        # Handle Waypoint Arrival Event
        elif msg == "WAYPOINT_REACHED":
            
            if nav_state == "SEARCHING_PILE":
                print("[STATE]: Arrived at center pile. Starting intake feeder...")
                pop32.write(b"START_INTAKE\n")
                nav_state = "COLLECTING"

            elif nav_state == "MOVING_TO_ARC" and final_target:
                print("[STATE]: Cleared central arc. Navigating to drop circle...")
                esp32.write(f"SET_GOAL,{final_target[0]:.1f},{final_target[1]:.1f}\n".encode('utf-8'))
                nav_state = "MOVING_TO_DROP"

            elif nav_state == "MOVING_TO_DROP":
                print("[STATE]: Arrived at drop zone. Opening right exit gate...")
                pop32.write(b"RELEASE_STONE\n")
                time.sleep(1.0) # Allow time for servo to cycle

                print("[STATE]: Stone released. Returning to central pile...")
                esp32.write(f"SET_GOAL,{CENTER_PILE[0]:.1f},{CENTER_PILE[1]:.1f}\n".encode('utf-8'))
                nav_state = "SEARCHING_PILE"

    # Display status on camera feed
    cv2.putText(frame, f"State: {nav_state}", (20, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imshow("Master Autonomous Controller", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()