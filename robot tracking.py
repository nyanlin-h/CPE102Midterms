import cv2
import numpy as np
import math
import serial
import time
import os



CENTER_PILE = (230.0, 240.0) 
AVOID_RADIUS = 175.0 
RIGHT_EXIT_OFFSET_PX = 24.0 

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

ROBOT_ARUCO_ID = 0

# Initialize ArUco Detector OUTSIDE the main loop for performance
aruco_dict = cv2.aruco.getPrebuiltDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# ==========================================
# 2. FILE & SERIAL INITIALIZATION
# ==========================================

drop_off_targets = {}
if os.path.exists("map_drop_off.txt"):
    with open("map_drop_off.txt", "r") as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) == 3:
                drop_off_targets[parts[0].strip()] = (float(parts[1]), float(parts[2]))

# Serial Connections
esp32 = serial.Serial(port='COM3', baudrate=115200, timeout=0.05) # Drivetrain & TCS3200
pop32 = serial.Serial(port='COM4', baudrate=115200, timeout=0.05) # Intake & Dual-Stage Exit Gate
time.sleep(2)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

# ==========================================
# 3. HELPER FUNCTIONS
# ==========================================

def get_robot_pose_aruco(frame, target_id):     #add here toooooooo
    """Detects ArUco marker and computes position (x, y) & heading in radians."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None and target_id in ids:
        idx = np.where(ids == target_id)[0][0]
        pts = corners[idx][0]  # Corners: [TL, TR, BR, BL]

        # Centroid calculation
        center_x = float(np.mean(pts[:, 0]))
        center_y = float(np.mean(pts[:, 1]))

        # Vector pointing from Top-Left to Top-Right
        dx = pts[1][0] - pts[0][0]
        dy = pts[1][1] - pts[0][1]  # Fixed index typo
        
        # Inverted dy for Cartesian system coordinate mapping
        heading_rad = math.atan2(-dy, dx)

        return (center_x, center_y), heading_rad, corners[idx]

    return None, None, None

def calculate_arc_waypoint(robot_pos, target_pos, center_pos, radius):
    """Calculates clearance arc waypoint to navigate around center obstacle."""
    angle_robot = math.atan2(robot_pos[1] - center_pos[1], robot_pos[0] - center_pos[0])
    angle_target = math.atan2(target_pos[1] - center_pos[1], target_pos[0] - center_pos[0])
    
    mid_angle = (angle_robot + angle_target) / 2.0
    arc_x = center_pos[0] + radius * math.cos(mid_angle)
    arc_y = center_pos[1] + radius * math.sin(mid_angle)
    return (arc_x, arc_y)

def get_side_drop_target(drop_target, robot_heading_rad):
    """Offsets target so right-side release gate aligns directly above drop zone."""
    tx, ty = drop_target
    target_x = tx + RIGHT_EXIT_OFFSET_PX * math.sin(robot_heading_rad)
    target_y = ty - RIGHT_EXIT_OFFSET_PX * math.cos(robot_heading_rad)
    return (target_x, target_y)


nav_state = "SEARCHING_PILE"
final_target = None
heading_rad = 0.0
robot_pt = None

# Set initial destination to central collection pile
esp32.write(f"SET_GOAL,{CENTER_PILE[0]:.1f},{CENTER_PILE[1]:.1f}\n".encode('utf-8'))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    
    curr_pt, curr_heading, marker_corners = get_robot_pose_aruco(frame, ROBOT_ARUCO_ID)   #get arcuo id and fix it hereee

    if curr_pt is not None:
        robot_pt = curr_pt
        heading_rad = curr_heading
        
        # Stream telemetry to ESP32
        esp32.write(f"POS,{robot_pt[0]:.1f},{robot_pt[1]:.1f},{heading_rad:.3f}\n".encode('utf-8'))

        # Draw visual tracking feedback
        cv2.polylines(frame, [np.int32(marker_corners)], True, (0, 255, 0), 2)
        cv2.circle(frame, (int(robot_pt[0]), int(robot_pt[1])), 5, (255, 0, 0), -1)
        
        arrow_end_x = int(robot_pt[0] + 30 * math.cos(-heading_rad))
        arrow_end_y = int(robot_pt[1] + 30 * math.sin(-heading_rad))
        cv2.arrowedLine(frame, (int(robot_pt[0]), int(robot_pt[1])), (arrow_end_x, arrow_end_y), (0, 0, 255), 2)

    # 2. Read Serial Commands (Decoupled from Vision Tracking so commands are never missed)
    if esp32.in_waiting > 0:
        msg = esp32.readline().decode('utf-8', errors='ignore').strip()

        # Handle Color Sensor Trigger
        if msg.startswith("DETECTED_COLOR,") and nav_state == "COLLECTING":
            detected_color = msg.split(",")[1]
            print(f"[STONE DETECTED]: {detected_color}")

            # Notify POP32 of current color status
            pop32.write(f"COLOR,{detected_color}\n".encode('utf-8'))

            if detected_color in drop_off_targets and robot_pt:
                raw_target = drop_off_targets[detected_color]
                
                # Align side exit gate offset
                final_target = get_side_drop_target(raw_target, heading_rad)
                
                # Calculate clearance arc using updated ArUco position (robot_pt)
                arc_pt = calculate_arc_waypoint(robot_pt, final_target, CENTER_PILE, AVOID_RADIUS)

                # Pause front intake rollers
                pop32.write(b"STOP_INTAKE\n")

                # Transmit goal waypoint to ESP32
                esp32.write(f"SET_GOAL,{arc_pt[0]:.1f},{arc_pt[1]:.1f}\n".encode('utf-8'))
                nav_state = "MOVING_TO_ARC"

        # Handle Navigation Events
        elif msg == "WAYPOINT_REACHED":
            
            if nav_state == "SEARCHING_PILE":
                print("[STATE]: Arrived at center pile. Starting intake feeder...")
                pop32.write(b"START_INTAKE\n")
                nav_state = "COLLECTING"

            elif nav_state == "MOVING_TO_ARC" and final_target:
                print("[STATE]: Central arc cleared. Navigating to drop circle...")
                esp32.write(f"SET_GOAL,{final_target[0]:.1f},{final_target[1]:.1f}\n".encode('utf-8'))
                nav_state = "MOVING_TO_DROP"

            elif nav_state == "MOVING_TO_DROP":
                print("[STATE]: Arrived at drop zone. Triggering release mechanism...")
                pop32.write(b"RELEASE_STONE\n")
                time.sleep(1.0) # Allow mechanism time to cycle

                print("[STATE]: Stone released. Returning to central pile...")
                esp32.write(f"SET_GOAL,{CENTER_PILE[0]:.1f},{CENTER_PILE[1]:.1f}\n".encode('utf-8'))
                nav_state = "SEARCHING_PILE"

    # Display HUD
    cv2.putText(frame, f"State: {nav_state}", (20, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imshow("Master Autonomous Controller (ArUco)", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()