import cv2
import numpy as np
import math
import serial
import time

# Import Custom Navigation Engine
from navigation import FieldNavigator

# ==========================================
# 1. FIELD & ROBOT CONFIGURATION
# ==========================================

CENTER_PILE = (230.0, 240.0) 
AVOID_RADIUS = 205.0            # Expanded boundary clearance for heavy sliding turn radius
RIGHT_EXIT_OFFSET_PX = 24.0 

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

ROBOT_APRILTAG_ID = 0

# UI Color Map Palette (BGR Format)
COLOR_PALETTE = {
    "Crimson": (0, 0, 255),
    "Cyan": (255, 255, 0),
    "Lime_Green": (0, 255, 0),
    "Marigold": (0, 165, 255),
    "Sky_Blue": (255, 191, 0),
    "Violet": (238, 130, 238)
}

# Initialize AprilTag Detector (36h11 Family)
apriltag_dict = cv2.aruco.getPrebuiltDictionary(cv2.aruco.DICT_APRILTAG_36h11)
apriltag_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(apriltag_dict, apriltag_params)

# Initialize Field Navigator
nav = FieldNavigator(map_filename="map_drop_off.txt", center_pile=CENTER_PILE, avoid_radius=AVOID_RADIUS, exit_offset=RIGHT_EXIT_OFFSET_PX)

# Unified ESP32 Serial Link
esp32 = serial.Serial(port='COM3', baudrate=115200, timeout=0.05)
time.sleep(2)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

# ==========================================
# 2. HELPER & VISUALIZATION FUNCTIONS
# ==========================================

def get_robot_pose_apriltag(frame, target_id):
    """Detects AprilTag marker and computes position (x, y) & heading in radians."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None and target_id in ids:
        idx = np.where(ids == target_id)[0][0]
        pts = corners[idx][0]  # Corners: [TL, TR, BR, BL]

        center_x = float(np.mean(pts[:, 0]))
        center_y = float(np.mean(pts[:, 1]))

        dx = pts[1][0] - pts[0][0]
        dy = pts[1][1] - pts[0][1]
        
        heading_rad = math.atan2(-dy, dx)
        return (center_x, center_y), heading_rad, corners[idx]

    return None, None, None

def draw_enhanced_field_ui(frame, drop_targets, center_pile, avoid_radius):
    """Draws a semi-transparent HUD showing collection zone, avoidance boundary, and color-matched drop zones."""
    overlay = frame.copy()
    cv2.circle(overlay, (int(center_pile[0]), int(center_pile[1])), 45, (0, 255, 255), -1) # Collection inner zone
    cv2.circle(overlay, (int(center_pile[0]), int(center_pile[1])), int(avoid_radius), (0, 0, 255), -1) # Avoidance zone
    
    # Blend layers (15% opacity for transparent zone overlay)
    cv2.addWeighted(overlay, 0.15, frame, 0.85, 0, frame)

    # Avoidance Boundary Ring
    cv2.circle(frame, (int(center_pile[0]), int(center_pile[1])), int(avoid_radius), (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, "AVOIDANCE ZONE", (int(center_pile[0]) - 55, int(center_pile[1]) - int(avoid_radius) - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

    # Color-Matched Target Circles & Crosshairs
    for color_name, (dx, dy) in drop_targets.items():
        center = (int(dx), int(dy))
        color = COLOR_PALETTE.get(color_name, (255, 255, 255))
        
        cv2.circle(frame, center, 16, color, 2, cv2.LINE_AA)
        cv2.line(frame, (center[0] - 6, center[1]), (center[0] + 6, center[1]), color, 1)
        cv2.line(frame, (center[0], center[1] - 6), (center[0], center[1] + 6), color, 1)
        
        cv2.putText(frame, color_name, (center[0] - 25, center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, color_name, (center[0] - 25, center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

# ==========================================
# 3. MAIN STATE MACHINE
# ==========================================

nav_state = "SEARCHING_PILE"
final_target = None
arc_pt = None
heading_rad = 0.0
robot_pt = None

# Command initial drive goal to central collection pile
esp32.write(f"SET_GOAL,{CENTER_PILE[0]:.1f},{CENTER_PILE[1]:.1f}\n".encode('utf-8'))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # --- A. DRAW HUD OVERLAY & TRACK APRILTAG ---
    draw_enhanced_field_ui(frame, nav.drop_off_targets, CENTER_PILE, AVOID_RADIUS)

    curr_pt, curr_heading, marker_corners = get_robot_pose_apriltag(frame, ROBOT_APRILTAG_ID)

    if curr_pt is not None:
        robot_pt = curr_pt
        heading_rad = curr_heading
        
        # Stream live telemetry to ESP32
        esp32.write(f"POS,{robot_pt[0]:.1f},{robot_pt[1]:.1f},{heading_rad:.3f}\n".encode('utf-8'))

        # Render robot pose indicators
        cv2.polylines(frame, [np.int32(marker_corners)], True, (255, 0, 0), 2)
        cv2.circle(frame, (int(robot_pt[0]), int(robot_pt[1])), 5, (0, 255, 0), -1)
        
        arrow_end_x = int(robot_pt[0] + 30 * math.cos(-heading_rad))
        arrow_end_y = int(robot_pt[1] + 30 * math.sin(-heading_rad))
        cv2.arrowedLine(frame, (int(robot_pt[0]), int(robot_pt[1])), (arrow_end_x, arrow_end_y), (0, 0, 255), 2)

    # --- B. DRAW DYNAMIC PATH VECTOR LINES ---
    if robot_pt is not None:
        start_pt = (int(robot_pt[0]), int(robot_pt[1]))

        # Path to Collection Center (Green Line)
        if nav_state in ["SEARCHING_PILE", "COLLECTING"]:
            target_pt = (int(CENTER_PILE[0]), int(CENTER_PILE[1]))
            cv2.line(frame, start_pt, target_pt, (0, 255, 0), 2)

        # Clearance Arc Path (Orange Line -> Magenta Line)
        elif nav_state == "MOVING_TO_ARC" and arc_pt:
            arc_target = (int(arc_pt[0]), int(arc_pt[1]))
            cv2.line(frame, start_pt, arc_target, (0, 165, 255), 2)
            if final_target:
                drop_target = (int(final_target[0]), int(final_target[1]))
                cv2.line(frame, arc_target, drop_target, (255, 0, 255), 2, cv2.LINE_AA)

        # Path directly to Drop Zone (Magenta Line)
        elif nav_state == "MOVING_TO_DROP" and final_target:
            drop_target = (int(final_target[0]), int(final_target[1]))
            cv2.line(frame, start_pt, drop_target, (255, 0, 255), 2)

    # --- C. PARSE SERIAL MESSAGES FROM ESP32 ---
    if esp32.in_waiting > 0:
        msg = esp32.readline().decode('utf-8', errors='ignore').strip()

        # TCS3200 Color Sensor Event
        if msg.startswith("DETECTED_COLOR,") and nav_state == "COLLECTING":
            detected_color = msg.split(",")[1]
            print(f"[STONE DETECTED]: {detected_color}")

            if detected_color in nav.drop_off_targets and robot_pt:
                # Calculate aligned exit gate target & clearance arc waypoint using FieldNavigator
                final_target = nav.get_side_drop_target(detected_color, heading_rad)
                arc_pt = nav.calculate_arc_waypoint(robot_pt, final_target)

                # Stop intake motor
                esp32.write(b"STOP_INTAKE\n")

                # Command ESP32 to clearance arc waypoint
                esp32.write(f"SET_GOAL,{arc_pt[0]:.1f},{arc_pt[1]:.1f}\n".encode('utf-8'))
                nav_state = "MOVING_TO_ARC"

        # Navigation State Transitions
        elif msg == "WAYPOINT_REACHED":
            time.sleep(0.3)  # Short pause to let heavy chassis settle on smooth floor
            
            if nav_state == "SEARCHING_PILE":
                print("[STATE]: Arrived at center pile. Running 12V intake feeder...")
                esp32.write(b"START_INTAKE\n")
                nav_state = "COLLECTING"

            elif nav_state == "MOVING_TO_ARC" and final_target:
                print("[STATE]: Clearance arc reached. Driving to drop zone...")
                esp32.write(f"SET_GOAL,{final_target[0]:.1f},{final_target[1]:.1f}\n".encode('utf-8'))
                nav_state = "MOVING_TO_DROP"

            elif nav_state == "MOVING_TO_DROP":
                print("[STATE]: Arrived at drop zone. Executing servo release sequence...")
                esp32.write(b"RELEASE_STONE\n")
                time.sleep(2.0)  # Wait for dual-stage gate cycle

                print("[STATE]: Stone released. Returning to central pile...")
                esp32.write(f"SET_GOAL,{CENTER_PILE[0]:.1f},{CENTER_PILE[1]:.1f}\n".encode('utf-8'))
                nav_state = "SEARCHING_PILE"

    # Render HUD status
    cv2.putText(frame, f"State: {nav_state}", (20, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imshow("Master Controller - AprilTag Tracking & Dynamic Paths", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()