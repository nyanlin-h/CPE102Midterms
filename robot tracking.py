import cv2
import numpy as np
import math
import serial
import time


from navigation import FieldNavigator


PX_PER_CM = 3.74
CM_PER_PX = 0.263

AVOID_RADIUS_CM = 35.0         
GATE_OFFSET_CM = 7.0           

CENTER_PILE = (230.0, 240.0)   
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

ROBOT_APRILTAG_ID = 0

# Low-Pass Filter Alpha (0.0 = Max Smooth, 1.0 = Raw Input)
EMA_ALPHA = 0.65

COLOR_PALETTE = {
    "Crimson": (0, 0, 255),
    "Cyan": (255, 255, 0),
    "Lime_Green": (0, 255, 0),
    "Marigold": (0, 165, 255),
    "Sky_Blue": (255, 191, 0),
    "Violet": (238, 130, 238)
}

apriltag_dict = cv2.aruco.getPrebuiltDictionary(cv2.aruco.DICT_APRILTAG_36h11)
apriltag_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(apriltag_dict, apriltag_params)

nav = FieldNavigator(
    map_filename="map_drop_off.txt", 
    center_pile=CENTER_PILE, 
    avoid_radius_cm=AVOID_RADIUS_CM, 
    exit_offset_cm=GATE_OFFSET_CM,
    px_per_cm=PX_PER_CM
)

esp32 = serial.Serial(port='COM3', baudrate=115200, timeout=0.05)
time.sleep(2)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)


def get_robot_pose_apriltag(frame, target_id):
    """
    Detects AprilTag marker and calculates position (x, y) & heading in radians.
    Inverts Y-delta to map OpenCV screen space into standard Cartesian angles.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None and target_id in ids:
        idx = np.where(ids == target_id)[0][0]
        pts = corners[idx][0]  # Corners: [TL, TR, BR, BL]

        center_x = float(np.mean(pts[:, 0]))
        center_y = float(np.mean(pts[:, 1]))

        dx = pts[1][0] - pts[0][0]
        dy = -(pts[1][1] - pts[0][1])  # Invert OpenCV Y-axis
        
        heading_rad = math.atan2(dy, dx)
        return (center_x, center_y), heading_rad, corners[idx]

    return None, None, None

def draw_enhanced_field_ui(frame, drop_targets, center_pile, avoid_radius_px):
    """Draws HUD showing collection zone, avoidance boundary, and color drop zones."""
    overlay = frame.copy()
    cv2.circle(overlay, (int(center_pile[0]), int(center_pile[1])), 45, (0, 255, 255), -1) 
    cv2.circle(overlay, (int(center_pile[0]), int(center_pile[1])), int(avoid_radius_px), (0, 0, 255), -1) 
    
    cv2.addWeighted(overlay, 0.15, frame, 0.85, 0, frame)

    cv2.circle(frame, (int(center_pile[0]), int(center_pile[1])), int(avoid_radius_px), (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, f"AVOIDANCE ZONE ({AVOID_RADIUS_CM:.0f}cm)", (int(center_pile[0]) - 80, int(center_pile[1]) - int(avoid_radius_px) - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

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

# Command initial goal to central pile
esp32.write(f"SET_GOAL,{CENTER_PILE[0]:.1f},{CENTER_PILE[1]:.1f}\n".encode('utf-8'))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    draw_enhanced_field_ui(frame, nav.drop_off_targets, CENTER_PILE, nav.avoid_radius)

    curr_pt, curr_heading, marker_corners = get_robot_pose_apriltag(frame, ROBOT_APRILTAG_ID)

    if curr_pt is not None:
        # Apply Exponential Moving Average (EMA) smoothing to eliminate vision jitter
        if robot_pt is None:
            robot_pt = curr_pt
            heading_rad = curr_heading
        else:
            robot_pt = (
                EMA_ALPHA * curr_pt[0] + (1.0 - EMA_ALPHA) * robot_pt[0],
                EMA_ALPHA * curr_pt[1] + (1.0 - EMA_ALPHA) * robot_pt[1]
            )
            
            # Continuous heading angle interpolation
            dh = curr_heading - heading_rad
            dh = math.atan2(math.sin(dh), math.cos(dh))
            heading_rad += EMA_ALPHA * dh
            heading_rad = math.atan2(math.sin(heading_rad), math.cos(heading_rad))

        # Stream smoothed pose telemetry to ESP32
        esp32.write(f"POS,{robot_pt[0]:.1f},{robot_pt[1]:.1f},{heading_rad:.3f}\n".encode('utf-8'))

        # Visual pose indicators
        cv2.polylines(frame, [np.int32(marker_corners)], True, (255, 0, 0), 2)
        cv2.circle(frame, (int(robot_pt[0]), int(robot_pt[1])), 5, (0, 255, 0), -1)
        
        # Render orientation vector (convert Cartesian heading back to screen coordinates)
        arrow_end_x = int(robot_pt[0] + 30 * math.cos(heading_rad))
        arrow_end_y = int(robot_pt[1] - 30 * math.sin(heading_rad)) 
        cv2.arrowedLine(frame, (int(robot_pt[0]), int(robot_pt[1])), (arrow_end_x, arrow_end_y), (0, 0, 255), 2)

    # Render dynamic navigation paths
    if robot_pt is not None:
        start_pt = (int(robot_pt[0]), int(robot_pt[1]))

        if nav_state in ["SEARCHING_PILE", "COLLECTING"]:
            target_pt = (int(CENTER_PILE[0]), int(CENTER_PILE[1]))
            cv2.line(frame, start_pt, target_pt, (0, 255, 0), 2)

        elif nav_state == "MOVING_TO_ARC" and arc_pt:
            arc_target = (int(arc_pt[0]), int(arc_pt[1]))
            cv2.line(frame, start_pt, arc_target, (0, 165, 255), 2)
            if final_target:
                drop_target = (int(final_target[0]), int(final_target[1]))
                cv2.line(frame, arc_target, drop_target, (255, 0, 255), 2, cv2.LINE_AA)

        elif nav_state == "MOVING_TO_DROP" and final_target:
            drop_target = (int(final_target[0]), int(final_target[1]))
            cv2.line(frame, start_pt, drop_target, (255, 0, 255), 2)

    # Process serial events from ESP32
    if esp32.in_waiting > 0:
        msg = esp32.readline().decode('utf-8', errors='ignore').strip()

        if msg.startswith("DETECTED_COLOR,") and nav_state == "COLLECTING":
            detected_color = msg.split(",")[1]
            print(f"[STONE DETECTED]: {detected_color}")

            if detected_color in nav.drop_off_targets and robot_pt:
                final_target = nav.get_side_drop_target(detected_color, heading_rad)
                arc_pt = nav.calculate_arc_waypoint(robot_pt, final_target)

                esp32.write(b"STOP_INTAKE\n")
                esp32.write(f"SET_GOAL,{arc_pt[0]:.1f},{arc_pt[1]:.1f}\n".encode('utf-8'))
                nav_state = "MOVING_TO_ARC"

        elif msg == "WAYPOINT_REACHED":
            time.sleep(0.3)
            
            if nav_state == "SEARCHING_PILE":
                print("[STATE]: Arrived at center pile. Starting intake feeder...")
                esp32.write(b"START_INTAKE\n")
                nav_state = "COLLECTING"

            elif nav_state == "MOVING_TO_ARC" and final_target:
                print("[STATE]: Arc waypoint cleared. Navigating to drop zone...")
                esp32.write(f"SET_GOAL,{final_target[0]:.1f},{final_target[1]:.1f}\n".encode('utf-8'))
                nav_state = "MOVING_TO_DROP"

            elif nav_state == "MOVING_TO_DROP":
                print("[STATE]: Arrived at drop target. Initiating release sequence...")
                esp32.write(b"RELEASE_STONE\n")
                time.sleep(2.0)

                print("[STATE]: Release complete. Returning to central pile...")
                esp32.write(f"SET_GOAL,{CENTER_PILE[0]:.1f},{CENTER_PILE[1]:.1f}\n".encode('utf-8'))
                nav_state = "SEARCHING_PILE"

        elif msg == "NAV_TIMEOUT_SAFETY_STOP":
            print("[CRITICAL WARNING]: ESP32 executed safety stop due to lost tracking link.")

    cv2.putText(frame, f"State: {nav_state}", (20, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imshow("Master Controller - AprilTag Tracking & Dynamic Paths", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()