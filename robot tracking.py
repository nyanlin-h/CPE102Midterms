import cv2
import numpy as np
import math
import socket
import time
import ctypes

# --- Windows High-DPI Scaling Fix ---
# Prevents Windows from cropping/clipping OpenCV display windows on scaled screens
try:
    ctypes.windll.shcore.SetProcessDpiAwareness(2)
except Exception:
    try:
        ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass

from navigation import FieldNavigator
from config import (CAMERA_INDEX, CENTER_PILE, ROBOT_APRILTAG_ID, 
                    AVOID_RADIUS_CM, ESP32_IP, UDP_PORT)

# --- UDP SOCKET CONFIGURATION ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", UDP_PORT))
sock.setblocking(False)

EMA_ALPHA = 0.65

COLOR_PALETTE = {
    "Crimson": (0, 0, 255),
    "Cyan": (255, 255, 0),
    "Lime_Green": (0, 255, 0),
    "Marigold": (0, 165, 255),
    "Sky_Blue": (255, 191, 0),
    "Violet": (238, 130, 238),
}

# --- APRILTAG DETECTOR ---
apriltag_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
detector = cv2.aruco.ArucoDetector(apriltag_dict, cv2.aruco.DetectorParameters())

nav = FieldNavigator()

# --- AUTO-DETECT NATIVE FULL-FIELD RESOLUTION ---
def get_native_camera(cam_idx):
    """Detects camera without forcing crop, falling back gracefully to standard full-FOV resolutions."""
    # Try DirectShow first, then default backend
    for backend in [cv2.CAP_DSHOW, cv2.CAP_ANY]:
        cap = cv2.VideoCapture(cam_idx, backend)
        if cap.isOpened():
            break

    if not cap.isOpened():
        return None, 0, 0

    # Try standard resolution sets (both 16:9 and native 4:3 full-sensor modes)
    test_resolutions = [
        (1280, 720),  # 16:9 HD
        (1280, 960),  # 4:3 HD
        (1024, 768),  # 4:3 Standard
        (800, 600),   # 4:3 Standard
        (640, 480)    # 4:3 VGA
    ]

    for w, h in test_resolutions:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        if actual_w > 0 and actual_h > 0:
            print(f"[CAMERA]: Active Full-FOV Resolution: {actual_w}x{actual_h} (Aspect Ratio: {actual_w/actual_h:.2f})")
            return cap, actual_w, actual_h

    # Fallback to default stream parameters
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return cap, actual_w, actual_h


cap, FRAME_WIDTH, FRAME_HEIGHT = get_native_camera(CAMERA_INDEX)

if cap is None or not cap.isOpened():
    for alt_idx in [0, 1, 2, 3]:
        cap, FRAME_WIDTH, FRAME_HEIGHT = get_native_camera(alt_idx)
        if cap and cap.isOpened():
            print(f"[CAMERA]: Connected on fallback index {alt_idx}")
            break

# --- ASPECT RATIO SANITIZED WINDOW ---
cv2.namedWindow("Master Controller", cv2.WINDOW_NORMAL)

# Dynamically scale window dimensions to fit screen without cropping
target_win_w = 960
target_win_h = int(target_win_w / (FRAME_WIDTH / FRAME_HEIGHT)) if FRAME_HEIGHT > 0 else 540
cv2.resizeWindow("Master Controller", target_win_w, target_win_h)


def send(text):
    """Transmits UTF-8 text packet over UDP to ESP32."""
    try:
        sock.sendto((text + "\n").encode("utf-8"), (ESP32_IP, UDP_PORT))
    except Exception as e:
        print(f"[UDP ERROR]: {e}")


def send_goal(pt):
    """Transmits target coordinates to ESP32."""
    if pt is not None:
        send(f"SET_GOAL,{pt[0]:.1f},{pt[1]:.1f}")


def get_robot_pose_apriltag(frame, target_id):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None and target_id in ids:
        idx = np.where(ids == target_id)[0][0]
        pts = corners[idx][0]
        center = (float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1])))
        dx = pts[1][0] - pts[0][0]
        dy = -(pts[1][1] - pts[0][1])
        return center, math.atan2(dy, dx), corners[idx]
    return None, None, None


def draw_field_ui(frame, drop_targets, center_pile, avoid_radius_px):
    overlay = frame.copy()
    c = (int(center_pile[0]), int(center_pile[1]))
    
    cv2.circle(overlay, c, 45, (0, 255, 255), -1)
    cv2.circle(overlay, c, int(avoid_radius_px), (0, 0, 255), -1)
    cv2.addWeighted(overlay, 0.15, frame, 0.85, 0, frame)

    cv2.circle(frame, c, int(avoid_radius_px), (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, f"AVOIDANCE ZONE ({AVOID_RADIUS_CM:.0f}cm)",
                (c[0] - 80, c[1] - int(avoid_radius_px) - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

    for name, (dx, dy) in drop_targets.items():
        p = (int(dx), int(dy))
        col = COLOR_PALETTE.get(name, (255, 255, 255))
        cv2.circle(frame, p, 16, col, 2, cv2.LINE_AA)
        cv2.line(frame, (p[0] - 6, p[1]), (p[0] + 6, p[1]), col, 1)
        cv2.line(frame, (p[0], p[1] - 6), (p[0], p[1] + 6), col, 1)
        cv2.putText(frame, name, (p[0] - 25, p[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, name, (p[0] - 25, p[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)


nav_state = "SEARCHING_PILE"
final_target = None
arc_pt = None
heading_rad = 0.0
robot_pt = None

last_goal_send_time = 0.0
last_ping_time = 0.0


def handle_message(msg):
    global nav_state, final_target, arc_pt, last_goal_send_time, heading_rad, robot_pt

    print(f"[UDP RECV]: {msg}")

    if msg.startswith("DETECTED_COLOR,") and nav_state in ("SEARCHING_PILE", "COLLECTING"):
        parts = msg.split(",")
        if len(parts) < 2: return
        detected = parts[1].strip()

        if detected not in COLOR_PALETTE or robot_pt is None:
            return

        target = nav.get_side_drop_target(detected, heading_rad)
        if target is None or target == (0.0, 0.0): return

        send("STOP_INTAKE")
        final_target = target
        arc_pt = nav.calculate_arc_waypoint(robot_pt, final_target)
        send_goal(arc_pt)
        last_goal_send_time = time.time()
        nav_state = "MOVING_TO_ARC"

    elif msg == "WAYPOINT_REACHED":
        if nav_state == "SEARCHING_PILE":
            send("START_INTAKE")
            nav_state = "COLLECTING"

        elif nav_state == "MOVING_TO_ARC" and final_target:
            send_goal(final_target)
            last_goal_send_time = time.time()
            nav_state = "MOVING_TO_DROP"

        elif nav_state == "MOVING_TO_DROP":
            send("RELEASE_STONE")
            nav_state = "RELEASING"

    elif msg == "RELEASE_DONE" and nav_state == "RELEASING":
        send_goal(CENTER_PILE)
        last_goal_send_time = time.time()
        nav_state = "SEARCHING_PILE"


# Handshake on start
send("PING")
send_goal(CENTER_PILE)
last_ping_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        print("[ERROR]: Camera feed disconnected.")
        break

    draw_field_ui(frame, nav.drop_off_targets, CENTER_PILE, nav.avoid_radius)

    curr_pt, curr_heading, marker_corners = get_robot_pose_apriltag(frame, ROBOT_APRILTAG_ID)

    if curr_pt is not None:
        if robot_pt is None:
            robot_pt = curr_pt
            heading_rad = curr_heading
        else:
            robot_pt = (EMA_ALPHA * curr_pt[0] + (1 - EMA_ALPHA) * robot_pt[0],
                        EMA_ALPHA * curr_pt[1] + (1 - EMA_ALPHA) * robot_pt[1])
            dh = math.atan2(math.sin(curr_heading - heading_rad), math.cos(curr_heading - heading_rad))
            heading_rad = math.atan2(math.sin(heading_rad + EMA_ALPHA * dh), math.cos(heading_rad + EMA_ALPHA * dh))

        send(f"POS,{robot_pt[0]:.1f},{robot_pt[1]:.1f},{heading_rad:.3f}")

        cv2.polylines(frame, [np.int32(marker_corners)], True, (255, 0, 0), 2)
        cv2.circle(frame, (int(robot_pt[0]), int(robot_pt[1])), 5, (0, 255, 0), -1)
        end = (int(robot_pt[0] + 30 * math.cos(heading_rad)),
               int(robot_pt[1] - 30 * math.sin(heading_rad)))
        cv2.arrowedLine(frame, (int(robot_pt[0]), int(robot_pt[1])), end, (0, 0, 255), 2)

    elif robot_pt is not None:
        send(f"POS,{robot_pt[0]:.1f},{robot_pt[1]:.1f},{heading_rad:.3f}")

    if time.time() - last_ping_time > 1.5:
        send("PING")
        last_ping_time = time.time()

    if time.time() - last_goal_send_time > 1.0:
        if nav_state in ("SEARCHING_PILE", "COLLECTING"):
            send_goal(CENTER_PILE)
        elif nav_state == "MOVING_TO_ARC" and arc_pt:
            send_goal(arc_pt)
        elif nav_state == "MOVING_TO_DROP" and final_target:
            send_goal(final_target)
        last_goal_send_time = time.time()

    if robot_pt is not None:
        start_pt = (int(robot_pt[0]), int(robot_pt[1]))
        if nav_state in ("SEARCHING_PILE", "COLLECTING"):
            cv2.line(frame, start_pt, (int(CENTER_PILE[0]), int(CENTER_PILE[1])), (0, 255, 0), 2)
        elif nav_state == "MOVING_TO_ARC" and arc_pt and final_target:
            a = (int(arc_pt[0]), int(arc_pt[1]))
            cv2.line(frame, start_pt, a, (0, 165, 255), 2)
            cv2.line(frame, a, (int(final_target[0]), int(final_target[1])), (255, 0, 255), 2, cv2.LINE_AA)
        elif nav_state == "MOVING_TO_DROP" and final_target:
            cv2.line(frame, start_pt, (int(final_target[0]), int(final_target[1])), (255, 0, 255), 2)

    try:
        while True:
            data, _ = sock.recvfrom(1024)
            lines = data.decode("utf-8", errors="ignore").splitlines()
            for line in lines:
                if line.strip(): handle_message(line.strip())
    except (BlockingIOError, ConnectionResetError):
        pass  

    cv2.putText(frame, f"State: {nav_state}", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.imshow("Master Controller", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        send("STOP_INTAKE")
        break

cap.release()
sock.close()
cv2.destroyAllWindows()