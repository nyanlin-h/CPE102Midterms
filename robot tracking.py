import cv2
import numpy as np
import math
import serial
import time

from navigation import FieldNavigator
from config import (CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT, CENTER_PILE,
                    ROBOT_APRILTAG_ID, SERIAL_PORT, SERIAL_BAUD, AVOID_RADIUS_CM)

EMA_ALPHA = 0.65   # 0.0 = max smooth, 1.0 = raw

COLOR_PALETTE = {
    "Crimson": (0, 0, 255),
    "Cyan": (255, 255, 0),
    "Lime_Green": (0, 255, 0),
    "Marigold": (0, 165, 255),
    "Sky_Blue": (255, 191, 0),
    "Violet": (238, 130, 238),
}

apriltag_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
detector = cv2.aruco.ArucoDetector(apriltag_dict, cv2.aruco.DetectorParameters())

nav = FieldNavigator()

esp32 = serial.Serial(port=SERIAL_PORT, baudrate=SERIAL_BAUD, timeout=0.05)
time.sleep(2)
esp32.reset_input_buffer()

cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)


def send(text):
    esp32.write((text + "\n").encode("utf-8"))


def send_goal(pt):
    send(f"SET_GOAL,{pt[0]:.1f},{pt[1]:.1f}")


def get_robot_pose_apriltag(frame, target_id):
    """Returns (centre_px, heading_rad, corners) or (None, None, None)."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None and target_id in ids:
        idx = np.where(ids == target_id)[0][0]
        pts = corners[idx][0]  # [TL, TR, BR, BL]
        center = (float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1])))
        dx = pts[1][0] - pts[0][0]
        dy = -(pts[1][1] - pts[0][1])  # invert screen Y
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


def handle_message(msg):
    global nav_state, final_target, arc_pt

    if msg.startswith("DETECTED_COLOR,") and nav_state == "COLLECTING":
        detected = msg.split(",")[1]
        print(f"[STONE DETECTED]: {detected}")
        target = nav.get_drop_target(detected)
        if target is None:
            print(f"[WARNING]: No drop zone mapped for '{detected}'. Ignoring.")
            return
        if robot_pt is None:
            return
        final_target = target
        arc_pt = nav.calculate_arc_waypoint(robot_pt, final_target)
        send("STOP_INTAKE")
        send_goal(arc_pt)
        nav_state = "MOVING_TO_ARC"

    elif msg == "WAYPOINT_REACHED":
        if nav_state == "SEARCHING_PILE":
            print("[STATE]: At center pile. Starting intake...")
            send("START_INTAKE")
            nav_state = "COLLECTING"

        elif nav_state == "MOVING_TO_ARC" and final_target:
            print("[STATE]: Arc cleared. Heading to drop zone...")
            send_goal(final_target)
            nav_state = "MOVING_TO_DROP"

        elif nav_state == "MOVING_TO_DROP":
            print("[STATE]: At drop zone. Releasing stone...")
            send("RELEASE_STONE")
            nav_state = "RELEASING"

    elif msg == "RELEASE_DONE" and nav_state == "RELEASING":
        print("[STATE]: Release complete. Returning to pile...")
        send_goal(CENTER_PILE)
        nav_state = "SEARCHING_PILE"

    elif msg == "NAV_TIMEOUT_SAFETY_STOP":
        print("[WARNING]: ESP32 safety stop (lost tracking link). Re-sending current goal.")
        goal = {"SEARCHING_PILE": CENTER_PILE, "MOVING_TO_ARC": arc_pt,
                "MOVING_TO_DROP": final_target}.get(nav_state)
        if goal:
            send_goal(goal)


# Initial goal
send_goal(CENTER_PILE)

while True:
    ret, frame = cap.read()
    if not ret:
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
            dh = math.atan2(math.sin(curr_heading - heading_rad),
                            math.cos(curr_heading - heading_rad))
            heading_rad = math.atan2(math.sin(heading_rad + EMA_ALPHA * dh),
                                     math.cos(heading_rad + EMA_ALPHA * dh))

        # Don't stream while the ESP32 is busy in its blocking release routine,
        # otherwise its serial buffer overflows.
        if nav_state != "RELEASING":
            send(f"POS,{robot_pt[0]:.1f},{robot_pt[1]:.1f},{heading_rad:.3f}")

        cv2.polylines(frame, [np.int32(marker_corners)], True, (255, 0, 0), 2)
        cv2.circle(frame, (int(robot_pt[0]), int(robot_pt[1])), 5, (0, 255, 0), -1)
        end = (int(robot_pt[0] + 30 * math.cos(heading_rad)),
               int(robot_pt[1] - 30 * math.sin(heading_rad)))
        cv2.arrowedLine(frame, (int(robot_pt[0]), int(robot_pt[1])), end, (0, 0, 255), 2)

    # Path overlay
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

    # Drain ALL pending serial lines each frame
    while esp32.in_waiting > 0:
        line = esp32.readline().decode("utf-8", errors="ignore").strip()
        if line:
            handle_message(line)

    cv2.putText(frame, f"State: {nav_state}", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imshow("Master Controller - AprilTag Tracking & Dynamic Paths", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        send("STOP_INTAKE")
        break

cap.release()
esp32.close()
cv2.destroyAllWindows()