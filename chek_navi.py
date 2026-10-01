import cv2
import numpy as np
import math
import os
from navigation import FieldNavigator

# 1. Create a dummy map file if missing
MAP_FILE = "map_drop_off.txt"
if not os.path.exists(MAP_FILE):
    with open(MAP_FILE, "w") as f:
        f.write("Crimson, 500, 100\n")
        f.write("Cyan, 500, 380\n")
        f.write("Lime_Green, 100, 400\n")

# 2. Instantiate Navigator
nav = FieldNavigator(map_filename=MAP_FILE)

# Color Palette mapping
COLOR_PALETTE = {
    "Crimson": (0, 0, 255),
    "Cyan": (255, 255, 0),
    "Lime_Green": (0, 255, 0),
}

width, height = 640, 480
frame = np.zeros((height, width, 3), dtype=np.uint8)

# 3. Simulated field state
robot_pos = (120.0, 120.0)
robot_heading = math.radians(45)
target_color = "Cyan"

# 4. Calculate Navigation Waypoint
target_pos = nav.get_side_drop_target(target_color, robot_heading)

if target_pos == (0.0, 0.0) or target_pos is None:
    print(f"Target '{target_color}' not found in drop off targets.")
else:
    arc_waypoint = nav.calculate_arc_waypoint(robot_pos, target_pos)

    # 5. Render Visualizations
    # Render Dynamic Selection Box (if calibrated)
    if nav.collection_box is not None:
        x1, y1, x2, y2 = nav.collection_box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 1, cv2.LINE_AA)

    # Center Pile & Avoidance Clearance Zone
    center_pt = (int(nav.center_pile[0]), int(nav.center_pile[1]))
    cv2.circle(frame, center_pt, int(nav.clearance_radius), (0, 0, 255), 1, cv2.LINE_AA)
    cv2.circle(frame, center_pt, int(nav.avoid_radius), (0, 0, 180), 2, cv2.LINE_AA)
    cv2.circle(frame, center_pt, 6, (0, 255, 255), -1)
    cv2.putText(frame, "Collection Center", (center_pt[0] - 50, center_pt[1] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    # Idle Anchor Point
    idle_pt = (int(nav.idle_target[0]), int(nav.idle_target[1]))
    cv2.circle(frame, idle_pt, 6, (255, 0, 255), -1)
    cv2.putText(frame, "Idle Anchor", (idle_pt[0] + 10, idle_pt[1] + 4), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

    # Drop Off Map Targets
    for name, pos in nav.drop_off_targets.items():
        pt = (int(pos[0]), int(pos[1]))
        col = COLOR_PALETTE.get(name, (200, 200, 200))
        cv2.circle(frame, pt, 10, col, 2, cv2.LINE_AA)
        cv2.putText(frame, name, (pt[0] + 14, pt[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)

    # Robot Heading & Vector
    r_pt = (int(robot_pos[0]), int(robot_pos[1]))
    cv2.circle(frame, r_pt, 8, (0, 255, 0), -1)
    head_end = (int(robot_pos[0] + 25 * math.cos(robot_heading)), 
                int(robot_pos[1] - 25 * math.sin(robot_heading)))
    cv2.arrowedLine(frame, r_pt, head_end, (0, 255, 0), 2)
    cv2.putText(frame, "Robot", (r_pt[0] - 20, r_pt[1] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    # Target & Arc Waypoint
    t_pt = (int(target_pos[0]), int(target_pos[1]))
    cv2.circle(frame, t_pt, 6, (255, 0, 255), -1)
    
    arc_pt = (int(arc_waypoint[0]), int(arc_waypoint[1]))
    cv2.circle(frame, arc_pt, 6, (0, 165, 255), -1)
    cv2.putText(frame, "Arc Waypoint", (arc_pt[0] + 10, arc_pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)

    # 6. Render Path
    cv2.line(frame, r_pt, arc_pt, (0, 165, 255), 2, cv2.LINE_AA)
    cv2.line(frame, arc_pt, t_pt, (255, 0, 255), 2, cv2.LINE_AA)

    # Display Output
    cv2.imshow("Field Navigation Pathing Visualizer", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()