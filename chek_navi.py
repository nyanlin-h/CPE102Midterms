import cv2
import numpy as np
import math
import os
from navigation import FieldNavigator

# 1. Create a dummy map file if it doesn't exist
MAP_FILE = "map_drop_off.txt"
if not os.path.exists(MAP_FILE):
    with open(MAP_FILE, "w") as f:
        f.write("Crimson, 500, 100\n")
        f.write("Cyan, 500, 380\n")
        f.write("Lime_Green, 100, 400\n")

# 2. Instantiate Navigator and setup canvas
nav = FieldNavigator(map_filename=MAP_FILE)

# Setup a 640x480 frame matching camera resolution
width, height = 640, 480
frame = np.zeros((height, width, 3), dtype=np.uint8)

# 3. Define simulated field positions
robot_pos = (120.0, 120.0)
robot_heading = math.radians(45)  # Heading in radians
target_color = "Cyan"

# 4. Calculate Navigation Waypoints
side_target = nav.get_side_drop_target(target_color, robot_heading)
arc_waypoint = nav.calculate_arc_waypoint(robot_pos, side_target)

# 5. Draw Visualizations
# Center Avoidance Zone & Pile
center_pt = (int(nav.center_pile[0]), int(nav.center_pile[1]))
cv2.circle(frame, center_pt, int(nav.avoid_radius), (0, 0, 150), 2)  # Avoidance Radius
cv2.circle(frame, center_pt, 8, (0, 255, 255), -1)                   # Center Pile
cv2.putText(frame, "Avoidance Zone", (center_pt[0] - 50, center_pt[1] - int(nav.avoid_radius) - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

# Drop targets from map file
for name, pos in nav.drop_off_targets.items():
    pt = (int(pos[0]), int(pos[1]))
    cv2.circle(frame, pt, 10, (255, 255, 255), 1)
    cv2.putText(frame, name, (pt[0] + 12, pt[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

# Robot Position and Heading Vector
r_pt = (int(robot_pos[0]), int(robot_pos[1]))
cv2.circle(frame, r_pt, 8, (0, 255, 0), -1)
head_end = (int(robot_pos[0] + 25 * math.cos(robot_heading)), 
            int(robot_pos[1] - 25 * math.sin(robot_heading)))
cv2.arrowedLine(frame, r_pt, head_end, (0, 255, 0), 2)
cv2.putText(frame, "Robot", (r_pt[0] - 20, r_pt[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

# Side Offset Target
st_pt = (int(side_target[0]), int(side_target[1]))
cv2.circle(frame, st_pt, 6, (255, 0, 255), -1)
cv2.putText(frame, "Offset Target", (st_pt[0] + 10, st_pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

# Arc Waypoint
arc_pt = (int(arc_waypoint[0]), int(arc_waypoint[1]))
cv2.circle(frame, arc_pt, 6, (0, 165, 255), -1)
cv2.putText(frame, "Arc Waypoint", (arc_pt[0] + 10, arc_pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)

# 6. Render Path Lines
# Leg 1: Robot to Arc Waypoint
cv2.line(frame, r_pt, arc_pt, (0, 165, 255), 2, cv2.LINE_AA)
# Leg 2: Arc Waypoint to Offset Target
cv2.line(frame, arc_pt, st_pt, (255, 0, 255), 2, cv2.LINE_AA)

# Display Window
cv2.imshow("Field Navigation Pathing Visualizer", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()