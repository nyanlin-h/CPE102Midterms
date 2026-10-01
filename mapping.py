import cv2
import numpy as np
import time
from config import CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT, MAP_FILENAME

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
time.sleep(2)

MIN_ZONE_AREA = 400   # px^2, adjust once robot/zone size is confirmed

# Update bounds here using output from colour_value_getter.py
# Format: name: (HSV low, HSV high, BGR draw colour)
colour_area = {
    "Violet": ((143, 73, 50), (163, 153, 130), (86, 50, 90)),
    "Cyan": ((86, 68, 154), (106, 148, 234), (194, 178, 112)),
    "Crimson": ((168, 93, 85), (179, 173, 165), (64, 60, 125)),
    "Marigold": ((0, 172, 205), (20, 255,255 ), (33, 107, 255)),
    "Sky_Blue": ((88, 191, 89), (108, 255, 169), (129, 96, 12)),
    "Lime_Green": ((59, 60, 129), (79, 140, 209), (123, 169, 103)),
}

drop_off_areas = {}
print(f"Mapping active. Press 's' to save points to {MAP_FILENAME}, 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for name, (lower, upper, bgr) in colour_area.items():
        mask = cv2.inRange(hsv_frame, np.array(lower), np.array(upper))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Use the largest blob only, so stray specks can't overwrite the real zone
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > MIN_ZONE_AREA:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    drop_off_areas[name] = (int(M["m10"] / M["m00"]),
                                            int(M["m01"] / M["m00"]))

        if name in drop_off_areas:
            coords = drop_off_areas[name]
            cv2.circle(frame, coords, 8, bgr, -1)
            cv2.putText(frame, f"{name}:({coords[0]},{coords[1]})",
                        (coords[0] - 30, coords[1] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 2)

    cv2.imshow("Overhead Mapping Calibration", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        if drop_off_areas:
            with open(MAP_FILENAME, "w") as f:
                for name, coords in drop_off_areas.items():
                    f.write(f"{name}, {coords[0]}, {coords[1]}\n")
            print(f"Saved {len(drop_off_areas)} zone(s) to '{MAP_FILENAME}'.")
            break
        else:
            print("No drop-off points detected! Can't save empty map.")
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()