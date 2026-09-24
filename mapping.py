import cv2
import numpy as np
import time

cap = cv2.VideoCapture(0)       #change ports if necessary
time.sleep(2)

# Update bounds here using output from colourvaluegetter
colour_area = {
    "Violet": ((125, 100, 80), (145, 200, 210), (145, 50, 180)),
    "Cyan": ((90, 180, 230), (100, 220, 250), (250, 220, 50)),
    "Crimson": ((170, 190, 230), (180, 210, 250), (65, 50, 240)),
    "Marigold": ((5, 210, 240), (15, 225, 255), (33, 107, 255)),
    "Sky_Blue": ((90, 200, 240), (100, 210, 255), (250, 225, 50)),
    "Lime_Green": ((25, 90, 45), (45, 175, 125), (40, 85, 75)),
}

drop_off_areas = {}
print("Mapping active. Press 's' to save points to map_drop_off.txt, 'q' to quit.")

while True:     
    ret, frame = cap.read()
    if not ret:
        break

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for name, (lower, upper, bgr) in colour_area.items():
        mask = cv2.inRange(hsv_frame, np.array(lower), np.array(upper))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 400:      #see if i need to adjust the boundary because we are still not too sure about the robot size
                M = cv2.moments(contour)
                if M["m00"] != 0: 
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    drop_off_areas[name] = (cX, cY) 

        if name in drop_off_areas:
            coords = drop_off_areas[name]
            cv2.circle(frame, coords, 8, bgr, -1) 
            cv2.putText(frame, f"{name}:({coords[0]},{coords[1]})", (coords[0] - 30, coords[1] - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 2)

    cv2.imshow("Overhead Mapping Calibration", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        if len(drop_off_areas) > 0:
            with open("map_drop_off.txt", "w") as f: 
                for name, coords in drop_off_areas.items():
                    f.write(f"{name}, {coords[0]}, {coords[1]}\n")
            print(f"Successfully saved {len(drop_off_areas)} zone(s) to 'map_drop_off.txt'.")
            break
        else:
            print("No drop-off points detected! Can't save empty map.")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()