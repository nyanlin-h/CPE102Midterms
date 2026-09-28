import cv2
import numpy as np
import time

cap = cv2.VideoCapture(2)       #change ports if necessary
time.sleep(2)

# Update bounds here using output from colourvaluegetter
colour_area = {
    "Violet": ((143, 73, 50), (163, 153, 130), (86, 50, 90)),        #HSVlow,hsvhigh, bgr
    "Cyan": ((86, 68, 154), (106, 148, 234), (194, 178, 112)),
    "Crimson": ((168, 93, 85), (179, 173, 165), (64, 60, 125)),
    "Marigold": ((86, 68, 154), (106, 148, 234), (194, 178, 112)),
    "Sky_Blue": ((88, 191, 89), (108, 255, 169), (129, 96, 12)),
    "Lime_Green": ((59, 60, 129), (79, 140, 209), (123, 169, 103)),     #add addtional for collection area, starting point so that i can put everything into the same code at the same time.
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