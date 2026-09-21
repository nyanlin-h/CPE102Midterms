import cv2
import numpy as np
import time

cap = cv2.VideoCapture(2) # check if needed to change ports
time.sleep(3)   # reduce this if we dont get config time (but test first)

colour_area = {
    "Violet" : ((125, 100, 80), (145, 200, 210), (145, 50, 180)),   #readjust all values later
    "Cyan" : ((90, 180, 230), (100, 220, 250), (250, 220, 50)),
    "Crimson" : ((170, 190, 230), (180, 210, 250), (65, 50, 240)),
    "Marigold" : ((0, 0, 0), (15, 225, 255), (33, 107, 255)),
    "Sky_Blue" : ((0, 0, 0), (0, 0, 0), (0, 0, 0)),
    "Lime_Green" : ((0, 0, 0), (0, 0, 0), (0, 0, 0))
}

print("checking drop off points... press 's' to save, 'q' to quit")


drop_off_areas = {}

while True:     
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    

    for name, (lower, upper, bgr) in colour_area.items():
        mask = cv2.inRange(hsv_frame, np.array(lower), np.array(upper))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 400:
                M = cv2.moments(contour)
                if M["m00"] != 0: 
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    # Update/add to map cache
                    drop_off_areas[name] = (cX, cY) 

        # Visual feedback: Draw anchors for items currently stored in memory
        if name in drop_off_areas:
            coords = drop_off_areas[name]
            cv2.circle(frame, coords, 8, bgr, -1) 
            cv2.putText(frame, f"{name}:({coords[0]},{coords[1]})", (coords[0] - 30, coords[1] - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 2)

    cv2.imshow("Overhead Static Map Calibration", frame)
    key = cv2.waitKey(1) & 0xFF

    # save data
    if key == ord('s'):
        if len(drop_off_areas) > 0:
            with open("map_drop_off.txt", "w") as f: 
                for name, coords in drop_off_areas.items():
                    f.write(f"{name}, {coords[0]}, {coords[1]}\n")
            print(f"\nSaved {len(drop_off_areas)} zones to 'map_drop_off.txt'") #dont forget to reconnect to the values of the stones
        else:
            print("No colors detected yet! Can't save empty map.")

    elif key == ord('q'):
        print("Canceled calibration")
        break

cap.release()
cv2.destroyAllWindows()
