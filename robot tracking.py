import cv2
import numpy as np
import math
import serial
import time

#connect to esp32 and its port adjust the port if needed
esp32 = serial.Serial(port='COM2', baudrate=115200, timeout=0.05)
time.sleep(3) 

# 2. Open overhead webcam feed channel
cap = cv2.VideoCapture(0)


# get 2 differently coloured stickers to track head and tail
lower_front_red = np.array([0, 150, 150])    # Front
upper_front_red = np.array([10, 255, 255])
lower_back_blue = np.array([100, 150, 150])  # Rear 
upper_back_blue = np.array([120, 255, 255])

def get_marker_center(frame, hsv_frame, lower_bound, upper_bound):
    #isolates and extract the colour pixel blobs
    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 100: # Filter small image noise reflections
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy)
    return None

print("Vision loop active. Press 'q' to terminate application window.")

while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Locate tracking assets on top of the robot frame
    front_pt = get_marker_center(frame, hsv, lower_front_red, upper_front_red)
    back_pt = get_marker_center(frame, hsv, lower_back_blue, upper_back_blue)
    
    if front_pt and back_pt:
        # Transform pixel to metric grid format by scalar 
        cm_per_pixel = 0.5 
        
        # We use the back tracking marker as the base anchor point (robotX, robotY)
        rx_cm = back_pt[0] * cm_per_pixel
        ry_cm = back_pt[1] * cm_per_pixel
        
        # Compute Heading direction vector in radians
        dx = front_pt[0] - back_pt[0]
        dy = front_pt[1] - back_pt[1]
        # Invert dy screen space coordinate frame maps downwards natively (recheck with webcam)
        heading_rad = math.atan2(-dy, dx) 
        
        # Format packet and transmit payload string over USB Serial: "ROBOT,X,Y,Heading\n"
        data_string = f"ROBOT,{rx_cm:.2f},{ry_cm:.2f},{heading_rad:.3f}\n"
        esp32.write(data_string.encode('utf-8'))
        
        # Visual diagnostic indicators on output canvas window frame
        cv2.circle(frame, front_pt, 8, (0, 0, 255), -1) # Red tag marker
        cv2.circle(frame, back_pt, 8, (255, 0, 0), -1)  # Blue tag marker
        cv2.line(frame, back_pt, front_pt, (0, 255, 0), 2)
        cv2.putText(frame, f"X:{rx_cm:.1f} Y:{ry_cm:.1f} H:{math.degrees(heading_rad):.0f}", 
                    (back_pt[0]-20, back_pt[1]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    cv2.imshow("Overhead Robot Localization Feed Engine", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
