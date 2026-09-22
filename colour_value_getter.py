import cv2
import numpy as np

def get_color_values(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:  # click on the drop off area

        bgr_pixel = frame[y, x]         # BGR comes from the original
        
        hsv_pixel = hsv_frame[y, x]     # HSV comes from the conversion

        print(f"--- Coordinates ({x}, {y}) ---")
        print(f"BGR: Blue={bgr_pixel[0]}, Green={bgr_pixel[1]}, Red={bgr_pixel[2]}")
        print(f"HSV: Hue={hsv_pixel[0]}, Saturation={hsv_pixel[1]}, Value={hsv_pixel[2]}")
        print("-" * 25)

cap = cv2.VideoCapture(0) #adjust port if needed

cv2.namedWindow("Webcam Color")
cv2.setMouseCallback("Webcam Color", get_color_values)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #convert to hsv from bgr

    cv2.imshow("Webcam Color", frame)

    if cv2.waitKey(1) & 0xFF == ord('e'):   #e to exit
        cap.release()
        cv2.destroyAllWindows()
        break

