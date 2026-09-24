import cv2
import numpy as np

def get_color_values(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:      #click on drop of areas
        bgr_pixel = frame[y, x]
        hsv_pixel = hsv_frame[y, x]

        h_margin, s_margin, v_margin = 10, 40, 40

        lower_h = max(0, int(hsv_pixel[0]) - h_margin)
        lower_s = max(0, int(hsv_pixel[1]) - s_margin)
        lower_v = max(0, int(hsv_pixel[2]) - v_margin)

        upper_h = min(179, int(hsv_pixel[0]) + h_margin)
        upper_s = min(255, int(hsv_pixel[1]) + s_margin)
        upper_v = min(255, int(hsv_pixel[2]) + v_margin)

        print(f"--- Clicked Location ({x}, {y}) ---")
        print(f"BGR: ({bgr_pixel[0]}, {bgr_pixel[1]}, {bgr_pixel[2]})")
        print(f"Python tuple format for code:")
        print(f"(({lower_h}, {lower_s}, {lower_v}), ({upper_h}, {upper_s}, {upper_v}), ({bgr_pixel[0]}, {bgr_pixel[1]}, {bgr_pixel[2]}))")
        print("-" * 35)

cap = cv2.VideoCapture(0)           # adjust port if necessary
cv2.namedWindow("Color Calibration Tool")
cv2.setMouseCallback("Color Calibration Tool", get_color_values)

print("Click on any color zone or tag to inspect HSV ranges. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.imshow("Color Calibration Tool", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()