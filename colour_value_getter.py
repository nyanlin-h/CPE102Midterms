import cv2
import numpy as np
from config import CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT


H_MARGIN = 12
S_MARGIN = 50
V_MARGIN = 50

sampled_color_name = "Sampled_Zone"   # rename before pasting into mapping.py

frame = None
hsv_frame = None


def get_color_values(event, x, y, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN or frame is None or hsv_frame is None:
        return

    # Sample 3x3 region to filter out single-pixel noise
    y_min, y_max = max(0, y - 1), min(hsv_frame.shape[0], y + 2)
    x_min, x_max = max(0, x - 1), min(hsv_frame.shape[1], x + 2)

    avg_hsv = np.mean(hsv_frame[y_min:y_max, x_min:x_max], axis=(0, 1))
    avg_bgr = np.mean(frame[y_min:y_max, x_min:x_max], axis=(0, 1)).astype(int)

    lower_h = max(0, int(avg_hsv[0]) - H_MARGIN)
    lower_s = max(30, int(avg_hsv[1]) - S_MARGIN)
    lower_v = max(30, int(avg_hsv[2]) - V_MARGIN)
    upper_h = min(179, int(avg_hsv[0]) + H_MARGIN)
    upper_s = min(255, int(avg_hsv[1]) + S_MARGIN)
    upper_v = min(255, int(avg_hsv[2]) + V_MARGIN)

    lower_bound = np.array([lower_h, lower_s, lower_v])
    upper_bound = np.array([upper_h, upper_s, upper_v])

    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
    cv2.imshow("HSV Mask Preview (Check Isolation)", mask)

    print(f"\n--- Clicked Location ({x}, {y}) ---")
    print(f"Average BGR: ({avg_bgr[0]}, {avg_bgr[1]}, {avg_bgr[2]})")
    print("Paste into mapping code:")
    print(f"\"{sampled_color_name}\": (({lower_h}, {lower_s}, {lower_v}), "
          f"({upper_h}, {upper_s}, {upper_v}), "
          f"({avg_bgr[0]}, {avg_bgr[1]}, {avg_bgr[2]})),")
    print("-" * 45)


cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
cv2.namedWindow("Color Calibration Tool")
cv2.setMouseCallback("Color Calibration Tool", get_color_values)

print("Click on any color zone to inspect HSV bounds and preview binary mask. Press 'q' to quit.")

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