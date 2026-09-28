import cv2
import numpy as np

# HSV Margins (Adjustable)
H_MARGIN = 12
S_MARGIN = 50
V_MARGIN = 50

last_mask = None
sampled_color_name = "Sampled_Zone"

def get_color_values(event, x, y, flags, param):
    global last_mask, frame, hsv_frame
    
    if event == cv2.EVENT_LBUTTONDOWN:
        # 1. Sample 3x3 region to filter out single-pixel noise
        y_min, y_max = max(0, y - 1), min(hsv_frame.shape[0], y + 2)
        x_min, x_max = max(0, x - 1), min(hsv_frame.shape[1], x + 2)
        
        avg_hsv = np.mean(hsv_frame[y_min:y_max, x_min:x_max], axis=(0, 1))
        avg_bgr = np.mean(frame[y_min:y_max, x_min:x_max], axis=(0, 1)).astype(int)

        # 2. Compute bounded HSV range
        lower_h = max(0, int(avg_hsv[0]) - H_MARGIN)
        lower_s = max(30, int(avg_hsv[1]) - S_MARGIN)
        lower_v = max(30, int(avg_hsv[2]) - V_MARGIN)

        upper_h = min(179, int(avg_hsv[0]) + H_MARGIN)
        upper_s = min(255, int(avg_hsv[1]) + S_MARGIN)
        upper_v = min(255, int(avg_hsv[2]) + V_MARGIN)

        lower_bound = np.array([lower_h, lower_s, lower_v])
        upper_bound = np.array([upper_h, upper_s, upper_v])

        # 3. Create live binary mask window to test range immediately
        last_mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
        cv2.imshow("HSV Mask Preview (Check Isolation)", last_mask)

        print(f"\n--- Clicked Location ({x}, {y}) ---")
        print(f"Average BGR: ({avg_bgr[0]}, {avg_bgr[1]}, {avg_bgr[2]})")
        print(f"Paste into mapping code:")
        print(f"\"{sampled_color_name}\": (({lower_h}, {lower_s}, {lower_v}), ({upper_h}, {upper_s}, {upper_v}), ({avg_bgr[0]}, {avg_bgr[1]}, {avg_bgr[2]})),")
        print("-" * 45)

cap = cv2.VideoCapture(2) 
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