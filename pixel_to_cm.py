import cv2
import math

points = []

def click_event(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("Pixel to CM Calibration", frame)
        
        if len(points) == 2:
            # Calculate pixel distance 
            dx = points[1][0] - points[0][0]
            dy = points[1][1] - points[0][1]
            pixel_distance = math.sqrt(dx**2 + dy**2)
            
            cv2.line(frame, points[0], points[1], (0, 255, 0), 2)
            cv2.imshow("Pixel to CM Calibration", frame)
            
            print(f"\n--- CALIBRATION RESULTS ---")
            print(f"Pixel Distance between points: {pixel_distance:.2f} pixels")
            
            # 
            real_cm = float(input("Enter physical distance between these two points in CM: "))
            
            ratio_px_per_cm = pixel_distance / real_cm
            ratio_cm_per_px = real_cm / pixel_distance
            
            print(f"\nCalibration Ratio:")
            print(f" -> {ratio_px_per_cm:.2f} pixels / cm")
            print(f" -> {ratio_cm_per_px:.4f} cm / pixel")
            print(f"---------------------------\n")

cap = cv2.VideoCapture(0)
ret, frame = cap.read()

if ret:
    cv2.namedWindow("Pixel to CM Calibration")
    cv2.setMouseCallback("Pixel to CM Calibration", click_event)
    print("Click on the START and END points of your ruler/known object on the frame.")
    cv2.imshow("Pixel to CM Calibration", frame)
    cv2.waitKey(0)

cap.release()
cv2.destroyAllWindows()