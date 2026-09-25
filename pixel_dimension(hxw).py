import cv2

cap = cv2.VideoCapture(0)#change port for the webcam

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(f"Camera Resolution: {width} x {height} pixels")

cap.release()