import cv2
import os
import numpy as np

CENTER_FILE = "center_config.txt"

# Selection state
ix, iy = -1, -1
drawing = False
bbox_center = None
bbox_rect = None  # (x1, y1, x2, y2)
selected_idle = None
step = "CENTER_BOX"  # State: "CENTER_BOX" -> "IDLE" -> "DONE"


def mouse_callback(event, x, y, flags, param):
    global ix, iy, drawing, bbox_center, bbox_rect, selected_idle, step

    if step == "CENTER_BOX":
        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            ix, iy = x, y
            bbox_rect = (x, y, x, y)

        elif event == cv2.EVENT_MOUSEMOVE and drawing:
            bbox_rect = (min(ix, x), min(iy, y), max(ix, x), max(iy, y))

        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False
            x1, y1, x2, y2 = min(ix, x), min(iy, y), max(ix, x), max(iy, y)
            bbox_rect = (x1, y1, x2, y2)
            # Calculate geometric center of the drawn box
            cx = float((x1 + x2) / 2.0)
            cy = float((y1 + y2) / 2.0)
            bbox_center = (cx, cy)
            print(f"Collection Region Selected: Box=({x1},{y1})-({x2},{y2}) -> Center=({cx:.1f}, {cy:.1f})")
            step = "IDLE"

    elif step == "IDLE":
        if event == cv2.EVENT_LBUTTONDOWN:
            selected_idle = (float(x), float(y))
            print(f"Idle Anchor Point Selected: X={x}, Y={y}")
            step = "DONE"


def main():
    global ix, iy, drawing, bbox_center, bbox_rect, selected_idle, step
    cap = cv2.VideoCapture(2)

    cv2.namedWindow("Set Calibration Region")
    cv2.setMouseCallback("Set Calibration Region", mouse_callback)

    print("--- DYNAMIC CALIBRATION INSTRUCTIONS ---")
    print("1. Click & Drag a box over the COLLECTION REGION.")
    print("2. Left-click to set the IDLE ANCHOR POINT.")
    print("3. Press 's' to save, 'r' to reset, or 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        # Draw dynamic bounding box & center point
        if bbox_rect is not None:
            x1, y1, x2, y2 = bbox_rect
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            if bbox_center is not None:
                cx, cy = int(bbox_center[0]), int(bbox_center[1])
                cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                cv2.putText(
                    frame,
                    f"Center: ({cx}, {cy})",
                    (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                )

        # Draw Idle Anchor Point
        if selected_idle is not None:
            pt = (int(selected_idle[0]), int(selected_idle[1]))
            cv2.circle(frame, pt, 8, (255, 0, 255), -1)
            cv2.putText(
                frame,
                f"Idle: {pt}",
                (pt[0] + 12, pt[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 255),
                1,
            )

        # Prompt UI Overlay
        if step == "CENTER_BOX":
            prompt = "STEP 1: Click & Drag Box over Collection Region"
        elif step == "IDLE":
            prompt = "STEP 2: Click Idle Anchor Point"
        else:
            prompt = "All set! Press 's' to Save | 'r' to Reset | 'q' to Quit"

        cv2.putText(frame, prompt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("Set Calibration Region", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            if bbox_center is not None and selected_idle is not None:
                x1, y1, x2, y2 = bbox_rect
                with open(CENTER_FILE, "w") as f:
                    f.write(f"Center, {bbox_center[0]:.1f}, {bbox_center[1]:.1f}\n")
                    f.write(f"Box, {x1}, {y1}, {x2}, {y2}\n")
                    f.write(f"Idle, {selected_idle[0]:.1f}, {selected_idle[1]:.1f}\n")
                print(f"Saved dynamic region calibration to {CENTER_FILE}")
                break
            else:
                print("Please complete region selection and idle point before saving!")

        elif key == ord("r"):
            bbox_center = None
            bbox_rect = None
            selected_idle = None
            step = "CENTER_BOX"
            print("Selection reset. Drag collection region box again.")

        elif key == ord("q"):
            print("Exited without saving.")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()