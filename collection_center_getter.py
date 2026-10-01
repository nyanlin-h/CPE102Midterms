import cv2
import os

CENTER_FILE = "center_config.txt"

# Store points for both locations
selected_center = None
selected_idle = None
step = "CENTER"  # State tracker: "CENTER" -> "IDLE" -> "DONE"


def click_event(event, x, y, flags, param):
    global selected_center, selected_idle, step

    if event == cv2.EVENT_LBUTTONDOWN:
        if step == "CENTER":
            selected_center = (x, y)
            print(f"Collection Center Selected: X={x}, Y={y}")
            step = "IDLE"  # Move to next step
        elif step == "IDLE":
            selected_idle = (x, y)
            print(f"Idle Anchor Point Selected: X={x}, Y={y}")
            step = "DONE"


def main():
    global selected_center, selected_idle, step
    cap = cv2.VideoCapture(0)  # Open webcam

    cv2.namedWindow("Set Calibration Points")
    cv2.setMouseCallback("Set Calibration Points", click_event)

    print("--- CALIBRATION INSTRUCTIONS ---")
    print("1. Left-click to set the COLLECTION CENTER.")
    print("2. Left-click to set the IDLE ANCHOR POINT.")
    print("3. Press 's' to save both points, 'r' to reset, or 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        # Draw Collection Center if set
        if selected_center is not None:
            cv2.circle(frame, selected_center, 8, (0, 255, 255), -1)
            cv2.putText(
                frame,
                f"Center: {selected_center}",
                (selected_center[0] + 12, selected_center[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
            )

        # Draw Idle Anchor Point if set
        if selected_idle is not None:
            cv2.circle(frame, selected_idle, 8, (255, 0, 255), -1)
            cv2.putText(
                frame,
                f"Idle Anchor: {selected_idle}",
                (selected_idle[0] + 12, selected_idle[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 255),
                1,
            )

        # Dynamic On-Screen Status Prompt
        if step == "CENTER":
            prompt = "STEP 1: Click Collection Center"
        elif step == "IDLE":
            prompt = "STEP 2: Click Idle Anchor Point"
        else:
            prompt = "All set! Press 's' to Save | 'r' to Reset | 'q' to Quit"

        cv2.putText(
            frame,
            prompt,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        cv2.imshow("Set Calibration Points", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            if selected_center is not None and selected_idle is not None:
                with open(CENTER_FILE, "w") as f:
                    f.write(f"Center, {selected_center[0]}, {selected_center[1]}\n")
                    f.write(f"Idle, {selected_idle[0]}, {selected_idle[1]}\n")
                print(f"Saved coordinates to {CENTER_FILE}:")
                print(f"  - Center: {selected_center}")
                print(f"  - Idle Anchor: {selected_idle}")
                break
            else:
                print("Please click BOTH points before saving!")
        elif key == ord("r"):
            # Reset clicks to try again
            selected_center = None
            selected_idle = None
            step = "CENTER"
            print("Reset selection. Click Collection Center again.")
        elif key == ord("q"):
            print("Exited without saving.")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()