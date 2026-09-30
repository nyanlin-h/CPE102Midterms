import cv2
import numpy as np

# Global state
roi_points = []
sampling = False
selected_bgr = None
selected_hsv = None
lower_hsv = None
upper_hsv = None


def mouse_callback(event, x, y, flags, param):
    global roi_points, sampling, selected_bgr, selected_hsv, lower_hsv, upper_hsv

    frame = param["frame"]

    if event == cv2.EVENT_LBUTTONDOWN:
        sampling = True
        roi_points = [(x, y)]

    elif event == cv2.EVENT_MOUSEMOVE and sampling:
        # Update point during drag
        if len(roi_points) > 1:
            roi_points[1] = (x, y)
        else:
            roi_points.append((x, y))

    elif event == cv2.EVENT_LBUTTONUP:
        sampling = False
        if len(roi_points) == 1 or roi_points[0] == (x, y):
            # Single click selection
            bgr_pixel = frame[y, x]
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv_pixel = hsv_frame[y, x]

            selected_bgr = bgr_pixel.tolist()
            selected_hsv = hsv_pixel.tolist()

            # Set tolerance (+/- 15 Hue, +/- 40 Saturation & Value)
            h, s, v = selected_hsv
            lower_hsv = [max(0, h - 15), max(40, s - 40), max(40, v - 40)]
            upper_hsv = [min(179, h + 15), min(255, s + 40), min(255, v + 40)]
        else:
            # Dragged bounding box selection (average region)
            x1, y1 = roi_points[0]
            x2, y2 = x, y
            x_min, x_max = min(x1, x2), max(x1, x2)
            y_min, y_max = min(y1, y2), max(y1, y2)

            roi_bgr = frame[y_min:y_max, x_min:x_max]
            if roi_bgr.size > 0:
                mean_bgr = cv2.mean(roi_bgr)[:3]
                selected_bgr = [int(c) for c in mean_bgr]

                roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
                # Compute min/max within selected region for precise bounds
                min_hsv = np.min(roi_hsv, axis=(0, 1))
                max_hsv = np.max(roi_hsv, axis=(0, 1))

                # Add padding tolerance
                lower_hsv = [
                    max(0, int(min_hsv[0]) - 10),
                    max(40, int(min_hsv[1]) - 30),
                    max(40, int(min_hsv[2]) - 30),
                ]
                upper_hsv = [
                    min(179, int(max_hsv[0]) + 10),
                    min(255, int(max_hsv[1]) + 30),
                    min(255, int(max_hsv[2]) + 30),
                ]
                selected_hsv = [
                    int((lower_hsv[i] + upper_hsv[i]) / 2) for i in range(3)
                ]


def main():
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    cv2.namedWindow("Webcam Feed")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Pass current frame to mouse callback
        cv2.setMouseCallback(
            "Webcam Feed", mouse_callback, param={"frame": frame}
        )

        display_frame = frame.copy()

        # Display sampled results on frame
        if selected_bgr is not None:
            text_bgr = f"BGR: {selected_bgr}"
            text_hsv = f"HSV: {selected_hsv}"
            text_lower = f"Lower HSV: {lower_hsv}"
            text_upper = f"Upper HSV: {upper_hsv}"

            # Color preview square
            cv2.rectangle(
                display_frame,
                (10, 10),
                (60, 60),
                tuple(selected_bgr),
                -1,
            )
            cv2.rectangle(
                display_frame, (10, 10), (60, 60), (255, 255, 255), 1
            )

            # Information text
            cv2.putText(
                display_frame,
                text_bgr,
                (70, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                display_frame,
                text_hsv,
                (70, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                display_frame,
                text_lower,
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )
            cv2.putText(
                display_frame,
                text_upper,
                (10, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )

            # Mask Preview Window
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(
                hsv_frame, np.array(lower_hsv), np.array(upper_hsv)
            )
            cv2.imshow("Mask Target Preview", mask)

        cv2.imshow("Webcam Feed", display_frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()