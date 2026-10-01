import os

CAMERA_INDEX = 2
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
MAP_FILENAME = "map_drop_off.txt"
CENTER_FILE = "center_config.txt"

# Default fallback values
CENTER_PILE = (230.0, 240.0)
IDLE_ANCHOR = (230.0, 240.0)
COLLECTION_BOX = None  # (x1, y1, x2, y2)

if os.path.exists(CENTER_FILE):
    try:
        with open(CENTER_FILE, "r") as f:
            for line in f:
                parts = line.strip().split(",")
                if parts[0] == "Center" and len(parts) == 3:
                    CENTER_PILE = (float(parts[1]), float(parts[2]))
                elif parts[0] == "Box" and len(parts) == 5:
                    COLLECTION_BOX = (int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]))
                elif parts[0] == "Idle" and len(parts) == 3:
                    IDLE_ANCHOR = (float(parts[1]), float(parts[2]))
        print(f"[CONFIG]: Dynamic CENTER_PILE = {CENTER_PILE}, Box = {COLLECTION_BOX}")
    except Exception as e:
        print(f"[CONFIG WARNING]: Failed to parse {CENTER_FILE}. Using defaults. Error: {e}")

AVOID_RADIUS_CM = 13.0
ARC_MARGIN_CM = 9.0
PX_PER_CM = 3.74
ROBOT_APRILTAG_ID = 0
SERIAL_PORT = "COM3"
SERIAL_BAUD = 115200

# --- Wi-Fi UDP Configuration ---
ESP32_IP = "10.44.167.27"
UDP_PORT = 8888