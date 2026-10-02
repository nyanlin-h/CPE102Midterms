import os

CAMERA_INDEX = 2
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
MAP_FILENAME = "map_drop_off.txt"
CENTER_FILE = "center_config.txt"

CENTER_PILE = (230.0, 240.0)
IDLE_ANCHOR = (230.0, 240.0)
COLLECTION_BOX = None

if os.path.exists(CENTER_FILE):
    try:
        with open(CENTER_FILE, "r") as f:
            for line in f:
                parts = line.strip().split(",")
                label = parts[0].strip()
                if label == "Center" and len(parts) == 3:
                    CENTER_PILE = (float(parts[1]), float(parts[2]))
                elif label == "Box" and len(parts) == 5:
                    COLLECTION_BOX = (int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]))
                elif label == "Idle" and len(parts) == 3:
                    IDLE_ANCHOR = (float(parts[1]), float(parts[2]))
    except Exception as e:
        print(f"[CONFIG WARNING]: {e}")

AVOID_RADIUS_CM = 13.0
ARC_MARGIN_CM = 9.0
PX_PER_CM = 3.74
ROBOT_APRILTAG_ID = 0
SERIAL_PORT = "COM3"
SERIAL_BAUD = 115200

ESP32_IP = "172.20.10.2"
UDP_PORT = 8888