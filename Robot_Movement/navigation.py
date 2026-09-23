import cv2

anchor_points = {
    "START": (0, 0),
    "CENTER": (0, 0),
    "RED": (0, 0),
    "GREEN": (0, 0),
    "PURPLE": (0, 0),
    "CYRAN": (0, 0),
    "ORANGE": (0, 0),
    "BLUE": (0, 0),
    "HOME": (0, 0)
}

def get_anchor_points(filename="map_drop_off.txt"):
    try:
        with open(filename, "r") as f:
            for line in f:
                parts = line.strip().split(",")
                if len(parts) == 3:
                    name = parts[0].strip()
                    x = int(parts[1].strip())
                    y = int(parts[2].strip())
                    anchor_points[name] = (x, y)
    except FileNotFoundError:
        pass
