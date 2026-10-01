import math
import os

CM_TO_PX = 3.74

AVOID_RADIUS_CM = 13.0
ARC_MARGIN_CM = 9.0
OFFSET_DISTANCE_CM = 15.0


class FieldNavigator:
    def __init__(self, map_filename="map_drop_off.txt", center_file="center_config.txt"):
        self.center_pile = (230.0, 240.0)
        self.idle_target = (230.0, 240.0)
        self.collection_box = None

        self.load_center_config(center_file)
        self.avoid_radius = AVOID_RADIUS_CM * CM_TO_PX
        self.clearance_radius = (AVOID_RADIUS_CM + ARC_MARGIN_CM) * CM_TO_PX
        self.drop_off_targets = self.load_map(map_filename)

    def load_center_config(self, filename):
        if os.path.exists(filename):
            try:
                with open(filename, "r") as f:
                    for line in f:
                        parts = line.strip().split(",")
                        if len(parts) >= 3:
                            label = parts[0].strip()
                            if label == "Center":
                                self.center_pile = (float(parts[1]), float(parts[2]))
                            elif label == "Idle":
                                self.idle_target = (float(parts[1]), float(parts[2]))
                            elif label == "Box" and len(parts) == 5:
                                self.collection_box = (int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]))
                return
            except Exception as e:
                print(f"[FieldNavigator] Error parsing {filename}: {e}")

    def load_map(self, filename):
        targets = {}
        if not os.path.exists(filename):
            return targets

        with open(filename, "r") as f:
            for line in f:
                parts = line.strip().split(",")
                if len(parts) == 3:
                    targets[parts[0].strip()] = (float(parts[1].strip()), float(parts[2].strip()))
        return targets

    def get_side_drop_target(self, color_name, robot_heading):
        if color_name not in self.drop_off_targets:
            return (0.0, 0.0)

        tx, ty = self.drop_off_targets[color_name]
        offset_px = OFFSET_DISTANCE_CM * CM_TO_PX
        perp_angle = robot_heading + (math.pi / 2.0)
        return (tx + offset_px * math.cos(perp_angle), ty - offset_px * math.sin(perp_angle))

    def calculate_arc_waypoint(self, robot_pos, target_pos):
        cx, cy = self.center_pile
        rx, ry = robot_pos
        tx, ty = target_pos

        # Guard against division-by-zero or atan2(0,0) invalid results
        if math.hypot(rx - cx, ry - cy) < 1.0 or math.hypot(tx - cx, ty - cy) < 1.0:
            return target_pos

        angle_robot = math.atan2(-(ry - cy), rx - cx)
        angle_target = math.atan2(-(ty - cy), tx - cx)

        diff = math.atan2(math.sin(angle_target - angle_robot), 
                          math.cos(angle_target - angle_robot))

        mid_angle = angle_robot + (diff / 2.0)
        arc_x = cx + self.clearance_radius * math.cos(mid_angle)
        arc_y = cy - self.clearance_radius * math.sin(mid_angle)

        return (arc_x, arc_y)