import math
import os


CM_TO_PX = 3.738  # Calibration factor: converts centimeters to frame pixels

# Updated for 19cm x 23cm chassis dimensions to prevent pile collisions
AVOID_RADIUS_CM = 45.0   # Clearance radius around collection area center
ARC_MARGIN_CM = 10.0     # Buffer distance for intermediate arc waypoints
OFFSET_DISTANCE_CM = 15.0 # Side target offset for stone drop-offs


class FieldNavigator:
    def __init__(self, map_filename="map_drop_off.txt", center_file="center_config.txt"):
        """
        Initializes field navigation, dynamic coordinates, and radius thresholds.
        """
        # Default fallback coordinates (in pixel space)
        self.center_pile = (320.0, 240.0)
        self.idle_target = (100.0, 100.0)

        # 1. Load calibration configuration (Center + Idle Anchor)
        self.load_center_config(center_file)

        # 2. Convert radius dimensions from cm to pixel values
        self.avoid_radius = AVOID_RADIUS_CM * CM_TO_PX
        self.clearance_radius = (AVOID_RADIUS_CM + ARC_MARGIN_CM) * CM_TO_PX

        # 3. Load drop-off target map coordinates
        self.drop_off_targets = self.load_map(map_filename)

    def load_center_config(self, filename):
        """
        Loads the central collection area and idle anchor point from center_config.txt.
        """
        if os.path.exists(filename):
            try:
                with open(filename, "r") as f:
                    for line in f:
                        parts = line.strip().split(",")
                        if len(parts) == 3:
                            label = parts[0].strip()
                            x = float(parts[1].strip())
                            y = float(parts[2].strip())

                            if label == "Center":
                                self.center_pile = (x, y)
                            elif label == "Idle":
                                self.idle_target = (x, y)

                print(f"[FieldNavigator] Loaded Center: {self.center_pile}, Idle Anchor: {self.idle_target}")
                return
            except Exception as e:
                print(f"[FieldNavigator] Error parsing {filename}: {e}")

        print("[FieldNavigator] center_config.txt not found/valid. Using default positions.")

    def load_map(self, filename):
        """
        Loads drop-off zone coordinates from map_drop_off.txt.
        """
        targets = {}
        if not os.path.exists(filename):
            print(f"[FieldNavigator] Warning: Map file '{filename}' not found.")
            return targets

        with open(filename, "r") as f:
            for line in f:
                parts = line.strip().split(",")
                if len(parts) == 3:
                    color_name = parts[0].strip()
                    x = float(parts[1].strip())
                    y = float(parts[2].strip())
                    targets[color_name] = (x, y)

        print(f"[FieldNavigator] Loaded {len(targets)} drop-off targets from {filename}.")
        return targets

    def get_side_drop_target(self, color_name, robot_heading):
        """
        Calculates a side-offset coordinate relative to the drop target 
        based on current robot orientation.
        """
        if color_name not in self.drop_off_targets:
            print(f"[FieldNavigator] Target color '{color_name}' not in drop-off map!")
            return (0.0, 0.0)

        tx, ty = self.drop_off_targets[color_name]
        offset_px = OFFSET_DISTANCE_CM * CM_TO_PX

        # Perpendicular offset angle
        perp_angle = robot_heading + (math.pi / 2.0)
        offset_x = tx + offset_px * math.cos(perp_angle)
        offset_y = ty - offset_px * math.sin(perp_angle)

        return (offset_x, offset_y)

    def calculate_arc_waypoint(self, robot_pos, target_pos):
        """
        Calculates an intermediate waypoint on the clearance boundary 
        to path around the collection area.
        """
        cx, cy = self.center_pile
        rx, ry = robot_pos
        tx, ty = target_pos

        # Radial angles relative to collection center (Y inverted for image coordinates)
        angle_robot = math.atan2(-(ry - cy), rx - cx)
        angle_target = math.atan2(-(ty - cy), tx - cx)

        # Normalized angular difference [-pi, pi]
        diff = math.atan2(math.sin(angle_target - angle_robot), 
                          math.cos(angle_target - angle_robot))

        # Angle bisector on clearance perimeter
        mid_angle = angle_robot + (diff / 2.0)

        # Compute point coordinates on clearance circle
        arc_x = cx + self.clearance_radius * math.cos(mid_angle)
        arc_y = cy - self.clearance_radius * math.sin(mid_angle)

        return (arc_x, arc_y)

    def get_idle_path(self, robot_pos):
        """
        Generates the intermediate arc waypoint and final target 
        for routing the robot safely back to the idle anchor point.
        """
        arc_pt = self.calculate_arc_waypoint(robot_pos, self.idle_target)
        return arc_pt, self.idle_target