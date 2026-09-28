import math
import os

class FieldNavigator:
    def __init__(self, map_filename="map_drop_off.txt", center_pile=(230.0, 240.0), avoid_radius=205.0, exit_offset=24.0):
        """
        Field Navigator Module
        :param map_filename: Path to file containing saved drop targets
        :param center_pile: (X, Y) pixel coordinates of center pile
        :param avoid_radius: Radius around center pile to avoid during transit
        :param exit_offset: Offset in pixels for side discharge gate
        """
        self.center_pile = center_pile
        self.avoid_radius = avoid_radius
        self.exit_offset = exit_offset
        self.drop_off_targets = {}
        
        self.load_map(map_filename)

    def load_map(self, filename):
        """Loads drop zone target coordinates from external map file."""
        self.drop_off_targets.clear()
        if os.path.exists(filename):
            with open(filename, "r") as f:
                for line in f:
                    parts = line.strip().split(",")
                    if len(parts) == 3:
                        name = parts[0].strip()
                        x = float(parts[1].strip())
                        y = float(parts[2].strip())
                        self.drop_off_targets[name] = (x, y)
            print(f"[NAV MODULE]: Successfully loaded {len(self.drop_off_targets)} drop target(s) from '{filename}'.")
        else:
            print(f"[NAV WARNING]: Map file '{filename}' not found.")

    def get_side_drop_target(self, drop_target_key, robot_heading_rad):
        """Offsets drop zone target so right-side release chute aligns over target center."""
        if drop_target_key not in self.drop_off_targets:
            return None
            
        tx, ty = self.drop_off_targets[drop_target_key]
        target_x = tx + self.exit_offset * math.sin(robot_heading_rad)
        target_y = ty - self.exit_offset * math.cos(robot_heading_rad)
        return (target_x, target_y)

    def calculate_arc_waypoint(self, robot_pos, target_pos):
        """Calculates clearance arc waypoint to navigate around central obstacle pile."""
        angle_robot = math.atan2(robot_pos[1] - self.center_pile[1], robot_pos[0] - self.center_pile[0])
        angle_target = math.atan2(target_pos[1] - self.center_pile[1], target_pos[0] - self.center_pile[0])
        
        mid_angle = (angle_robot + angle_target) / 2.0
        arc_x = self.center_pile[0] + self.avoid_radius * math.cos(mid_angle)
        arc_y = self.center_pile[1] + self.avoid_radius * math.sin(mid_angle)
        return (arc_x, arc_y)