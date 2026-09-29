import math
import os
from config import (MAP_FILENAME, CENTER_PILE, AVOID_RADIUS_CM,
                    ARC_MARGIN_CM, PX_PER_CM)


class FieldNavigator:
    def __init__(self, map_filename=MAP_FILENAME, center_pile=CENTER_PILE,
                 avoid_radius_cm=AVOID_RADIUS_CM, arc_margin_cm=ARC_MARGIN_CM,
                 px_per_cm=PX_PER_CM):
        """
        Field navigator: loads drop zones and routes around the centre pile.
        The robot has no side chute, so it drives to the zone centre itself.
        """
        self.center_pile = center_pile
        self.px_per_cm = px_per_cm
        self.avoid_radius = avoid_radius_cm * px_per_cm
        self.clearance_radius = self.avoid_radius + arc_margin_cm * px_per_cm
        self.drop_off_targets = {}
        self.load_map(map_filename)

    def load_map(self, filename):
        self.drop_off_targets.clear()
        if not os.path.exists(filename):
            print(f"[NAV WARNING]: Map file '{filename}' not found.")
            return
        with open(filename, "r") as f:
            for line in f:
                parts = line.strip().split(",")
                if len(parts) == 3:
                    self.drop_off_targets[parts[0].strip()] = (
                        float(parts[1]), float(parts[2]))
        print(f"[NAV MODULE]: Loaded {len(self.drop_off_targets)} drop target(s) from '{filename}'.")

    def get_drop_target(self, name):
        """Centre of the named drop zone (pixels), or None if unknown."""
        return self.drop_off_targets.get(name)

    def calculate_arc_waypoint(self, robot_pos, target_pos):
        """
        Point on the clearance circle around the pile, at the midpoint angle
        (shortest way round) between the robot and the target.
        """
        cx, cy = self.center_pile
        angle_robot = math.atan2(-(robot_pos[1] - cy), robot_pos[0] - cx)
        angle_target = math.atan2(-(target_pos[1] - cy), target_pos[0] - cx)

        diff = math.atan2(math.sin(angle_target - angle_robot),
                          math.cos(angle_target - angle_robot))
        mid_angle = angle_robot + diff / 2.0

        arc_x = cx + self.clearance_radius * math.cos(mid_angle)
        arc_y = cy - self.clearance_radius * math.sin(mid_angle)  # screen Y is down
        return (arc_x, arc_y)