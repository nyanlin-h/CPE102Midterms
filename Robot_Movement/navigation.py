import math
import os

class FieldNavigator:
    def __init__(self, map_filename="map_drop_off.txt", center_pile=(230.0, 240.0), avoid_radius_cm=35.0, exit_offset_cm=7.0, px_per_cm=3.74):
        """
        Field Navigator Module using Real-World Scale Calibration and Safe Arc Routing.
        :param map_filename: Path to file containing saved drop targets
        :param center_pile: (X, Y) pixel coordinates of center pile
        :param avoid_radius_cm: Clearance radius around center pile in centimeters
        :param exit_offset_cm: Physical distance from AprilTag center to side release gate in centimeters
        :param px_per_cm: Calibrated camera pixel scale ratio (3.74 px/cm)
        """
        self.center_pile = center_pile
        self.px_per_cm = px_per_cm
        
        # Convert physical centimeter parameters to pixel dimensions
        self.avoid_radius = avoid_radius_cm * self.px_per_cm   # 35.0 cm * 3.74 = 130.9 px
        self.exit_offset = exit_offset_cm * self.px_per_cm     # 7.0 cm * 3.74 = 26.18 px
        
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
        """
        Offsets drop zone target so right-side release chute aligns over target center.
        Applies Y-axis inversion to match OpenCV screen coordinates.
        """
        if drop_target_key not in self.drop_off_targets:
            return None
            
        tx, ty = self.drop_off_targets[drop_target_key]
        # Invert sin/cos Y-components to match OpenCV inverted Y-down system
        target_x = tx + self.exit_offset * math.sin(robot_heading_rad)
        target_y = ty + self.exit_offset * math.cos(robot_heading_rad)
        return (target_x, target_y)

    def calculate_arc_waypoint(self, robot_pos, target_pos):
        """
        Calculates an outer clearance arc point avoiding the center pile by taking
        the shortest angular path and adding an extra 10cm safety margin.
        """
        # Invert Y delta calculations for standard Cartesian math
        angle_robot = math.atan2(-(robot_pos[1] - self.center_pile[1]), robot_pos[0] - self.center_pile[0])
        angle_target = math.atan2(-(target_pos[1] - self.center_pile[1]), target_pos[0] - self.center_pile[0])
        
        diff = angle_target - angle_robot
        # Shortest angle wrapping across circle boundary
        while diff > math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        
        mid_angle = angle_robot + (diff / 2.0)
        
        # Add extra 10cm (37.4px) buffer to guarantee clear perimeter routing
        clearance_radius = self.avoid_radius + (10.0 * self.px_per_cm) 
        
        arc_x = self.center_pile[0] + clearance_radius * math.cos(mid_angle)
        # Convert Y back to OpenCV frame coordinates (downward Y)
        arc_y = self.center_pile[1] - clearance_radius * math.sin(mid_angle)
        return (arc_x, arc_y)