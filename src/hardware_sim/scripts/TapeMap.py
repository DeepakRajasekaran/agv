import numpy as np

def make_arc(cx, cy, r, start_deg, end_deg, num_pts=20):
    angles = np.linspace(np.radians(start_deg), np.radians(end_deg), num_pts)
    return [(cx + r * np.cos(a), cy + r * np.sin(a)) for a in angles]

class TapeMap:
    def __init__(self):
        # Precise Industrial Track Geometry (Total Width = 10m)
        
        # Outer Corner Arcs
        arc_tl = make_arc(-3.0, 0.0, 2.0, 180, 90)    # Top-Left R2000
        arc_bl = make_arc(-4.0, -1.0, 1.0, 270, 180)  # Bottom-Left R1000
        arc_tr = make_arc(3.0, 0.0, 2.0, 90, 0)       # Top-Right R2000
        arc_br = make_arc(4.0, -1.0, 1.0, 0, -90)     # Bottom-Right R1000
        
        # Central Crossover Arcs
        arc_md = make_arc(-1.1, 1.0, 1.0, 90, 0)      # Middle Shortcut Down R1000
        arc_md_merge = make_arc(-1.1, -1.0, 1.0, 0, -90) # Middle Shortcut Down Merge R1000
        arc_mu = make_arc(1.1, -1.0, 1.0, 270, 180)   # Middle Shortcut Up R1000
        arc_mu_merge = make_arc(1.1, 1.0, 1.0, 180, 90) # Middle Shortcut Up Merge R1000
        
        # Define continuous path loops (clock-wise track flow)
        
        # Path 1: The continuous 10m outer track
        self.path_outer = (
            [(-1.1, 2.0), (3.0, 2.0)] +
            arc_tr +
            [(5.0, -1.0)] +
            arc_br +
            [(4.0, -2.0), (-4.0, -2.0)] +
            arc_bl +
            [(-5.0, 0.0)] +
            arc_tl +
            [(-3.0, 2.0), (-1.1, 2.0)] # Overlap for seamless PID tracking
        )
        
        # Path 2: The middle shortcut traveling South/West
        self.path_mid_down = (
            [(-3.0, 2.0)] +             # Start securely on top edge
            arc_md +                    # Take shortcut down
            [(-0.1, 1.0), (-0.1, -1.0)] +
            arc_md_merge +
            [(-1.1, -2.0), (-4.0, -2.0)] + # Join bottom edge
            arc_bl +                    # Continue West around the left loop
            [(-5.0, 0.0)] +
            arc_tl +
            [(-3.0, 2.0)]
        )
        
        # Path 3: The middle shortcut traveling North/East
        self.path_mid_up = (
            [(3.0, -2.0)] +             # Start securely on bottom edge
            arc_mu +                    # Take shortcut up
            [(0.1, -1.0), (0.1, 1.0)] +
            arc_mu_merge +
            [(1.1, 2.0), (3.0, 2.0)] +  # Join top edge
            arc_tr +                    # Continue East around the right loop
            [(5.0, -1.0)] +
            arc_br +
            [(4.0, -2.0), (3.0, -2.0)]
        )
        
        # Start state
        self.active_path = self.path_outer
        self.active_branch = "OUTER"
        
        # Junction target for PID turn execution
        self.junction_node = (-1.1, 2.0)
        
        # Marker configurations for triggering controller state transitions
        self.markers = [
            {"pos": (-2.5, 2.05), "is_left": False}, # Approaching top shortcut from left
            {"pos": (2.5, -2.05), "is_left": False}, # Approaching bot shortcut from right
            {"pos": (0.05, -1.5), "is_left": True},  # Approaching bot merge going down
            {"pos": (-0.05, 1.5), "is_left": True}   # Approaching top merge going up
        ]
        self.marker_radius = 0.25
        
        # Precise RFID tag coordinates placed directly prior to junction forks
        self.tags = [
            {"pos": (-2.0, 2.0), "tag_id": "TAG_TOP"},   # Before the downward fork
            {"pos": (2.0, -2.0), "tag_id": "TAG_BOT"},   # Before the upward fork
            {"pos": (5.0, 0.0), "tag_id": "TAG_RIGHT"},  # Right edge midpoint
            {"pos": (-5.0, 0.0), "tag_id": "TAG_LEFT"}   # Left edge midpoint
        ]
        self.tag_radius = 0.25

    def get_closest_point(self, robot_pos, path):
        min_dist = float('inf')
        best_point = np.array(path[0])
        
        for i in range(len(path) - 1):
            a = np.array(path[i])
            b = np.array(path[i+1])
            ab = b - a
            ap = np.array(robot_pos) - a
            ab_len_sq = np.sum(ab**2)
            if ab_len_sq == 0:
                t = 0.0
            else:
                t = np.dot(ap, ab) / ab_len_sq
                t = np.clip(t, 0.0, 1.0)
            closest = a + t * ab
            dist = np.linalg.norm(np.array(robot_pos) - closest)
            if dist < min_dist:
                min_dist = dist
                best_point = closest
                
        return best_point, min_dist

    def compute_lateral_error(self, robot_x, robot_y, robot_yaw, path=None):
        # Check for merging back to OUTER branch
        if self.active_branch == "MID_DOWN":
            # Merges back on bottom edge (West-bound)
            if robot_x < -2.0 and robot_y < -1.5:
                self.active_branch = "OUTER"
                self.active_path = self.path_outer
                self.junction_node = (-1.1, 2.0)
        elif self.active_branch == "MID_UP":
            # Merges back on top edge (East-bound)
            if robot_x > 2.0 and robot_y > 1.5:
                self.active_branch = "OUTER"
                self.active_path = self.path_outer
                self.junction_node = (1.1, -2.0)

        if path is None:
            path = self.active_path
            
        closest_pt, dist = self.get_closest_point((robot_x, robot_y), path)
        w = closest_pt - np.array([robot_x, robot_y])
        l_vec = np.array([-np.sin(robot_yaw), np.cos(robot_yaw)])
        lateral_error = np.dot(w, l_vec)
        
        # Clip to ±0.25m which matches safety.lost_threshold in params.yaml.
        # The physical MGS1600 sensor saturates at ±90mm, but in simulation we extend
        # the range so the PID receives proportional error signal all the way to the fault
        # boundary. This prevents the controller from treating a 10cm drift identically to
        # a 25cm drift, allowing stronger corrective action during recovery maneuvers.
        return float(np.clip(lateral_error, -0.25, 0.25))

    def select_branch(self, command):
        # Topological state transition routing
        if command in ("STRAIGHT", "LEFT", "TURN_LEFT"):
            if self.active_branch == "OUTER":
                pass # Keep following outer loop
        elif command in ("TURN_RIGHT", "RIGHT"):
            if self.active_branch == "OUTER":
                # Currently at top edge, take shortcut South towards TAG_BOT
                if np.linalg.norm(np.array(self.junction_node) - np.array([-1.1, 2.0])) < 0.5:
                    self.active_path = self.path_mid_down
                    self.active_branch = "MID_DOWN"
                    self.junction_node = (2.0, -2.0) # Next major decision is at bot
                # Currently at bottom edge, take shortcut North towards TAG_TOP
                elif np.linalg.norm(np.array(self.junction_node) - np.array([1.1, -2.0])) < 0.5:
                    self.active_path = self.path_mid_up
                    self.active_branch = "MID_UP"
                    self.junction_node = (-1.1, 2.0) # Next major decision is at top
