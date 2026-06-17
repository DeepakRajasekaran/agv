#!/usr/bin/env python3
import json
import psycopg2
import numpy as np

def make_arc(cx, cy, r, start_deg, end_deg, num_pts=20):
    angles = np.linspace(np.radians(start_deg), np.radians(end_deg), num_pts)
    return [[float(cx + r * np.cos(a)), float(cy + r * np.sin(a))] for a in angles]

def main():
    # Generate coordinates for each edge
    edges_data = {}

    # Edge 1 (1 -> 2): TAG_LEFT (-5,0) -> TAG_TOP (-2,2)
    # arc_tl + [-2.0, 2.0]
    edges_data[1] = make_arc(-3.0, 0.0, 2.0, 180, 90) + [[-2.0, 2.0]]

    # Edge 2 (2 -> 3): TAG_TOP (-2,2) -> junction (-1.1, 2)
    edges_data[2] = [[-2.0, 2.0], [-1.1, 2.0]]

    # Edge 3 (3 -> 4): junction (-1.1, 2) -> waypoint (3, 2)
    edges_data[3] = [[-1.1, 2.0], [3.0, 2.0]]

    # Edge 4 (4 -> 5): waypoint (3, 2) -> TAG_RIGHT (5, 0)
    # arc_tr
    edges_data[4] = make_arc(3.0, 0.0, 2.0, 90, 0)

    # Edge 5 (5 -> 6): TAG_RIGHT (5, 0) -> TAG_BOT (2, -2)
    # straight down + arc_br + straight to 2,-2
    edges_data[5] = [[5.0, 0.0], [5.0, -1.0]] + make_arc(4.0, -1.0, 1.0, 0, -90) + [[4.0, -2.0], [3.0, -2.0], [2.0, -2.0]]

    # Edge 6 (6 -> 7): TAG_BOT (2, -2) -> junction (1.1, -2)
    edges_data[6] = [[2.0, -2.0], [1.1, -2.0]]

    # Edge 7 (7 -> 8): junction (1.1, -2) -> waypoint (-4, -2)
    edges_data[7] = [[1.1, -2.0], [-4.0, -2.0]]

    # Edge 8 (8 -> 1): waypoint (-4, -2) -> TAG_LEFT (-5, 0)
    # arc_bl + straight up
    edges_data[8] = make_arc(-4.0, -1.0, 1.0, 270, 180) + [[-5.0, -1.0], [-5.0, 0.0]]

    # Edge 9 (3 -> 9): junction (-1.1, 2) -> waypoint (-0.1, 1)
    # arc_md
    edges_data[9] = make_arc(-1.1, 1.0, 1.0, 90, 0)

    # Edge 10 (9 -> 10): waypoint (-0.1, 1) -> waypoint (-0.1, -2)
    # straight + arc_md_merge
    edges_data[10] = [[-0.1, 1.0], [-0.1, -1.0]] + make_arc(-1.1, -1.0, 1.0, 0, -90)

    # Edge 11 (10 -> 1): waypoint (-0.1, -2) -> TAG_LEFT (-5, 0)
    # straight from merge end to left edge + arc_bl + straight to TAG_LEFT
    edges_data[11] = [[-1.1, -2.0], [-4.0, -2.0]] + make_arc(-4.0, -1.0, 1.0, 270, 180) + [[-5.0, -1.0], [-5.0, 0.0]]

    # Edge 12 (7 -> 11): junction (1.1, -2) -> waypoint (0.1, -1)
    # arc_mu
    edges_data[12] = make_arc(1.1, -1.0, 1.0, 270, 180)

    # Edge 13 (11 -> 12): waypoint (0.1, -1) -> waypoint (0.1, 2)
    # straight + arc_mu_merge
    edges_data[13] = [[0.1, -1.0], [0.1, 1.0]] + make_arc(1.1, 1.0, 1.0, 180, 90)

    # Edge 14 (12 -> 5): waypoint (0.1, 2) -> TAG_RIGHT (5, 0)
    # straight from merge end to right edge + arc_tr
    edges_data[14] = [[1.1, 2.0], [3.0, 2.0]] + make_arc(3.0, 0.0, 2.0, 90, 0)

    # Update database
    try:
        conn = psycopg2.connect(
            dbname="map_db",
            user="nav_user",
            password="nav_password",
            host="172.18.0.1",
            port=5432
        )
        cursor = conn.cursor()
        
        for edge_id, points in edges_data.items():
            points_json = json.dumps(points)
            cursor.execute(
                "UPDATE edges SET control_points = %s WHERE id = %s;",
                (points_json, edge_id)
            )
            print(f"Updated Edge ID {edge_id} with {len(points)} control points.")
            
        conn.commit()
        cursor.close()
        conn.close()
        print("Database detailed map population complete!")
    except Exception as e:
        print(f"Error updating database: {e}")

if __name__ == '__main__':
    main()
