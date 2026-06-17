-- Clear existing dummy data
TRUNCATE TABLE edges, vertices RESTART IDENTITY CASCADE;

-- Insert Vertices with Tag IDs in the type column
-- ID 1: TAG_LEFT
INSERT INTO vertices (pos_x, pos_y, type) VALUES (-5.0, 0.0, 'TAG_LEFT');
-- ID 2: TAG_TOP
INSERT INTO vertices (pos_x, pos_y, type) VALUES (-2.0, 2.0, 'TAG_TOP');
-- ID 3: JUNCTION_TOP
INSERT INTO vertices (pos_x, pos_y, type) VALUES (-1.1, 2.0, 'junction');
-- ID 4: OUTER_TOP_RIGHT
INSERT INTO vertices (pos_x, pos_y, type) VALUES (3.0, 2.0, 'waypoint');
-- ID 5: TAG_RIGHT
INSERT INTO vertices (pos_x, pos_y, type) VALUES (5.0, 0.0, 'TAG_RIGHT');
-- ID 6: TAG_BOT
INSERT INTO vertices (pos_x, pos_y, type) VALUES (2.0, -2.0, 'TAG_BOT');
-- ID 7: JUNCTION_BOT
INSERT INTO vertices (pos_x, pos_y, type) VALUES (1.1, -2.0, 'junction');
-- ID 8: OUTER_BOT_LEFT
INSERT INTO vertices (pos_x, pos_y, type) VALUES (-4.0, -2.0, 'waypoint');
-- ID 9: SHORTCUT_DOWN_1
INSERT INTO vertices (pos_x, pos_y, type) VALUES (-0.1, 1.0, 'waypoint');
-- ID 10: SHORTCUT_DOWN_2
INSERT INTO vertices (pos_x, pos_y, type) VALUES (-0.1, -2.0, 'waypoint');
-- ID 11: SHORTCUT_UP_1
INSERT INTO vertices (pos_x, pos_y, type) VALUES (0.1, -1.0, 'waypoint');
-- ID 12: SHORTCUT_UP_2
INSERT INTO vertices (pos_x, pos_y, type) VALUES (0.1, 2.0, 'waypoint');

-- Insert Edges (Clockwise flow)
-- Outer Loop
INSERT INTO edges (start_vertex_id, end_vertex_id, is_bidirectional, curve_type) VALUES (1, 2, FALSE, 'line');
INSERT INTO edges (start_vertex_id, end_vertex_id, is_bidirectional, curve_type) VALUES (2, 3, FALSE, 'line');
INSERT INTO edges (start_vertex_id, end_vertex_id, is_bidirectional, curve_type) VALUES (3, 4, FALSE, 'line');
INSERT INTO edges (start_vertex_id, end_vertex_id, is_bidirectional, curve_type) VALUES (4, 5, FALSE, 'line');
INSERT INTO edges (start_vertex_id, end_vertex_id, is_bidirectional, curve_type) VALUES (5, 6, FALSE, 'line');
INSERT INTO edges (start_vertex_id, end_vertex_id, is_bidirectional, curve_type) VALUES (6, 7, FALSE, 'line');
INSERT INTO edges (start_vertex_id, end_vertex_id, is_bidirectional, curve_type) VALUES (7, 8, FALSE, 'line');
INSERT INTO edges (start_vertex_id, end_vertex_id, is_bidirectional, curve_type) VALUES (8, 1, FALSE, 'line');

-- Shortcut Down (Branching from JUNCTION_TOP (3) to TAG_LEFT (1))
INSERT INTO edges (start_vertex_id, end_vertex_id, is_bidirectional, curve_type) VALUES (3, 9, FALSE, 'line');
INSERT INTO edges (start_vertex_id, end_vertex_id, is_bidirectional, curve_type) VALUES (9, 10, FALSE, 'line');
INSERT INTO edges (start_vertex_id, end_vertex_id, is_bidirectional, curve_type) VALUES (10, 1, FALSE, 'line');

-- Shortcut Up (Branching from JUNCTION_BOT (7) to TAG_RIGHT (5))
INSERT INTO edges (start_vertex_id, end_vertex_id, is_bidirectional, curve_type) VALUES (7, 11, FALSE, 'line');
INSERT INTO edges (start_vertex_id, end_vertex_id, is_bidirectional, curve_type) VALUES (11, 12, FALSE, 'line');
INSERT INTO edges (start_vertex_id, end_vertex_id, is_bidirectional, curve_type) VALUES (12, 5, FALSE, 'line');
