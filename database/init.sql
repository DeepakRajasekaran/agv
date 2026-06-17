-- Initialize Map Schema

CREATE TABLE vertices (
    id SERIAL PRIMARY KEY,
    pos_x FLOAT NOT NULL,
    pos_y FLOAT NOT NULL,
    type VARCHAR(50) DEFAULT 'junction'
);

CREATE TABLE edges (
    id SERIAL PRIMARY KEY,
    start_vertex_id INTEGER REFERENCES vertices(id) ON DELETE CASCADE,
    end_vertex_id INTEGER REFERENCES vertices(id) ON DELETE CASCADE,
    is_bidirectional BOOLEAN DEFAULT FALSE,
    curve_type VARCHAR(50) DEFAULT 'line',
    control_points JSONB
);

CREATE TABLE missions (
    id SERIAL PRIMARY KEY,
    goal_vertex_id INTEGER REFERENCES vertices(id) ON DELETE CASCADE,
    status VARCHAR(50) DEFAULT 'pending',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Insert dummy data for initial map loading
INSERT INTO vertices (pos_x, pos_y, type) VALUES (0, 0, 'junction');
INSERT INTO vertices (pos_x, pos_y, type) VALUES (5, 0, 'junction');
INSERT INTO edges (start_vertex_id, end_vertex_id, is_bidirectional, curve_type) VALUES (1, 2, TRUE, 'line');
