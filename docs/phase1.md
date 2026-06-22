Here is the complete end-to-end software, backend, and UI architecture for Phase 1 (Simulation Focus). This model builds upon the MuJoCo/ROS 2 base and integrates industrial-grade fleet management concepts like multi-map handling and mission sequencing.

---

### 1. Core Software & Simulation Architecture (ROS 2 Jazzy + MuJoCo)

The software layer must be strictly deterministic. We treat the MuJoCo simulation not just as an animation, but as a rigid data provider that exactly mimics the byte-streams of real hardware.

#### A. The Emulated Hardware Interface (`hardware_sim_node.py`)

This is the bridge that keeps the C++ control logic ignorant of whether it is in a simulation or reality.

* **Physics Loop (MuJoCo):** Runs at 1000Hz internally to calculate precise tire-slip and caster sway. It exposes a 100Hz tick rate to ROS 2.
* **Sensor Emulation (`TapeMap.py` Integration):**
* Instead of a physical magnetic sensor, the node drops a mathematical perpendicular line from the AGV's center of rotation to the nearest `control_point` of the active track in the database.
* It calculates the `cross_track_error` (e.g., +15mm right, -12mm left).
* It injects Gaussian noise ($\sigma = 0.5mm$) to simulate electrical sensor jitter, forcing your PID controller to handle realistic, imperfect data.
* It publishes a custom message `agv_msgs/MagneticTrackData` at 100Hz.


* **RFID/Tag Emulation:** When the AGV's pose falls within a 50mm radius of a map vertex (a topological tag), it publishes the tag's ID and coordinates on `/sensor/tag_read`.

#### B. The Navigation & Control Stack (C++)

* **Line Follower Controller (`path_follower_node`):**
* A ROS 2 Lifecycle node operating at a strict 50Hz.
* Subscribes to `agv_msgs/MagneticTrackData`.
* **PID Logic:** Converts the cross-track error into a rotational velocity command (`angular.z`). If the error exceeds the fault boundary ($\pm$250mm), the state machine instantly transitions to `FAULT_LINE_LOST` and commands zero velocity (`linear.x = 0`).


* **Topological Router (`nav_server_node`):**
* An Action Server (`nav2_msgs/action/NavigateToPose`).
* When given a destination tag, it uses Dijkstra's algorithm to calculate the shortest path along the topological graph (e.g., `TAG_START` $\rightarrow$ `TAG_A` $\rightarrow$ `TAG_B`).
* It retrieves the geometric curves (`control_points`) between these tags from the database and publishes the active trajectory.



---

### 2. Backend & Data Layer (Fleet & Mission Management)

To support industrial features like mission sequencing and multi-map handling, the backend (`map_studio_backend.py`) must function as a mini-Fleet Management System (FMS) inspired by the **VDA 5050** interoperability standard.

#### A. Multi-Map Management

Industrial facilities change layouts frequently. The backend must handle full CRUD (Create, Read, Update, Delete) operations for maps.

* **Database Schema (PostgreSQL `map_db`):**
* `maps` table: `id`, `name` (e.g., "Warehouse A", "Assembly Line 1"), `version_hash`, `is_active`.
* `vertices` table: `id`, `map_id`, `x`, `y`, `type` (e.g., `CHARGING_STATION`, `JUNCTION`).
* `edges` table: `id`, `map_id`, `start_vertex`, `end_vertex`, `control_points` (JSONB curve data), `speed_limit`.


* **Map Loading:** When a new map is selected via the API, the backend commands the AGV software to drop its current memory graph, loads the new active map into the Dijkstra solver, and resets the simulation environment to the new starting tag.

#### B. Mission & Action Sequencing

An AGV does not just "drive to a point." It executes a sequence of complex tasks. We implement this using a **Task Graph** or **Behavior Tree** model in the backend.

* **Mission Structure:** A Mission is an array of sequential Actions.
* *Action 1:* `Maps_TO` (Target: `TAG_B`)
* *Action 2:* `ALIGN_TO_DOCK` (Target: `LIFTER_1`)
* *Action 3:* `WAIT` (Condition: `EXTERNAL_IO_TRIGGER` or `TIME = 30s`)
* *Action 4:* `Maps_TO` (Target: `TAG_C`)


* **Execution Engine:** The backend pushes these actions to the AGV one by one. It waits for the ROS 2 Action Server to return a `SUCCEEDED` result before pushing the next action. If an action fails (e.g., path blocked), the backend pauses the sequence and triggers an operator alert.

---

### 3. UI Frontend Architecture (For Stitch / React Development)

The UI must be treated as an industrial SCADA (Supervisory Control and Data Acquisition) terminal. It should be modular, high-contrast, and focused entirely on spatial awareness and task control.

#### A. Core Dashboard Layout

* **Global Header:**
* Active Map Selector (Dropdown).
* Global E-Stop Button (Prominent, red, overriding all commands).
* Fleet Status Summary (e.g., "1 AGV Active, 0 Faults").


* **Left Sidebar (Mission Control):**
* **Mission Builder:** A drag-and-drop interface. The user selects a sequence of blocks: `[Drive to Tag]` $\rightarrow$ `[Wait]` $\rightarrow$ `[Drive to Tag]`.
* **Active Mission Queue:** A list showing the current mission, highlighting the active step, and showing pending steps.


* **Center Canvas (Map Studio - Konva.js):**
* A 2D vector rendering of the topological map.
* **Layer 1 (Grid):** Standard coordinate grid for reference.
* **Layer 2 (Graph):** The vertices (dots) and edges (lines/curves representing the magnetic tape).
* **Layer 3 (Live Telemetry):** An icon representing the AGV. It moves in real-time by polling the API for its pose (`x`, `y`, `theta`).
* **Interactivity:** Clicking an edge allows the user to adjust the Bezier curve `control_points`. Clicking a vertex allows them to reassign its tag ID or type.


* **Right Sidebar (Telemetry & Diagnostics):**
* **State Machine Status:** Displays the current ROS 2 state (e.g., `FOLLOW_LINE`, `READ_TAG`, `IDLE`).
* **Live Sensor Feed:** A visual gauge showing the `cross_track_error` (e.g., a needle moving left/right from center).
* **Fault Log:** A scrolling list of warnings and errors (e.g., "Warning: Tape signal weak", "Fault: Obstacle detected").



#### B. Key UI Features & Workflows

* **Simulated Fault Injection Panel:** For testing, the UI should have a hidden "Developer Panel" with buttons to send HTTP requests to the backend to inject faults:
* `[Inject Tape Loss]` $\rightarrow$ Tells `hardware_sim_node.py` to stop publishing MGS data.
* `[Inject Obstacle]` $\rightarrow$ Triggers the simulated safety LiDAR.


* **Map Versioning & Saving:** When a user modifies the curves or tags in the Canvas, a "Save Map Revision" button pushes a JSON payload to the backend to update the PostgreSQL database and create a new version hash.

---

Understanding how the internal C++ State Machine transitions during these simulated sequences is critical before building the UI logic. Here is an interactive breakdown of the core control logic.

[Interactive visualization of the AGV State Machine logic]