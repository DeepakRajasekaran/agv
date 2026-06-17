import React, { useState, useRef, useEffect } from "react";
import "./App.css";
import CADSketcher from "./components/CADSketcher";
import CADToolbar from "./components/CADToolbar";
import GridCanvas from "./components/GridCanvas";
import GlobalHeader from "./components/GlobalHeader";
import LeftSidebar from "./components/LeftSidebar";
import RightSidebar from "./components/RightSidebar";
import SimulationModal from "./components/SimulationModal";
import { Layer, Group, Circle, Line, Rect, Text } from 'react-konva';

const API_BASE = "http://localhost:8080";

function App() {
  // ── CAD / Map Editor State ──────────────────────────────────────────────
  const [activeTool, setActiveTool] = useState("select");
  const [isBidirectional, setIsBidirectional] = useState(false);
  const [sketches, setSketches] = useState([]);
  const [constraints, setConstraints] = useState([]);
  const [dimensions, setDimensions] = useState([]);
  const [fixedPoints, setFixedPoints] = useState([]);
  const [overlay, setOverlay] = useState(null);
  const [editMode, setEditMode] = useState(false);

  // ── Live Data State ─────────────────────────────────────────────────────
  const [activeMap, setActiveMap] = useState({ vertices: [], edges: [] });
  const [selectedItem, setSelectedItem] = useState(null);
  const [robotPose, setRobotPose] = useState(null);
  const [plannedPath, setPlannedPath] = useState([]);

  // ── Layout / View State ─────────────────────────────────────────────────
  const [activeView, setActiveView] = useState("mission_control");

  // ── Diagnostics State ───────────────────────────────────────────────────
  const [diagnostics, setDiagnostics] = useState({
    state: 'IDLE',
    cross_track_error: 0.0,
    last_tag: '',
    nav_vel: { linear: 0, angular: 0 },
    fault_log: [],
  });

  // ── Mission State ───────────────────────────────────────────────────────
  const [mission, setMission] = useState(null);

  // ── Sim Mode (kept for compatibility) ───────────────────────────────────
  const [simMode, setSimMode] = useState(false);
  
  // ── Sim Lifecycle State ─────────────────────────────────────────────────
  const [simStatus, setSimStatus] = useState("unknown");
  const [isSimModalOpen, setIsSimModalOpen] = useState(false);
  const [isSimRestarting, setIsSimRestarting] = useState(false);

  const sketcherRef = useRef(null);
  const SCALE_M = 100; // 100 pixels = 1 meter

  // ── Fetch Active Map on Mount ───────────────────────────────────────────
  useEffect(() => {
    fetch(`${API_BASE}/api/map`)
      .then(res => res.json())
      .then(data => {
        if (data && data.vertices) {
          setActiveMap(data);
        }
      })
      .catch(err => console.error("Error loading active map:", err));
  }, []);

  // ── Polling: Robot Pose, Plan, Diagnostics ──────────────────────────────
  useEffect(() => {
    const interval = setInterval(() => {
      // Robot Pose
      fetch(`${API_BASE}/api/robot_pose`)
        .then(res => res.json())
        .then(data => {
          if (data && typeof data.x === 'number') {
            setRobotPose(data);
          }
        })
        .catch(() => {});

      // Planned Path
      fetch(`${API_BASE}/api/plan`)
        .then(res => res.json())
        .then(data => {
          if (Array.isArray(data)) {
            setPlannedPath(data);
          }
        })
        .catch(() => {});

      // Diagnostics
      fetch(`${API_BASE}/api/diagnostics`)
        .then(res => res.json())
        .then(data => {
          if (data && data.state) {
            setDiagnostics(data);
          }
        })
        .catch(() => {});

      // Mission Status
      fetch(`${API_BASE}/api/mission/status`)
        .then(res => res.json())
        .then(data => {
          if (data && data.status !== 'IDLE') {
            setMission(data);
          }
        })
        .catch(() => {});
        
      // Sim Status
      fetch(`${API_BASE}/api/sim/status`)
        .then(res => res.json())
        .then(data => {
          if (data && data.status) {
            setSimStatus(data.status);
          }
        })
        .catch(() => setSimStatus("stopped"));
    }, 200);
    return () => clearInterval(interval);
  }, []);

  // ── Handlers ────────────────────────────────────────────────────────────
  const undo = () => {
    console.log("Undo not fully implemented in lightweight version yet.");
  };

  const handleClearSketch = () => {
    if (window.confirm("Clear map?")) {
      setSketches([]);
      setConstraints([]);
      setDimensions([]);
      setFixedPoints([]);
    }
  };

  const handleLoadMap = async () => {
    if (!simMode) {
      alert("Enable Sim Mode first!");
      return;
    }
    console.log("Loading map into simulator...", sketches);
    alert("Map paths loaded! Simulation restarting...");
  };

  const handleNavigateTo = (vertex) => {
    fetch(`${API_BASE}/api/navigate`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ x: vertex.x, y: vertex.y }),
    })
      .then(res => res.json())
      .then(data => {
        if (data.success) {
          console.log(`Navigation goal sent to ${vertex.type} (${vertex.x}, ${vertex.y})`);
        } else {
          console.error("Failed to send navigation goal");
        }
      })
      .catch(err => console.error("Error sending navigation goal:", err));
  };

  const handleEstop = () => {
    fetch(`${API_BASE}/api/estop`, { method: "POST" })
      .then(res => res.json())
      .then(data => console.log("E-Stop:", data))
      .catch(err => console.error("E-Stop error:", err));
  };

  const handleNewMission = () => {
    console.log("New Mission requested");
    // TODO: Open mission builder modal
  };

  const handleInjectFault = (type) => {
    fetch(`${API_BASE}/api/sim/inject_fault`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ type }),
    })
      .then(res => res.json())
      .then(data => console.log("Fault injected:", data))
      .catch(err => console.error("Fault injection error:", err));
  };

  const handleOpenSimModal = (isRestart) => {
    setIsSimRestarting(isRestart);
    setIsSimModalOpen(true);
  };

  const handleSimAction = (headless) => {
    setIsSimModalOpen(false);
    const endpoint = isSimRestarting ? '/api/sim/restart' : '/api/sim/start';
    
    fetch(`${API_BASE}${endpoint}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ headless }),
    })
      .then(res => res.json())
      .then(data => console.log(`Simulation ${isSimRestarting ? 'restarted' : 'started'}:`, data))
      .catch(err => console.error("Sim action error:", err));
  };

  return (
    <div style={{ display: 'flex', flexDirection: 'column', height: '100vh', width: '100vw', background: 'var(--color-surface)' }}>
      {/* ── Global Header ──────────────────────────────────────────── */}
      <GlobalHeader
        activeMapName="ASSEMBLY LINE 1"
        onEstop={handleEstop}
        editMode={editMode}
        onToggleEdit={(mode) => {
          setEditMode(mode);
          if (!mode) setActiveTool("select");
        }}
        simStatus={simStatus}
        onOpenSimModal={handleOpenSimModal}
      />
      
      <SimulationModal 
        isOpen={isSimModalOpen} 
        onClose={() => setIsSimModalOpen(false)} 
        onStart={handleSimAction} 
        isRestart={isSimRestarting} 
      />

      {/* ── Main Content Area ──────────────────────────────────────── */}
      <div style={{
        display: 'flex',
        flex: 1,
        paddingTop: 'var(--header-height)',
      }}>
        {/* ── Left Sidebar ─────────────────────────────────────────── */}
        <LeftSidebar
          activeView={activeView}
          onViewChange={setActiveView}
          mission={mission}
          onNewMission={handleNewMission}
          activeMap={activeMap}
          onNavigateToVertex={handleNavigateTo}
          faultCount={diagnostics.fault_log.filter(e => e.level === 'error').length}
        />

        {/* ── Center Canvas (Map Studio) ───────────────────────────── */}
        <main style={{
          flex: 1,
          marginLeft: 'var(--sidebar-left-width)',
          marginRight: 'var(--sidebar-right-width)',
          height: 'calc(100vh - var(--header-height))',
          position: 'relative',
          overflow: 'hidden',
          display: 'flex',
          flexDirection: 'column',
        }}>
          {/* Canvas Label Overlay */}
          <div style={{
            position: 'absolute',
            top: 'var(--space-md)',
            left: 'var(--space-md)',
            zIndex: 10,
            display: 'flex',
            alignItems: 'center',
            background: 'var(--color-surface)',
            border: '1px solid var(--color-outline-variant)',
            borderRadius: 'var(--radius-default)',
            padding: 'var(--space-sm) var(--space-md)',
            padding: 'var(--space-sm) var(--space-md)',
            gap: 8,
          }}>
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="var(--color-on-surface-variant)" strokeWidth="2">
              <polygon points="1 6 1 22 8 18 16 22 23 18 23 2 16 6 8 2 1 6" /><line x1="8" y1="2" x2="8" y2="18" /><line x1="16" y1="6" x2="16" y2="22" />
            </svg>
            <span className="font-label-caps" style={{ color: 'var(--color-on-surface)' }}>
              MAP STUDIO : {editMode ? 'EDIT MODE' : 'LIVE VIEW'}
            </span>
          </div>

          {/* Edit Mode Toggle */}
          <div style={{
            position: 'absolute',
            top: 'var(--space-md)',
            right: 'var(--space-md)',
            zIndex: 10,
            display: 'flex',
            alignItems: 'center',
            gap: 8,
          }}>
            {editMode ? (
              <>
                <CADToolbar
                  activeTool={activeTool}
                  setActiveTool={setActiveTool}
                  isBidirectional={isBidirectional}
                  setIsBidirectional={setIsBidirectional}
                  undo={undo}
                  handleClearSketch={handleClearSketch}
                />
                <button
                  className="btn btn-secondary"
                  onClick={handleLoadMap}
                  style={{ fontSize: 11 }}
                >
                  SAVE REVISION
                </button>
              </>
            ) : null}
          </div>

          {/* GridCanvas with all Konva layers */}
          <div style={{ flex: 1, position: 'relative', overflow: 'hidden' }}>
            <GridCanvas>
              {({ scale, stagePos, SCALE_M, setOverlay }) => (
                <>
                  {/* Active Database Map Layer */}
                  <Layer key="active-map-layer">
                    {activeMap.edges.map(edge => {
                      // If detailed path points are available, draw a thick virtual track
                      if (edge.control_points && edge.control_points.length > 0) {
                        const points = [];
                        edge.control_points.forEach(pt => {
                          points.push(pt[0] * SCALE_M, -pt[1] * SCALE_M);
                        });
                        return (
                          <Line
                            key={`map-edge-${edge.id}`}
                            points={points}
                            stroke="#FACC15" // Industrial Yellow for paths
                            strokeWidth={5}
                            lineCap="round"
                            lineJoin="round"
                            opacity={0.6}
                            onClick={() => { if (activeView === 'map_studio') setSelectedItem({ ...edge, type: 'EDGE' }) }}
                            onTap={() => { if (activeView === 'map_studio') setSelectedItem({ ...edge, type: 'EDGE' }) }}
                            onMouseEnter={e => { if (activeView === 'map_studio') e.target.getStage().container().style.cursor = 'pointer' }}
                            onMouseLeave={e => { if (activeView === 'map_studio') e.target.getStage().container().style.cursor = 'default' }}
                          />
                        );
                      }

                      // Fallback to straight line connecting vertices
                      const vStart = activeMap.vertices.find(v => v.id === edge.start_vertex_id);
                      const vEnd = activeMap.vertices.find(v => v.id === edge.end_vertex_id);
                      if (!vStart || !vEnd) return null;
                      return (
                        <Line
                          key={`map-edge-${edge.id}`}
                          points={[
                            vStart.x * SCALE_M,
                            -vStart.y * SCALE_M,
                            vEnd.x * SCALE_M,
                            -vEnd.y * SCALE_M
                          ]}
                          stroke="#FACC15"
                          strokeWidth={3 / scale}
                          opacity={0.6}
                          dash={edge.is_bidirectional ? [8 / scale, 4 / scale] : null}
                          onClick={() => { if (activeView === 'map_studio') setSelectedItem({ ...edge, type: 'EDGE' }) }}
                          onTap={() => { if (activeView === 'map_studio') setSelectedItem({ ...edge, type: 'EDGE' }) }}
                          onMouseEnter={e => { if (activeView === 'map_studio') e.target.getStage().container().style.cursor = 'pointer' }}
                          onMouseLeave={e => { if (activeView === 'map_studio') e.target.getStage().container().style.cursor = 'default' }}
                        />
                      );
                    })}

                    {activeMap.vertices.map(vertex => (
                      <Group
                        key={`map-vertex-${vertex.id}`}
                        x={vertex.x * SCALE_M}
                        y={-vertex.y * SCALE_M}
                        onClick={() => {
                          if (activeView === 'map_studio') setSelectedItem(vertex);
                          if (!editMode && activeView === 'mission_control') handleNavigateTo(vertex);
                        }}
                        onTap={() => {
                          if (activeView === 'map_studio') setSelectedItem(vertex);
                          if (!editMode && activeView === 'mission_control') handleNavigateTo(vertex);
                        }}
                        onMouseEnter={e => {
                          const stage = e.target.getStage();
                          stage.container().style.cursor = 'pointer';
                        }}
                        onMouseLeave={e => {
                          const stage = e.target.getStage();
                          stage.container().style.cursor = 'default';
                        }}
                      >
                        {/* Vertex Outer Ring */}
                        <Circle
                          radius={10 / scale}
                          fill="transparent"
                          stroke={vertex.type === 'junction' ? '#FACC15' : '#4edea3'}
                          strokeWidth={2 / scale}
                        />
                        {/* Vertex Inner Dot */}
                        <Circle
                          radius={4 / scale}
                          fill={vertex.type === 'junction' ? '#FACC15' : '#4edea3'}
                        />
                        {/* Vertex Label */}
                        <Text
                          text={`V-${String(vertex.id).padStart(2, '0')}`}
                          fill="var(--color-on-surface-variant, #d1c6ab)"
                          fontFamily="JetBrains Mono, monospace"
                          fontSize={9 / scale}
                          y={14 / scale}
                          align="center"
                          offsetX={20 / scale}
                          width={40 / scale}
                        />
                      </Group>
                    ))}
                  </Layer>

                  {/* Planned/Generated Path Layer */}
                  {plannedPath.length > 0 && (
                    <Layer key="planned-path-layer">
                      <Line
                        points={plannedPath.flatMap(pt => [pt.x * SCALE_M, -pt.y * SCALE_M])}
                        stroke="#00e5ff"
                        strokeWidth={4 / scale}
                        dash={[8 / scale, 6 / scale]}
                        lineCap="round"
                        lineJoin="round"
                      />
                      {/* Waypoint markers */}
                      {plannedPath.map((pt, idx) => (
                        <Circle
                          key={`plan-pt-${idx}`}
                          x={pt.x * SCALE_M}
                          y={-pt.y * SCALE_M}
                          radius={2.5 / scale}
                          fill="#00e5ff"
                        />
                      ))}
                    </Layer>
                  )}

                  {/* Live Robot Pose Layer */}
                  {robotPose && (
                    <Layer key="live-robot-layer">
                      <Group
                        x={robotPose.x * SCALE_M}
                        y={-robotPose.y * SCALE_M}
                        rotation={-robotPose.theta * 180 / Math.PI}
                      >
                        {/* AGV Chassis */}
                        <Rect
                          x={-25 / scale}
                          y={-18 / scale}
                          width={50 / scale}
                          height={36 / scale}
                          fill="rgba(250, 204, 21, 0.2)"
                          stroke="#FACC15"
                          strokeWidth={2 / scale}
                          cornerRadius={4 / scale}
                        />
                        {/* Direction Arrow */}
                        <Line
                          points={[
                            -10 / scale, 0,
                            15 / scale, 0,
                            8 / scale, -5 / scale,
                            15 / scale, 0,
                            8 / scale, 5 / scale
                          ]}
                          stroke="#FACC15"
                          strokeWidth={2.5 / scale}
                        />
                        {/* Left Wheel */}
                        <Rect
                          x={-12 / scale}
                          y={-22 / scale}
                          width={24 / scale}
                          height={4 / scale}
                          fill="var(--color-deep-slate, #0F172A)"
                          stroke="#FACC15"
                          strokeWidth={0.5 / scale}
                        />
                        {/* Right Wheel */}
                        <Rect
                          x={-12 / scale}
                          y={18 / scale}
                          width={24 / scale}
                          height={4 / scale}
                          fill="var(--color-deep-slate, #0F172A)"
                          stroke="#FACC15"
                          strokeWidth={0.5 / scale}
                        />
                      </Group>
                    </Layer>
                  )}

                  {/* CAD Editor (only in edit mode) */}
                  {editMode && (
                    <Layer key="cad-sketcher-layer">
                      <CADSketcher
                        ref={sketcherRef}
                        sketches={sketches}
                        setSketches={setSketches}
                        dimensions={dimensions}
                        setDimensions={setDimensions}
                        fixedPoints={fixedPoints}
                        setFixedPoints={setFixedPoints}
                        constraints={constraints}
                        setConstraints={setConstraints}
                        scale={scale}
                        SCALE_M={SCALE_M}
                        activeTool={activeTool}
                        setOverlay={setOverlay}
                        pushToHistory={() => {}}
                      />
                    </Layer>
                  )}
                </>
              )}
            </GridCanvas>

            {/* Render interactive overlays (like dimension inputs) */}
            {overlay && (
               <overlay.component {...overlay.props} />
            )}
          </div>

          {/* Simulation Control Floating Panel */}
          <div style={{
            position: 'absolute',
            bottom: 'var(--space-md)',
            right: 'var(--space-md)',
            zIndex: 30,
            background: 'var(--color-panel-bg)',
            border: '1px solid var(--color-panel-border)',
            borderRadius: 'var(--radius-default)',
            padding: 'var(--space-md)',
            width: 220,
            boxShadow: '0 4px 12px rgba(0,0,0,0.4)',
          }}>
            <div style={{
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'space-between',
              marginBottom: 'var(--space-sm)',
            }}>
              <span className="font-label-caps" style={{
                color: 'var(--color-on-surface)',
                borderTop: '2px solid var(--color-text-muted)',
                paddingTop: 4,
                fontSize: 10,
              }}>
                SIMULATION CONTROL
              </span>
              <span style={{ color: 'var(--color-on-surface-variant)', fontSize: 14 }}>⠿</span>
            </div>
            <div style={{ display: 'flex', flexDirection: 'column', gap: 'var(--space-sm)' }}>
              <button
                className="btn-ghost"
                onClick={() => handleInjectFault('tape_loss')}
                style={{ justifyContent: 'flex-start', fontSize: 11 }}
              >
                <span className="indicator-dot indicator-dot-error" />
                INJECT TAPE LOSS
              </button>
              <button
                className="btn-ghost"
                onClick={() => handleInjectFault('obstacle')}
                style={{ justifyContent: 'flex-start', fontSize: 11 }}
              >
                <span className="indicator-dot indicator-dot-error" />
                INJECT OBSTACLE
              </button>
            </div>
          </div>
        </main>

        {/* ── Right Sidebar (Diagnostics / Properties) ───────────────── */}
        <RightSidebar diagnostics={diagnostics} activeView={activeView} selectedItem={selectedItem} />
      </div>
    </div>
  );
}

export default App;
