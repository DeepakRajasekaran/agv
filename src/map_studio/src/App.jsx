import React, { useState, useRef, useEffect } from "react";
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

  // ── Navigation Event Listener ───────────────────────────────────────────
  useEffect(() => {
    const handleNavigate = (e) => {
      if (e.detail) {
        setActiveView(e.detail);
      }
    };
    window.addEventListener("navigate", handleNavigate);
    return () => window.removeEventListener("navigate", handleNavigate);
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
    <div className="bg-surface dark:bg-surface text-on-surface h-screen overflow-hidden flex flex-col font-body-md select-none">
      {/* ── Global Header ──────────────────────────────────────────── */}
      <GlobalHeader
        activeMapName="ASSEMBLY LINE 1"
        activeView={activeView}
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
      <div className="flex flex-1 pt-16">
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
        <main className="flex-1 ml-0 md:ml-64 mr-0 md:mr-80 h-[calc(100vh-64px)] relative bg-[#0F172A] overflow-hidden flex flex-col">
          {/* Canvas Label Overlay */}
          <div className="absolute top-md left-md z-10 flex items-center bg-surface border border-outline-variant rounded p-sm shadow-md opacity-90">
            <span className="material-symbols-outlined text-[20px] text-on-surface-variant mr-2">map</span>
            <span className="font-headline-sm text-headline-sm font-label-caps text-label-caps text-on-surface">
              MAP STUDIO : {editMode ? 'EDIT MODE' : 'LIVE VIEW'}
            </span>
          </div>

          {/* Edit Mode Toggle / Toolbar */}
          <div className="absolute top-4 left-1/2 -translate-x-1/2 z-10">
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
          <div className="absolute bottom-6 right-6 z-30 bg-[#1E293B] border border-[#334155] rounded shadow-lg p-4 w-64 cursor-move">
            <div className="flex items-center justify-between mb-2 cursor-grab">
              <span className="font-label-caps text-[12px] font-bold text-[#dae2fd] tracking-[0.05em] border-t-2 border-[#94A3B8] pt-1">SIMULATION CONTROL</span>
              <span className="material-symbols-outlined text-[16px] text-[#d1c6ab]">drag_indicator</span>
            </div>
            <div className="flex flex-col gap-2">
              <button 
                onClick={() => handleInjectFault('tape_loss')}
                className="bg-transparent border border-[#4d4632] text-[#dae2fd] font-label-caps text-[12px] font-bold tracking-[0.05em] py-2 px-4 rounded hover:bg-[#222a3d] hover:border-[#facc15] transition-all text-left flex items-center"
              >
                <span className="w-2 h-2 bg-[#EF4444] rounded-full mr-2"></span>
                INJECT TAPE LOSS
              </button>
              <button 
                onClick={() => handleInjectFault('obstacle')}
                className="bg-transparent border border-[#4d4632] text-[#dae2fd] font-label-caps text-[12px] font-bold tracking-[0.05em] py-2 px-4 rounded hover:bg-[#222a3d] hover:border-[#facc15] transition-all text-left flex items-center"
              >
                <span className="w-2 h-2 bg-[#EF4444] rounded-full mr-2"></span>
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
