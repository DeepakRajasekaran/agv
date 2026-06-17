import React, { useState, useCallback, useRef, useEffect } from 'react';
import { Group, Line, Circle, Text, Rect, Wedge } from 'react-konva';
import { applyDimensionUpdate } from '../utils/cadConstraints';
import { applyConstraint, validateConstraints, isFullyConstrained, propagateConstraints } from '../utils/cadSolver';
import { vec, isPointSafe } from '../utils/cadMath';
import { RULER } from './GridCanvas';

/** Unwrap propagateConstraints result — now returns {sketches, error} */
function runSolver(sketches, constraints, dimensions, fixedPoints, SCALE_M, referenceVertices) {
  const result = propagateConstraints(sketches, constraints, dimensions, fixedPoints, SCALE_M, referenceVertices);
  if (Array.isArray(result)) return { sketches: result, error: null };
  return result;
}

/**
 * Centralized vertex resolver for sketches.
 */
function resolveCadPoint(s, part) {
  if (!s) return { x: 0, y: 0 };
  if (s.type === 'line') {
    if (part === 'start') return { x: s.points[0], y: s.points[1] };
    if (part === 'end') return { x: s.points[2], y: s.points[3] };
    if (part === 'mid') return { x: (s.points[0] + s.points[2]) / 2, y: (s.points[1] + s.points[3]) / 2 };
  }
  if (s.type === 'circle') {
    if (part === 'center') return { x: s.center[0], y: s.center[1] };
    if (part === 'rad') return { x: s.center[0] + s.radius, y: s.center[1] };
    return { x: s.center[0], y: s.center[1] };
  }
  if (s.type === 'sector') {
    if (part === 'center') return { x: s.center[0], y: s.center[1] };
    if (part === 'start') return { x: s.center[0] + s.radius * Math.cos(s.startAngle), y: s.center[1] + s.radius * Math.sin(s.startAngle) };
    if (part === 'end') return { x: s.center[0] + s.radius * Math.cos(s.startAngle + s.sweepAngle), y: s.center[1] + s.radius * Math.sin(s.startAngle + s.sweepAngle) };
    if (part === 'mid_arc') return { x: s.center[0] + s.radius * Math.cos(s.startAngle + s.sweepAngle/2), y: s.center[1] + s.radius * Math.sin(s.startAngle + s.sweepAngle/2) };
    if (part === 'rad') return { x: s.center[0] + s.radius * Math.cos(s.startAngle), y: s.center[1] + s.radius * Math.sin(s.startAngle) }; // fallback for radius dim
    return { x: s.center[0], y: s.center[1] };
  }
  if (s.type === 'rect') {
    const [x1, y1] = s.start;
    const [x2, y2] = s.end;
    if (part === 'start' || part === 'p1') return { x: x1, y: y1 };
    if (part === 'p2') return { x: x2, y: y1 };
    if (part === 'p3' || part === 'end') return { x: x2, y: y2 };
    if (part === 'p4') return { x: x1, y: y2 };
    if (part === 'center') return { x: (x1 + x2) / 2, y: (y1 + y2) / 2 };
    if (part === 'mid_top') return { x: (x1 + x2) / 2, y: y1 };
    if (part === 'mid_bottom') return { x: (x1 + x2) / 2, y: y2 };
    if (part === 'mid_left') return { x: x1, y: (y1 + y2) / 2 };
    if (part === 'mid_right') return { x: x2, y: (y1 + y2) / 2 };
  }
  return { x: 0, y: 0 };
}

/**
/**
 * Arc-based Angle Annotation — draws an arc between two vectors with a degree label.
 */
function AngleAnnotation({ cx, cy, r1x, r1y, r2x, r2y, label, scale, onClick }) {
  const safeScale = scale && scale > 0 ? scale : 1;
  const a1 = Math.atan2(r1y - cy, r1x - cx);
  const a2 = Math.atan2(r2y - cy, r2x - cx);
  const arcR = 32 / safeScale;
  
  // Build arc path
  let sweep = a2 - a1;
  if (sweep < 0) sweep += 2 * Math.PI;
  if (sweep > Math.PI) { sweep = 2 * Math.PI - sweep; }
  
  const midAng = a1 + sweep / 2;
  const arcStartX = cx + arcR * Math.cos(a1);
  const arcStartY = cy + arcR * Math.sin(a1);
  const arcEndX = cx + arcR * Math.cos(a2);
  const arcEndY = cy + arcR * Math.sin(a2);
  const largeArc = sweep > Math.PI ? 1 : 0;
  
  const textX = cx + (arcR + 14 / safeScale) * Math.cos(midAng);
  const textY = cy + (arcR + 14 / safeScale) * Math.sin(midAng);
  
  const arrowSize = 5 / safeScale;
  const tangA2x = -Math.sin(a2);
  const tangA2y = Math.cos(a2);
  
  return (
    <Group>
      <Line
        points={[
          cx + (arcR - 5/safeScale) * Math.cos(a1), cy + (arcR - 5/safeScale) * Math.sin(a1),
          arcStartX, arcStartY
        ]}
        stroke="#666" strokeWidth={1/safeScale} listening={false}
      />
      <Line
        points={[
          cx + (arcR - 5/safeScale) * Math.cos(a2), cy + (arcR - 5/safeScale) * Math.sin(a2),
          arcEndX, arcEndY
        ]}
        stroke="#666" strokeWidth={1/safeScale} listening={false}
      />
      {/* Arc drawn as a Konva Arc */}
      <Line
        points={[arcStartX, arcStartY, arcEndX, arcEndY]}
        stroke="rgba(255,255,255,0)"
        strokeWidth={0}
        listening={false}
      />
      {/* Approximate arc with short line segments */}
      {Array.from({ length: 21 }, (_, k) => {
        const t0 = a1 + (k / 20) * sweep;
        const t1 = a1 + ((k + 1) / 20) * sweep;
        return (
          <Line
            key={k}
            points={[cx + arcR * Math.cos(t0), cy + arcR * Math.sin(t0), cx + arcR * Math.cos(t1), cy + arcR * Math.sin(t1)]}
            stroke="#aaa" strokeWidth={1.5 / safeScale} listening={false}
          />
        );
      })}
      {/* Arrow at arc end */}
      <Line
        points={[arcEndX, arcEndY, arcEndX + tangA2x * arrowSize - (arcEndX - cx) / arcR * arrowSize * 0.4, arcEndY + tangA2y * arrowSize - (arcEndY - cy) / arcR * arrowSize * 0.4,
                arcEndX + tangA2x * arrowSize + (arcEndX - cx) / arcR * arrowSize * 0.4, arcEndY + tangA2y * arrowSize + (arcEndY - cy) / arcR * arrowSize * 0.4]}
        fill="#aaa" closed stroke="#aaa" strokeWidth={0.5/safeScale} listening={false}
      />
      <Text
        x={textX} y={textY}
        text={label + '°'}
        fill="#00e5ff"
        fontSize={14 / safeScale}
        align="center"
        offsetX={(label.length * 3.5 + 8) / safeScale}
        offsetY={6 / safeScale}
        onClick={onClick}
        onTap={onClick}
        onMouseEnter={e => { e.target.getStage().container().style.cursor = 'pointer'; }}
        onMouseLeave={e => { e.target.getStage().container().style.cursor = 'default'; }}
      />
    </Group>
  );
}

/**
 * ISO Standard Dimension Annotation.
 */
function Annotation({ x1, y1, x2, y2, label, scale, tolerance = '', symbol = '', onClick }) {
  const safeScale = scale && scale > 0 ? scale : 1;
  const dx = x2 - x1;
  const dy = y2 - y1;
  const dist = Math.sqrt(dx * dx + dy * dy);
  if (!Number.isFinite(dist) || dist < 0.001) return null;

  const nx = -dy / dist;
  const ny = dx / dist;

  // Heuristic: Push dimension away from origin (0,0) to keep it outside robot-centric shapes
  const midX_raw = (x1 + x2) / 2;
  const midY_raw = (y1 + y2) / 2;
  const dotProd = midX_raw * nx + midY_raw * ny;
  const flip = dotProd < 0 ? -1 : 1;
  const fnx = nx * flip;
  const fny = ny * flip;

  const dimOffset = 22 / safeScale;
  const gap = 4 / safeScale;
  const ext = 8 / safeScale;

  const p1 = { x: x1 + fnx * gap, y: y1 + fny * gap };
  const p2 = { x: x2 + fnx * gap, y: y2 + fny * gap };
  const d1 = { x: x1 + fnx * dimOffset, y: y1 + fny * dimOffset };
  const d2 = { x: x2 + fnx * dimOffset, y: y2 + fny * dimOffset };
  const e1 = { x: x1 + fnx * (dimOffset + ext), y: y1 + fny * (dimOffset + ext) };
  const e2 = { x: x2 + fnx * (dimOffset + ext), y: y2 + fny * (dimOffset + ext) };

  const arrowSize = 6 / safeScale;
  const fullText = (symbol || '') + label + (tolerance ? ' ' + tolerance : '');
  const midX = (d1.x + d2.x) / 2;
  const midY = (d1.y + d2.y) / 2;
  const angle = Math.atan2(dy, dx) * (180 / Math.PI);

  return (
    <Group>
      <Line points={[p1.x, p1.y, e1.x, e1.y]} stroke="#666" strokeWidth={1 / safeScale} listening={false} />
      <Line points={[p2.x, p2.y, e2.x, e2.y]} stroke="#666" strokeWidth={1 / safeScale} listening={false} />
      <Line points={[d1.x, d1.y, d2.x, d2.y]} stroke="#aaa" strokeWidth={1.5 / safeScale} listening={false} />
      
      {/* Arrows */}
      <Line points={[d1.x, d1.y, d1.x + (dx/dist)*arrowSize + nx*arrowSize*0.3, d1.y + (dy/dist)*arrowSize + ny*arrowSize*0.3, d1.x + (dx/dist)*arrowSize - nx*arrowSize*0.3, d1.y + (dy/dist)*arrowSize - ny*arrowSize*0.3]} fill="#aaa" closed listening={false} />
      <Line points={[d2.x, d2.y, d2.x - (dx/dist)*arrowSize + nx*arrowSize*0.3, d2.y - (dy/dist)*arrowSize + ny*arrowSize*0.3, d2.x - (dx/dist)*arrowSize - nx*arrowSize*0.3, d2.y - (dy/dist)*arrowSize - ny*arrowSize*0.3]} fill="#aaa" closed listening={false} />

      <Text 
        x={midX + nx * 8 / safeScale} 
        y={midY + ny * 8 / safeScale} 
        text={fullText} 
        fill="white" 
        fontSize={15 / safeScale} 
        align="center"
        offsetX={(fullText.length * 3.5) / safeScale}
        rotation={angle > 90 || angle < -90 ? angle + 180 : angle}
        onClick={onClick}
        onTap={onClick}
        onMouseEnter={(e) => {
          const container = e.target.getStage().container();
          container.style.cursor = 'pointer';
          e.target.fill('#00e5ff');
        }}
        onMouseLeave={(e) => {
          const container = e.target.getStage().container();
          container.style.cursor = 'default';
          e.target.fill('white');
        }}
      />
    </Group>
  );
}

function DimOverlay({ x, y, value, onCommit, onCancel }) {
  const [val, setVal] = useState(value);
  const ref = useRef(null);

  useEffect(() => { ref.current?.focus(); }, []);

  const handleKey = (e) => {
    if (e.key === 'Enter') onCommit(val, '0');
    if (e.key === 'Escape') onCancel();
  };

  return (
    <div style={{
      position: 'absolute', left: x, top: y, transform: 'translate(-50%, -50%)',
      background: '#1a1a1a', border: '1px solid #00e5ff', padding: '10px', borderRadius: '4px',
      display: 'flex', gap: '8px', pointerEvents: 'auto', boxShadow: '0 4px 20px rgba(0,0,0,0.5)'
    }}>
      <input ref={ref} autoFocus type="text" value={val} 
        onChange={e => setVal(e.target.value)} 
        onBlur={e => { if (e.target.value === '') setVal('0'); }}
        onKeyDown={handleKey} 
        style={{ background: '#000', color: '#00e5ff', border: '1px solid #333', padding: '4px', width: '60px', outline: 'none' }} />
    </div>
  );
}

const CADSketcher = React.forwardRef(({ sketches, setSketches, dimensions, setDimensions, fixedPoints, setFixedPoints, constraints = [], setConstraints, setCadBatchSafe, referenceVertices = [], pushToHistory, scale, SCALE_M, activeTool, setOverlay, isConstructionMode, isSubtractionMode, isUnionMode }, ref) => {
  // 1. Foundation State & Refs (Must be first to avoid TDZ ReferenceErrors in dependency arrays)
  const [newShape, setNewShape] = useState(null);
  const [dimSelection, setDimSelection] = useState([]);
  const [constraintSelection, setConstraintSelection] = useState([]);
  const [selectedSketchIds, setSelectedSketchIds] = useState([]);
  const [selectedEdge, setSelectedEdge] = useState(null); // { id, edgeIdx }
  const [selectionBox, setSelectionBox] = useState(null);
  const stageGroupRef = useRef(null);
  const pendingMarquee = useRef(null); 
  const drawingState = useRef({ step: 0, isDragging: false, startPos: null, startSnap: null, lastClickTime: 0 });

  const sketchesRef      = useRef(sketches);
  const constraintsRef   = useRef(constraints);
  const dimensionsRef    = useRef(dimensions);
  const fixedPointsRef   = useRef(fixedPoints);

  useEffect(() => { sketchesRef.current    = sketches;    }, [sketches]);
  useEffect(() => { constraintsRef.current = constraints; }, [constraints]);
  useEffect(() => { dimensionsRef.current  = dimensions;  }, [dimensions]);
  useEffect(() => { fixedPointsRef.current = fixedPoints; }, [fixedPoints]);

  // Clear overlay when CADSketcher unmounts (e.g. exiting editor)
  useEffect(() => {
    return () => {
      if (setOverlay) setOverlay(null);
    };
  }, [setOverlay]);

  // Force Konva redraw when state changes
  useEffect(() => {
    if (stageGroupRef.current) {
      const layer = stageGroupRef.current.getLayer();
      if (layer) layer.batchDraw();
    }
  }, [sketches, constraints, dimensions, fixedPoints]);

  const deleteSelected = useCallback(() => {
    if (!selectedSketchIds.length) return;
    
    // Filter out shapes
    const updatedSketches = sketchesRef.current.filter(s => !selectedSketchIds.includes(s.id));
    
    // Filter out tied constraints
    const updatedConstraints = constraintsRef.current.filter(c => {
      const involvesSelected = (c.v1 && selectedSketchIds.includes(c.v1.sketchId)) || 
                               (c.v2 && selectedSketchIds.includes(c.v2.sketchId));
      return !involvesSelected;
    });

    // Filter out tied dims
    const updatedDimensions = dimensionsRef.current.filter(d => {
      const involvesSelected = (d.v1 && selectedSketchIds.includes(d.v1.sketchId)) || 
                               (d.v2 && selectedSketchIds.includes(d.v2.sketchId));
      return !involvesSelected;
    });

    // Filter out fixed points
    const updatedFixed = fixedPointsRef.current.filter(f => !selectedSketchIds.includes(f.sketchId));

    pushToHistory();
    const solveResult = runSolver(updatedSketches, updatedConstraints, updatedDimensions, updatedFixed, SCALE_M, referenceVertices);
    
    if (setCadBatchSafe) {
      setCadBatchSafe({
        sketches: solveResult.error ? updatedSketches : solveResult.sketches,
        constraints: updatedConstraints,
        dimensions: updatedDimensions,
        fixedPoints: updatedFixed
      });
    } else {
      setSketches(solveResult.error ? updatedSketches : solveResult.sketches);
      if (setConstraints) setConstraints(updatedConstraints);
      if (setDimensions) setDimensions(updatedDimensions);
      if (setFixedPoints) setFixedPoints(updatedFixed);
    }
    setSelectedSketchIds([]);
  }, [selectedSketchIds, pushToHistory, SCALE_M, referenceVertices, setCadBatchSafe, setSketches, setConstraints, setDimensions, setFixedPoints]);

  const toggleConstructionSelection = useCallback(() => {
    if (!selectedSketchIds.length) return;
    pushToHistory();

    let nextSketches = [...sketchesRef.current];
    let nextConstraints = [...constraintsRef.current];

    // If a specific edge of a shape is selected, we must AUTO-EXPLODE
    if (selectedEdge) {
      const target = nextSketches.find(s => s.id === selectedEdge.id);
      if (target && target.type === 'rect') {
        // Remove rect and add 4 lines
        nextSketches = nextSketches.filter(s => s.id !== target.id);
        const [x1, y1] = target.start;
        const [x2, y2] = target.end;
        const lineData = [
          { p: [x1, y1, x2, y1], field: 'top' },
          { p: [x2, y1, x2, y2], field: 'right' },
          { p: [x2, y2, x1, y2], field: 'bottom' },
          { p: [x1, y2, x1, y1], field: 'left' }
        ];

        const newLines = lineData.map((data, i) => {
          const id = `ExpLine_${Date.now()}_${i}`;
          return {
            id, type: 'line', points: data.p,
            construction: i === selectedEdge.edgeIdx ? !target.construction : target.construction,
            op: target.op
          };
        });

        nextSketches.push(...newLines);

        // Add constraints to hold the "rectangle" together
        for (let i = 0; i < 4; i++) {
          const l1 = newLines[i];
          const l2 = newLines[(i + 1) % 4];
          nextConstraints.push({ type: 'coincident', v1: { sketchId: l1.id, part: 'end' }, v2: { sketchId: l2.id, part: 'start' } });
          nextConstraints.push({ type: i % 2 === 0 ? 'horizontal' : 'vertical', v1: { sketchId: l1.id, part: 'start' }, v2: { sketchId: l1.id, part: 'end' } });
        }

        // Remap existing constraints and dimensions from target vertices to new line vertices
        const remapVertex = (v) => {
           if (v.sketchId !== target.id) return v;
           // p1: start of L0 / end of L3
           if (v.part === 'p1' || v.part === 'start') return { sketchId: newLines[0].id, part: 'start' };
           if (v.part === 'p2') return { sketchId: newLines[1].id, part: 'start' };
           if (v.part === 'p3' || v.part === 'end') return { sketchId: newLines[2].id, part: 'start' };
           if (v.part === 'p4') return { sketchId: newLines[3].id, part: 'start' };
           // Mids
           if (v.part === 'mid_top') return { sketchId: newLines[0].id, part: 'mid' };
           if (v.part === 'mid_right') return { sketchId: newLines[1].id, part: 'mid' };
           if (v.part === 'mid_bottom') return { sketchId: newLines[2].id, part: 'mid' };
           if (v.part === 'mid_left') return { sketchId: newLines[3].id, part: 'mid' };
           return v;
        };

        nextConstraints = nextConstraints.map(c => ({ ...c, v1: remapVertex(c.v1), v2: remapVertex(c.v2) }));
        const nextDimensions = dimensionsRef.current.map(d => ({ ...d, v1: remapVertex(d.v1), v2: remapVertex(d.v2) }));
        const nextFixed = fixedPointsRef.current.map(f => remapVertex(f));

        const solveResult = runSolver(nextSketches, nextConstraints, nextDimensions, nextFixed, SCALE_M, referenceVertices);
        setCadBatchSafe({ sketches: solveResult.sketches, constraints: nextConstraints, dimensions: nextDimensions, fixedPoints: nextFixed });
        setSelectedSketchIds([]);
        setSelectedEdge(null);
        return;
      }
    }

    // Default: Toggle selected shapes
    const updated = nextSketches.map(s => selectedSketchIds.includes(s.id) ? { ...s, construction: !s.construction } : s);
    setSketches(updated);
    setSelectedEdge(null);
  }, [selectedSketchIds, selectedEdge, pushToHistory, setSketches, setCadBatchSafe, SCALE_M, referenceVertices]);

  React.useImperativeHandle(ref, () => ({
    deleteSelection: deleteSelected,
    toggleConstruction: toggleConstructionSelection,
    hasSelection: selectedSketchIds.length > 0
  }));

  // Keyboard delete listener
  useEffect(() => {
    const handleKeyDown = (e) => {
      if (document.activeElement.tagName === 'INPUT' || document.activeElement.tagName === 'TEXTAREA') return;
      if (e.key === 'Delete' || e.key === 'Backspace') {
        deleteSelected();
      }
      if (e.key === 'Escape') {
        if (drawingState.current.step !== 0 || selectionBox) {
          drawingState.current = { step: 0, isDragging: false, startPos: null, lastClickTime: 0 };
          setNewShape(null);
          setSelectionBox(null);
        }
        setDimSelection([]);
        setConstraintSelection([]);
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [deleteSelected]);

  // 1. Internal Helpers (Moved to top of component body to prevent ReferenceErrors)
  const getVertices = useCallback(() => {
    const v = [{ x: 0, y: 0, sketchId: 'origin', part: 'origin', type: 'origin' }, ...referenceVertices];
    sketches.forEach(s => {
      if (s.type === 'line') {
        v.push({ ...resolveCadPoint(s, 'start'), sketchId: s.id, part: 'start' });
        v.push({ ...resolveCadPoint(s, 'end'), sketchId: s.id, part: 'end' });
        v.push({ ...resolveCadPoint(s, 'mid'), sketchId: s.id, part: 'mid' });
      } else if (s.type === 'circle') {
        const center = resolveCadPoint(s, 'center');
        v.push({ ...center, sketchId: s.id, part: 'center' });
        v.push({ ...resolveCadPoint(s, 'rad'), sketchId: s.id, part: 'rad' });
      } else if (s.type === 'sector') {
        v.push({ ...resolveCadPoint(s, 'center'), sketchId: s.id, part: 'center' });
        v.push({ ...resolveCadPoint(s, 'start'), sketchId: s.id, part: 'start' });
        v.push({ ...resolveCadPoint(s, 'end'), sketchId: s.id, part: 'end' });
        v.push({ ...resolveCadPoint(s, 'mid_arc'), sketchId: s.id, part: 'mid_arc' });
      } else if (s.type === 'rect') {
        ['p1', 'p2', 'p3', 'p4', 'mid_top', 'mid_bottom', 'mid_left', 'mid_right', 'center'].forEach(p => v.push({ ...resolveCadPoint(s, p), sketchId: s.id, part: p }));
      }
    });
    return v;
  }, [referenceVertices, sketches]);

  const snapPos = useCallback((rawX, rawY) => {
    const threshold = 18 / scale;
    let best = { x: rawX, y: rawY, snapped: false };
    let minDist = threshold;

    getVertices().forEach(v => {
      const d = vec.dist(v, { x: rawX, y: rawY });
      if (d < minDist) { minDist = d; best = { ...v, snapped: true }; }
    });

    if (best.snapped) return best;

    sketches.forEach(s => {
      if (s.type === 'line') {
        const p = vec.project({x: rawX, y: rawY}, {x: s.points[0], y: s.points[1]}, {x: s.points[2], y: s.points[3]});
        const d = vec.dist(p, {x: rawX, y: rawY});
        if (d < minDist) { minDist = d; best = { ...p, snapped: true, sketchId: s.id, type: 'edge' }; }
      } else if (s.type === 'circle') {
        const c = { x: s.center[0], y: s.center[1] };
        const d = vec.dist(c, { x: rawX, y: rawY });
        if (d > 0) {
          const ratio = s.radius / d;
          const p = { x: c.x + (rawX - c.x) * ratio, y: c.y + (rawY - c.y) * ratio };
          const distToEdge = Math.abs(d - s.radius);
          if (distToEdge < minDist) { minDist = distToEdge; best = { ...p, snapped: true, sketchId: s.id, type: 'edge' }; }
        }
      } else if (s.type === 'sector') {
        const c = { x: s.center[0], y: s.center[1] };
        const p1 = { x: c.x + s.radius * Math.cos(s.startAngle), y: c.y + s.radius * Math.sin(s.startAngle) };
        const p2 = { x: c.x + s.radius * Math.cos(s.startAngle + s.sweepAngle), y: c.y + s.radius * Math.sin(s.startAngle + s.sweepAngle) };
        
        const proj1 = vec.project({x: rawX, y: rawY}, c, p1);
        const d1 = vec.dist(proj1, {x: rawX, y: rawY});
        if (d1 < minDist) { 
          minDist = d1; 
          best = { ...proj1, snapped: true, sketchId: s.id, type: 'edge', line: { id: s.id, type: 'line', points: [c.x, c.y, p1.x, p1.y], part1: 'center', part2: 'start' } }; 
        }
        
        const proj2 = vec.project({x: rawX, y: rawY}, c, p2);
        const d2 = vec.dist(proj2, {x: rawX, y: rawY});
        if (d2 < minDist) { 
          minDist = d2; 
          best = { ...proj2, snapped: true, sketchId: s.id, type: 'edge', line: { id: s.id, type: 'line', points: [c.x, c.y, p2.x, p2.y], part1: 'center', part2: 'end' } }; 
        }
      }
    });
    return best;
  }, [getVertices, scale, sketches]);

  const commitDim = useCallback((v1, v2, valStr, tolStr, symbol = '') => {
    const val = parseFloat(valStr);
    if (!isNaN(val)) {
      const newDim = { id: Date.now(), v1, v2, value: val, tolerance: tolStr ? '±' + tolStr : '', symbol, label: val.toFixed(3) };
      const allDims = [...dimensionsRef.current, newDim];
      let prePositioned = applyDimensionUpdate(sketchesRef.current, v1, v2, val, SCALE_M, dimensionsRef.current, fixedPointsRef.current, referenceVertices);
      const solveResult = runSolver(prePositioned, constraintsRef.current, allDims, fixedPointsRef.current, SCALE_M, referenceVertices);
      if (solveResult.error) {
        alert('⚠️ Dimension Error:\n\n' + solveResult.error);
        setOverlay(null);
        return;
      }
      pushToHistory();
      setSketches(solveResult.sketches);
      setDimensions(allDims);
    }
    setOverlay(null);
  }, [SCALE_M, pushToHistory, referenceVertices, setDimensions, setOverlay, setSketches]);

  const editDim = useCallback((dimId, v1, v2, valStr, tolStr) => {
    const val = parseFloat(valStr);
    if (!isNaN(val)) {
      const updatedDimensions = dimensionsRef.current.map(d =>
        d.id === dimId ? { ...d, value: val, tolerance: tolStr ? '±' + tolStr : '', label: val.toFixed(3) } : d
      );
      let prePositioned = applyDimensionUpdate(sketchesRef.current, v1, v2, val, SCALE_M, dimensionsRef.current, fixedPointsRef.current, referenceVertices);
      const solveResult = runSolver(prePositioned, constraintsRef.current, updatedDimensions, fixedPointsRef.current, SCALE_M, referenceVertices);
      if (solveResult.error) {
        alert('⚠️ Dimension Error:\n\n' + solveResult.error);
        setOverlay(null);
        return;
      }
      pushToHistory();
      setSketches(solveResult.sketches);
      setDimensions(updatedDimensions);
    }
    setOverlay(null);
  }, [SCALE_M, pushToHistory, referenceVertices, setDimensions, setOverlay, setSketches]);

  const finalizeShape = useCallback((finalPos) => {
    if (!newShape) return;
    
    let finalized = { ...newShape };
    if (finalPos) {
       if (finalized.type === 'line') finalized.points = [finalized.points[0], finalized.points[1], finalPos.x, finalPos.y];
       if (finalized.type === 'circle') finalized.radius = vec.dist({x: finalized.center[0], y: finalized.center[1]}, finalPos);
       if (finalized.type === 'rect') finalized.end = [finalPos.x, finalized.y];
    }

    if (finalized.type === 'line') {
      const d = vec.dist({x: finalized.points[0], y: finalized.points[1]}, {x: finalized.points[2], y: finalized.points[3]});
      if (d < 5 / scale) {
         setNewShape(null);
         drawingState.current = { step: 0, isDragging: false, startPos: null };
         return;
      }
    }

    drawingState.current = { step: 0, isDragging: false, startPos: null };
    pushToHistory();

    const typeCount = sketchesRef.current.filter(s => s.type === finalized.type).length + 1;
    const typeLabels = { line: 'Line', circle: 'Circle', rect: 'Rect', sector: 'Sector' };
    const humanId = `${typeLabels[finalized.type] || 'Shape'} ${typeCount}`;

    let newConstraints = [...constraintsRef.current];
    if (finalized.type !== 'sector' && drawingState.current.startSnap) {
       // For non-sectors, snap the start/p1/center to what the user clicked on
       newConstraints.push({ type: 'coincident', v1: { sketchId: humanId, part: finalized.type === 'rect' ? 'p1' : (finalized.type === 'circle' ? 'center' : 'start') }, v2: drawingState.current.startSnap });
    } else if (finalized.type === 'sector' && drawingState.current.startSnap) {
       // For sectors, the first click was the CENTER — snap center, not start
       newConstraints.push({ type: 'coincident', v1: { sketchId: humanId, part: 'center' }, v2: drawingState.current.startSnap });
    }
    const snapMeta = finalPos && finalPos.snapped && finalPos.sketchId && finalized.type !== 'sector'
      ? { sketchId: finalPos.sketchId, part: finalPos.part, type: finalPos.type } : null;
    if (snapMeta) {
       // Sector finalPos is just the sweep-end click — not meaningful for snapping
       newConstraints.push({ type: 'coincident', v1: { sketchId: humanId, part: finalized.type === 'rect' ? 'p3' : (finalized.type === 'circle' ? 'rad' : 'end') }, v2: snapMeta });
    }

    const solveResult = runSolver([...sketchesRef.current, { ...finalized, id: humanId, name: humanId }], newConstraints, dimensionsRef.current, fixedPointsRef.current, SCALE_M, referenceVertices);

    drawingState.current = { step: 0, isDragging: false, startPos: null, startSnap: null, lastClickTime: drawingState.current.lastClickTime };

    if (setCadBatchSafe) {
      setCadBatchSafe({
        sketches: solveResult.sketches,
        constraints: newConstraints
      });
    } else {
      setSketches(solveResult.sketches);
      if (setConstraints) setConstraints(newConstraints);
    }
    setNewShape(null);
  }, [newShape, pushToHistory, setSketches, scale, SCALE_M, referenceVertices, setConstraints, setCadBatchSafe]);


  // 2. Lifecycle & Interaction Hooks
  useEffect(() => {
    if (drawingState.current.step !== 0 || selectionBox) {
      drawingState.current = { step: 0, isDragging: false, startPos: null, lastClickTime: 0 };
      setNewShape(null);
      setSelectionBox(null);
    }
    setDimSelection([]);
    setConstraintSelection([]);
    if (activeTool !== 'select') setSelectedSketchIds([]);
  }, [activeTool]);

  useEffect(() => {
    const isConstraint = ['coincide', 'coincident', 'equal', 'vertical', 'horizontal', 'parallel', 'perpendicular', 'angle'].includes(activeTool);
    if (!isConstraint || constraintSelection.length === 0) return;

    const currentSketches    = sketchesRef.current;
    const currentConstraints = constraintsRef.current;
    const currentDimensions  = dimensionsRef.current;
    const currentFixed       = fixedPointsRef.current;

    const isSingleTarget = ['vertical', 'horizontal'].includes(activeTool);

    if (isSingleTarget && constraintSelection.length >= 1) {
      let updated = [...currentSketches];
      let updatedConstraints = [...currentConstraints];
      let anyError = null;

      constraintSelection.forEach(v => {
        const { sketches: res, newConstraint, error } = applyConstraint(
          updated, activeTool, v, null,
          currentFixed, currentDimensions, SCALE_M, referenceVertices, updatedConstraints
        );
        if (error) { anyError = error; return; }
        updated = res;
        if (newConstraint) updatedConstraints.push(newConstraint);
      });

      if (anyError) {
        alert('⚠️ Constraint Error:\n\n' + anyError);
      } else {
        pushToHistory();
        const solveResult = runSolver(updated, updatedConstraints, currentDimensions, currentFixed, SCALE_M, referenceVertices);
        if (solveResult.error) {
          alert('⚠️ Constraint Error:\n\n' + solveResult.error);
        } else {
          setSketches(solveResult.sketches);
          if (setConstraints && updatedConstraints.length > currentConstraints.length) setConstraints(updatedConstraints);
        }
      }
      setConstraintSelection([]);

    } else if (!isSingleTarget && constraintSelection.length >= 2) {
      const v1 = constraintSelection[0];
      const v2 = constraintSelection[1];
      
      if (activeTool === 'angle') {
        const s1 = currentSketches.find(sk => sk.id === v1.sketchId);
        const s2 = currentSketches.find(sk => sk.id === v2.sketchId);
        if (!s1 || !s2 || s1.type !== 'line' || s2.type !== 'line') {
           alert('Angle constraint requires selecting two lines.');
           setConstraintSelection([]);
           return;
        }

        const sharedCoincide = currentConstraints.some(c => 
          (c.type === 'coincide' || c.type === 'coincident') &&
          ((c.v1.sketchId === s1.id && c.v2.sketchId === s2.id) ||
           (c.v1.sketchId === s2.id && c.v2.sketchId === s1.id))
        );

        const points1 = [{x: s1.points[0], y: s1.points[1]}, {x: s1.points[2], y: s1.points[3]}];
        const points2 = [{x: s2.points[0], y: s2.points[1]}, {x: s2.points[2], y: s2.points[3]}];
        const overlaps = points1.some(p1 => points2.some(p2 => Math.abs(p1.x - p2.x) < 1e-4 && Math.abs(p1.y - p2.y) < 1e-4));

        if (!sharedCoincide && !overlaps) {
           alert('Angle constraint can only be applied to connected lines. Please add a coincide constraint first.');
           setConstraintSelection([]);
           return;
        }

        const stage = stageGroupRef.current?.getStage();
        if (stage) {
           const pAbs = stage.getAbsoluteTransform().point({x: (v1.x+v2.x)/2, y: (v1.y+v2.y)/2});
           const rect = stage.container().getBoundingClientRect();
           setConstraintSelection([]);
           setOverlay({
             component: DimOverlay,
             props: {
               x: rect.left + pAbs.x, y: rect.top + pAbs.y,
               value: '90', tolerance: '',
               onCommit: (vStr, tStr) => {
                  setOverlay(null);
                  const angleVal = parseFloat(vStr);
                  if (isNaN(angleVal)) return;
                  const newConstraint = { type: 'angle', v1, v2, value: angleVal, id: Date.now() };
                  pushToHistory();
                  const allConstraints = [...currentConstraints, newConstraint];
                  const solveResult = runSolver(currentSketches, allConstraints, currentDimensions, currentFixed, SCALE_M, referenceVertices);
                  if (solveResult.error) {
                    alert('⚠️ Constraint Error:\n\n' + solveResult.error);
                  } else {
                    setSketches(solveResult.sketches);
                    if (setConstraints) setConstraints(allConstraints);
                    // Also store as a visible dimension annotation
                    const angleDim = { id: Date.now() + 1, v1, v2, value: angleVal, label: angleVal.toFixed(1), symbol: '°', isAngle: true };
                    if (setDimensions) setDimensions(prev => [...prev, angleDim]);
                  }
               },
               onCancel: () => setOverlay(null)
             }
           });
        }
        return;
      }

      const { sketches: res, newConstraint, error } = applyConstraint(
        currentSketches, activeTool, v1, v2,
        currentFixed, currentDimensions, SCALE_M, referenceVertices, currentConstraints
      );

      if (error) {
        alert('⚠️ Constraint Error:\n\n' + error);
      } else if (newConstraint) {
        pushToHistory();
        const allConstraints = [...currentConstraints, newConstraint];
        const solveResult = runSolver(res, allConstraints, currentDimensions, currentFixed, SCALE_M, referenceVertices);
        if (solveResult.error) {
          alert('⚠️ Constraint Error:\n\n' + solveResult.error);
        } else {
          setSketches(solveResult.sketches);
          if (setConstraints) setConstraints(allConstraints);
        }
      }
      setConstraintSelection([]);
    }
  // eslint-disable-next-line
  }, [activeTool, constraintSelection]);

  const handleMouseDown = useCallback((e) => {
    if (!activeTool) return;
    const now = Date.now();
    const isDoubleClick = (now - drawingState.current.lastClickTime < 350);
    drawingState.current.lastClickTime = now;
    
    const stage = e.target.getStage();
    if (!stage) return;
    const pos = stage.getRelativePointerPosition();
    const snapped = snapPos(pos.x, pos.y);
    const snapMeta = snapped.snapped && snapped.sketchId ? { sketchId: snapped.sketchId, part: snapped.part, type: snapped.type } : null;

    if (activeTool === 'select') {
      if (snapped.snapped && snapped.sketchId && !snapped.sketchId.startsWith('ref-') && snapped.sketchId !== 'origin') {
        // Clicked directly on a sketch vertex
        pendingMarquee.current = null;
        setSelectedSketchIds(prev => {
          if (e.evt.shiftKey) {
            return prev.includes(snapped.sketchId) ? prev.filter(id => id !== snapped.sketchId) : [...prev, snapped.sketchId];
          }
          return [snapped.sketchId];
        });
      } else {
        // Clicked on empty space — arm a potential marquee drag
        if (isDoubleClick) {
          setSelectionBox({ startX: pos.x, startY: pos.y, endX: pos.x, endY: pos.y });
          pendingMarquee.current = null;
        } else {
          // Arm a potential marquee drag: it will become visible only once the user drags a few pixels
          pendingMarquee.current = { x: pos.x, y: pos.y, shiftKey: e.evt.shiftKey };
        }
        if (!e.evt.shiftKey) setSelectedSketchIds([]);
        e.cancelBubble = true;
      }
      return;
    }

    if (activeTool === 'dimension') {
      if (dimSelection.length === 0) {
        if (snapped.snapped) {
          if (snapped.type === 'edge') {
            const s = sketches.find(sk => sk.id === snapped.sketchId);
            if (!s) return;
            if (s.type === 'circle') {
              const v1 = { x: s.center[0], y: s.center[1], sketchId: s.id, part: 'center' };
              const v2 = { x: s.center[0] + s.radius, y: s.center[1], sketchId: s.id, part: 'rad' };
              const pAbs = stage.getAbsoluteTransform().point(v2);
              const rect = stage.container().getBoundingClientRect();
              setOverlay({ 
                component: DimOverlay, 
                props: { 
                  x: rect.left + pAbs.x, y: rect.top + pAbs.y, 
                  value: (s.radius/SCALE_M).toFixed(3),
                  onCommit: (vStr) => commitDim(v1, v2, vStr, '', '⌀ ')
                } 
              });
            } else if (s.type === 'line') {
              setDimSelection([{ sketchId: s.id, type: 'edge', line: s }]);
            } else if (s.type === 'sector' && snapped.line) {
              setDimSelection([{ sketchId: s.id, type: 'edge', line: snapped.line }]);
            }
          } else {
            setDimSelection([snapped]);
          }
        }
      } else if (dimSelection.length === 1) {
        const first = dimSelection[0];
        
        if (first.type === 'edge') {
          if (snapped.type === 'edge') {
            const s2 = sketches.find(sk => sk.id === snapped.sketchId);
            if (!s2) return;
            const s2Line = snapped.line || s2;
            if (s2Line.type !== 'line') return;
            
            if (s2.id === first.sketchId && (!snapped.line || snapped.line.part2 === first.line.part2)) return;
            
            const s1 = first.line;

            // Validation: ensure they share a point
            const sharedCoincide = constraintsRef.current.some(c => 
              (c.type === 'coincide' || c.type === 'coincident') &&
              ((c.v1.sketchId === s1.id && c.v2.sketchId === s2.id) || 
               (c.v1.sketchId === s2.id && c.v2.sketchId === s1.id))
            );
            const sharePoint = 
              (Math.abs(s1.points[0] - s2Line.points[0]) < 0.001 && Math.abs(s1.points[1] - s2Line.points[1]) < 0.001) ||
              (Math.abs(s1.points[0] - s2Line.points[2]) < 0.001 && Math.abs(s1.points[1] - s2Line.points[3]) < 0.001) ||
              (Math.abs(s1.points[2] - s2Line.points[0]) < 0.001 && Math.abs(s1.points[3] - s2Line.points[1]) < 0.001) ||
              (Math.abs(s1.points[2] - s2Line.points[2]) < 0.001 && Math.abs(s1.points[3] - s2Line.points[3]) < 0.001);

            if (!sharedCoincide && !sharePoint) {
              alert('⚠️ To measure an angle, the two lines must be connected (share a coincident point).');
              setDimSelection([]);
              return;
            }

            const dx1 = s1.points[2] - s1.points[0], dy1 = s1.points[3] - s1.points[1];
            const dx2 = s2Line.points[2] - s2Line.points[0], dy2 = s2Line.points[3] - s2Line.points[1];
            let angleRad = Math.acos((dx1*dx2 + dy1*dy2) / (Math.sqrt(dx1*dx1+dy1*dy1) * Math.sqrt(dx2*dx2+dy2*dy2)));
            let currentAngle = (angleRad * 180 / Math.PI).toFixed(1);
            
            const v1 = { sketchId: s1.id, part: s1.part2 || 'start', x: s1.points[0], y: s1.points[1] };
            const v2 = { sketchId: s2Line.id, part: s2Line.part2 || 'start', x: s2Line.points[0], y: s2Line.points[1] };
            
            const pAbs = stage.getAbsoluteTransform().point({x: pos.x, y: pos.y});
            const rect = stage.container().getBoundingClientRect();
            setOverlay({
              component: DimOverlay,
              props: {
                x: rect.left + pAbs.x, y: rect.top + pAbs.y,
                value: currentAngle,
                onCommit: (vStr) => {
                  setOverlay(null);
                  const angleVal = parseFloat(vStr);
                  if (isNaN(angleVal)) return;
                  const newConstraint = { type: 'angle', v1, v2, value: angleVal, id: Date.now() };
                  pushToHistory();
                  const allConstraints = [...constraintsRef.current, newConstraint];
                  const solveResult = runSolver(sketchesRef.current, allConstraints, dimensionsRef.current, fixedPointsRef.current, SCALE_M, referenceVertices);
                  if (solveResult.error) {
                    alert('⚠️ Constraint Error:\n\n' + solveResult.error);
                  } else {
                    setSketches(solveResult.sketches);
                    if (setConstraints) setConstraints(allConstraints);
                    // Also store as a visible dimension annotation
                    const angleDim = { id: Date.now() + 1, v1, v2, value: angleVal, label: angleVal.toFixed(1), symbol: '°', isAngle: true };
                    if (setDimensions) setDimensions(prev => [...prev, angleDim]);
                  }
                },
                onCancel: () => setOverlay(null)
              }
            });
            setDimSelection([]);
          } else if (!snapped.snapped) {
            const s = first.line;
            const v1 = { x: s.points[0], y: s.points[1], sketchId: s.id, part: s.part1 || 'start' };
            const v2 = { x: s.points[2], y: s.points[3], sketchId: s.id, part: s.part2 || 'end' };
            const pAbs = stage.getAbsoluteTransform().point({x: pos.x, y: pos.y});
            const rect = stage.container().getBoundingClientRect();
            setOverlay({ 
              component: DimOverlay, 
              props: { 
                x: rect.left + pAbs.x, y: rect.top + pAbs.y, 
                value: (vec.dist(v1, v2)/SCALE_M).toFixed(3),
                onCommit: (vStr) => commitDim(v1, v2, vStr, '')
              } 
            });
            setDimSelection([]);
          } else {
            setDimSelection([snapped]);
          }
        } else {
          if (snapped.snapped && !snapped.type && (first.sketchId !== snapped.sketchId || first.part !== snapped.part)) {
            const pAbs = stage.getAbsoluteTransform().point({x: pos.x, y: pos.y});
            const rect = stage.container().getBoundingClientRect();
            setOverlay({ 
              component: DimOverlay, 
              props: { 
                x: rect.left + pAbs.x, y: rect.top + pAbs.y, 
                value: (vec.dist(first, snapped)/SCALE_M).toFixed(3),
                onCommit: (vStr) => commitDim(first, snapped, vStr, '')
              } 
            });
            setDimSelection([]);
          }
        }
      }
      return;
    }

    // ── Anchor / Fixed-point tool ──
    if (activeTool === 'anchor') {
      if (!snapped.snapped || !snapped.sketchId) return;
      const existingIdx = fixedPoints.findIndex(f => f.sketchId === snapped.sketchId && f.part === snapped.part);
      if (existingIdx !== -1) {
        const updatedFixed = fixedPoints.filter((_, i) => i !== existingIdx);
        if (setFixedPoints) setFixedPoints(updatedFixed);
      } else {
        const newFixed = { sketchId: snapped.sketchId, part: snapped.part, x: snapped.x, y: snapped.y };
        const updatedFixed = [...fixedPoints, newFixed];
        if (setFixedPoints) setFixedPoints(updatedFixed);
        const solveResult = runSolver(sketches, constraints, dimensions, updatedFixed, SCALE_M, referenceVertices);
        if (!solveResult.error) setSketches(solveResult.sketches);
      }
      return;
    }

    const isConstraintTool = ['coincide', 'coincident', 'equal', 'vertical', 'horizontal', 'parallel', 'perpendicular'].includes(activeTool);
    if (isConstraintTool) {
      if (!snapped.snapped) return;
      
      let targetSnap = snapped;
      if (snapped.type === 'edge') {
         if (['parallel', 'perpendicular', 'equal'].includes(activeTool)) {
            const edgePart = snapped.line && snapped.line.part2 ? snapped.line.part2 : 'start';
            targetSnap = { sketchId: snapped.sketchId, part: edgePart, x: snapped.x, y: snapped.y, snapped: true, type: 'edge', line: snapped.line };
         } else {
            return;
         }
      }

      setConstraintSelection(prev => {
        if (prev.length === 1 && prev[0].sketchId === targetSnap.sketchId) return prev;
        return [...prev, targetSnap];
      });
      return;
    }

    if (activeTool === 'line' || activeTool === 'circle' || activeTool === 'rect' || activeTool === 'sector') {
      const shapeProps = { construction: isConstructionMode, op: isSubtractionMode ? 'subtract' : (isUnionMode ? 'union' : 'add') };
      
      if (drawingState.current.step === 0) {
        drawingState.current = { step: 1, isDragging: false, startPos: { x: pos.x, y: pos.y }, startSnap: snapMeta, lastClickTime: drawingState.current.lastClickTime };
        if (activeTool === 'line') setNewShape({ type: 'line', points: [snapped.x, snapped.y, snapped.x, snapped.y], ...shapeProps });
        else if (activeTool === 'circle') setNewShape({ type: 'circle', center: [snapped.x, snapped.y], radius: 0, ...shapeProps });
        else if (activeTool === 'rect') setNewShape({ type: 'rect', start: [snapped.x, snapped.y], end: [snapped.x, snapped.y], ...shapeProps });
        else if (activeTool === 'sector') setNewShape({ type: 'sector', center: [snapped.x, snapped.y], radius: 0, startAngle: 0, sweepAngle: 0, ...shapeProps });
      } else if (activeTool === 'sector' && drawingState.current.step === 1) {
        // Sector step 1 → 2: lock startAngle from current mouse position
        const dx = pos.x - newShape.center[0];
        const dy = pos.y - newShape.center[1];
        const startAngle = Math.atan2(dy, dx);
        setNewShape(prev => ({ ...prev, startAngle, sweepAngle: 0 }));
        drawingState.current.step = 2;
      } else if (activeTool === 'sector' && drawingState.current.step === 2 && newShape) {
        // Sector step 2 → finalize on second click
        finalizeShape(snapped);
      } else if (activeTool === 'line' && newShape) {
        // Chain line: Continue to next segment
        const typeCount = sketchesRef.current.filter(s => s.type === 'line').length + 1;
        const humanId = `Line ${typeCount}`;
        const finalLine = { ...newShape, id: humanId, name: humanId, points: [newShape.points[0], newShape.points[1], snapped.x, snapped.y] };
        
        let newConstraints = [...constraintsRef.current];
        if (drawingState.current.startSnap) {
           newConstraints.push({ type: 'coincident', v1: { sketchId: humanId, part: 'start' }, v2: drawingState.current.startSnap });
        }
        if (snapMeta) {
           newConstraints.push({ type: 'coincident', v1: { sketchId: humanId, part: 'end' }, v2: snapMeta });
        }

        const solveResult = runSolver([...sketchesRef.current, finalLine], newConstraints, dimensionsRef.current, fixedPointsRef.current, SCALE_M, referenceVertices);
        
        pushToHistory();
        if (setCadBatchSafe) {
          setCadBatchSafe({
            sketches: solveResult.sketches,
            constraints: newConstraints
          });
        } else {
          setSketches(solveResult.sketches);
          if (setConstraints) setConstraints(newConstraints);
        }
        
        drawingState.current.startPos = { x: snapped.x, y: snapped.y };
        drawingState.current.startSnap = { sketchId: humanId, part: 'end' };
        setNewShape({ type: 'line', points: [snapped.x, snapped.y, snapped.x, snapped.y], ...shapeProps });
      } else if (activeTool !== 'line' && newShape) {
        // Circle / Rect: Second click finishes it
        finalizeShape(snapped);
      }
    }
  }, [activeTool, snapPos, sketches, dimSelection, constraintSelection, SCALE_M, setOverlay, fixedPoints, dimensions, setSketches, pushToHistory, isConstructionMode, isSubtractionMode, isUnionMode, newShape, finalizeShape]);

  const handleMouseMove = useCallback((e) => {
    if (activeTool === 'select') {
      const stage = e.target.getStage();
      if (!stage) return;
      const pos = stage.getRelativePointerPosition();

      // Promote pending marquee to active once mouse drags beyond threshold
      if (pendingMarquee.current && e.evt.buttons === 1) {
        const dx = Math.abs(pos.x - pendingMarquee.current.x);
        const dy = Math.abs(pos.y - pendingMarquee.current.y);
        if (dx > 4 || dy > 4) {
          setSelectionBox({ startX: pendingMarquee.current.x, startY: pendingMarquee.current.y, endX: pos.x, endY: pos.y });
          pendingMarquee.current = null;
        }
        return;
      }

      if (selectionBox) {
        setSelectionBox(prev => ({ ...prev, endX: pos.x, endY: pos.y }));
        return;
      }
      return;
    }

    if (drawingState.current.step === 0 || !newShape) return;
    const stage = e.target.getStage();
    if (!stage) return;
    const pos = stage.getRelativePointerPosition();
    
    // Detect drag
    if (drawingState.current.startPos) {
       const dist = vec.dist(drawingState.current.startPos, pos);
       if (e.evt.buttons === 1 && dist > 5) {
          drawingState.current.isDragging = true;
       }
    }

    const snapped = snapPos(pos.x, pos.y);
    if (newShape.type === 'line') setNewShape({ ...newShape, points: [newShape.points[0], newShape.points[1], snapped.x, snapped.y] });
    else if (newShape.type === 'circle') setNewShape({ ...newShape, radius: vec.dist({x: newShape.center[0], y: newShape.center[1]}, snapped) });
    else if (newShape.type === 'rect') setNewShape({ ...newShape, end: [snapped.x, snapped.y] });
    else if (newShape.type === 'sector') {
       const dx = pos.x - newShape.center[0];
       const dy = pos.y - newShape.center[1];
       let ang = Math.atan2(dy, dx);
       
       if (drawingState.current.step === 1) {
           // Step 1: only update radius — no startAngle locking yet
           const r = vec.dist({x: newShape.center[0], y: newShape.center[1]}, pos);
           setNewShape({ ...newShape, radius: r });
       } else if (drawingState.current.step === 2) {
           // Step 2: startAngle was locked on click; now drag the sweep
           let rawSweep = ang - newShape.startAngle;
           if (rawSweep < 0) rawSweep += 2 * Math.PI;
           setNewShape({ ...newShape, sweepAngle: rawSweep });
       }
    }
  }, [newShape, snapPos, selectionBox, activeTool]);

  const handleMouseUp = useCallback((e) => {
    // Discard the pending marquee if user didn't drag far enough (it was just a click)
    pendingMarquee.current = null;

    // Finish active marquee selection drag
    if (activeTool === 'select' && selectionBox) {
      const { startX, startY, endX, endY } = selectionBox;
      if (Math.abs(startX - endX) > 2 && Math.abs(startY - endY) > 2) {
        const minX = Math.min(startX, endX);
        const maxX = Math.max(startX, endX);
        const minY = Math.min(startY, endY);
        const maxY = Math.max(startY, endY);
        
        const inBox = (px, py) => px >= minX && px <= maxX && py >= minY && py <= maxY;
        
        const newlySelected = sketchesRef.current.filter(s => {
          if (s.type === 'line') {
            return inBox(s.points[0], s.points[1]) || inBox(s.points[2], s.points[3]);
          } else if (s.type === 'circle') {
            return inBox(s.center[0], s.center[1]);
          } else if (s.type === 'rect') {
            return inBox(s.start[0], s.start[1]) || inBox(s.end[0], s.end[1]);
          }
          return false;
        }).map(s => s.id);

        if (newlySelected.length) {
          setSelectedSketchIds(prev => Array.from(new Set([...prev, ...newlySelected])));
        }
      }
      setSelectionBox(null);
      return;
    }

    // Only auto-finalize for Circle/Rect via drag. 
    // Line (Polyline) requires explicit clicks or dblclick to finalize.
    if (drawingState.current.step === 1 && drawingState.current.isDragging && newShape && activeTool !== 'line' && activeTool !== 'sector') {
      finalizeShape();
    }
  }, [selectionBox, newShape, finalizeShape, activeTool]);

  const handleDblClick = useCallback((e) => {
     if (drawingState.current.step === 1 && activeTool === 'line') {
        finalizeShape();
     }
  }, [activeTool, finalizeShape]);

  return (
    <Group ref={stageGroupRef} onMouseDown={handleMouseDown} onMouseMove={handleMouseMove} onMouseUp={handleMouseUp} onDblClick={handleDblClick}>
      <Circle radius={10000} fill="rgba(0,0,0,0)" />
      {sketches.map(s => {
        const isSelected = selectedSketchIds.includes(s.id);
        const baseColor = isFullyConstrained(s, dimensions, fixedPoints) ? '#aaaaaa' : 'white';
        const opColor = s.op === 'subtract' ? '#FF5252' : baseColor;
        const color = isSelected ? '#00e5ff' : (s.construction ? '#666' : opColor);
        const strokeWidth = (isSelected ? 3 : 2) / scale;
        const dash = s.construction ? [5/scale, 5/scale] : null;
        return (
        <Group key={s.id}>
          {s.type === 'line' && <Line points={s.points} stroke={color} strokeWidth={strokeWidth} dash={dash} shadowBlur={isSelected ? 5 : 0} shadowColor="#00e5ff" />}
          {s.type === 'circle' && <Circle x={s.center[0]} y={s.center[1]} radius={s.radius} stroke={color} strokeWidth={strokeWidth} dash={dash} shadowBlur={isSelected ? 5 : 0} shadowColor="#00e5ff" />}
          {s.type === 'rect' && (() => {
            const [x1, y1] = s.start;
            const [x2, y2] = s.end;
            const edges = [
              [x1, y1, x2, y1], [x2, y1, x2, y2], [x2, y2, x1, y2], [x1, y2, x1, y1]
            ];
            return edges.map((pts, i) => {
              const edgeSelected = isSelected && selectedEdge?.id === s.id && selectedEdge?.edgeIdx === i;
              const edgeColor = edgeSelected ? '#00e5ff' : color;
              const edgeWidth = (edgeSelected ? 4 : (isSelected ? 3 : 2)) / scale;
              return <Line key={`${s.id}-e${i}`} points={pts} stroke={edgeColor} strokeWidth={edgeWidth} dash={dash} />;
            });
          })()}
          {s.type === 'sector' && <Wedge x={s.center[0]} y={s.center[1]} radius={s.radius} angle={s.sweepAngle * 180 / Math.PI} rotation={s.startAngle * 180 / Math.PI} stroke={color} strokeWidth={strokeWidth} dash={dash} shadowBlur={isSelected ? 5 : 0} shadowColor="#00e5ff" />}
        </Group>
      )})}
      {newShape && (
        <Group>
          {newShape.type === 'line' && <Line points={newShape.points} stroke="yellow" strokeWidth={2 / scale} dash={[5/scale, 5/scale]} />}
          {newShape.type === 'circle' && <Circle x={newShape.center[0]} y={newShape.center[1]} radius={newShape.radius} stroke="yellow" strokeWidth={2 / scale} dash={[5/scale, 5/scale]} />}
          {newShape.type === 'rect' && <Line points={[newShape.start[0], newShape.start[1], newShape.end[0], newShape.start[1], newShape.end[0], newShape.end[1], newShape.start[0], newShape.end[1], newShape.start[0], newShape.start[1]]} stroke="yellow" strokeWidth={2 / scale} closed dash={[5/scale, 5/scale]} />}
          {newShape.type === 'sector' && <Wedge x={newShape.center[0]} y={newShape.center[1]} radius={newShape.radius} angle={newShape.sweepAngle * 180 / Math.PI} rotation={newShape.startAngle * 180 / Math.PI} stroke="yellow" strokeWidth={2 / scale} dash={[5/scale, 5/scale]} />}
        </Group>
      )}
      {dimensions.map(d => {
        if (d.isAngle) {
          // Render as an arc annotation
          const s1 = sketches.find(sk => sk.id === d.v1.sketchId);
          const s2 = sketches.find(sk => sk.id === d.v2.sketchId);
          if (!s1 || !s2) return null;
          // Find shared vertex (center of arc)
          const p1s = resolveCadPoint(s1, 'start'), p1e = resolveCadPoint(s1, 'end');
          const p2s = resolveCadPoint(s2, 'start'), p2e = resolveCadPoint(s2, 'end');
          const isCenter = d.v1.part === 'center' || d.v2.part === 'center';
          let cx, cy, r1x, r1y, r2x, r2y;
          if (isCenter) {
            // Sector sweep angle — center is the sector center
            const sc = s1.id === d.v1.sketchId ? s1 : s2;
            cx = sc.center[0]; cy = sc.center[1];
            r1x = sc.center[0] + sc.radius * Math.cos(sc.startAngle);
            r1y = sc.center[1] + sc.radius * Math.sin(sc.startAngle);
            r2x = sc.center[0] + sc.radius * Math.cos(sc.startAngle + sc.sweepAngle);
            r2y = sc.center[1] + sc.radius * Math.sin(sc.startAngle + sc.sweepAngle);
          } else {
            // Lines sharing a vertex
            const distThresh = 2;
            const isS1S2 = Math.hypot(p1s.x - p2s.x, p1s.y - p2s.y) < distThresh;
            const isS1E2 = Math.hypot(p1s.x - p2e.x, p1s.y - p2e.y) < distThresh;
            const isE1S2 = Math.hypot(p1e.x - p2s.x, p1e.y - p2s.y) < distThresh;
            const isE1E2 = Math.hypot(p1e.x - p2e.x, p1e.y - p2e.y) < distThresh;
            if (isS1S2) { cx = p1s.x; cy = p1s.y; r1x = p1e.x; r1y = p1e.y; r2x = p2e.x; r2y = p2e.y; }
            else if (isS1E2) { cx = p1s.x; cy = p1s.y; r1x = p1e.x; r1y = p1e.y; r2x = p2s.x; r2y = p2s.y; }
            else if (isE1S2) { cx = p1e.x; cy = p1e.y; r1x = p1s.x; r1y = p1s.y; r2x = p2e.x; r2y = p2e.y; }
            else if (isE1E2) { cx = p1e.x; cy = p1e.y; r1x = p1s.x; r1y = p1s.y; r2x = p2s.x; r2y = p2s.y; }
            else {
              // Fallback: use midpoints
              cx = (p1s.x + p1e.x) / 2; cy = (p1s.y + p1e.y) / 2;
              r1x = p1e.x; r1y = p1e.y; r2x = p2e.x; r2y = p2e.y;
            }
          }
          return <AngleAnnotation key={d.id} cx={cx} cy={cy} r1x={r1x} r1y={r1y} r2x={r2x} r2y={r2y} label={d.label} scale={scale} onClick={(e) => {
            e.cancelBubble = true;
            const stage = e.target.getStage();
            const rect = stage.container().getBoundingClientRect();
            const pAbs = stage.getAbsoluteTransform().point({ x: cx, y: cy });
            setOverlay({ component: DimOverlay, props: {
              x: rect.left + pAbs.x, y: rect.top + pAbs.y,
              value: d.value.toFixed(1),
              onCommit: (vStr) => {
                const newVal = parseFloat(vStr);
                if (!isNaN(newVal)) {
                  const updatedDimensions = dimensionsRef.current.map(dd => dd.id === d.id ? { ...dd, value: newVal, label: newVal.toFixed(1) } : dd);
                  // Update the matching angle constraint value too
                  const updatedConstraints = constraintsRef.current.map(c => c.type === 'angle' && c.v1.sketchId === d.v1.sketchId && c.v2.sketchId === d.v2.sketchId ? { ...c, value: newVal } : c);
                  const solveResult = runSolver(sketchesRef.current, updatedConstraints, updatedDimensions, fixedPointsRef.current, SCALE_M, referenceVertices);
                  if (!solveResult.error) {
                    setSketches(solveResult.sketches);
                    if (setConstraints) setConstraints(updatedConstraints);
                    if (setDimensions) setDimensions(updatedDimensions);
                  }
                }
                setOverlay(null);
              },
              onCancel: () => setOverlay(null)
            }});
          }} />;
        }
        const s1 = sketches.find(sk => sk.id === d.v1.sketchId);
        const s2 = sketches.find(sk => sk.id === d.v2.sketchId);
        if (!s1 || !s2) return null;
        const p1 = resolveCadPoint(s1, d.v1.part);
        const p2 = resolveCadPoint(s2, d.v2.part);
        return <Annotation key={d.id} x1={p1.x} y1={p1.y} x2={p2.x} y2={p2.y} label={d.label} symbol={d.symbol} tolerance={d.tolerance} scale={scale} 
                  onClick={(e) => {
                     e.cancelBubble = true;
                     const stage = e.target.getStage();
                     const rect = stage.container().getBoundingClientRect();
                     const pAbs = stage.getAbsoluteTransform().point({x: (p1.x+p2.x)/2, y: (p1.y+p2.y)/2});
                     
                     // Must use live coordinates, not stale snapshot coords, to calculate distance vectors correctly
                     const liveV1 = { ...d.v1, x: p1.x, y: p1.y };
                     const liveV2 = { ...d.v2, x: p2.x, y: p2.y };
                     
                     setOverlay({
                         component: DimOverlay,
                         props: {
                             x: rect.left + pAbs.x, y: rect.top + pAbs.y,
                             value: d.value.toFixed(3), tolerance: d.tolerance ? d.tolerance.replace('±', '') : '',
                             onCommit: (vStr, tStr) => editDim(d.id, liveV1, liveV2, vStr, tStr),
                             onCancel: () => setOverlay(null)
                         }
                     });
                  }}
               />;
      })}
      
      {/* Visual Feedback for Selections */}
      {[...dimSelection, ...constraintSelection].map((v, i) => {
        if (v.type === 'edge' || (['parallel', 'perpendicular', 'equal'].includes(activeTool) && v.sketchId)) {
           if (v.line && v.line.points) {
              return <Line key={`sel-${i}`} points={v.line.points} stroke="rgba(255,165,0,0.8)" strokeWidth={4/scale} listening={false} />;
           }
           const sk = sketches.find(s => s.id === v.sketchId);
           if (sk && sk.type === 'line') {
              return <Line key={`sel-${i}`} points={sk.points} stroke="rgba(255,165,0,0.8)" strokeWidth={4/scale} listening={false} />;
           }
        }
        if (v.x !== undefined && v.y !== undefined) {
          return <Circle key={`sel-${i}`} x={v.x} y={v.y} radius={8 / scale} fill="rgba(255,165,0,0.8)" listening={false} stroke="white" strokeWidth={1.5/scale} />;
        }
        return null;
      })}

      {/* Marquee Selection Box */}
      {selectionBox && (
        <Rect
          x={Math.min(selectionBox.startX, selectionBox.endX)}
          y={Math.min(selectionBox.startY, selectionBox.endY)}
          width={Math.abs(selectionBox.startX - selectionBox.endX)}
          height={Math.abs(selectionBox.startY - selectionBox.endY)}
          fill="rgba(0, 229, 255, 0.15)"
          stroke="#00e5ff"
          strokeWidth={1 / scale}
          dash={[4 / scale, 2 / scale]}
          listening={false}
        />
      )}

      {getVertices().map((v, i) => {
        const isFixed = (fixedPoints || []).some(f => f.sketchId === v.sketchId && f.part === v.part);
        if (v.type === 'origin') {
          return (
            <Group key={`v-${i}`} x={v.x} y={v.y} listening={false}>
              <Circle radius={6 / scale} fill="white" stroke="black" strokeWidth={1 / scale} />
              <Circle radius={3 / scale} fill="black" />
            </Group>
          );
        }
        return (
          <Circle
            key={`v-${i}`} x={v.x} y={v.y}
            radius={isFixed ? 6 / scale : 4 / scale}
            fill={isFixed ? '#ff9800' : 'white'}
            listening={false}
            stroke={isFixed ? '#ff5c00' : 'black'}
            strokeWidth={isFixed ? 1.5 / scale : 0.5 / scale}
          />
        );
      })}
    </Group>
  );
});

export default CADSketcher;
