import React, { useRef, useEffect, useCallback, useState } from 'react';
import { Stage, Layer, Line, Circle } from 'react-konva';

const RULER = 30;   // px — ruler strip width/height (matches Qt margin=30)
const SCALE_M = 100; // px per metre in world coords

// ─── Pick a "nice" step given a raw world-unit step ───────────────────────────
function niceStep(raw) {
  if (raw <= 0) return 0.1;
  const base = Math.pow(10, Math.floor(Math.log10(raw)));
  const r    = raw / base;
  if (r < 2)      return 1   * base;
  else if (r < 5) return 2   * base;
  else             return 5   * base;
}

// ─── Draw one ruler strip on a <canvas> ──────────────────────────────────────
function drawRuler(canvas, axis, stageX, stageY, viewScale, w, h) {
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  canvas.width  = axis === 'x' ? w : RULER;
  canvas.height = axis === 'x' ? RULER : h;
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Background
  ctx.fillStyle = '#2d2d2d';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // Border line
  ctx.strokeStyle = '#444';
  ctx.lineWidth = 1;
  if (axis === 'x') {
    ctx.beginPath(); ctx.moveTo(0, RULER - 1); ctx.lineTo(w, RULER - 1); ctx.stroke();
  } else {
    ctx.beginPath(); ctx.moveTo(RULER - 1, 0); ctx.lineTo(RULER - 1, h); ctx.stroke();
  }

  // Adaptive tick step (mirrors Qt: raw = 80 * m_px)
  const m_px   = 1.0 / (viewScale * SCALE_M);        // world metres per canvas px
  const rawStep = 80 * m_px;
  const stepM   = niceStep(rawStep);                  // world metres per tick
  const stepPx  = stepM * SCALE_M * viewScale;        // canvas px per tick

  ctx.font      = '9px monospace';
  ctx.fillStyle = '#ccc';

  if (axis === 'x') {
    // World 0 is at canvas x = stageX
    const startM = Math.floor(-stageX / (SCALE_M * viewScale) / stepM) * stepM;
    for (let m = startM; ; m += stepM) {
      const cx = stageX + m * SCALE_M * viewScale;
      if (cx > w) break;
      if (cx < RULER) continue;
      ctx.strokeStyle = '#666';
      ctx.lineWidth   = 1;
      ctx.beginPath(); ctx.moveTo(cx, RULER * 0.5); ctx.lineTo(cx, RULER); ctx.stroke();
      const label = Math.abs(m) < 1e-10 ? '0' : parseFloat(m.toPrecision(4)).toString();
      ctx.textAlign    = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(label, cx, RULER * 0.28);
    }
    // "X" label at start
    ctx.fillStyle = '#FF1744'; ctx.font = 'bold 9px monospace';
    ctx.textAlign = 'left'; ctx.fillText('X', RULER + 4, 14);
  } else {
    // Y axis: world Y positive-up ↔ canvas Y inverted
    // World 0 is at canvas y = stageY
    const startM = Math.floor(-stageY / (SCALE_M * viewScale) / stepM) * stepM - stepM;
    for (let m = startM; ; m += stepM) {
      const cy = stageY + m * SCALE_M * viewScale;   // note: positive m → downward in canvas
      if (cy > h) break;
      if (cy < RULER) continue;
      ctx.strokeStyle = '#666';
      ctx.lineWidth   = 1;
      ctx.beginPath(); ctx.moveTo(RULER * 0.5, cy); ctx.lineTo(RULER, cy); ctx.stroke();
      const worldY    = -m; // invert: positive world Y = upward
      const label     = Math.abs(worldY) < 1e-10 ? '0' : parseFloat(worldY.toPrecision(4)).toString();
      ctx.save();
      ctx.translate(RULER * 0.32, cy);
      ctx.rotate(-Math.PI / 2);
      ctx.textAlign    = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillStyle    = '#ccc';
      ctx.font         = '9px monospace';
      ctx.fillText(label, 0, 0);
      ctx.restore();
    }
    // "Y" label
    ctx.fillStyle = '#00C853'; ctx.font = 'bold 9px monospace';
    ctx.textAlign = 'center'; ctx.fillText('Y', RULER / 2, RULER + 14);
  }

  // Corner square
  if (axis === 'x') {
    ctx.fillStyle = '#2d2d2d';
    ctx.fillRect(0, 0, RULER, RULER);
  }
}

// ─── Konva grid layer (mirrors BaseGridScene.drawBackground) ─────────────────
function GridLayer({ scale, stagePos, width, height }) {
  // Adaptive step: aim for ~60-120px between grid lines
  const targetPx = 60;
  const rawStep  = targetPx / (SCALE_M * scale);
  const stepM    = niceStep(rawStep);
  const majorMul = 5;  // every 5 minor = 1 major

  const minWorldX = -stagePos.x / (SCALE_M * scale);
  const maxWorldX = (width  - stagePos.x) / (SCALE_M * scale);
  // Y inverted: canvas down = world down, but we flipped in worldToCanvas: y=-world*SCALE
  const minWorldY = (stagePos.y - height) / (SCALE_M * scale);
  const maxWorldY =  stagePos.y           / (SCALE_M * scale);

  const lines = [];
  const startX = Math.floor(minWorldX / stepM) * stepM;
  const endX   = Math.ceil (maxWorldX / stepM) * stepM;
  const startY = Math.floor(minWorldY / stepM) * stepM;
  const endY   = Math.ceil (maxWorldY / stepM) * stepM;

  for (let x = startX; x <= endX; x = parseFloat((x + stepM).toPrecision(10))) {
    const isMajor = Math.round(x / stepM) % majorMul === 0;
    lines.push(
      <Line key={`gx${x.toFixed(6)}`}
        points={[x * SCALE_M, -20000, x * SCALE_M, 20000]}
        stroke={isMajor ? '#4a4a4a' : '#353535'}
        strokeWidth={(isMajor ? 1.0 : 0.6) / scale}
        listening={false}
      />
    );
  }
  for (let y = startY; y <= endY; y = parseFloat((y + stepM).toPrecision(10))) {
    const isMajor = Math.round(y / stepM) % majorMul === 0;
    lines.push(
      <Line key={`gy${y.toFixed(6)}`}
        points={[-20000, -y * SCALE_M, 20000, -y * SCALE_M]}
        stroke={isMajor ? '#4a4a4a' : '#353535'}
        strokeWidth={(isMajor ? 1.0 : 0.6) / scale}
        listening={false}
      />
    );
  }

  return <Layer listening={false}>{lines}</Layer>;
}

// ─── GridCanvas ───────────────────────────────────────────────────────────────
// Render-prop component. Uses two <canvas> rulers + Konva Stage.
// Usage:
//   <GridCanvas>
//     {({ scale, stagePos, SCALE_M }) => (
//       <Layer>…user content…</Layer>
//     )}
//   </GridCanvas>
//
const GridCanvas = ({ children, style = {}, initialScale = 1, draggable = true, stagePos: externalStagePos, onStagePosChange }) => {
  const containerRef = useRef(null);
  const xRulerRef    = useRef(null);
  const yRulerRef    = useRef(null);

  const [size, setSize]       = useState({ w: 0, h: 0 }); // Start with 0 to detect first measurement
  const [scale, setScale]     = useState(initialScale);
  const [internalStagePos, setInternalStagePos] = useState({ x: 0, y: 0 });
  const [overlay, setOverlay] = useState(null); // { component, props }
  const hasCentered = useRef(false);

  const stagePos = externalStagePos || internalStagePos;
  const setStagePos = (pos) => {
    if (!Number.isFinite(pos.x) || !Number.isFinite(pos.y)) return;
    if (onStagePosChange) onStagePosChange(pos);
    else setInternalStagePos(pos);
  };

  const setScaleSafe = (val) => {
    if (!Number.isFinite(val) || val <= 0) return;
    setScale(val);
  };

  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const obs = new ResizeObserver(() => {
      const w = el.clientWidth;
      const h = el.clientHeight;
      setSize({ w, h });
    });
    obs.observe(el);
    return () => obs.disconnect();
  }, []);

  useEffect(() => {
    drawRuler(xRulerRef.current, 'x', stagePos.x, stagePos.y, scale, size.w, size.h);
    drawRuler(yRulerRef.current, 'y', stagePos.x, stagePos.y, scale, size.w, size.h);

    // One-time centering upon initiation/refresh when size is first known
    if (size.w > 0 && size.h > 0 && !hasCentered.current) {
      const canvasW = size.w - RULER;
      const canvasH = size.h - RULER;
      const centeredPos = { x: canvasW / 2, y: canvasH / 2 };
      
      // If we are using internal state, set it directly
      // If external, call the callback
      if (onStagePosChange) {
        onStagePosChange(centeredPos);
      } else {
        setInternalStagePos(centeredPos);
      }
      
      hasCentered.current = true;
    }
  }, [stagePos, scale, size, onStagePosChange]);

  const handleWheel = useCallback((e) => {
    e.evt.preventDefault();
    const stage = e.target.getStage();
    if (!stage) return;
    const oldScale = stage.scaleX();
    const pointer  = stage.getPointerPosition();
    if (!pointer) return;
    const mouseAt = { x: (pointer.x - stage.x()) / oldScale, y: (pointer.y - stage.y()) / oldScale };
    const factor   = e.evt.deltaY < 0 ? 1.1 : 0.9;
    const newScale = Math.max(0.05, Math.min(200, oldScale * factor));
    const newPos = { x: pointer.x - mouseAt.x * newScale, y: pointer.y - mouseAt.y * newScale };
    setScaleSafe(newScale);
    setStagePos(newPos);
  }, []);

  const handleDragEnd = useCallback((e) => {
    if (e.target !== e.target.getStage()) return; 
    setStagePos({ x: e.target.x(), y: e.target.y() });
  }, []);

  const canvasW = size.w - RULER;
  const canvasH = size.h - RULER;

  return (
    <div ref={containerRef} style={{ position: 'relative', width: '100%', height: '100%', overflow: 'hidden', background: '#1c1c1c', ...style }}>
      <div style={{ position: 'absolute', top: 0, left: 0, width: RULER, height: RULER, background: '#2d2d2d', zIndex: 10, borderRight: '1px solid #444', borderBottom: '1px solid #444' }} />
      <canvas ref={xRulerRef} style={{ position: 'absolute', top: 0, left: RULER, zIndex: 9 }} />
      <canvas ref={yRulerRef} style={{ position: 'absolute', top: RULER, left: 0, zIndex: 9 }} />

      <div style={{ position: 'absolute', top: RULER, left: RULER }}>
        <Stage
          width={canvasW}
          height={canvasH}
          x={stagePos.x}
          y={stagePos.y}
          scaleX={scale}
          scaleY={scale}
          draggable={draggable}
          onWheel={handleWheel}
          onDragEnd={handleDragEnd}
          onContextMenu={(e) => e.evt.preventDefault()}
        >
          <GridLayer scale={scale} stagePos={stagePos} width={canvasW} height={canvasH} />
          <Layer listening={false}>
            <Line points={[0, 0, SCALE_M, 0]} stroke="#D32F2F" strokeWidth={3 / scale} />
            <Line points={[0, 0, 0, -SCALE_M]} stroke="#388E3C" strokeWidth={3 / scale} />
            <Circle x={0} y={0} radius={3 / scale} fill="white" />
          </Layer>
          {typeof children === 'function' ? children({ scale, stagePos, SCALE_M, setOverlay, setScale: setScaleSafe, setStagePos }) : children}
        </Stage>
      </div>

      {/* RENDER OVERLAY OUTSIDE STAGE */}
      {overlay && (
         <div style={{ position: 'absolute', top: 0, left: 0, width: '100%', height: '100%', pointerEvents: 'none', zIndex: 1000 }}>
            <overlay.component {...overlay.props} onCancel={() => setOverlay(null)} />
         </div>
      )}

      {/* Floating Zoom Controls */}
      <div className="absolute bottom-6 right-6 bg-surface-container border-2 border-outline-variant rounded-lg flex flex-col z-10 shadow-lg">
        <button 
          onClick={() => setScaleSafe(Math.min(200, scale * 1.2))} 
          title="Zoom In" 
          className="p-2 hover:bg-surface-container-high text-on-surface border-b-2 border-outline-variant transition-colors flex items-center justify-center"
        >
          <span className="material-symbols-outlined">add</span>
        </button>
        <div 
          onClick={() => {
            setScaleSafe(initialScale);
            if (size.w > 0 && size.h > 0) {
              const canvasW = size.w - RULER;
              const canvasH = size.h - RULER;
              const centeredPos = { x: canvasW / 2, y: canvasH / 2 };
              if (onStagePosChange) onStagePosChange(centeredPos);
              else setInternalStagePos(centeredPos);
            }
          }}
          title="Reset View"
          className="p-2 font-data-mono text-center text-xs text-primary border-b-2 border-outline-variant cursor-pointer hover:bg-surface-container-high transition-colors"
        >
          {Math.round(scale * 100)}%
        </div>
        <button 
          onClick={() => setScaleSafe(Math.max(0.05, scale / 1.2))} 
          title="Zoom Out" 
          className="p-2 hover:bg-surface-container-high text-on-surface transition-colors flex items-center justify-center"
        >
          <span className="material-symbols-outlined">remove</span>
        </button>
      </div>
    </div>
  );
};

export { GridCanvas, SCALE_M, RULER };
export default GridCanvas;
