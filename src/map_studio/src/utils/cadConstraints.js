import { isFullyConstrained } from './cadSolver';

/**
 * Geometric constraint utility for SafetyStudio CAD.
 * This handles updating sketch coordinates/dimensions based on user numeric entry.
 */

export const applyDimensionUpdate = (sketches, vA, vB, newValue, SCALE_M, dimensions = [], fixedPoints = [], referenceVertices = []) => {
  const updated = [...sketches];
  const targetPx = newValue * SCALE_M;

  // Determine which vertex should act as anchor (v1) and which should move (v2)
  let v1 = vA, v2 = vB;
  const sA = sketches.find(s => s.id === vA.sketchId);
  const sB = sketches.find(s => s.id === vB.sketchId);
  
  const isA_Fixed = vA.type === 'reference' || vA.type === 'origin' || (sA && isFullyConstrained(sA, dimensions, fixedPoints));
  const isB_Fixed = vB.type === 'reference' || vB.type === 'origin' || (sB && isFullyConstrained(sB, dimensions, fixedPoints));

  if (isB_Fixed && !isA_Fixed) {
     v1 = vB; v2 = vA;
  }

  const dx = v2.x - v1.x;
  const dy = v2.y - v1.y;
  if (!Number.isFinite(dx) || !Number.isFinite(dy)) return sketches;
  
  const currentDist = Math.sqrt(dx * dx + dy * dy);
  if (!Number.isFinite(currentDist) || currentDist < 0.001) return sketches; // Avoid divide by zero

  const ratio = targetPx / currentDist;
  if (!Number.isFinite(ratio)) return sketches;

  const newX = v1.x + dx * ratio;
  const newY = v1.y + dy * ratio;

  // Now find the sketch matching v2.sketchId and update it
  const idx = updated.findIndex(s => s.id === v2.sketchId);
  if (idx === -1) return sketches; // E.g., if v2 was a reference, it won't be found in sketches

  const s = { ...updated[idx] };

  const moveAllCoincident = (oldX, oldY, nX, nY, ignoreIds = []) => {
    updated.forEach((ns, i) => {
       if (ignoreIds.includes(ns.id)) return;
       let changed = false;
       const next = {...ns};
       const thresh = 0.005;
       if (next.type === 'line') {
          if (Math.abs(next.points[0] - oldX) < thresh && Math.abs(next.points[1] - oldY) < thresh) {
             next.points = [nX, nY, next.points[2], next.points[3]]; changed = true;
          } else if (Math.abs(next.points[2] - oldX) < thresh && Math.abs(next.points[3] - oldY) < thresh) {
             next.points = [next.points[0], next.points[1], nX, nY]; changed = true;
          }
       } else if (next.type === 'circle') {
          if (Math.abs(next.center[0] - oldX) < thresh && Math.abs(next.center[1] - oldY) < thresh) {
             next.center = [nX, nY]; changed = true;
          }
       } else if (next.type === 'rect') {
          const x1 = next.start[0], y1 = next.start[1], x2 = next.end[0], y2 = next.end[1];
          const pts = [[x1, y1], [x2, y1], [x2, y2], [x1, y2], [(x1+x2)/2, y1], [(x1+x2)/2, y2], [x1, (y1+y2)/2], [x2, (y1+y2)/2], [(x1+x2)/2, (y1+y2)/2]];
          if (pts.some(p => Math.abs(p[0] - oldX) < thresh && Math.abs(p[1] - oldY) < thresh)) {
             next.start = [next.start[0] + (nX-oldX), next.start[1] + (nY-oldY)];
             next.end   = [next.end[0] + (nX-oldX), next.end[1] + (nY-oldY)];
             changed = true;
          }
       }
       if (changed) updated[i] = next;
    });
  };

  if (s.type === 'line') {
    if (v2.part === 'start') {
      moveAllCoincident(s.points[0], s.points[1], newX, newY, [s.id]);
      s.points = [newX, newY, s.points[2], s.points[3]];
    }
    if (v2.part === 'end') {
      moveAllCoincident(s.points[2], s.points[3], newX, newY, [s.id]);
      s.points = [s.points[0], s.points[1], newX, newY];
    }
  } else if (s.type === 'circle') {
    if (v2.part === 'center') {
       s.center = [newX, newY];
    } else {
       // It's a radius update
       s.radius = targetPx;
    }
  } else if (s.type === 'rect') {
    const xMin = Math.min(s.start[0], s.end[0]);
    const xMax = Math.max(s.start[0], s.end[0]);
    const yMin = Math.min(s.start[1], s.end[1]);
    const yMax = Math.max(s.start[1], s.end[1]);

    if (v2.part === 'p1' || v2.part === 'start') {
      moveAllCoincident(s.start[0], s.start[1], newX, newY, [s.id]);
      s.start = [newX, newY];
    } else if (v2.part === 'p3' || v2.part === 'end') {
      moveAllCoincident(s.end[0], s.end[1], newX, newY, [s.id]);
      s.end = [newX, newY];
    } else if (v2.part === 'p2') {
      moveAllCoincident(xMax, yMin, newX, newY, [s.id]);
      s.start = [s.start[0], newY];
      s.end = [newX, s.end[1]];
    } else if (v2.part === 'p4') {
      moveAllCoincident(xMin, yMax, newX, newY, [s.id]);
      s.start = [newX, s.start[1]];
      s.end = [s.end[0], newY];
    } else if (v2.part === 'mid_top') {
      moveAllCoincident((xMin+xMax)/2, yMin, newX, newY, [s.id]);
      if (s.start[1] < s.end[1]) s.start[1] = newY; else s.end[1] = newY;
    } else if (v2.part === 'mid_bottom') {
      moveAllCoincident((xMin+xMax)/2, yMax, newX, newY, [s.id]);
      if (s.start[1] > s.end[1]) s.start[1] = newY; else s.end[1] = newY;
    } else if (v2.part === 'mid_left') {
      moveAllCoincident(xMin, (yMin+yMax)/2, newX, newY, [s.id]);
      if (s.start[0] < s.end[0]) s.start[0] = newX; else s.end[0] = newX;
    } else if (v2.part === 'mid_right') {
      moveAllCoincident(xMax, (yMin+yMax)/2, newX, newY, [s.id]);
      if (s.start[0] > s.end[0]) s.start[0] = newX; else s.end[0] = newX;
    } else if (v2.part === 'center') {
       const offsetX = newX - (s.start[0] + s.end[0]) / 2;
       const offsetY = newY - (s.start[1] + s.end[1]) / 2;
       moveAllCoincident((s.start[0] + s.end[0]) / 2, (s.start[1] + s.end[1]) / 2, newX, newY, [s.id]);
       s.start = [s.start[0] + offsetX, s.start[1] + offsetY];
       s.end   = [s.end[0] + offsetX, s.end[1] + offsetY];
    }
  }

  updated[idx] = s;
  return updated;
};
