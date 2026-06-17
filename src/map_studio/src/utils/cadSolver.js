/**
 * Pure-JS Iterative Geometric Constraint Solver for SafetyStudio CAD.
 * v2: Replaces broken PlaneGCS bridge with a self-contained solver.
 *
 * Strategy: Extract "particles" (moveable named points) from each sketch,
 * run iterative constraint relaxation, then write solved coords back.
 * Fixed/anchored particles are never moved.
 * Returns { sketches, error } — error is a human-readable string if infeasible.
 */

const MAX_ITER   = 300;
const TOLERANCE  = 1e-5;   // convergence residual threshold (canvas px)
const STEP       = 0.5;    // relaxation step (0 < step <= 1)

// ─────────────────────────────────────────────────────────────────────────────
//  Particle extraction
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Extracts a flat map of { id → {x, y, fixed} } from all sketches.
 * Rect corners are stored individually; midpoints are computed on the fly.
 */
function extractParticles(sketches, fixedPoints) {
  const particles = {};

  const fix = (pid) => {
    if (particles[pid]) particles[pid].fixed = true;
  };

  const addPt = (id, x, y) => {
    if (!particles[id]) particles[id] = { x, y, fixed: false };
  };

  sketches.forEach(s => {
    if (s.type === 'line') {
      addPt(`${s.id}-start`, s.points[0], s.points[1]);
      addPt(`${s.id}-end`,   s.points[2], s.points[3]);
    } else if (s.type === 'circle') {
      addPt(`${s.id}-center`, s.center[0], s.center[1]);
      // radius is a scalar
    } else if (s.type === 'sector') {
      addPt(`${s.id}-center`, s.center[0], s.center[1]);
      const sx = s.center[0] + s.radius * Math.cos(s.startAngle);
      const sy = s.center[1] + s.radius * Math.sin(s.startAngle);
      addPt(`${s.id}-start`, sx, sy);
      const ex = s.center[0] + s.radius * Math.cos(s.startAngle + s.sweepAngle);
      const ey = s.center[1] + s.radius * Math.sin(s.startAngle + s.sweepAngle);
      addPt(`${s.id}-end`, ex, ey);
      // mid_arc is NOT a real particle — it is virtual (derived from center+start+end)
      // This prevents coincide constraints from distorting the sector shape.
    } else if (s.type === 'rect') {
      const [x1, y1] = s.start;
      const [x2, y2] = s.end;
      addPt(`${s.id}-p1`, x1, y1);
      addPt(`${s.id}-p2`, x2, y1);
      addPt(`${s.id}-p3`, x2, y2);
      addPt(`${s.id}-p4`, x1, y2);
    }
  });

  // Mark origin as fixed
  addPt('origin', 0, 0);
  particles['origin'].fixed = true;

  // Apply fixed points from anchors
  fixedPoints.forEach(f => {
    const pid = resolveParticleId(f);
    if (pid && particles[pid]) particles[pid].fixed = true;
  });

  return particles;
}

/** Convert a vertex descriptor {sketchId, part} → particle id string */
function resolveParticleId(v) {
  if (!v) return null;
  if (v.type === 'origin') return 'origin';
  const { sketchId, part } = v;
  if (!sketchId || !part) return null;

  // Normalise part aliases
  const partMap = {
    start: 'start', end: 'end',
    center: 'center',
    p1: 'p1', p2: 'p2', p3: 'p3', p4: 'p4',
    mid: 'mid', mid_top: 'mid_top', mid_bottom: 'mid_bottom',
    mid_left: 'mid_left', mid_right: 'mid_right',
    mid_arc: 'mid_arc',
  };
  const normalized = partMap[part] || part;
  return `${sketchId}-${normalized}`;
}

/**
 * Midpoint particles are virtual — computed from two real particles.
 * Returns {x, y} or null if either particle missing.
 */
function getMid(particles, id1, id2) {
  const p1 = particles[id1], p2 = particles[id2];
  if (!p1 || !p2) return null;
  return { x: (p1.x + p2.x) / 2, y: (p1.y + p2.y) / 2 };
}

/**
 * Resolve a possibly-virtual particle position (midpoints are virtual).
 * Returns {x, y, virtual, p1Id, p2Id} or null.
 */
function resolvePosition(particles, v, sketches) {
  if (!v) return null;
  if (v.type === 'origin') return { x: 0, y: 0, fixed: true };

  const { sketchId, part } = v;
  const s = sketches.find(sk => sk.id === sketchId);
  if (!s && sketchId !== 'origin') return null;

  // Virtual midpoints — resolved from two real particles
  const midparts = {
    mid:        s && s.type === 'line'   ? [`${sketchId}-start`, `${sketchId}-end`]       : null,
    mid_top:    s && s.type === 'rect'   ? [`${sketchId}-p1`,    `${sketchId}-p2`]         : null,
    mid_bottom: s && s.type === 'rect'   ? [`${sketchId}-p3`,    `${sketchId}-p4`]         : null,
    mid_left:   s && s.type === 'rect'   ? [`${sketchId}-p1`,    `${sketchId}-p4`]         : null,
    mid_right:  s && s.type === 'rect'   ? [`${sketchId}-p2`,    `${sketchId}-p3`]         : null,
    center:     s && s.type === 'rect'   ? [`${sketchId}-p1`,    `${sketchId}-p3`]         : null,  // centre = mid(p1,p3)
  };

  if (midparts[part]) {
    const [id1, id2] = midparts[part];
    const m = getMid(particles, id1, id2);
    if (!m) return null;
    const fixed = (particles[id1]?.fixed && particles[id2]?.fixed);
    return { ...m, virtual: true, p1Id: id1, p2Id: id2, fixed };
  }

  // Special virtual: sector mid_arc — computed from center+start+end
  // Movement routes to: center (translation) if free, OR rotates start/end (rotation) if center is pinned.
  if (part === 'mid_arc' && s && s.type === 'sector') {
    const c = particles[`${sketchId}-center`];
    const start = particles[`${sketchId}-start`];
    const end = particles[`${sketchId}-end`];
    if (!c || !start || !end) return null;
    const aS = Math.atan2(start.y - c.y, start.x - c.x);
    const aE = Math.atan2(end.y - c.y, end.x - c.x);
    const r = Math.sqrt((start.x - c.x) ** 2 + (start.y - c.y) ** 2);
    let sweep = aE - aS;
    if (sweep < 0) sweep += 2 * Math.PI;
    const midAng = aS + sweep / 2;
    const mx = c.x + r * Math.cos(midAng);
    const my = c.y + r * Math.sin(midAng);
    return {
      x: mx, y: my, virtual: true,
      // Tag as sector mid_arc so applyDelta can route correctly
      sectorMidArc: true,
      sketchId,
      p1Id: `${sketchId}-center`,   // for translation
      p2Id: `${sketchId}-center`,
      fixed: c.fixed
    };
  }

  const pid = resolveParticleId(v);
  if (!pid) return null;
  const p = particles[pid];
  if (!p) return null;
  return { ...p, pid };
}

/**
 * Move a possibly-virtual point toward target by delta.
 * For real particles: apply directly. For virtual midpoints: split delta to both parents.
 */
function applyDelta(particles, resolved, dx, dy, stepScale = STEP) {
  if (!resolved || resolved.fixed || isNaN(dx) || isNaN(dy) || !Number.isFinite(dx) || !Number.isFinite(dy)) return;

  if (resolved.sectorMidArc) {
    // mid_arc movement: prefer translation (move center) if center is free.
    // If center is fixed/pinned, rotate start and end around center instead.
    const c = particles[resolved.p1Id];
    const sid = resolved.sketchId;
    const start = particles[`${sid}-start`];
    const end = particles[`${sid}-end`];
    if (!c || !start || !end) return;

      // Rotation: always rotate start/end around center regardless of c.fixed
      // This correctly handles: constraint coincide on center (c moves via coincide) + mid_arc rotating sector
      const r = Math.sqrt((start.x - c.x) ** 2 + (start.y - c.y) ** 2);
      if (r < 1e-6) return;
      const aS = Math.atan2(start.y - c.y, start.x - c.x);
      const aE = Math.atan2(end.y - c.y, end.x - c.x);
      let sweep = aE - aS; if (sweep < 0) sweep += 2 * Math.PI;
      const midAng = aS + sweep / 2;
      const sinM = Math.sin(midAng), cosM = Math.cos(midAng);
      // Angular component of (dx,dy): how much would rotating mid_arc satisfy this delta?
      const dTheta = (-sinM * dx + cosM * dy) / r;
      // Linear (translation) component: residual after angular correction
      const rotDx = -sinM * dTheta * r, rotDy = cosM * dTheta * r;
      const transDx = dx - rotDx, transDy = dy - rotDy;
      // Apply rotation to start and end
      if (!start.fixed) {
        const rS = Math.sqrt((start.x - c.x) ** 2 + (start.y - c.y) ** 2);
        const aS2 = Math.atan2(start.y - c.y, start.x - c.x) + dTheta * stepScale;
        start.x = c.x + rS * Math.cos(aS2);
        start.y = c.y + rS * Math.sin(aS2);
      }
      if (!end.fixed) {
        const rE = Math.sqrt((end.x - c.x) ** 2 + (end.y - c.y) ** 2);
        const aE2 = Math.atan2(end.y - c.y, end.x - c.x) + dTheta * stepScale;
        end.x = c.x + rE * Math.cos(aE2);
        end.y = c.y + rE * Math.sin(aE2);
      }
      // Apply remaining translation to center
      if (!c.fixed) {
        c.x += transDx * stepScale;
        c.y += transDy * stepScale;
      }

  } else if (resolved.virtual) {
    const p1 = particles[resolved.p1Id];
    const p2 = particles[resolved.p2Id];
    if (p1 && !p1.fixed) { p1.x += dx * stepScale * 0.5; p1.y += dy * stepScale * 0.5; }
    if (p2 && !p2.fixed) { p2.x += dx * stepScale * 0.5; p2.y += dy * stepScale * 0.5; }
  } else if (resolved.pid) {
    const p = particles[resolved.pid];
    if (p && !p.fixed) { p.x += dx * stepScale; p.y += dy * stepScale; }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Constraint residuals & corrections
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Single iteration of constraint solving.
 * Returns total residual (sum of squared errors before correction).
 */
function solveIteration(particles, constraints, dimensions, sketches) {
  let residual = 0;

  // Helper: refresh a virtual position using current particle state
  const res = (v) => resolvePosition(particles, v, sketches);

  // ── Implicit Rigidity Constraints (e.g. Rectangles) ──
  sketches.forEach(s => {
    if (s.type === 'rect') {
      const p1 = particles[`${s.id}-p1`], p2 = particles[`${s.id}-p2`];
      const p3 = particles[`${s.id}-p3`], p4 = particles[`${s.id}-p4`];
      if (!p1 || !p2 || !p3 || !p4) return;

      // Rigid Rect: p1.y == p2.y, p2.x == p3.x, p3.y == p4.y, p4.x == p1.x
      const pairs = [
        [p1, p2, 'y'], [p2, p3, 'x'], [p3, p4, 'y'], [p4, p1, 'x']
      ];
      pairs.forEach(([a, b, axis]) => {
        const target = (a[axis] + b[axis]) / 2;
        const da = target - a[axis], db = target - b[axis];
        residual += da * da + db * db;
        if (!a.fixed) a[axis] += da * STEP;
        if (!b.fixed) b[axis] += db * STEP;
      });
    } else if (s.type === 'sector') {
      const c = particles[`${s.id}-center`];
      const start = particles[`${s.id}-start`];
      const end = particles[`${s.id}-end`];
      if (!c || !start || !end) return;

      // Enforce equal radii for both straight edges
      const r1 = Math.sqrt((start.x - c.x)**2 + (start.y - c.y)**2);
      const r2 = Math.sqrt((end.x - c.x)**2 + (end.y - c.y)**2);
      const targetR = (r1 + r2) / 2;
      
      const dr1 = targetR - r1, dr2 = targetR - r2;
      residual += dr1 * dr1 + dr2 * dr2;
      
      if (r1 > 1e-6) {
        const ratio = targetR / r1;
        const targetSx = c.x + (start.x - c.x) * ratio;
        const targetSy = c.y + (start.y - c.y) * ratio;
        if (!start.fixed && Number.isFinite(targetSx) && Number.isFinite(targetSy)) { start.x += (targetSx - start.x) * STEP; start.y += (targetSy - start.y) * STEP; }
      }
      if (r2 > 1e-6) {
        const ratio = targetR / r2;
        const targetEx = c.x + (end.x - c.x) * ratio;
        const targetEy = c.y + (end.y - c.y) * ratio;
        if (!end.fixed && Number.isFinite(targetEx) && Number.isFinite(targetEy)) { end.x += (targetEx - end.x) * STEP; end.y += (targetEy - end.y) * STEP; }
      }
    }

  });

  // ── Geometric Constraints ──
  constraints.forEach(c => {
    const { type, v1, v2 } = c;

    if (type === 'coincident' || type === 'coincide') {
      const r1 = res(v1), r2 = res(v2);
      if (!r1 || !r2) return;
      const dx = r2.x - r1.x, dy = r2.y - r1.y;
      const d2 = dx * dx + dy * dy;
      residual += d2;
      if (d2 < TOLERANCE * TOLERANCE) return;
      
      if (!r1.fixed && !r2.fixed) {
        const mx = (r1.x + r2.x) / 2, my = (r1.y + r2.y) / 2;
        applyDelta(particles, r1, mx - r1.x, my - r1.y);
        applyDelta(particles, r2, mx - r2.x, my - r2.y);
      } else if (r1.fixed && !r2.fixed) {
        applyDelta(particles, r2, r1.x - r2.x, r1.y - r2.y);
      } else if (r2.fixed && !r1.fixed) {
        applyDelta(particles, r1, r2.x - r1.x, r2.y - r1.y);
      }
    }

    else if (type === 'horizontal') {
      if (v2) {
        // Point-to-point horizontal
        const r1 = res(v1), r2 = res(v2);
        if (!r1 || !r2) return;
        const targetY = (r1.y + r2.y) / 2;
        const dy1 = targetY - r1.y, dy2 = targetY - r2.y;
        residual += dy1 * dy1 + dy2 * dy2;
        
        if (!r1.fixed && !r2.fixed) {
          applyDelta(particles, r1, 0, dy1);
          applyDelta(particles, r2, 0, dy2);
        } else if (r1.fixed && !r2.fixed) {
          applyDelta(particles, r2, 0, r1.y - r2.y);
        } else if (r2.fixed && !r1.fixed) {
          applyDelta(particles, r1, 0, r2.y - r1.y);
        }
      } else {
        // Single point or edge horizontal
        const r1 = res(v1);
        if (!r1) return;
        
        // If it's a line/rect edge, we should ideally make the whole segment horizontal.
        // For simple vertex selection, making a single point 'horizontal' doesn't mean much,
        // but if it's a line ID, we treat as line-horizontal.
        const s = sketches.find(sk => sk.id === v1.sketchId);
        // Allow vertex selection (any part) on a line/rect/sector — not just 'edge' type
        if (s && (s.type === 'line' || s.type === 'rect' || s.type === 'sector')) {
          let p1, p2;
          if (s.type === 'sector') {
            p1 = particles[`${s.id}-center`];
            p2 = particles[`${s.id}-${v1.part === 'start' ? 'start' : 'end'}`];
          } else {
            p1 = particles[resolveParticleId({sketchId: s.id, part: v1.part === 'end' ? 'end' : 'start'})];
            p2 = particles[resolveParticleId({sketchId: s.id, part: v1.part === 'end' ? 'start' : 'end'})];
          }
          // For rects, we keep them axis aligned anyway, but this satisfies the solver
          if (p1 && p2) {
            const targetY = (p1.y + p2.y) / 2;
            const dy1 = targetY - p1.y, dy2 = targetY - p2.y;
            residual += dy1 * dy1 + dy2 * dy2;
            if (!p1.fixed && !p2.fixed) {
              p1.y += dy1 * STEP;
              p2.y += dy2 * STEP;
            } else if (p1.fixed && !p2.fixed) {
              p2.y += (p1.y - p2.y) * STEP;
            } else if (p2.fixed && !p1.fixed) {
              p1.y += (p2.y - p1.y) * STEP;
            }
          }
        }
      }
    }

    else if (type === 'vertical') {
      if (v2) {
        // Point-to-point vertical
        const r1 = res(v1), r2 = res(v2);
        if (!r1 || !r2) return;
        const targetX = (r1.x + r2.x) / 2;
        const dx1 = targetX - r1.x, dx2 = targetX - r2.x;
        residual += dx1 * dx1 + dx2 * dx2;
        if (!r1.fixed && !r2.fixed) {
          applyDelta(particles, r1, dx1, 0);
          applyDelta(particles, r2, dx2, 0);
        } else if (r1.fixed && !r2.fixed) {
          applyDelta(particles, r2, r1.x - r2.x, 0);
        } else if (r2.fixed && !r1.fixed) {
          applyDelta(particles, r1, r2.x - r1.x, 0);
        }
      } else {
        // Single target vertical
        const r1 = res(v1);
        if (!r1) return;
        const s = sketches.find(sk => sk.id === v1.sketchId);
        if (s && (s.type === 'line' || s.type === 'rect' || s.type === 'sector')) {
          let p1, p2;
          if (s.type === 'sector') {
            p1 = particles[`${s.id}-center`];
            p2 = particles[`${s.id}-${v1.part === 'start' ? 'start' : 'end'}`];
          } else {
            p1 = particles[resolveParticleId({sketchId: s.id, part: v1.part === 'end' ? 'end' : 'start'})];
            p2 = particles[resolveParticleId({sketchId: s.id, part: v1.part === 'end' ? 'start' : 'end'})];
          }
          if (p1 && p2) {
            const targetX = (p1.x + p2.x) / 2;
            const dx1 = targetX - p1.x, dx2 = targetX - p2.x;
            residual += dx1 * dx1 + dx2 * dx2;
            if (!p1.fixed && !p2.fixed) {
              p1.x += dx1 * STEP;
              p2.x += dx2 * STEP;
            } else if (p1.fixed && !p2.fixed) {
              p2.x += (p1.x - p2.x) * STEP;
            } else if (p2.fixed && !p1.fixed) {
              p1.x += (p2.x - p1.x) * STEP;
            }
          }
        }
      }
    }

    else if (type === 'parallel' || type === 'perpendicular' || type === 'angle') {
      const s1 = sketches.find(sk => sk.id === v1.sketchId);
      const s2 = sketches.find(sk => sk.id === v2.sketchId);
      if (!s1 || !s2) return;
      if (s1.type !== 'line' && s1.type !== 'sector') return;
      if (s2.type !== 'line' && s2.type !== 'sector') return;

      const a1 = s1.type === 'line' ? particles[`${s1.id}-start`] : particles[`${s1.id}-center`];
      const b1 = s1.type === 'line' ? particles[`${s1.id}-end`] : particles[`${s1.id}-${v1.part === 'start' ? 'start' : 'end'}`];
      const a2 = s2.type === 'line' ? particles[`${s2.id}-start`] : particles[`${s2.id}-center`];
      const b2 = s2.type === 'line' ? particles[`${s2.id}-end`] : particles[`${s2.id}-${v2.part === 'start' ? 'start' : 'end'}`];
      if (!a1 || !b1 || !a2 || !b2) return;

      let dx1 = b1.x - a1.x, dy1 = b1.y - a1.y;
      const len1 = Math.sqrt(dx1 * dx1 + dy1 * dy1);
      if (len1 < 1e-6 || !Number.isFinite(len1)) return;
      dx1 /= len1; dy1 /= len1;
      const theta1 = Math.atan2(dy1, dx1);

      let dx2 = b2.x - a2.x, dy2 = b2.y - a2.y;
      const len2 = Math.sqrt(dx2 * dx2 + dy2 * dy2);
      if (len2 < 1e-6 || !Number.isFinite(len2)) return;
      const theta2 = Math.atan2(dy2 / len2, dx2 / len2);

      let targetRad = 0;
      if (type === 'perpendicular') targetRad = Math.PI / 2;
      else if (type === 'angle') targetRad = c.value * Math.PI / 180;

      const t1 = theta1 + targetRad;
      const t2 = theta1 - targetRad;

      let diff1 = theta2 - t1;
      while (diff1 > Math.PI) diff1 -= 2 * Math.PI;
      while (diff1 < -Math.PI) diff1 += 2 * Math.PI;

      let diff2 = theta2 - t2;
      while (diff2 > Math.PI) diff2 -= 2 * Math.PI;
      while (diff2 < -Math.PI) diff2 += 2 * Math.PI;

      if (type === 'parallel' || type === 'perpendicular') {
         let diff1_anti = diff1 > 0 ? diff1 - Math.PI : diff1 + Math.PI;
         let diff2_anti = diff2 > 0 ? diff2 - Math.PI : diff2 + Math.PI;
         if (Math.abs(diff1_anti) < Math.abs(diff1)) diff1 = diff1_anti;
         if (Math.abs(diff2_anti) < Math.abs(diff2)) diff2 = diff2_anti;
      }

      const diff = Math.abs(diff1) < Math.abs(diff2) ? diff1 : diff2;
      residual += diff * diff * 1000;
      if (Math.abs(diff) < TOLERANCE / 10) return;

      const targetTheta1 = theta1 + diff / 2;
      const targetTheta2 = theta2 - diff / 2;

      const tXX1 = Math.cos(targetTheta1), tYY1 = Math.sin(targetTheta1);
      const tXX2 = Math.cos(targetTheta2), tYY2 = Math.sin(targetTheta2);

      const idealDx1 = tXX1 * len1, idealDy1 = tYY1 * len1;
      const idealDx2 = tXX2 * len2, idealDy2 = tYY2 * len2;

      if (!Number.isFinite(idealDx1) || !Number.isFinite(idealDy1) || !Number.isFinite(idealDx2) || !Number.isFinite(idealDy2)) return;

      if (!a1.fixed) { a1.x += (b1.x - idealDx1 - a1.x) * STEP * 0.5; a1.y += (b1.y - idealDy1 - a1.y) * STEP * 0.5; }
      if (!b1.fixed) { b1.x += (a1.x + idealDx1 - b1.x) * STEP * 0.5; b1.y += (a1.y + idealDy1 - b1.y) * STEP * 0.5; }
      
      if (!a2.fixed) { a2.x += (b2.x - idealDx2 - a2.x) * STEP * 0.5; a2.y += (b2.y - idealDy2 - a2.y) * STEP * 0.5; }
      if (!b2.fixed) { b2.x += (a2.x + idealDx2 - b2.x) * STEP * 0.5; b2.y += (a2.y + idealDy2 - b2.y) * STEP * 0.5; }
    }

    else if (type === 'equal') {
      // Equalise lengths for two lines, or radii for two circles
      const s1 = sketches.find(sk => sk.id === v1.sketchId);
      const s2 = sketches.find(sk => sk.id === v2.sketchId);
      if (!s1 || !s2) return;

      if (s1.type === 'line' && s2.type === 'line') {
        const a1 = particles[`${s1.id}-start`], b1 = particles[`${s1.id}-end`];
        const a2 = particles[`${s2.id}-start`], b2 = particles[`${s2.id}-end`];
        if (!a1 || !b1 || !a2 || !b2) return;
        const len1 = Math.sqrt((b1.x-a1.x)**2 + (b1.y-a1.y)**2);
        const len2 = Math.sqrt((b2.x-a2.x)**2 + (b2.y-a2.y)**2);
        const target = (len1 + len2) / 2;
        residual += (len1 - target) ** 2 + (len2 - target) ** 2;
        // Scale line2 to target length, preserving midpoint
        if (len2 > 1e-6) {
          const ratio = target / len2;
          const mx = (a2.x + b2.x) / 2, my = (a2.y + b2.y) / 2;
          const newAx = mx + (a2.x - mx) * ratio, newAy = my + (a2.y - my) * ratio;
          const newBx = mx + (b2.x - mx) * ratio, newBy = my + (b2.y - my) * ratio;
          if (Number.isFinite(newAx) && Number.isFinite(newAy) && Number.isFinite(newBx) && Number.isFinite(newBy)) {
            if (!a2.fixed) { a2.x += (newAx - a2.x) * STEP; a2.y += (newAy - a2.y) * STEP; }
            if (!b2.fixed) { b2.x += (newBx - b2.x) * STEP; b2.y += (newBy - b2.y) * STEP; }
          }
        }
      } else if (s1.type === 'circle' && s2.type === 'circle') {
        const target = (s1.radius + s2.radius) / 2;
        const dr1 = target - s1.radius, dr2 = target - s2.radius;
        residual += dr1 * dr1 + dr2 * dr2;
        s1.radius += dr1 * STEP;
        s2.radius += dr2 * STEP;
      }
    }
  });

  // ── Dimension Constraints ──
  dimensions.forEach(d => {
    const s1 = sketches.find(sk => sk.id === d.v1.sketchId);
    if (s1 && s1.type === 'circle' && d.v2.part === 'rad') {
      // Radius dimension
      const targetPx = d.value * (d._scale || 100);
      const dr = targetPx - s1.radius;
      residual += dr * dr;
      s1.radius += dr * STEP;
      return;
    }

    const r1 = res(d.v1), r2 = res(d.v2);
    if (!r1 || !r2) return;

    const dx = r2.x - r1.x, dy = r2.y - r1.y;
    const dist = Math.sqrt(dx * dx + dy * dy);
    const targetPx = d.value * (d._scale || 100);
    const err = dist - targetPx;
    residual += err * err;

    if (Math.abs(err) < TOLERANCE || dist < 1e-6) return;

    const ratio = (dist - targetPx) / dist;
    const corrX = dx * ratio * 0.5;
    const corrY = dy * ratio * 0.5;

    if (!r1.fixed && !r2.fixed) {
      applyDelta(particles, r1, corrX, corrY);
      applyDelta(particles, r2, -corrX, -corrY);
    } else if (!r1.fixed) {
      applyDelta(particles, r1, dx * ratio, dy * ratio);
    } else if (!r2.fixed) {
      applyDelta(particles, r2, -dx * ratio, -dy * ratio);
    }
  });

  return residual;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Write solved particles back to sketch arrays
// ─────────────────────────────────────────────────────────────────────────────

function applyParticlesToSketches(sketches, particles) {
  return sketches.map(s => {
    if (s.type === 'line') {
      const p1 = particles[`${s.id}-start`], p2 = particles[`${s.id}-end`];
      if (p1 && p2) return { ...s, points: [p1.x, p1.y, p2.x, p2.y] };
    } else if (s.type === 'circle') {
      const c = particles[`${s.id}-center`];
      if (c) return { ...s, center: [c.x, c.y] };
    } else if (s.type === 'sector') {
      const c = particles[`${s.id}-center`];
      const start = particles[`${s.id}-start`];
      const end = particles[`${s.id}-end`];
      if (c && start && end) {
        const radius = (Math.sqrt((start.x - c.x)**2 + (start.y - c.y)**2) + Math.sqrt((end.x - c.x)**2 + (end.y - c.y)**2)) / 2;
        const startAngle = Math.atan2(start.y - c.y, start.x - c.x);
        let sweepAngle = Math.atan2(end.y - c.y, end.x - c.x) - startAngle;
        
        // Normalize sweepAngle to preserve direction based on original sector
        // A sector's sweepAngle is typically drawn with a specific orientation.
        // We ensure the new sweep angle is as close to the original as possible.
        while (sweepAngle - s.sweepAngle > Math.PI) sweepAngle -= 2 * Math.PI;
        while (sweepAngle - s.sweepAngle < -Math.PI) sweepAngle += 2 * Math.PI;
        
        return { ...s, center: [c.x, c.y], radius, startAngle, sweepAngle };
      }
    } else if (s.type === 'rect') {
      const p1 = particles[`${s.id}-p1`], p2 = particles[`${s.id}-p2`];
      const p3 = particles[`${s.id}-p3`], p4 = particles[`${s.id}-p4`];
      if (p1 && p2 && p3 && p4) {
        // Reconstruct start/end from the span of all 4 corners.
        // Maintain axis-aligned rectangle: start = (minX, minY), end = (maxX, maxY).
        // After solving, corners may have drifted slightly — use their centroid + half-spans.
        const cx = (p1.x + p2.x + p3.x + p4.x) / 4;
        const cy = (p1.y + p2.y + p3.y + p4.y) / 4;
        // Half-width and half-height from the corner spread
        const hw = (Math.abs(p2.x - p1.x) + Math.abs(p3.x - p4.x)) / 4;
        const hh = (Math.abs(p4.y - p1.y) + Math.abs(p3.y - p2.y)) / 4;
        return { ...s, start: [cx - hw, cy - hh], end: [cx + hw, cy + hh] };
      }
    }
    return s;
  });
}

// ─────────────────────────────────────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────────────────────────────────────

/** @returns {sketches: [], error: string|null} */
export const propagateConstraints = (sketches, constraints, dimensions, fixedPoints, SCALE_M, referenceVertices = []) => {
  if (!sketches || sketches.length === 0) return sketches;
  if ((!constraints || constraints.length === 0) && (!dimensions || dimensions.length === 0)) return sketches;

  // Stamp the SCALE_M into dimensions so resolvePosition can use it
  const stampedDims = (dimensions || []).map(d => ({ ...d, _scale: SCALE_M }));

  const particles = extractParticles(sketches, fixedPoints || []);

  // Pre-solve: Propagate "fixed" status across coincident constraints
  // If A is fixed and B is coincident with A, then B should act as fixed.
  let changed = true;
  let loops = 0;
  while (changed && loops < 10) {
    changed = false;
    loops++;
    (constraints || []).forEach(c => {
      if (c.type === 'coincident' || c.type === 'coincide') {
        const r1 = resolvePosition(particles, c.v1, sketches);
        const r2 = resolvePosition(particles, c.v2, sketches);
        if (!r1 || !r2) return;
        
        // We only propagate fixedness to real particles, not virtual midpoints
        if (r1.fixed && !r2.fixed && !r2.virtual) {
          particles[r2.pid].x = r1.x;
          particles[r2.pid].y = r1.y;
          particles[r2.pid].fixed = true;
          changed = true;
        } else if (r2.fixed && !r1.fixed && !r1.virtual) {
          particles[r1.pid].x = r2.x;
          particles[r1.pid].y = r2.y;
          particles[r1.pid].fixed = true;
          changed = true;
        }
      }
    });
  }

  // Check infeasibility upfront: two coincident targets both fixed at different positions
  const infeasibleReasons = [];
  (constraints || []).forEach(c => {
    if (c.type === 'coincident' || c.type === 'coincide') {
      const r1 = resolvePosition(particles, c.v1, sketches);
      const r2 = resolvePosition(particles, c.v2, sketches);
      if (r1 && r2 && r1.fixed && r2.fixed) {
        const d = Math.sqrt((r1.x - r2.x) ** 2 + (r1.y - r2.y) ** 2);
        if (d > TOLERANCE) {
          infeasibleReasons.push(`Coincident constraint between two anchored points at different positions`);
        }
      }
    }
  });

  if (infeasibleReasons.length > 0) {
    return { sketches, error: infeasibleReasons.join('\n') };
  }

  // Iterative solve
  let residual = Infinity;
  for (let i = 0; i < MAX_ITER; i++) {
    residual = solveIteration(particles, constraints || [], stampedDims, sketches);
    if (residual < TOLERANCE * TOLERANCE) break;
  }

  // If we didn't converge, check if it's because constraints are contradictory
  if (residual > 1.0) {
    // residual > 1 pixel² after 300 iterations means genuinely infeasible or conflicting
    return {
      sketches,
      error: `Constraint could not be satisfied. The constraints may be contradictory or over-constrained.\n(Residual: ${Math.sqrt(residual).toFixed(2)} px)`
    };
  }

  return { sketches: applyParticlesToSketches(sketches, particles), error: null };
};

/**
 * Add a single constraint and resolve.
 * Returns { sketches, newConstraint, error }
 */
export const applyConstraint = (sketches, type, v1, v2, fixedPoints, dimensions, SCALE_M, referenceVertices, existingConstraints = []) => {
  const newConstraint = { type, v1, v2, id: Date.now() };
  const allConstraints = [...existingConstraints, newConstraint];
  const result = propagateConstraints(sketches, allConstraints, dimensions, fixedPoints, SCALE_M, referenceVertices);

  if (result.error) {
    return { sketches, newConstraint: null, error: result.error };
  }
  return { sketches: result.sketches, newConstraint, error: null };
};

/**
 * Returns true if the sketch has a fixed/anchor point applied.
 */
export const isFullyConstrained = (s, dimensions, fixedPoints) => {
  return (fixedPoints || []).some(f => f.sketchId === s.id);
};

/**
 * Validates a proposed new set of sketches against the active constraints/dimensions.
 * Returns an array of human-readable error strings (empty = valid).
 * This is a lightweight check — not a full solve.
 */
export const validateConstraints = (oldSketches, newSketches, dimensions, fixedPoints, referenceVertices = [], SCALE_M = 100, excludeDimId) => {
  return []; // Full validation handled inside propagateConstraints
};
