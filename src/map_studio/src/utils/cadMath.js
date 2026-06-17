/**
 * CAD Vector Math Library for SafetyStudio
 * Provides robust geometric calculations for drafting and constraints.
 */

export const vec = {
  add: (a, b) => ({ x: a.x + b.x, y: a.y + b.y }),
  sub: (a, b) => ({ x: a.x - b.x, y: a.y - b.y }),
  mul: (a, s) => ({ x: a.x * s, y: a.y * s }),
  div: (a, s) => ({ x: a.x / s, y: a.y / s }),
  magSq: (a) => a.x * a.x + a.y * a.y,
  mag: (a) => Math.sqrt(a.x * a.x + a.y * a.y),
  normalize: (a) => {
    const l = Math.sqrt(a.x * a.x + a.y * a.y);
    return l > 0 ? { x: a.x / l, y: a.y / l } : { x: 0, y: 0 };
  },
  dot: (a, b) => a.x * b.x + a.y * b.y,
  cross: (a, b) => a.x * b.y - a.y * b.x,
  dist: (a, b) => Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2)),
  angle: (a, b) => Math.atan2(vec.cross(a, b), vec.dot(a, b)),
  rotate: (a, angle) => ({
    x: a.x * Math.cos(angle) - a.y * Math.sin(angle),
    y: a.x * Math.sin(angle) + a.y * Math.cos(angle)
  }),
  perp: (a) => ({ x: -a.y, y: a.x }), // Counter-clockwise perpendicular
  project: (p, a, b) => {
    const ap = vec.sub(p, a);
    const ab = vec.sub(b, a);
    const l2 = vec.magSq(ab);
    if (l2 === 0) return a;
    const t = Math.max(0, Math.min(1, vec.dot(ap, ab) / l2));
    return vec.add(a, vec.mul(ab, t));
  }
};

/**
 * Normalizes an angle to [0, 2PI)
 */
export const normalizeAngle = (a) => {
  let res = a % (Math.PI * 2);
  if (res < 0) res += Math.PI * 2;
  return res;
};

/**
 * Finite Number Guard
 */
export const isPointSafe = (p) => p && Number.isFinite(p.x) && Number.isFinite(p.y);
