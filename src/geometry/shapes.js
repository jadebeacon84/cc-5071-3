import { Transform } from '../core/transform.js';
import { Vec2 } from '../core/vec2.js';
import { EPS_LEN, polygonArea2, signedArea2 } from '../core/geom.js';
import { classifyRawPoints } from './validate.js';

// Convex polygon whose local vertices are ordered CCW about the local origin
// (the local origin is the mass/rotation centre, not necessarily the centroid).
export class ConvexShape {
  constructor(localVerts, name = 'convex') {
    if (localVerts.length < 3) {
      throw new Error('ConvexShape needs at least 3 non-degenerate vertices');
    }
    this.name = name;
    this.localVerts = localVerts.map((v) => v.clone());
    this.worldVerts = localVerts.map((v) => v.clone());
    this.xf = new Transform();
    this.updateTransform(this.xf);
  }

  updateTransform(xf) {
    this.xf = xf;
    this.worldVerts = this.localVerts.map((v) => xf.localToWorld(v));
  }

  // Furthest vertex along world direction d. Ties resolve to the lowest index
  // so support queries are deterministic for repeated queries.
  support(d) {
    let best = this.worldVerts[0];
    let bestDot = Vec2.dot(best, d);
    for (let i = 1; i < this.worldVerts.length; i++) {
      const v = this.worldVerts[i];
      const s = Vec2.dot(v, d);
      if (s > bestDot) {
        bestDot = s;
        best = v;
      }
    }
    return { point: best.clone(), index: this.worldVerts.indexOf(best), value: bestDot };
  }

  supportLocal(dLocal) {
    let best = this.localVerts[0];
    let bestDot = Vec2.dot(best, dLocal);
    for (let i = 1; i < this.localVerts.length; i++) {
      const s = Vec2.dot(this.localVerts[i], dLocal);
      if (s > bestDot) {
        bestDot = s;
        best = this.localVerts[i];
      }
    }
    return { point: best.clone(), index: this.localVerts.indexOf(best), value: bestDot };
  }

  // World-space CCW edges {a, b, n} where n is the outward unit normal.
  edges() {
    const out = [];
    const n = this.worldVerts.length;
    for (let i = 0; i < n; i++) {
      const a = this.worldVerts[i];
      const b = this.worldVerts[(i + 1) % n];
      const tangent = Vec2.sub(b, a);
      // CCW verts: the outward normal is the left perpendicular of the edge.
      const normal = Vec2.normalize(Vec2.perpLeft(tangent));
      out.push({ a, b, tangent, normal, index: i });
    }
    return out;
  }

  aabb() {
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (const v of this.worldVerts) {
      minX = Math.min(minX, v.x);
      minY = Math.min(minY, v.y);
      maxX = Math.max(maxX, v.x);
      maxY = Math.max(maxY, v.y);
    }
    return { minX, minY, maxX, maxY };
  }

  area() {
    return Math.abs(polygonArea2(this.localVerts)) * 0.5;
  }

  // Geometric centroid in local frame.
  centroidLocal() {
    const n = this.localVerts.length;
    const c = new Vec2();
    let areaSum = 0;
    for (let i = 0; i < n; i++) {
      const a = this.localVerts[i];
      const b = this.localVerts[(i + 1) % n];
      const cross = Vec2.cross(a, b);
      areaSum += cross;
      c.x += (a.x + b.x) * cross;
      c.y += (a.y + b.y) * cross;
    }
    const area = areaSum * 0.5;
    if (Math.abs(area) < EPS_LEN * EPS_LEN) return new Vec2();
    return new Vec2(c.x / (6 * area), c.y / (6 * area));
  }

  // Polygon moment of inertia about the local origin, mass = 1.
  inertiaLocal() {
    const n = this.localVerts.length;
    let num = 0;
    let den = 0;
    for (let i = 0; i < n; i++) {
      const a = this.localVerts[i];
      const b = this.localVerts[(i + 1) % n];
      const cross = Math.abs(Vec2.cross(a, b));
      num += cross * (Vec2.dot(a, a) + Vec2.dot(a, b) + Vec2.dot(b, b));
      den += cross;
    }
    return den <= EPS_LEN * EPS_LEN ? 1 : num / (6 * den);
  }
}

export class BoxShape extends ConvexShape {
  constructor(halfWidth, halfHeight) {
    super(
      [
        new Vec2(-halfWidth, -halfHeight),
        new Vec2(halfWidth, -halfHeight),
        new Vec2(halfWidth, halfHeight),
        new Vec2(-halfWidth, halfHeight)
      ],
      `box ${halfWidth.toFixed(2)}x${halfHeight.toFixed(2)}`
    );
    this.halfWidth = halfWidth;
    this.halfHeight = halfHeight;
  }
}

// Right-angle wedge (triangular prism cross section): right angle at (-hx,-hy).
export class WedgeShape extends ConvexShape {
  constructor(halfWidth, halfHeight) {
    super(
      [
        new Vec2(-halfWidth, -halfHeight),
        new Vec2(halfWidth, -halfHeight),
        new Vec2(-halfWidth, halfHeight)
      ],
      `wedge ${halfWidth.toFixed(2)}x${halfHeight.toFixed(2)}`
    );
    this.halfWidth = halfWidth;
    this.halfHeight = halfHeight;
  }
}

// Any convex prop: raw points are deduplicated and a robust Andrew hull is built,
// so duplicated vertices and collinear input cannot produce a broken polygon.
// A degenerate input (point/line/two-point/non-finite) throws an error carrying
// an explicit `status` from validate.js, so callers never get a silently bad body.
export class RotatedConvex extends ConvexShape {
  constructor(rawPoints, name = 'rotated-convex') {
    const diagnosis = classifyRawPoints(rawPoints);
    let hull;
    try {
      hull = buildHull(rawPoints);
    } catch (err) {
      err.status = err.status ?? diagnosis.status;
      err.diagnosis = diagnosis;
      err.hullBlocked = diagnosis.hullBlocked ?? true;
      throw err;
    }
    super(hull, name);
    this.diagnosis = diagnosis;
  }
}

function dedupePoints(points) {
  const out = [];
  for (const p of points) {
    let seen = false;
    for (const q of out) {
      if (Vec2.dist2(p, q) <= EPS_LEN * EPS_LEN) {
        seen = true;
        break;
      }
    }
    if (!seen) out.push(p.clone());
  }
  return out;
}

// Andrew monotone-chain hull. Collinear points on an edge are dropped; the hull
// is returned CCW. Throws if the input is degenerate (line/point) or contains
// a non-finite coordinate, so a NaN vertex can never be silently dropped.
export function buildHull(rawPoints) {
  for (const p of rawPoints) {
    if (!Number.isFinite(p.x) || !Number.isFinite(p.y)) {
      const err = new Error('convex hull input contains a non-finite coordinate');
      err.status = 'non-finite';
      throw err;
    }
  }
  const pts = dedupePoints(rawPoints);
  if (pts.length < 3) throw new Error('convex hull needs >= 3 distinct points');
  pts.sort((a, b) => (a.x !== b.x ? a.x - b.x : a.y - b.y));

  const cross = (o, a, b) => signedArea2(o, a, b);
  const lower = [];
  for (const p of pts) {
    while (
      lower.length >= 2 &&
      cross(lower[lower.length - 2], lower[lower.length - 1], p) <= EPS_LEN * EPS_LEN
    ) {
      lower.pop();
    }
    lower.push(p);
  }
  const upper = [];
  for (let i = pts.length - 1; i >= 0; i--) {
    const p = pts[i];
    while (
      upper.length >= 2 &&
      cross(upper[upper.length - 2], upper[upper.length - 1], p) <= EPS_LEN * EPS_LEN
    ) {
      upper.pop();
    }
    upper.push(p);
  }
  lower.pop();
  upper.pop();
  const hull = lower.concat(upper);
  if (hull.length < 3) throw new Error('points are collinear: no convex hull exists');
  if (polygonArea2(hull) < 0) hull.reverse();
  return hull;
}
