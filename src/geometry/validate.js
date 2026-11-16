import { Vec2 } from '../core/vec2.js';
import { EPS_LEN, polygonArea2, signedArea2 } from '../core/geom.js';

// Explicit status for a convex polygon. A degenerate input is never silently
// accepted: callers (CLI, UI, release checks) get a machine-readable state
// instead of a half-built shape.
//
// status:
//   'valid'            CCW, strictly convex, positive area, all edges non-zero
//   'point'            all points coincide (zero-dimensional)
//   'line'             distinct points but collinear (one-dimensional)
//   'concave'          polygon has a reflex vertex / non-positive turn
//   'bad-winding'      area <= 0 with otherwise non-degenerate geometry
//   'non-finite'       a coordinate is NaN / Infinity
//   'insufficient'     fewer than 3 distinct vertices
export function validateConvex(verts) {
  if (!Array.isArray(verts) || verts.length < 3) {
    return { ok: false, status: 'insufficient', reason: 'need >= 3 vertices', vertexCount: verts?.length ?? 0 };
  }
  for (const v of verts) {
    if (!Number.isFinite(v.x) || !Number.isFinite(v.y)) {
      return { ok: false, status: 'non-finite', reason: 'non-finite coordinate', vertexCount: verts.length };
    }
  }

  const distinct = countDistinct(verts);
  if (distinct < 2) {
    return { ok: false, status: 'point', reason: 'all points coincide', vertexCount: verts.length };
  }

  const area2 = polygonArea2(verts);
  const scale = maxCoordinate(verts);
  const areaTol = (EPS_LEN * Math.max(1, scale)) ** 2;
  if (Math.abs(area2) <= areaTol) {
    return collinearStatus(verts, scale, areaTol);
  }
  if (area2 < 0) {
    return { ok: false, status: 'bad-winding', reason: 'vertices are clockwise', vertexCount: verts.length, area: area2 * 0.5 };
  }

  // Strict convexity: every CCW turn at a vertex must be strictly positive.
  // Threshold scales with the two incident edges so large polygons are not
  // penalised by absolute round-off.
  const n = verts.length;
  const turnTol = areaTol;
  for (let i = 0; i < n; i++) {
    const a = verts[i];
    const b = verts[(i + 1) % n];
    const c = verts[(i + 2) % n];
    const edgeLen = Math.max(Vec2.dist(a, b), Vec2.dist(b, c), EPS_LEN);
    const cross = signedArea2(a, b, c);
    if (cross <= -turnTol * edgeLen) {
      return {
        ok: false,
        status: 'concave',
        reason: `reflex turn at vertex ${(i + 1) % n}`,
        vertex: (i + 1) % n,
        vertexCount: n,
        area: area2 * 0.5
      };
    }
  }

  return {
    ok: true,
    status: 'valid',
    vertexCount: n,
    distinct,
    area: area2 * 0.5
  };
}

function countDistinct(verts) {
  const tol2 = EPS_LEN * EPS_LEN;
  let count = 0;
  for (let i = 0; i < verts.length; i++) {
    let seen = false;
    for (let j = 0; j < i; j++) {
      if (Vec2.dist2(verts[i], verts[j]) <= tol2) {
        seen = true;
        break;
      }
    }
    if (!seen) count++;
  }
  return count;
}

function maxCoordinate(verts) {
  let m = EPS_LEN;
  for (const v of verts) m = Math.max(m, Math.abs(v.x), Math.abs(v.y));
  return m;
}

function collinearStatus(verts, scale, areaTol) {
  // Distinguish a true line (all points on one line) from generic insufficiency.
  const p0 = verts[0];
  let dir = null;
  for (const v of verts) {
    const d = Vec2.sub(v, p0);
    if (d.len2() > EPS_LEN * EPS_LEN) {
      dir = d;
      break;
    }
  }
  if (!dir) {
    return { ok: false, status: 'point', reason: 'all points coincide', vertexCount: verts.length };
  }
  for (const v of verts) {
    const cross = Math.abs(Vec2.cross(dir, Vec2.sub(v, p0)));
    if (cross > areaTol * scale) {
      return { ok: false, status: 'concave', reason: 'zero signed area but non-collinear points', vertexCount: verts.length };
    }
  }
  return { ok: false, status: 'line', reason: 'points are collinear: no 2D convex body', vertexCount: verts.length };
}

// Classify a raw point set the way RotatedConvex/buildHull sees it, before any
// hull construction. Lets the CLI and UI report why a prop cannot be built.
export function classifyRawPoints(points) {
  const result = validateConvex(points);
  if (result.ok) return result;
  // Duplicate-only collapse and collinear sets are the two hull-blocking
  // degeneracies; surface them with the same vocabulary the hull throws.
  if (result.status === 'line' || result.status === 'point' || result.status === 'insufficient') {
    return { ...result, hullBlocked: true };
  }
  return { ...result, hullBlocked: false };
}
