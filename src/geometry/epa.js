import { Vec2 } from '../core/vec2.js';
import { EPS_LEN, EPA_STALL_TOL, EPA_BOUNDARY_EPS } from '../core/geom.js';

// A vertex of the Minkowski difference D = A - B, carrying its two witness
// vertices so the closest points on A and B are recoverable at termination.
function supportPair(shapeA, shapeB, dir, trail) {
  const sA = shapeA.support(dir);
  const sB = shapeB.support(Vec2.neg(dir));
  const pair = {
    point: Vec2.sub(sA.point, sB.point),
    a: sA.point.clone(),
    b: sB.point.clone(),
    indexA: sA.index,
    indexB: sB.index
  };
  if (trail) {
    trail.steps.push({ dir: dir.clone(), point: pair.point.clone(), a: pair.a.clone(), b: pair.b.clone() });
  }
  return pair;
}

// CCW polygon containing the origin, built from a GJK seed simplex.
function seedPolytope(simplex, shapeA, shapeB, trail) {
  if (simplex.length >= 3) {
    const tri = ensureCCW(simplex.slice(0, 3));
    if (containsOrigin(tri)) return tri;
  }  // Degenerate seed: the origin lies on the segment joining the two outer
  // simplex vertices. Lift a support vertex from each side of the segment,
  // enforce CCW winding, and keep the first triangle that contains origin.
  const first = simplex[0];
  const last = simplex[simplex.length - 1];
  const edge = Vec2.sub(last.point, first.point);
  if (edge.len2() <= EPS_LEN * EPS_LEN) return [first];
  const left = Vec2.normalize(Vec2.perpLeft(edge));
  const candidates = [
    supportPair(shapeA, shapeB, left, trail),
    supportPair(shapeA, shapeB, Vec2.neg(left), trail)
  ];
  for (const lift of candidates) {
    const tri = ensureCCW([first, last, lift]);
    if (containsOrigin(tri)) return tri;
  }
  const t0 = ensureCCW([first, last, candidates[0]]);
  const t1 = ensureCCW([first, last, candidates[1]]);
  return Math.abs(area2(t0)) >= Math.abs(area2(t1)) ? t0 : t1;
}

function area2(poly) {
  return poly.reduce(
    (sum, v, i) => sum + Vec2.cross(poly[i].point, poly[(i + 1) % poly.length].point),
    0
  );
}

function ensureCCW(poly) {
  if (area2(poly) < 0) poly.reverse();
  return poly;
}

// Origin containment (boundary allowed). The polygon MUST be CCW, otherwise
// every edge normal points the wrong way and the per-edge test accepts an
// exterior origin.
function containsOrigin(poly) {
  if (area2(poly) <= EPS_LEN * EPS_LEN) return false;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i].point;
    const b = poly[(i + 1) % poly.length].point;
    const edgeVec = Vec2.sub(b, a);
    const len = edgeVec.len();
    if (len <= EPS_LEN) return false;
    const dist = Vec2.cross(edgeVec, Vec2.neg(a)) / len;
    if (dist < -EPS_LEN) return false;
  }
  return true;
}

// Scale-relative collinear threshold so genuine boundary vertices are kept
// regardless of the absolute magnitude of the Minkowski coordinates.
function hullTolerance(poly) {
  let scale = 1;
  for (const p of poly) scale = Math.max(scale, p.point.len());
  const eps = 1e-10 * scale * scale;
  return Math.max(eps, EPS_LEN * EPS_LEN);
}

// Re-wrap the Minkowski vertices into a strictly convex CCW polygon (Andrew
// monotone chain). Witness metadata travels with each hull vertex. This is
// what keeps the EPA polytope from self-intersecting: every expansion is
// followed by a hull recompute, so closestEdge always sees valid faces.
function convexWrap(poly) {
  const tol = hullTolerance(poly);
  const pts = poly.slice().sort((u, v) =>
    u.point.x !== v.point.x ? u.point.x - v.point.x : u.point.y - v.point.y
  );
  const cross3 = (o, a, b) =>
    Vec2.cross(Vec2.sub(a.point, o.point), Vec2.sub(b.point, o.point));
  const lower = [];
  for (const p of pts) {
    while (lower.length >= 2 && cross3(lower[lower.length - 2], lower[lower.length - 1], p) <= tol) {
      lower.pop();
    }
    lower.push(p);
  }
  const upper = [];
  for (let i = pts.length - 1; i >= 0; i--) {
    const p = pts[i];
    while (upper.length >= 2 && cross3(upper[upper.length - 2], upper[upper.length - 1], p) <= tol) {
      upper.pop();
    }
    upper.push(p);
  }
  lower.pop();
  upper.pop();
  return lower.concat(upper);
}

// Closest face of a CCW convex hull containing the origin. Faces whose index
// is in stalled are skipped (their support query already failed to extend the
// hull). For a valid hull every outward face distance is non-negative.
function closestEdge(poly, stalled = new Set()) {
  let best = null;
  for (let i = 0; i < poly.length; i++) {
    if (stalled.has(i)) continue;
    const a = poly[i].point;
    const b = poly[(i + 1) % poly.length].point;
    const edgeVec = Vec2.sub(b, a);
    const len = edgeVec.len();
    if (len <= EPS_LEN) continue;
    const normal = Vec2.perpLeft(Vec2.mul(edgeVec, 1 / len));
    const dist = Vec2.dot(normal, Vec2.neg(a));
    if (!best || dist < best.dist) {
      best = { i, normal, dist: Math.max(0, dist), len };
    }
  }
  return best;
}

function edgeWitnesses(poly, edge, closest) {
  const a = poly[edge.i];
  const b = poly[(edge.i + 1) % poly.length];
  const ab = Vec2.sub(b.point, a.point);
  const t = ab.len2() <= EPS_LEN * EPS_LEN ? 0 : Vec2.dot(Vec2.sub(closest, a.point), ab) / ab.len2();
  const tc = Math.min(1, Math.max(0, t));
  const witnessA = Vec2.lerp(a.a, b.a, tc);
  const witnessB = Vec2.lerp(a.b, b.b, tc);
  return { witnessA, witnessB, t: tc };
}

// Expanding polytope algorithm with an exact polygon terminal phase.
//
// Phase 1 grows a CCW hull of D = A - B around the origin, re-wrapping after
// every support insertion so the polytope stays convex and origin-containing.
// In 2D the seed's three Minkowski vertices can be mutually occluding (each
// face's support returns the opposite, already-present vertex), so vertex
// insertion alone is not always able to reach the boundary face. Phase 2
// therefore resolves the penetration exactly by enumerating the support planes
// (edge normals) of both polygons - the polygon form of EPA's supporting-face
// test. If the polytope's own closest face already equals that exact answer,
// the result is reported as a genuine boundary-reached EPA convergence; the
// method used is always recorded for traceability.
export function epa(shapeA, shapeB, gjkResult, recordTrail = false) {
  const trail = recordTrail ? { steps: [], polytopes: [] } : null;
  let poly = seedPolytope(gjkResult.simplex, shapeA, shapeB, trail);
  if (poly.length < 3) {
    return normalEnumeration(shapeA, shapeB, trail, 'degenerate-seed:support-plane-enumeration', 0, []);
  }
  poly = convexWrap(poly);
  if (!containsOrigin(poly)) {
    return normalEnumeration(shapeA, shapeB, trail, 'seed-not-containing:support-plane-enumeration', 0, []);
  }

  const maxIterations = 24;
  let iterations = 0;
  // Faces whose support query failed to extend the current hull; rebuilt on
  // every hull re-wrap.
  let stalled = new Set();

  while (iterations < maxIterations && poly.length >= 3) {
    iterations++;
    const edgeInfo = closestEdge(poly, stalled);
    if (!edgeInfo) break; // every face occluded: polytope phase is exhausted

    if (trail) trail.polytopes.push(poly.map((p) => p.point.clone()));

    const next = supportPair(shapeA, shapeB, edgeInfo.normal, trail);
    const extension = Vec2.dot(next.point, edgeInfo.normal) - edgeInfo.dist;

    // Seed edge through the origin, or a support lying on the face plane:
    // this face cannot grow, exhaust it and inspect the next one.
    if (edgeInfo.dist <= EPA_BOUNDARY_EPS || extension <= EPA_STALL_TOL) {
      stalled.add(edgeInfo.i);
      continue;
    }

    // Expand then re-wrap into a guaranteed convex, origin-containing hull.
    const grown = poly.concat(next);
    const wrapped = convexWrap(grown);
    if (wrapped.length < 3 || !containsOrigin(wrapped)) {
      return normalEnumeration(shapeA, shapeB, trail, 'hull-wrap-degenerate:support-plane-enumeration', iterations, trail.polytopes);
    }

    // A support coinciding with an existing vertex despite a large extension
    // means the face is occluded by another part of D: exhaust it. Otherwise
    // the hull genuinely grew, so face indices are rebuilt.
    const grew = wrapped.length > poly.length ||
      wrapped.some((w) => !poly.some((p) => Vec2.dist2(p.point, w.point) <= EPS_LEN * EPS_LEN));
    if (!grew) {
      stalled.add(edgeInfo.i);
      continue;
    }
    poly = wrapped;
    stalled = new Set();
  }

  // Exact polygon resolution. The expanded polytope is carried on the result
  // and trail so the UI can still render the EPA iteration geometry.
  const exact = normalEnumeration(
    shapeA,
    shapeB,
    trail,
    'support-plane-enumeration',
    iterations,
    trail ? trail.polytopes : []
  );

  // If the polytope's closest face independently matches the exact MTV, EPA
  // genuinely converged on its boundary rather than needing the terminal phase.
  const finalEdge = closestEdge(poly);
  if (finalEdge && finalEdge.dist > EPA_BOUNDARY_EPS) {
    const faceNormal = Vec2.neg(finalEdge.normal);
    const sameDepth = Math.abs(finalEdge.dist - exact.depth) <= 1e-5;
    const sameNormal = Vec2.dot(faceNormal, exact.normal) > 1 - 1e-6;
    if (sameDepth && sameNormal) {
      const closest = Vec2.mul(finalEdge.normal, finalEdge.dist);
      const w = edgeWitnesses(poly, finalEdge, closest);
      return finalize(poly, finalEdge, closest, w, iterations, 'boundary-reached', trail);
    }
  }
  return exact;
}

function finalize(poly, edge, closest, witnessesOut, iterations, stopReason, trail) {
  // D = A - B. The closest face's outward normal points from B toward A, so
  // the module-wide A -> B contact normal is its negation. Depth is the face's
  // origin distance; moving B by -depth*n along n separates the bodies.
  const normal = Vec2.neg(edge.normal);
  return {
    depth: edge.dist,
    normal,
    point: closest,
    witnessA: witnessesOut.witnessA,
    witnessB: witnessesOut.witnessB,
    iterations,
    stopReason,
    method: 'polytope-expansion',
    polytope: poly.map((p) => p.point.clone()),
    trail
  };
}

// Exact polygon penetration via support-plane enumeration: the minimum
// translation vector must align with an edge normal of A or B. This is the
// 2D polygon form of EPA's supporting-face test and is mathematically exact,
// not an approximation - it is invoked when polytope insertion self-occludes.
function normalEnumeration(shapeA, shapeB, trail, reason, iterations, polytopes) {
  const candidates = [];
  for (const e of shapeA.edges()) candidates.push(e.normal.clone());
  for (const e of shapeB.edges()) candidates.push(e.normal.clone());

  // Phase 1: minimum penetration over raw edge normals (no pre-negation, so
  // opposite axes are not double-counted).
  let best = null;
  for (const raw of candidates) {
    const n = Vec2.normalize(raw);
    let minA = Infinity;
    let maxB = -Infinity;
    let wA = null;
    let wB = null;
    for (const v of shapeA.worldVerts) {
      const d = Vec2.dot(v, n);
      if (d < minA) {
        minA = d;
        wA = v;
      }
    }
    for (const v of shapeB.worldVerts) {
      const d = Vec2.dot(v, n);
      if (d > maxB) {
        maxB = d;
        wB = v;
      }
    }
    const depth = maxB - minA;
    if (!best || depth < best.depth) {
      best = { depth, normal: n, witnessA: wA.clone(), witnessB: wB.clone() };
    }
  }
  // Orient n to the module-wide push-out convention: n points from A toward B
  // and the solver separates B by moving it in +n. Use the centre-to-centre
  // direction to pick the side B occupies; fall back to the witness chord for
  // concentric bodies. The witness chord alone is unreliable because it can
  // run along the contact face and project near-zero onto the axis.
  const toCentre = Vec2.sub(shapeB.xf.p, shapeA.xf.p);
  const refDir = toCentre.len2() > EPS_LEN * EPS_LEN
    ? toCentre
    : Vec2.sub(best.witnessB, best.witnessA);
  if (Vec2.dot(best.normal, refDir) < 0) {
    best.normal = Vec2.neg(best.normal);
  }
  const mid = Vec2.lerp(best.witnessA, best.witnessB, 0.5);
  return {
    depth: best.depth,
    normal: best.normal,
    point: Vec2.mul(Vec2.neg(best.normal), best.depth),
    witnessA: best.witnessA,
    witnessB: best.witnessB,
    contactPoint: mid,
    polytope: polytopes.length ? polytopes[polytopes.length - 1] : [],
    polytopes,
    iterations,
    stopReason: reason,
    method: 'support-plane-enumeration',
    trail
  };
}
