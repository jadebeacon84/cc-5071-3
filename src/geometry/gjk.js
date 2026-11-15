import { Vec2 } from '../core/vec2.js';
import { EPS, EPS_LEN } from '../core/geom.js';

// A vertex of the Minkowski difference D = A - B stores the difference point
// plus its two witness vertices, so closest points on A and B are recoverable.
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
  if (trail) trail.steps.push({ dir: dir.clone(), ...clonePair(pair) });
  return pair;
}

function clonePair(p) {
  return {
    point: p.point.clone(),
    a: p.a.clone(),
    b: p.b.clone(),
    indexA: p.indexA,
    indexB: p.indexB
  };
}

// Closest point of a simplex (1..3 vertices) to the origin with barycentric
// weights and the active feature, used for distance reporting at termination.
function closestOnSimplex(s) {
  if (s.length === 1) {
    return { point: s[0].point.clone(), weights: [1], feature: 'vertex', keep: [0] };
  }
  if (s.length === 2) {
    const ab = Vec2.sub(s[1].point, s[0].point);
    const len2 = ab.len2();
    let t = len2 <= EPS_LEN * EPS_LEN ? 0 : -Vec2.dot(s[0].point, ab) / len2;
    if (t <= 1e-10) {
      return { point: s[0].point.clone(), weights: [1, 0], feature: 'vertex', keep: [0] };
    }
    if (t >= 1 - 1e-10) {
      return { point: s[1].point.clone(), weights: [0, 1], feature: 'vertex', keep: [1] };
    }
    t = Math.min(1, Math.max(0, t));
    return {
      point: Vec2.lerp(s[0].point, s[1].point, t),
      weights: [1 - t, t],
      feature: 'edge',
      keep: [0, 1]
    };
  }
  const inside = originInTriangle(s[0].point, s[1].point, s[2].point);
  if (inside.inside) {
    return { point: new Vec2(0, 0), weights: inside.weights, feature: 'interior', keep: [0, 1, 2] };
  }
  let best = null;
  for (let i = 0; i < 3; i++) {
    const a = s[i].point;
    const b = s[(i + 1) % 3].point;
    const ab = Vec2.sub(b, a);
    const len2 = ab.len2();
    const t = len2 <= EPS_LEN * EPS_LEN ? 0 : Math.min(1, Math.max(0, -Vec2.dot(a, ab) / len2));
    const q = Vec2.lerp(a, b, t);
    const d2 = q.len2();
    if (!best || d2 < best.d2) best = { d2, t, edge: i, point: q };
  }
  const i = best.edge;
  const j = (i + 1) % 3;
  const weights = [0, 0, 0];
  weights[i] = 1 - best.t;
  weights[j] = best.t;
  if (best.t <= 1e-10) return { point: s[i].point.clone(), weights, feature: 'vertex', keep: [i] };
  if (best.t >= 1 - 1e-10) return { point: s[j].point.clone(), weights, feature: 'vertex', keep: [j] };
  return { point: best.point, weights, feature: 'edge', keep: [i, j] };
}

// Winding-independent barycentric containment with boundary tolerance.
function originInTriangle(p0, p1, p2) {
  const pts = [p0, p1, p2];
  const areas = [Vec2.cross(p1, p2), Vec2.cross(p2, p0), Vec2.cross(p0, p1)];
  const total = areas[0] + areas[1] + areas[2];
  for (let i = 0; i < 3; i++) {
    const a = pts[i];
    const b = pts[(i + 1) % 3];
    const edge = Vec2.sub(b, a);
    const edgeLen = edge.len();
    if (edgeLen <= EPS_LEN) continue;
    const signedDist = Vec2.cross(edge, Vec2.neg(a)) / edgeLen;
    const oriented = total >= 0 ? signedDist : -signedDist;
    if (oriented < -EPS_LEN * 2) return { inside: false };
  }
  const weights = [0, 0, 0];
  if (Math.abs(total) > EPS_LEN * EPS_LEN) {
    weights[0] = areas[0] / total;
    weights[1] = areas[1] / total;
    weights[2] = areas[2] / total;
  } else {
    weights[0] = weights[1] = weights[2] = 1 / 3;
  }
  return { inside: true, weights };
}

function witnesses(simplex, weights) {
  const a = new Vec2();
  const b = new Vec2();
  for (let i = 0; i < simplex.length; i++) {
    const w = weights[i] || 0;
    a.x += simplex[i].a.x * w;
    a.y += simplex[i].a.y * w;
    b.x += simplex[i].b.x * w;
    b.y += simplex[i].b.y * w;
  }
  return { witnessA: a, witnessB: b };
}

// Line simplex update. Given A (newest) and B, return the next search direction
// toward the origin and the reduced simplex (1 or 2 vertices).
function updateLine(simplex) {
  const A = simplex[0].point;
  const B = simplex[1].point;
  const AB = Vec2.sub(B, A);
  const AO = Vec2.neg(A);
  const len2 = AB.len2();
  // Projection parameter of the origin onto the infinite line through A, B.
  const t = len2 <= EPS_LEN * EPS_LEN ? 0 : Vec2.dot(AO, AB) / len2;
  if (t > 0 && t < 1) {
    // Origin projects onto the open segment. The perpendicular direction is
    // normally toward the origin; if it vanishes the origin lies on the line,
    // which is ambiguous in 2D (boundary touch or a real overlap spanning the
    // line), so pick either perpendicular and let the next support vertex build
    // a triangle that resolves containment, or stall on a duplicate support.
    const perp = triple(AB, AO, AB);
    if (perp.len2() <= EPS_LEN * EPS_LEN) {
      const lift = Vec2.perpLeft(AB);
      return { simplex, dir: Vec2.normalize(lift.len2() > 0 ? lift : new Vec2(0, 1)) };
    }
    return { simplex, dir: Vec2.normalize(perp) };
  }
  // Origin projects outside the segment: the closest feature is an endpoint.
  // Reduce to that vertex and search straight toward the origin.
  if (t >= 1) return { simplex: [simplex[1]], dir: Vec2.normalize(Vec2.neg(B)) };
  return { simplex: [simplex[0]], dir: Vec2.normalize(AO) };
}

// Triangle simplex update. Winding-independent: the origin is a strict interior
// point only when it lies on the interior side of every edge by a non-trivial
// margin. An origin lying exactly on an edge (or through a vertex) is boundary
// touching, not overlap, and is routed to the distance path with a zero gap.
function updateTriangle(simplex) {
  const A = simplex[0].point;
  const B = simplex[1].point;
  const C = simplex[2].point;
  const pts = [A, B, C];

  const areas = [Vec2.cross(B, C), Vec2.cross(C, A), Vec2.cross(A, B)];
  const total = areas[0] + areas[1] + areas[2];
  const degenerate = Math.abs(total) <= EPS_LEN * EPS_LEN;
  if (!degenerate) {
    // Boundary band scales with the simplex: floating-point round-off on the
    // edge distance is ~ edgeLength * EPS, so an origin within a few ulps of an
    // edge is treated as on the edge (touch), while a genuine shallow overlap
    // sits comfortably outside this band.
    const scale = Math.max(Math.abs(A.len()), Math.abs(B.len()), Math.abs(C.len()), EPS_LEN);
    const boundaryBand = scale * EPS * 16;
    let onBoundary = false;
    let interior = true;
    for (let i = 0; i < 3; i++) {
      const a = pts[i];
      const b = pts[(i + 1) % 3];
      const edge = Vec2.sub(b, a);
      const len = edge.len();
      if (len <= EPS_LEN) {
        interior = false;
        break;
      }
      // Signed perpendicular distance of the origin from edge i, oriented to
      // be positive on the triangle's interior side. Normalize -0 to 0.
      const signedDist = Vec2.cross(edge, Vec2.neg(a)) / len;
      const oriented = (total >= 0 ? signedDist : -signedDist) || 0;
      if (oriented < -boundaryBand) {
        interior = false;
        break;
      }
      // The origin is only "on the boundary" when it both sits on the edge line
      // and projects inside the edge segment. A zero distance with the projection
      // outside the span means the origin is beyond a vertex, not on this face,
      // so it cannot establish boundary touching here.
      if (Math.abs(oriented) <= boundaryBand) {
        const u = len * len <= EPS_LEN * EPS_LEN ? 0 : Vec2.dot(Vec2.neg(a), edge) / (len * len);
        if (u >= 0 && u <= 1) onBoundary = true;
      }
    }
    if (interior && !onBoundary) return { contain: true };
    if (interior && onBoundary) return { touch: true };
  }

  // Not contained: pick the edge whose closest point is nearest the origin.
  // The next search direction points from that closest point to the origin.
  let best = null;
  for (let i = 0; i < 3; i++) {
    const a = pts[i];
    const b = pts[(i + 1) % 3];
    const ab = Vec2.sub(b, a);
    const len2 = ab.len2();
    const t = len2 <= EPS_LEN * EPS_LEN ? 0 : Math.min(1, Math.max(0, -Vec2.dot(a, ab) / len2));
    const q = Vec2.lerp(a, b, t);
    const d2 = q.len2();
    if (!best || d2 < best.d2) best = { d2, i, t, q };
  }
  const i0 = best.i;
  const i1 = (best.i + 1) % 3;
  // Preserve the newest-vertex-first ordering the line update expects.
  const ordered = i0 === 0 ? [simplex[i0], simplex[i1]] : [simplex[i1], simplex[i0]];
  if (best.t <= 1e-10 || best.t >= 1 - 1e-10) {
    // Closest feature is an endpoint: reduce to that vertex.
    const vert = best.t <= 1e-10 ? ordered[0] : ordered[1];
    return { simplex: [vert], dir: Vec2.normalize(Vec2.neg(vert.point)) };
  }
  return { simplex: ordered, dir: Vec2.normalize(Vec2.neg(best.q)) };
}

// Triple product a x (b x c) = b(a.c) - c(a.b).
function triple(a, b, c) {
  return Vec2.sub(Vec2.mul(b, Vec2.dot(a, c)), Vec2.mul(c, Vec2.dot(a, b)));
}

// GJK intersection + distance. Standard 2D simplex evolution: each iteration
// adds a support vertex toward the origin, then the line/triangle update keeps
// only the features the origin can still lie beyond and yields the next search
// direction. Intersection is reported the moment a triangle encloses the
// origin; otherwise the loop stalls on a support vertex already in the simplex
// and the surviving feature gives the exact separating distance/normal.
export function gjk(shapeA, shapeB, recordTrail = false) {
  const trail = recordTrail ? { steps: [] } : null;
  // First support on D = A - B is taken toward B.
  let dir = Vec2.normalize(Vec2.sub(shapeB.xf.p, shapeA.xf.p));
  if (dir.len2() === 0) dir = new Vec2(1, 0);

  let simplex = [supportPair(shapeA, shapeB, dir, trail)];
  dir = Vec2.neg(simplex[0].point);
  if (dir.len2() <= EPS_LEN * EPS_LEN) {
    // Origin coincides with a support vertex: boundary touch, zero gap.
    return touching(simplex, 1, 'origin-on-simplex', trail);
  }
  dir = Vec2.normalize(dir);

  let iterations = 0;
  const maxIterations = 32;
  let stopReason = 'max-iterations';
  let enclosedFlag = false;
  let boundaryFlag = false;

  while (iterations < maxIterations) {
    iterations++;
    if (dir.len2() <= EPS_LEN * EPS_LEN) {
      enclosedFlag = true;
      stopReason = 'origin-on-simplex';
      break;
    }
    const next = supportPair(shapeA, shapeB, dir, trail);

    // In GJK the support not crossing the origin does not end the search: that
    // vertex simply joins the simplex and the feature update picks the next
    // direction. The distance search only terminates when the support is
    // already a simplex vertex (the closest feature cannot improve) or the
    // origin is enclosed / lies on the simplex.
    if (simplex.some((v) => v.indexA === next.indexA && v.indexB === next.indexB)) {
      stopReason = 'duplicate-support';
      break;
    }

    simplex.unshift(next);

    if (simplex.length === 2) {
      const r = updateLine(simplex);
      simplex = r.simplex;
      dir = r.dir;
      // Zero direction means the origin lies on the surviving segment/vertex:
      // a boundary candidate. Touch vs shallow overlap is resolved by SAT.
      if (dir.len2() <= EPS_LEN * EPS_LEN) {
        boundaryFlag = true;
        stopReason = 'origin-on-simplex';
        break;
      }
    } else {
      const r = updateTriangle(simplex);
      if (r.contain) {
        enclosedFlag = true;
        stopReason = 'origin-enclosed';
        break;
      }
      if (r.touch) {
        // Origin lies on a triangle edge/vertex: a boundary candidate rather
        // than a strict interior point. SAT resolves touch vs shallow overlap.
        boundaryFlag = true;
        stopReason = 'origin-on-simplex';
        break;
      }
      simplex = r.simplex;
      dir = r.dir;
    }
  }

  if (enclosedFlag) {
    return enclosed(simplex, iterations, stopReason, trail);
  }

  const closest = closestOnSimplex(simplex);
  const { witnessA, witnessB } = witnesses(simplex, closest.weights);
  const delta = Vec2.sub(witnessB, witnessA);
  let normal = null;
  let distance = 0;
  // Convention: contact normal points from A toward B. An edge feature defines
  // a separating axis (negate the closest point on D = A - B). A vertex feature
  // falls back to the witness-to-witness direction.
  if (closest.feature === 'edge' && closest.point.len2() > EPS_LEN * EPS_LEN) {
    normal = Vec2.normalize(Vec2.neg(closest.point));
    distance = closest.point.len();
  } else if (delta.len2() > EPS_LEN * EPS_LEN) {
    normal = Vec2.normalize(delta);
    distance = Vec2.dot(delta, normal);
  } else {
    // Coincident witnesses: pick any axis, distance is effectively zero.
    normal = new Vec2(1, 0);
  }

  return {
    intersect: false,
    boundary: boundaryFlag,
    simplex: simplex.map(clonePair),
    closest: closest.point.clone(),
    weights: closest.weights.slice(),
    witnessA,
    witnessB,
    distance,
    normal,
    iterations,
    stopReason,
    trail
  };
}

function enclosed(simplex, iterations, stopReason, trail) {
  const closest = closestOnSimplex(simplex);
  const { witnessA, witnessB } = witnesses(simplex, closest.weights);
  return {
    intersect: true,
    boundary: false,
    simplex: simplex.map(clonePair),
    closest: closest.point.clone(),
    weights: closest.weights.slice(),
    witnessA,
    witnessB,
    distance: 0,
    normal: null,
    iterations,
    stopReason,
    trail
  };
}

// Boundary candidate: the origin lies on (not strictly inside) the simplex.
// The shapes at least touch; whether they merely share a point or have a real
// shallow overlap is resolved by the SAT reference in the narrow phase. Until
// then it is reported as non-intersecting with a zero gap and a boundary flag.
function touching(simplex, iterations, stopReason, trail) {
  const closest = closestOnSimplex(simplex);
  const { witnessA, witnessB } = witnesses(simplex, closest.weights);
  const delta = Vec2.sub(witnessB, witnessA);
  const normal = delta.len2() > EPS_LEN * EPS_LEN
    ? Vec2.normalize(delta)
    : new Vec2(1, 0);
  return {
    intersect: false,
    boundary: true,
    simplex: simplex.map(clonePair),
    closest: closest.point.clone(),
    weights: closest.weights.slice(),
    witnessA,
    witnessB,
    distance: 0,
    normal,
    iterations,
    stopReason,
    trail
  };
}
