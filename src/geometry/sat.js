import { Vec2 } from '../core/vec2.js';
import { EPS_LEN } from '../core/geom.js';

// Independent reference implementation: SAT over every edge normal.
// This is deliberately a separate algorithm path from GJK/EPA so that
// distances, penetration depth and normal direction can be cross-checked.
// Normal convention, identical to GJK/EPA output: n points from A toward B.
export function satReference(shapeA, shapeB) {
  const axesA = shapeA.edges().map((e) => ({ n: e.normal.clone(), source: 'A', edge: e.index }));
  const axesB = shapeB.edges().map((e) => ({ n: e.normal.clone(), source: 'B', edge: e.index }));
  const axes = axesA.concat(axesB);

  let best = null;
  for (const axis of axes) {
    const n = axis.n;
    let minA = Infinity;
    let maxA = -Infinity;
    let minB = Infinity;
    let maxB = -Infinity;
    let wA = null;
    let wB = null;
    for (const v of shapeA.worldVerts) {
      const d = Vec2.dot(v, n);
      if (d < minA) {
        minA = d;
        wA = v;
      }
      maxA = Math.max(maxA, d);
    }
    for (const v of shapeB.worldVerts) {
      const d = Vec2.dot(v, n);
      if (d > maxB) {
        maxB = d;
        wB = v;
      }
      minB = Math.min(minB, d);
    }
    // gap > 0 means disjoint along this axis.
    const gap = minA - maxB;
    if (!best || gap > best.gap) {
      best = { gap, n, axis, wA: wA.clone(), wB: wB.clone(), minA, maxA, minB, maxB };
    }
  }

  const separated = best.gap > -EPS_LEN;
  // Module-wide convention: the contact normal is the push-out direction for
  // B (the solver moves B in +n). Pick B's side from the centre-to-centre
  // direction, falling back to the witness chord for concentric bodies. The
  // witness chord alone can run along the contact and project near-zero.
  const toCentre = Vec2.sub(shapeB.xf.p, shapeA.xf.p);
  const refDir = toCentre.len2() > EPS_LEN * EPS_LEN
    ? toCentre
    : Vec2.sub(best.wB, best.wA);
  let n = best.n.clone();
  if (Vec2.dot(n, refDir) < 0) n = Vec2.neg(n);
  if (separated) {
    return {
      intersect: false,
      distance: Math.max(0, best.gap),
      depth: 0,
      normal: best.gap >= EPS_LEN ? n : null,
      witnessA: best.wA,
      witnessB: best.wB,
      axisSource: best.axis.source,
      edgeIndex: best.axis.edge
    };
  }
  // Overlapping: max gap is the least-negative penetration axis.
  return {
    intersect: true,
    distance: 0,
    depth: -best.gap,
    normal: n,
    witnessA: best.wA,
    witnessB: best.wB,
    axisSource: best.axis.source,
    edgeIndex: best.axis.edge
  };
}
