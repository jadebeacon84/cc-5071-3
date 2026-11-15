import { gjk } from './gjk.js';
import { epa } from './epa.js';
import { satReference } from './sat.js';
import { buildManifold, buildSeparatedContact } from './manifold.js';

// Narrow phase for one (A, B) pair. GJK first decides strict separation with an
// exact gap; only when it places the origin on a simplex boundary (ambiguous
// between exact touch and a shallow overlap) does the independent SAT reference
// resolve the signed gap. Overlaps then run EPA for depth/normal and build a
// clipping manifold; separated pairs within slop keep a single resting contact.
// AABB broadphase never reaches here, so no contact is born from box overlap.
export function detectContact(shapeA, shapeB, recordTrail = false) {
  const g = gjk(shapeA, shapeB, recordTrail);
  let sat = null;
  let penetration = null;
  let manifold = null;

  let overlap = g.intersect;
  if (!overlap && g.boundary) {
    sat = satReference(shapeA, shapeB);
    overlap = sat.intersect;
  }

  if (overlap) {
    penetration = epa(shapeA, shapeB, g, recordTrail);
    manifold = buildManifold(shapeA, shapeB, penetration);
  } else {
    const separated = buildSeparatedContact(shapeA, shapeB, g);
    if (!separated) return null;
    manifold = separated;
  }

  return { gjk: g, epa: penetration, sat, manifold, overlap };
}

// Resolve only the touch-vs-overlap question for a boundary GJK result, without
// building a manifold. Used by callers that need a boolean plus the SAT gap.
export function resolveBoundary(shapeA, shapeB, g) {
  if (g.intersect) return { overlap: true, sat: satReference(shapeA, shapeB) };
  if (!g.boundary) return { overlap: false, sat: null };
  const sat = satReference(shapeA, shapeB);
  return { overlap: sat.intersect, sat };
}
