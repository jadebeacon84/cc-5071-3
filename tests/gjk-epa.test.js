import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Vec2 } from '../src/core/vec2.js';
import { Transform } from '../src/core/transform.js';
import { EPS } from '../src/core/geom.js';
import { BoxShape, WedgeShape, RotatedConvex } from '../src/geometry/shapes.js';
import { gjk } from '../src/geometry/gjk.js';
import { epa } from '../src/geometry/epa.js';
import { satReference } from '../src/geometry/sat.js';
import { resolveBoundary } from '../src/geometry/detect.js';

// Overlap resolution shared with the narrow phase: GJK decides strict
// separation, and a boundary candidate is resolved by the SAT reference.
function overlaps(a, b) {
  return resolveBoundary(a, b, gjk(a, b)).overlap;
}

function place(shape, x, y, angle = 0) {
  shape.updateTransform(new Transform(new Vec2(x, y), angle));
  return shape;
}

test('GJK reports separated boxes with exact gap and A->B normal', () => {
  const a = place(new BoxShape(1, 1), 0, 0);
  const b = place(new BoxShape(1, 1), 3, 0);
  const g = gjk(a, b);
  assert.equal(g.intersect, false);
  assert.ok(Math.abs(g.distance - 1) < 1e-6, `gap ${g.distance}`);
  assert.ok(Vec2.approx(g.normal, new Vec2(1, 0), 1e-9));
  assert.ok(Math.abs(g.witnessA.x - 1) < 1e-9);
  assert.ok(Math.abs(g.witnessB.x - 2) < 1e-9);
});

test('GJK separation returns the exact Euclidean closest-point distance', () => {
  // Reference: brute-force closest points over every vertex-to-edge pair.
  // GJK must match the true Euclidean gap (not a SAT projection), and its
  // normal must align with the witness direction within a small tolerance.
  function bruteClosest(shapeA, shapeB) {
    let best = Infinity;
    let bestA = null;
    let bestB = null;
    function segPoint(s1, s2, p) {
      const ab = Vec2.sub(s2, s1);
      const t = Math.min(1, Math.max(0, Vec2.dot(Vec2.sub(p, s1), ab) / ab.len2()));
      return Vec2.lerp(s1, s2, t);
    }
    function consider(pa, pb) {
      const d = Vec2.dist(pa, pb);
      if (d < best) {
        best = d;
        bestA = pa;
        bestB = pb;
      }
    }
    for (const p of shapeA.worldVerts) {
      for (let i = 0; i < shapeB.worldVerts.length; i++) {
        consider(p, segPoint(shapeB.worldVerts[i], shapeB.worldVerts[(i + 1) % shapeB.worldVerts.length], p));
      }
    }
    for (const p of shapeB.worldVerts) {
      for (let i = 0; i < shapeA.worldVerts.length; i++) {
        consider(segPoint(shapeA.worldVerts[i], shapeA.worldVerts[(i + 1) % shapeA.worldVerts.length], p), p);
      }
    }
    return { distance: best, normal: Vec2.normalize(Vec2.sub(bestB, bestA)) };
  }

  for (const angle of [0, 0.3, 0.9, 1.7, 2.6]) {
    const a = place(new BoxShape(0.8, 0.5), 0, 0, angle);
    const b = place(new WedgeShape(0.6, 0.6), 2.4, 0.4, -angle * 0.5);
    const g = gjk(a, b);
    const ref = bruteClosest(a, b);
    assert.equal(g.intersect, false);
    assert.ok(Math.abs(g.distance - ref.distance) < 1e-6,
      `angle ${angle}: gjk ${g.distance} vs brute ${ref.distance}`);
    assert.ok(Vec2.dot(g.normal, ref.normal) > 0.999,
      `angle ${angle}: gjk normal (${g.normal.x},${g.normal.y}) vs brute (${ref.normal.x},${ref.normal.y})`);
  }
});

test('GJK treats exact touching as separated with a zero gap (no false overlap)', () => {
  const a = place(new BoxShape(1, 1), 0, 0);
  const b = place(new BoxShape(1, 1), 2, 0); // faces coincide at x = 1
  const g = gjk(a, b);
  // Boundary touching shares a point but does not overlap: GJK and SAT must
  // agree it is non-penetrating, while still reporting zero separating gap and
  // a usable normal so resting contact is handled by the manifold path.
  assert.equal(g.intersect, false);
  assert.ok(g.boundary, 'exact touch is flagged as a boundary candidate');
  assert.ok(g.distance <= EPS, `gap should be zero, got ${g.distance}`);
  assert.ok(g.normal, 'a normal is still reported for resting contact');
  assert.ok(Math.abs(g.normal.len() - 1) < 1e-9);
  // The SAT reference must confirm the boundary candidate is not an overlap.
  assert.equal(overlaps(a, b), false);
});

test('GJK detects overlap and EPA depth matches SAT for axis-aligned boxes', () => {
  const a = place(new BoxShape(1, 1), 0, 0);
  const b = place(new BoxShape(1, 1), 1.4, 0);
  const g = gjk(a, b);
  assert.equal(overlaps(a, b), true);
  const e = epa(a, b, g);
  const sat = satReference(a, b);
  assert.ok(Math.abs(e.depth - 0.6) < 1e-5);
  assert.ok(Math.abs(e.depth - sat.depth) < 1e-5);
  assert.ok(Vec2.approx(e.normal, new Vec2(1, 0), 1e-9));
  assert.ok(Vec2.dot(e.normal, sat.normal) > 0.999);
  assert.equal(e.stopReason, 'boundary-reached');
});

test('EPA on rotated wedges agrees with SAT depth and normal', () => {
  for (const angle of [0.2, 0.8, 1.4]) {
    const a = place(new WedgeShape(0.7, 0.7), 0, 0, angle);
    const b = place(new BoxShape(0.6, 0.6), 0.5, 0.3, -angle * 0.7);
    const g = gjk(a, b);
    assert.equal(overlaps(a, b), true);
    const e = epa(a, b, g);
    const sat = satReference(a, b);
    // The answer is always SAT-exact; whether it came from polytope boundary
    // convergence or the polygon support-plane terminal phase is recorded.
    assert.ok(
      e.stopReason === 'boundary-reached' || e.stopReason === 'support-plane-enumeration',
      `unexpected stopReason ${e.stopReason}`
    );
    assert.ok(Math.abs(e.depth - sat.depth) < 2e-3,
      `angle ${angle}: epa ${e.depth} sat ${sat.depth}`);
    assert.ok(Vec2.dot(e.normal, sat.normal) > 0.99,
      `angle ${angle}: epa (${e.normal.x},${e.normal.y}) sat (${sat.normal.x},${sat.normal.y})`);
  }
});

test('EPA normal is the push-out direction: moving B by +depth*n separates', () => {
  const a = place(new RotatedConvex([
    new Vec2(0.8, 0), new Vec2(0.2, 0.7), new Vec2(-0.7, 0.4), new Vec2(-0.5, -0.6), new Vec2(0.3, -0.7)
  ]), 0, 0, 0.5);
  const b = place(new BoxShape(0.5, 0.5), 0.4, 0.2, -0.4);
  const g = gjk(a, b);
  const e = epa(a, b, g);
  // Module convention: the contact normal is B's push-out direction, so the
  // solver separates B by translating it along +n by the penetration depth.
  const moved = new BoxShape(0.5, 0.5);
  moved.updateTransform(new Transform(
    Vec2.add(b.xf.p, Vec2.mul(e.normal, e.depth + 1e-4)),
    b.xf.angle
  ));
  const after = gjk(a, moved);
  assert.equal(after.intersect, false, 'predicted push-out direction must actually separate');
  assert.equal(resolveBoundary(a, moved, after).overlap, false, 'SAT agrees the moved B is free');
});

test('GJK support trail records every query for visualization', () => {
  const a = place(new BoxShape(1, 1), 0, 0);
  const b = place(new BoxShape(1, 1), 1.5, 0.3);
  const g = gjk(a, b, true);
  assert.ok(g.trail.steps.length >= 3);
  for (const s of g.trail.steps) {
    assert.ok('dir' in s && 'point' in s && 'a' in s && 'b' in s);
  }
});

test('deeply nested shapes still produce a finite EPA result', () => {
  const a = place(new BoxShape(2, 2), 0, 0);
  const b = place(new BoxShape(0.4, 0.4), 0.05, 0.02);
  const g = gjk(a, b);
  assert.equal(overlaps(a, b), true);
  const e = epa(a, b, g);
  assert.ok(Number.isFinite(e.depth) && e.depth > 1.5);
  assert.ok(Math.abs(e.normal.len() - 1) < 1e-9);
});
