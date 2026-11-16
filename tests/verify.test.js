import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Vec2 } from '../src/core/vec2.js';
import { Transform } from '../src/core/transform.js';
import { BoxShape, WedgeShape } from '../src/geometry/shapes.js';
import { gjk } from '../src/geometry/gjk.js';
import { epa } from '../src/geometry/epa.js';
import { resolveBoundary } from '../src/geometry/detect.js';
import { buildManifold } from '../src/geometry/manifold.js';
import { separatingVector, verifyManifold } from '../src/geometry/verify.js';

function place(shape, x, y, angle = 0) {
  shape.updateTransform(new Transform(new Vec2(x, y), angle));
  return shape;
}

function overlappingManifold(a, b) {
  const g = gjk(a, b);
  assert.equal(resolveBoundary(a, b, g).overlap, true);
  const e = epa(a, b, g);
  return buildManifold(a, b, e);
}

test('separating vector of disjoint boxes equals the GJK gap along the axis', () => {
  const a = place(new BoxShape(1, 1), 0, 0);
  const b = place(new BoxShape(1, 1), 3, 0);
  const sep = separatingVector(a, b);
  assert.equal(sep.state, 'separate');
  assert.ok(Math.abs(sep.magnitude - 1) < 1e-9);
  assert.ok(Vec2.approx(sep.vector, new Vec2(1, 0), 1e-9));
  assert.ok(Vec2.approx(sep.normal, new Vec2(1, 0), 1e-9));
});

test('exact touching yields a zero separating vector with touch state', () => {
  const a = place(new BoxShape(1, 1), 0, 0);
  const b = place(new BoxShape(1, 1), 2, 0);
  const sep = separatingVector(a, b);
  assert.equal(sep.state, 'touch');
  assert.equal(sep.magnitude, 0);
  assert.equal(sep.vector.len2(), 0);
});

test('overlap separating vector points B out and its magnitude matches SAT depth', () => {
  const a = place(new BoxShape(1, 1), 0, 0);
  const b = place(new BoxShape(1, 1), 1.4, 0);
  const sep = separatingVector(a, b);
  assert.equal(sep.state, 'overlap');
  assert.ok(Math.abs(sep.magnitude - 0.6) < 1e-6);
  assert.ok(Vec2.approx(sep.normal, new Vec2(1, 0), 1e-9));
  // Applying the vector actually separates the bodies.
  const moved = place(new BoxShape(1, 1), 1.4 + sep.vector.x + 1e-4, sep.vector.y);
  assert.equal(resolveBoundary(a, moved, gjk(a, moved)).overlap, false);
});

test('a genuine face-to-face manifold passes every validity predicate', () => {
  const a = place(new BoxShape(1, 1), 0, 0);
  const b = place(new BoxShape(1, 1), 1.4, 0);
  const verdict = verifyManifold(a, b, overlappingManifold(a, b));
  assert.equal(verdict.ok, true, JSON.stringify(verdict.failedChecks));
  assert.equal(verdict.separating.state, 'overlap');
  assert.ok(Math.abs(verdict.distance.gjk) < 1e-9);
});

test('a fabricated manifold is rejected even when its numbers look plausible', () => {
  const a = place(new BoxShape(1, 1), 0, 0);
  const b = place(new BoxShape(1, 1), 5, 5); // far apart, no real contact
  const fake = {
    normal: new Vec2(1, 0),
    tangent: new Vec2(0, 1),
    contacts: [{
      point: new Vec2(9, 9),
      depth: 0.3,
      anchorA: a.xf.worldToLocal(new Vec2(9, 9)),
      anchorB: b.xf.worldToLocal(new Vec2(9, 9))
    }]
  };
  const verdict = verifyManifold(a, b, fake);
  assert.equal(verdict.ok, false);
  assert.ok(verdict.failedChecks.includes('separated-within-slop'));
});

test('a manifold with a tampered normal fails the live push-out predicate', () => {
  const a = place(new BoxShape(1, 1), 0, 0);
  const b = place(new BoxShape(1, 1), 1.4, 0);
  const m = overlappingManifold(a, b);
  // Flip the normal: moving B along it now drives B deeper into A.
  m.normal = Vec2.neg(m.normal);
  for (const c of m.contacts) c.normal = m.normal;
  const verdict = verifyManifold(a, b, m);
  assert.equal(verdict.ok, false);
  assert.ok(
    verdict.failedChecks.includes('normal-matches-sat') ||
    verdict.failedChecks.includes('pushout-actually-separates')
  );
});

test('rotated wedge contact manifold verifies with finite bounded depths', () => {
  const a = place(new WedgeShape(0.7, 0.7), 0, 0, 0.4);
  const b = place(new BoxShape(0.5, 0.5), 0.5, 0.2, -0.3);
  const verdict = verifyManifold(a, b, overlappingManifold(a, b));
  assert.equal(verdict.ok, true, JSON.stringify(verdict.failedChecks));
  assert.ok(verdict.checks.find((c) => c.name === 'anchors-reproduce-point').ok);
});
