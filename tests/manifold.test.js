import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Vec2 } from '../src/core/vec2.js';
import { Transform } from '../src/core/transform.js';
import { BoxShape, WedgeShape, RotatedConvex } from '../src/geometry/shapes.js';
import { gjk } from '../src/geometry/gjk.js';
import { epa } from '../src/geometry/epa.js';
import { resolveBoundary } from '../src/geometry/detect.js';
import { buildManifold, buildSeparatedContact } from '../src/geometry/manifold.js';

function place(shape, x, y, angle = 0) {
  shape.updateTransform(new Transform(new Vec2(x, y), angle));
  return shape;
}

function manifoldOf(a, b) {
  const g = gjk(a, b);
  const { overlap } = resolveBoundary(a, b, g);
  assert.equal(overlap, true, 'fixture must overlap');
  const e = epa(a, b, g);
  return { g, e, m: buildManifold(a, b, e) };
}

test('manifold normal is the push-out direction and moves B out of A', () => {
  const a = place(new BoxShape(1, 1), 0, 0);
  const b = place(new BoxShape(1, 1), 1.4, 0);
  const { e, m } = manifoldOf(a, b);
  assert.ok(Vec2.approx(m.normal, e.normal, 1e-9));
  for (const c of m.contacts) {
    assert.ok(c.depth > 0, 'penetration depth is positive');
    const moved = new BoxShape(1, 1);
    moved.updateTransform(new Transform(
      Vec2.add(b.xf.p, Vec2.mul(m.normal, c.depth + 1e-4)),
      b.xf.angle
    ));
    assert.equal(resolveBoundary(a, moved, gjk(a, moved)).overlap, false);
  }
});

test('face-to-face overlap yields two contacts spanning the reference edge', () => {
  const a = place(new BoxShape(1, 1), 0, 0);
  const b = place(new BoxShape(1, 1), 1.4, 0);
  const { m } = manifoldOf(a, b);
  assert.equal(m.contacts.length, 2);
  const span = Math.abs(Vec2.dot(
    Vec2.sub(m.contacts[1].point, m.contacts[0].point),
    m.tangent
  ));
  // The two contacts must preserve meaningful tangential extent (edge length 2),
  // not collapse onto a single arbitrary point.
  assert.ok(span > 1.5, `span ${span}`);
});

test('near-coplanar edge-edge contact selects a stable deterministic reference', () => {
  const run = () => {
    // Two wide shallow boxes overlapping slightly along x with near-parallel
    // (tiny relative rotation) contacting edges: an edge-edge near-coplanar case.
    const a = place(new BoxShape(0.7, 0.5), 0, 0, 0.003);
    const b = place(new BoxShape(0.7, 0.5), 1.38, 0, -0.002);
    return manifoldOf(a, b).m;
  };
  const first = run();
  for (let k = 0; k < 5; k++) {
    const again = run();
    assert.equal(again.refEdgeIndex, first.refEdgeIndex);
    assert.equal(again.refIsA, first.refIsA);
  }
  assert.ok(first.contacts.length >= 1);
});

test('duplicate and collinear hull input still produces a valid manifold', () => {
  const a = place(new RotatedConvex([
    new Vec2(0, 0.5), new Vec2(0, 0.5),
    new Vec2(0.48, 0.16), new Vec2(0.3, -0.4),
    new Vec2(0.15, -0.4), new Vec2(-0.3, -0.4), new Vec2(-0.48, 0.16)
  ], 'pentagon'), 0, 0);
  const b = place(new BoxShape(0.4, 0.4), 0.35, 0.1, 0.2);
  const { m } = manifoldOf(a, b);
  assert.ok(m.contacts.length >= 1 && m.contacts.length <= 2);
  for (const c of m.contacts) {
    assert.ok(Number.isFinite(c.depth) && c.depth >= 0);
    assert.ok(Math.abs(c.normal.len() - 1) < 1e-9);
  }
});

test('manifold reduction keeps at most two contacts with the widest span', () => {
  const a = place(new WedgeShape(0.8, 0.8), 0, 0, 0.4);
  const b = place(new BoxShape(0.5, 0.5), 0.55, 0.2, -0.3);
  const { m } = manifoldOf(a, b);
  assert.ok(m.contacts.length <= 2);
  if (m.contacts.length === 2) {
    const span = Math.abs(Vec2.dot(
      Vec2.sub(m.contacts[1].point, m.contacts[0].point),
      m.tangent
    ));
    assert.ok(span > 0);
  }
});

test('every contact stores local anchors on both bodies and a tangent key', () => {
  const a = place(new BoxShape(1, 1), 0, 0);
  const b = place(new BoxShape(1, 1), 1.4, 0);
  const { m } = manifoldOf(a, b);
  for (const c of m.contacts) {
    assert.ok(c.anchorA && c.anchorB);
    // Anchors are local points: transforming them back reproduces the world point.
    const wA = a.xf.localToWorld(c.anchorA);
    const wB = b.xf.localToWorld(c.anchorB);
    assert.ok(Vec2.approx(wA, c.point, 1e-7));
    assert.ok(Vec2.approx(wB, c.point, 1e-7));
    assert.equal(typeof c.tangentS, 'number');
  }
});

test('separated contact within slop returns a single non-penetrating point', () => {
  const a = place(new BoxShape(1, 1), 0, 0);
  const b = place(new BoxShape(1, 1), 2.005, 0); // gap 0.005 < CONTACT_SLOP
  const g = gjk(a, b);
  assert.equal(g.intersect, false);
  const sep = buildSeparatedContact(a, b, g);
  assert.ok(sep);
  assert.equal(sep.contacts.length, 1);
  assert.ok(sep.contacts[0].depth <= 0);
});

test('separated contact beyond slop returns null (no contact from mere proximity)', () => {
  const a = place(new BoxShape(1, 1), 0, 0);
  const b = place(new BoxShape(1, 1), 3, 0);
  const g = gjk(a, b);
  assert.equal(buildSeparatedContact(a, b, g), null);
});
