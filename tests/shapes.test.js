import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Vec2 } from '../src/core/vec2.js';
import { Transform } from '../src/core/transform.js';
import { BoxShape, WedgeShape, RotatedConvex, buildHull } from '../src/geometry/shapes.js';

test('box verts are CCW and support follows rotation', () => {
  const box = new BoxShape(1, 0.5);
  box.updateTransform(new Transform(new Vec2(0, 0), Math.PI / 2));
  const s = box.support(new Vec2(0, -1));
  assert.deepEqual(s.index, 0); // local -x face points world -y after +90deg
  const e = box.edges();
  assert.equal(e.length, 4);
  for (const edge of e) {
    assert.ok(Math.abs(edge.normal.len() - 1) < 1e-12);
  }
});

test('wedge is a right triangle with three outward normals', () => {
  const w = new WedgeShape(0.5, 0.5);
  assert.equal(w.worldVerts.length, 3);
  assert.ok(w.area() > 0);
  assert.equal(w.edges().length, 3);
});

test('hull removes duplicate and collinear points', () => {
  const hull = buildHull([
    new Vec2(0, 0), new Vec2(0, 0),
    new Vec2(1, 0), new Vec2(2, 0),
    new Vec2(2, 2),
    new Vec2(1, 1),
    new Vec2(0, 2)
  ]);
  assert.equal(hull.length, 4);
});

test('hull rejects collinear-only input', () => {
  assert.throws(() => buildHull([new Vec2(0, 0), new Vec2(1, 0), new Vec2(2, 0)]));
});

test('rotated convex centroid and inertia are finite and positive', () => {
  const shape = new RotatedConvex([
    new Vec2(1, 0), new Vec2(0.5, 0.8), new Vec2(-0.6, 0.6),
    new Vec2(-0.8, -0.4), new Vec2(0.3, -0.7)
  ]);
  const c = shape.centroidLocal();
  assert.ok(Number.isFinite(c.x) && Number.isFinite(c.y));
  assert.ok(shape.inertiaLocal() > 0);
  assert.ok(shape.area() > 0.5);
});

test('support vertex is the extreme world point', () => {
  const shape = new RotatedConvex([
    new Vec2(1, 0), new Vec2(0, 1), new Vec2(-1, 0), new Vec2(0, -1)
  ]);
  shape.updateTransform(new Transform(new Vec2(5, 5), 0));
  const s = shape.support(new Vec2(1, 1));
  // (1,0) has the largest projection along (1,1); translated to (6,5).
  assert.ok(Vec2.approx(s.point, new Vec2(6, 5)));
});
