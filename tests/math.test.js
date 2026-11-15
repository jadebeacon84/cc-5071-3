import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Vec2 } from '../src/core/vec2.js';
import { Mat22 } from '../src/core/mat22.js';
import { Transform } from '../src/core/transform.js';
import { closestOnSegment, signedArea2, polygonArea2 } from '../src/core/geom.js';

test('Vec2 arithmetic and cross convention', () => {
  assert.ok(Vec2.approx(Vec2.add(new Vec2(1, 2), new Vec2(3, -1)), new Vec2(4, 1)));
  assert.equal(Vec2.cross(new Vec2(1, 0), new Vec2(0, 1)), 1);
  assert.equal(Vec2.cross(new Vec2(0, 1), new Vec2(1, 0)), -1);
  assert.ok(Vec2.approx(Vec2.perpLeft(new Vec2(1, 0)), new Vec2(0, 1)));
  assert.ok(Vec2.approx(Vec2.perpRight(new Vec2(1, 0)), new Vec2(0, -1)));
});

test('Mat22 round trip through transpose', () => {
  const r = Mat22.fromAngle(0.7);
  const v = new Vec2(2.3, -1.1);
  assert.ok(Vec2.approx(r.transposeMulVec(r.mulVec(v)), v, 1e-12));
  assert.ok(Math.abs(r.angle() - 0.7) < 1e-12);
});

test('Transform local/world mapping is invertible', () => {
  const xf = new Transform(new Vec2(3, -2), 1.1);
  const local = new Vec2(1.5, 0.7);
  const world = xf.localToWorld(local);
  assert.ok(Vec2.approx(xf.worldToLocal(world), local, 1e-12));
  assert.ok(Vec2.approx(xf.directionToLocal(xf.directionToWorld(local)), local, 1e-12));
});

test('closestOnSegment clamps to endpoints and interpolates', () => {
  const a = new Vec2(0, 0);
  const b = new Vec2(10, 0);
  assert.equal(closestOnSegment(a, b, new Vec2(5, 3)).t, 0.5);
  assert.equal(closestOnSegment(a, b, new Vec2(-4, 9)).t, 0);
  assert.equal(closestOnSegment(a, b, new Vec2(40, -9)).t, 1);
});

test('signed area sign: CCW positive', () => {
  assert.ok(signedArea2(new Vec2(0, 0), new Vec2(1, 0), new Vec2(0, 1)) > 0);
  assert.ok(polygonArea2([new Vec2(0, 0), new Vec2(1, 0), new Vec2(1, 1), new Vec2(0, 1)]) > 0);
});
